/**
 * map_odom_localizer.cpp
 *
 * Localizes a robot within a pre-built .ply map using FastVGICP.
 *
 * ── INITIALIZATION MODES (param: "init_mode") ────────────────────────────────
 *
 *  "tf_prior"      Original behaviour. Requires the EKF to publish
 *                  map→base_link before the first scan can be processed.
 *
 *  "full_pose"     You know x, y, and yaw (heading in radians).
 *                  Set init_x, init_y, init_yaw.  One ICP call locks in the
 *                  pose and seeds the EKF; normal TF-prior tracking follows.
 *
 *  "position_only" You know x and y but NOT the heading.
 *                  Set init_x, init_y.  The node tries init_heading_candidates
 *                  evenly-spaced yaws (coarse ICP each), picks the best, then
 *                  refines with a full-resolution ICP.
 *
 *  "global"        No prior at all.  A grid of init_search_step metres is
 *                  built over every occupied map cell; a full heading sweep is
 *                  run at each cell.  The best candidate is then refined.
 *                  ⚠ Can take 10–90 s on large maps — this is expected.
 *
 * ── TRACKING (all modes) ──────────────────────────────────────────────────────
 *  Once initialization succeeds the node enters TRACKING state:
 *    1. Try to get map→base_link from the EKF TF tree as the ICP prior.
 *    2. If TF is unavailable (EKF still cold-starting), fall back to the last
 *       accepted pose.  (TF_PRIOR mode skips this fallback — original behaviour.)
 *    3. Publish PoseWithCovarianceStamped for the EKF on every good scan.
 */

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/transforms.h>

#ifdef USE_CUDA_VGICP
  #include <fast_vgicp/cuda/fast_vgicp_cuda.cuh>
  using VGICPVariant = fast_gicp::FastVGICPCuda<pcl::PointXYZI, pcl::PointXYZI>;
#else
  #include <fast_gicp/gicp/fast_vgicp.hpp>
  using VGICPVariant = fast_gicp::FastVGICP<pcl::PointXYZI, pcl::PointXYZI>;
#endif

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

// ─────────────────────────────────────────────────────────────────────────────
//  Enumerations & free helpers
// ─────────────────────────────────────────────────────────────────────────────

enum class InitMode {
    TF_PRIOR,       ///< Original: always require EKF TF prior
    FULL_POSE,      ///< x, y, yaw known
    POSITION_ONLY,  ///< x, y known; heading unknown → heading sweep
    GLOBAL,         ///< Nothing known → grid search over map
};

enum class LocalizerState {
    NEEDS_INIT,  ///< Waiting for the first successful localization
    TRACKING,    ///< Normal closed-loop operation
};

/// Build a ground-plane Isometry3d from (x, y, yaw).
static Eigen::Isometry3d makeXYYaw(double x, double y, double yaw)
{
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.translation()     = Eigen::Vector3d(x, y, 0.0);
    T.linear()          = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
                              .toRotationMatrix();
    return T;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Node
// ─────────────────────────────────────────────────────────────────────────────

class MapOdomLocalizer : public rclcpp::Node
{
public:
    MapOdomLocalizer() : Node("map_odom_localizer")
    {
        loadParams();

        // Give the TF buffer time to cache messages while the heavy map loads.
        tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        loadMap();
        buildVGICP();

        pub_pose_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "localizer/pose", rclcpp::QoS(10));

        // SensorDataQoS (best-effort, depth 1): drop stale scans if ICP is slow.
        sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            lidar_topic_, rclcpp::SensorDataQoS(),
            std::bind(&MapOdomLocalizer::cloudCallback, this, std::placeholders::_1));

        logInitMode();
    }

private:

    // =========================================================================
    //  Parameter loading
    // =========================================================================

    void loadParams()
    {
        // ── Core ───────────────────────────────────
        declare_parameter("map_frame",              "map");
        declare_parameter("base_frame",             "base_link");
        declare_parameter("lidar_topic",            "/ouster/points");
        declare_parameter("map_ply_path",           "");
        declare_parameter("voxel_leaf_map",         0.3);
        declare_parameter("voxel_leaf_scan",        0.1);
        declare_parameter("vgicp_resolution",       1);
        declare_parameter("vgicp_max_iterations",   64);
        declare_parameter("vgicp_max_corresp_dist", 1.5);
        declare_parameter("max_fitness_accept",     0.5);

        // ── Initialization mode ──────────────────────────────────────────────
        // "tf_prior" | "full_pose" | "position_only" | "global"
        declare_parameter("init_mode", "tf_prior");

        // Known position (used by full_pose and position_only)
        declare_parameter("init_x",   0.0);
        declare_parameter("init_y",   0.0);
        // Known heading in radians — only used for full_pose
        declare_parameter("init_yaw", 0.0);

        // Heading candidates to try per position (position_only and global)
        declare_parameter("init_heading_candidates", 16);

        // Grid step in metres for global search
        declare_parameter("global_search_step", 5.0);

        // ICP iteration budget for the *search* phase.
        // Fewer iterations = faster sweep; the best candidate is always
        // refined to vgicp_max_iterations afterwards.
        declare_parameter("init_search_max_iter", 20);

        // Override the default EKF starting pose
        declare_parameter("ekf_reset_topic", "/set_pose");

        // ── Read values ──────────────────────────────────────────────────────
        map_frame_    = get_parameter("map_frame").as_string();
        base_frame_   = get_parameter("base_frame").as_string();
        lidar_topic_  = get_parameter("lidar_topic").as_string();
        map_ply_path_ = get_parameter("map_ply_path").as_string();

        voxel_leaf_map_  = get_parameter("voxel_leaf_map").as_double();
        voxel_leaf_scan_ = get_parameter("voxel_leaf_scan").as_double();
        max_fitness_     = get_parameter("max_fitness_accept").as_double();

        vgicp_resolution_ = get_parameter("vgicp_resolution").as_int();
        vgicp_max_iter_   = get_parameter("vgicp_max_iterations").as_int();
        vgicp_corr_dist_  = get_parameter("vgicp_max_corresp_dist").as_double();

        init_x_   = get_parameter("init_x").as_double();
        init_y_   = get_parameter("init_y").as_double();
        init_yaw_ = get_parameter("init_yaw").as_double();

        init_heading_candidates_ = get_parameter("init_heading_candidates").as_int();
        global_search_step_      = get_parameter("global_search_step").as_double();
        init_search_max_iter_    = get_parameter("init_search_max_iter").as_int();

        ekf_reset_topic_ = get_parameter("ekf_reset_topic").as_string();

        const std::string mode_str = get_parameter("init_mode").as_string();
        if      (mode_str == "full_pose")     init_mode_ = InitMode::FULL_POSE;
        else if (mode_str == "position_only") init_mode_ = InitMode::POSITION_ONLY;
        else if (mode_str == "global")        init_mode_ = InitMode::GLOBAL;
        else {
            if (mode_str != "tf_prior")
                RCLCPP_WARN(get_logger(),
                    "Unknown init_mode '%s' — defaulting to 'tf_prior'.", mode_str.c_str());
            init_mode_ = InitMode::TF_PRIOR;
        }

        // TF_PRIOR skips NEEDS_INIT entirely (original behaviour).
        if (init_mode_ == InitMode::TF_PRIOR)
            state_ = LocalizerState::TRACKING;
    }

    void logInitMode() const
    {
        switch (init_mode_) {
            case InitMode::TF_PRIOR:
                RCLCPP_INFO(get_logger(),
                    "[init] mode=TF_PRIOR — waiting for EKF map→base_link TF.");
                break;
            case InitMode::FULL_POSE:
                RCLCPP_INFO(get_logger(),
                    "[init] mode=FULL_POSE  x=%.2f  y=%.2f  yaw=%.3f rad",
                    init_x_, init_y_, init_yaw_);
                break;
            case InitMode::POSITION_ONLY:
                RCLCPP_INFO(get_logger(),
                    "[init] mode=POSITION_ONLY  x=%.2f  y=%.2f  "
                    "(sweeping %d headings on first scan)",
                    init_x_, init_y_, init_heading_candidates_);
                break;
            case InitMode::GLOBAL:
                RCLCPP_INFO(get_logger(),
                    "[init] mode=GLOBAL  grid_step=%.1f m  headings=%d  "
                    "⚠ may take 10–90 s on first scan for large maps",
                    global_search_step_, init_heading_candidates_);
                break;
        }
    }

    // =========================================================================
    //  Map loading
    // =========================================================================

    void loadMap()
    {
        if (map_ply_path_.empty()) {
            RCLCPP_FATAL(get_logger(), "map_ply_path is not set. Cannot localise.");
            rclcpp::shutdown();
            return;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr raw_xyz(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPLYFile<pcl::PointXYZ>(map_ply_path_, *raw_xyz) < 0) {
            RCLCPP_FATAL(get_logger(), "Failed to load PLY map: %s", map_ply_path_.c_str());
            rclcpp::shutdown();
            return;
        }
        if (raw_xyz->empty()) {
            RCLCPP_FATAL(get_logger(),
                "Map loaded but is empty — PLY may have XYZRGB fields. "
                "Re-save as XYZ-only or change the load type.");
            rclcpp::shutdown();
            return;
        }

        // Convert XYZ → XYZI (intensity unused; set to 0)
        pcl::PointCloud<pcl::PointXYZI>::Ptr raw_xyzi(new pcl::PointCloud<pcl::PointXYZI>);
        raw_xyzi->reserve(raw_xyz->size());
        for (const auto & p : *raw_xyz) {
            pcl::PointXYZI pi;
            pi.x = p.x;  pi.y = p.y;  pi.z = p.z;  pi.intensity = 0.0f;
            raw_xyzi->push_back(pi);
        }

        map_cloud_ = downsample(raw_xyzi, voxel_leaf_map_);
        RCLCPP_INFO(get_logger(), "Map loaded: %zu pts (downsampled from %zu)",
            map_cloud_->size(), raw_xyzi->size());

        // Pre-compute bbox (needed for logging in global search)
        computeMapBBox();
    }

    void computeMapBBox()
    {
        map_min_x_ = map_min_y_ =  std::numeric_limits<float>::max();
        map_max_x_ = map_max_y_ = -std::numeric_limits<float>::max();
        for (const auto & pt : *map_cloud_) {
            map_min_x_ = std::min(map_min_x_, pt.x);
            map_max_x_ = std::max(map_max_x_, pt.x);
            map_min_y_ = std::min(map_min_y_, pt.y);
            map_max_y_ = std::max(map_max_y_, pt.y);
        }
        RCLCPP_INFO(get_logger(),
            "Map bbox: x=[%.1f, %.1f]  y=[%.1f, %.1f]  "
            "(%.0f × %.0f m)",
            map_min_x_, map_max_x_, map_min_y_, map_max_y_,
            static_cast<double>(map_max_x_ - map_min_x_),
            static_cast<double>(map_max_y_ - map_min_y_));
    }

    // =========================================================================
    //  VGICP setup
    // =========================================================================

    void buildVGICP()
    {
        vgicp_ = std::make_shared<VGICPVariant>();
        vgicp_->setResolution(vgicp_resolution_);
        vgicp_->setMaxCorrespondenceDistance(vgicp_corr_dist_);
        vgicp_->setMaximumIterations(vgicp_max_iter_);
        vgicp_->setTransformationEpsilon(1e-4);
        vgicp_->setEuclideanFitnessEpsilon(1e-4);
#ifndef USE_CUDA_VGICP
        vgicp_->setNumThreads(0);
        RCLCPP_INFO(get_logger(), "Matcher: FastVGICP (CPU, all threads)");
#else
        RCLCPP_INFO(get_logger(), "Matcher: cuVGICP (CUDA GPU)");
#endif
        vgicp_->setInputTarget(map_cloud_);

        pub_ekf_reset_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            ekf_reset_topic_, rclcpp::QoS(1).reliable());
    }

    // =========================================================================
    //  Main scan callback
    // =========================================================================

    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        const rclcpp::Time stamp = msg->header.stamp;

        // Transform scan into base_link frame and downsample.
        auto scan_base = toBaseFrame(msg, stamp);
        if (!scan_base) return;
        const auto scan_ds = downsample(*scan_base, voxel_leaf_scan_);

        // ── First-scan bootstrap ─────────────────────────────────────────────
        if (state_ == LocalizerState::NEEDS_INIT) {
            runInitialization(scan_ds, stamp);
            return;
        }

        // ── Normal tracking ──────────────────────────────────────────────────
        auto prior = getPrior(stamp);
        if (!prior) return;

        // fast_gicp caches source covariances after the first setInputSource call,
        // so re-setting with the same cloud on every tick is fine (no-op if the
        // pointer matches). The call is cheap either way.
        vgicp_->setInputSource(scan_ds);
        pcl::PointCloud<pcl::PointXYZI> aligned;
        vgicp_->align(aligned, prior->matrix().cast<float>());

        const double fitness   = vgicp_->getFitnessScore();
        const bool   converged = vgicp_->hasConverged();

        if (!converged || fitness > max_fitness_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "[TRACKING] ICP failed — converged=%s  fitness=%.4f  thresh=%.4f. "
                "Scan dropped.",
                converged ? "true" : "false", fitness, max_fitness_);
            return;
        }

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
            "[TRACKING] fitness=%.4f", fitness);

        Eigen::Isometry3d result;
        result.matrix() = vgicp_->getFinalTransformation().cast<double>();
        last_good_pose_  = result;   // always keep a fallback prior
        publishPose(result, stamp, fitness);
    }

    // =========================================================================
    //  Initialization dispatcher
    // =========================================================================

    void runInitialization(const pcl::PointCloud<pcl::PointXYZI>::Ptr & scan,
                           const rclcpp::Time & stamp)
    {
        RCLCPP_INFO(get_logger(), "[init] Processing first scan for initialization…");

        // Set source ONCE here — all tryAlignAt calls reuse the cached
        // source covariances without re-preprocessing.
        vgicp_->setInputSource(scan);

        switch (init_mode_) {

            // ── MODE 1: Full pose known ───────────────────────────────────────
            case InitMode::FULL_POSE: {
                auto guess = makeXYYaw(init_x_, init_y_, init_yaw_);
                auto [pose, fitness] = tryAlignAt(guess, vgicp_max_iter_);

                if (fitness > max_fitness_) {
                    RCLCPP_WARN(get_logger(),
                        "[init] FULL_POSE ICP fitness=%.4f > threshold=%.4f. "
                        "Check init_x/init_y/init_yaw. Will retry on next scan.",
                        fitness, max_fitness_);
                    return;  // stay in NEEDS_INIT
                }

                RCLCPP_INFO(get_logger(),
                    "[init] FULL_POSE locked in — fitness=%.4f  "
                    "pos=(%.2f, %.2f)",
                    fitness,
                    pose.translation().x(), pose.translation().y());
                finishInit(pose, stamp, fitness);
                return;
            }

            // ── MODE 2: Position known, heading unknown ───────────────────────
            case InitMode::POSITION_ONLY: {
                auto coarse = headingSweep(init_x_, init_y_);
                if (!coarse) return;  // warning already logged inside sweep

                // Refine winner with full iteration budget
                auto [refined, fitness] = tryAlignAt(*coarse, vgicp_max_iter_);
                RCLCPP_INFO(get_logger(),
                    "[init] POSITION_ONLY refined fitness=%.4f  "
                    "pos=(%.2f, %.2f)  yaw=%.3f rad",
                    fitness,
                    refined.translation().x(), refined.translation().y(),
                    std::atan2(refined.rotation()(1, 0), refined.rotation()(0, 0)));
                finishInit(refined, stamp, fitness);
                return;
            }

            // ── MODE 3: Nothing known — grid search ───────────────────────────
            case InitMode::GLOBAL: {
                auto coarse = globalSearch();
                if (!coarse) return;  // warning already logged inside search

                auto [refined, fitness] = tryAlignAt(*coarse, vgicp_max_iter_);
                RCLCPP_INFO(get_logger(),
                    "[init] GLOBAL refined fitness=%.4f  "
                    "pos=(%.2f, %.2f)  yaw=%.3f rad",
                    fitness,
                    refined.translation().x(), refined.translation().y(),
                    std::atan2(refined.rotation()(1, 0), refined.rotation()(0, 0)));
                finishInit(refined, stamp, fitness);
                return;
            }

            case InitMode::TF_PRIOR:
                // Should never reach here — state_ is set to TRACKING in loadParams().
                state_ = LocalizerState::TRACKING;
                return;
        }
    }

    /// Transition to TRACKING and publish the initial pose for the EKF.
    void finishInit(const Eigen::Isometry3d & pose, const rclcpp::Time & stamp, double fitness)
    {
        last_good_pose_ = pose;
        state_           = LocalizerState::TRACKING;
        publishPose(pose, stamp, fitness);
        // Also hard-reset the EKF to this pose so it doesn't spend
        // several seconds dragging in from (0, 0).
        // robot_localization treats a message on its set_pose topic as
        // an immediate filter reinitialisation.
        RCLCPP_INFO(get_logger(),
            "[init] Resetting EKF to (%.2f, %.2f) via '%s'",
            pose.translation().x(), pose.translation().y(),
            ekf_reset_topic_.c_str());
        pub_ekf_reset_->publish(buildPoseMsg(pose, stamp, fitness));

        RCLCPP_INFO(get_logger(),
            "[init] ✓ Initialization complete — now in TRACKING mode.");
    }

    // =========================================================================
    //  Heading sweep  (MODE 2 and per-cell in MODE 3)
    // =========================================================================

    /// Try init_heading_candidates_ evenly-spaced yaws at (x, y).
    /// vgicp_ source must already be set before calling.
    /// Returns the coarse-ICP pose with the lowest fitness, or nullopt if none
    /// beat max_fitness_.
    std::optional<Eigen::Isometry3d>
    headingSweep(double x, double y)
    {
        double best_fitness = std::numeric_limits<double>::max();
        std::optional<Eigen::Isometry3d> best_pose;

        const double step = 2.0 * M_PI / init_heading_candidates_;
        for (int i = 0; i < init_heading_candidates_; ++i) {
            const double yaw = i * step;
            auto [pose, fitness] = tryAlignAt(makeXYYaw(x, y, yaw),
                                              init_search_max_iter_);
            RCLCPP_DEBUG(get_logger(),
                "[init/sweep] yaw=%.2f rad  fitness=%.4f", yaw, fitness);
            if (fitness < best_fitness) {
                best_fitness = fitness;
                best_pose    = pose;
            }
        }

        RCLCPP_INFO(get_logger(),
            "[init/sweep] Done at (%.2f, %.2f) — best fitness=%.4f",
            x, y, best_fitness);

        if (best_fitness > max_fitness_) {
            RCLCPP_WARN(get_logger(),
                "[init/sweep] Best fitness %.4f > threshold %.4f. "
                "Verify init_x / init_y. Will retry on next scan.",
                best_fitness, max_fitness_);
            return std::nullopt;
        }
        return best_pose;
    }

    // =========================================================================
    //  Global grid search  (MODE 3)
    // =========================================================================

    /// Grid over every occupied map cell × heading sweep.
    /// vgicp_ source must already be set before calling.
    ///
    /// Candidate positions are derived directly from the map point cloud:
    /// each cell that contains ≥1 map point becomes one candidate (at the
    /// cell centre).  Empty cells (open space, voids) are skipped, which
    /// keeps the call count proportional to the mapped area, not the bbox.
    std::optional<Eigen::Isometry3d> globalSearch()
    {
        // ── Build the set of occupied grid cells ─────────────────────────────
        // Two int32_t packed into a uint64_t — sign-safe via uint32_t reinterpret.
        auto cellKey = [this](float x, float y) -> uint64_t {
            const auto ix = static_cast<int32_t>(std::floor(x / global_search_step_));
            const auto iy = static_cast<int32_t>(std::floor(y / global_search_step_));
            return (static_cast<uint64_t>(static_cast<uint32_t>(ix)) << 32)
                 |  static_cast<uint64_t>(static_cast<uint32_t>(iy));
        };

        std::unordered_set<uint64_t> occupied;
        occupied.reserve(map_cloud_->size());
        for (const auto & pt : *map_cloud_)
            occupied.insert(cellKey(pt.x, pt.y));

        // Convert cells back to metric centre positions
        struct CandXY { float x, y; };
        std::vector<CandXY> candidates;
        candidates.reserve(occupied.size());
        for (const uint64_t key : occupied) {
            const auto ix = static_cast<int32_t>(static_cast<uint32_t>(key >> 32));
            const auto iy = static_cast<int32_t>(static_cast<uint32_t>(key & 0xFFFF'FFFFu));
            candidates.push_back({
                (ix + 0.5f) * static_cast<float>(global_search_step_),
                (iy + 0.5f) * static_cast<float>(global_search_step_)
            });
        }

        const size_t total =
            candidates.size() * static_cast<size_t>(init_heading_candidates_);
        RCLCPP_INFO(get_logger(),
            "[init/global] %zu occupied cells × %d headings = %zu ICP calls. "
            "Map extents: %.0f × %.0f m — standing by…",
            candidates.size(), init_heading_candidates_, total,
            static_cast<double>(map_max_x_ - map_min_x_),
            static_cast<double>(map_max_y_ - map_min_y_));

        double best_fitness = std::numeric_limits<double>::max();
        std::optional<Eigen::Isometry3d> best_pose;
        const double yaw_step = 2.0 * M_PI / init_heading_candidates_;

        size_t count = 0;
        for (const auto & c : candidates) {
            for (int h = 0; h < init_heading_candidates_; ++h) {
                auto [pose, fitness] = tryAlignAt(
                    makeXYYaw(c.x, c.y, h * yaw_step),
                    init_search_max_iter_);

                if (fitness < best_fitness) {
                    best_fitness = fitness;
                    best_pose    = pose;
                }
                // Progress heartbeat every 100 calls
                if (++count % 100 == 0)
                    RCLCPP_INFO(get_logger(),
                        "[init/global] %zu / %zu  best_fitness=%.4f",
                        count, total, best_fitness);
            }
        }

        RCLCPP_INFO(get_logger(),
            "[init/global] Search complete — best fitness=%.4f", best_fitness);

        if (best_fitness > max_fitness_) {
            RCLCPP_WARN(get_logger(),
                "[init/global] No candidate below threshold=%.4f. "
                "Try: increasing max_fitness_accept, reducing global_search_step, "
                "or checking the map / sensor calibration.",
                max_fitness_);
            return std::nullopt;
        }
        return best_pose;
    }

    // =========================================================================
    //  Low-level ICP helper
    // =========================================================================

    /// Run one alignment at @p guess using @p max_iter iterations.
    ///
    /// PRECONDITION: vgicp_->setInputSource() must have been called by the
    /// caller before entering any search loop.  fast_gicp caches source
    /// covariances, so calling align() repeatedly with different guesses is
    /// efficient.
    ///
    /// @p max_iter is restored to vgicp_max_iter_ after the call so that
    /// the tracking path always uses the full budget.
    std::pair<Eigen::Isometry3d, double>
    tryAlignAt(const Eigen::Isometry3d & guess, int max_iter)
    {
        vgicp_->setMaximumIterations(max_iter);

        pcl::PointCloud<pcl::PointXYZI> aligned;
        vgicp_->align(aligned, guess.matrix().cast<float>());

        const double fitness = vgicp_->getFitnessScore();
        Eigen::Isometry3d result;
        result.matrix() = vgicp_->getFinalTransformation().cast<double>();

        vgicp_->setMaximumIterations(vgicp_max_iter_);  // restore
        return {result, fitness};
    }

    // =========================================================================
    //  Prior acquisition for TRACKING
    // =========================================================================

    /// For TF_PRIOR mode: strictly requires TF (original behaviour).
    /// For all other modes:
    ///   1. Try the EKF TF.
    ///   2. Fall back to last_good_pose_ while the EKF is still cold-starting
    ///      (it needs our published poses before it can converge and publish TF).
    std::optional<Eigen::Isometry3d> getPrior(const rclcpp::Time & stamp)
    {
        try {
            const auto tf = tf_buffer_->lookupTransform(
                map_frame_, base_frame_, stamp,
                rclcpp::Duration::from_seconds(0.05));
            return tf2::transformToEigen(tf);
        } catch (const tf2::TransformException & ex) {

            if (init_mode_ == InitMode::TF_PRIOR) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10'000,
                    "[TRACKING] Waiting for EKF map→base_link TF: %s", ex.what());
                return std::nullopt;
            }

            // EKF may still be warming up — use our last accepted pose.
            if (last_good_pose_) {
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                    "[TRACKING] TF unavailable — using last accepted pose as prior.");
                return last_good_pose_;
            }

            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "[TRACKING] TF unavailable and no prior pose yet — dropping scan.");
            return std::nullopt;
        }
    }

    // =========================================================================
    //  Utilities
    // =========================================================================

    pcl::PointCloud<pcl::PointXYZI>::Ptr
    downsample(const pcl::PointCloud<pcl::PointXYZI>::Ptr & in, double leaf)
    {
        auto out = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        pcl::VoxelGrid<pcl::PointXYZI> vg;
        vg.setLeafSize(static_cast<float>(leaf),
                       static_cast<float>(leaf),
                       static_cast<float>(leaf));
        vg.setInputCloud(in);
        vg.filter(*out);
        return out;
    }

    std::optional<pcl::PointCloud<pcl::PointXYZI>::Ptr>
    toBaseFrame(const sensor_msgs::msg::PointCloud2::SharedPtr & msg,
                const rclcpp::Time & stamp)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*msg, *cloud);

        if (msg->header.frame_id == base_frame_) return cloud;

        try {
            const auto tf_msg = tf_buffer_->lookupTransform(
                base_frame_, msg->header.frame_id, stamp,
                rclcpp::Duration::from_seconds(0.1));
            const Eigen::Affine3d T = tf2::transformToEigen(tf_msg);
            auto out = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
            pcl::transformPointCloud(*cloud, *out, T.matrix().cast<float>());
            return out;
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "[toBaseFrame] Sensor TF failed: %s  "
                "(Is '%s'→'%s' published?)",
                ex.what(), msg->header.frame_id.c_str(), base_frame_.c_str());
            return std::nullopt;
        }
    }

    geometry_msgs::msg::PoseWithCovarianceStamped
    buildPoseMsg(const Eigen::Isometry3d & map_to_base, const rclcpp::Time & stamp, double fitness)
    {
        geometry_msgs::msg::PoseWithCovarianceStamped msg;
        msg.header.stamp    = stamp;
        msg.header.frame_id = map_frame_;

        const Eigen::Quaterniond q(map_to_base.rotation());
        msg.pose.pose.position.x    = map_to_base.translation().x();
        msg.pose.pose.position.y    = map_to_base.translation().y();
        msg.pose.pose.position.z    = map_to_base.translation().z();
        msg.pose.pose.orientation.x = q.x();
        msg.pose.pose.orientation.y = q.y();
        msg.pose.pose.orientation.z = q.z();
        msg.pose.pose.orientation.w = q.w();

        // Baseline covariance consumed by the EKF
        double cov = 0.01 + (0.1 * fitness);  // increase covariance for worse fits (tunable)
        msg.pose.covariance[0]  = cov;  // X
        msg.pose.covariance[7]  = cov;  // Y
        msg.pose.covariance[14] = cov;  // Z
        msg.pose.covariance[21] = cov;  // Roll
        msg.pose.covariance[28] = cov;  // Pitch
        msg.pose.covariance[35] = cov;  // Yaw

        return msg;
    }

    void publishPose(const Eigen::Isometry3d & map_to_base, const rclcpp::Time & stamp, double fitness)
    {
        pub_pose_->publish(buildPoseMsg(map_to_base, stamp, fitness));
    }

    // =========================================================================
    //  Member variables
    // =========================================================================

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr             sub_cloud_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_ekf_reset_;
    std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // ── Core config ──────────────────────────────────────────────────────────
    std::string map_frame_, base_frame_, lidar_topic_, map_ply_path_;
    double      voxel_leaf_map_, voxel_leaf_scan_, vgicp_corr_dist_, max_fitness_;
    int         vgicp_resolution_, vgicp_max_iter_;

    // ── Init config ──────────────────────────────────────────────────────────
    InitMode init_mode_               = InitMode::TF_PRIOR;
    double   init_x_                  = 0.0;
    double   init_y_                  = 0.0;
    double   init_yaw_                = 0.0;
    int      init_heading_candidates_ = 16;
    double   global_search_step_      = 5.0;
    int      init_search_max_iter_    = 20;

    std::string ekf_reset_topic_;

    // ── Runtime state ─────────────────────────────────────────────────────────
    LocalizerState                   state_          = LocalizerState::NEEDS_INIT;
    std::optional<Eigen::Isometry3d> last_good_pose_;

    // ── Map & matcher ─────────────────────────────────────────────────────────
    std::shared_ptr<VGICPVariant>        vgicp_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud_;
    float map_min_x_{0}, map_max_x_{0}, map_min_y_{0}, map_max_y_{0};
};

// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapOdomLocalizer>());
    rclcpp::shutdown();
    return 0;
}