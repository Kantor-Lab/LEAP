/**
 * map_odom_localizer.cpp
 *
 * Localizes a robot within a pre-built .ply map using FastVGICP.
 *
 * ── INITIALIZATION MODES (param: "init_mode") ────────────────────────────────
 *
 * "tf_prior"                 Original behaviour. Requires the EKF to publish
 * map→base_link before the first scan can be processed.
 *
 * "seeded_position_heading"  MODE 1 — you know x, y, AND yaw (e.g. from a
 * GPS+compass fix, or a saved pose). Set init_x, init_y, init_yaw.
 * ICP spirals outward in POSITION ring by ring, and at every ring
 * point tries a small WINDOW of headings around the seeded yaw
 * (seed_heading_candidates, spanning ±seed_heading_tolerance_deg,
 * tried centre-out — seeded yaw first) rather than trusting it
 * exactly, since GPS/compass headings are rarely perfectly accurate.
 * Much cheaper than a full 360° sweep since the window is narrow.
 * Set seed_heading_candidates=1 to fall back to trusting the seeded
 * yaw exactly (the original behaviour). Stops as soon as any
 * (position, heading) candidate beats max_fitness_accept.
 * Good when the seed is trusted but may be off by a few metres
 * (GPS noise, stale odometry, etc) and a few degrees of heading.
 * (legacy alias: "full_pose")
 *
 * "seeded_position"          MODE 2 — you know x, y but NOT yaw. Set init_x,
 * init_y.  Same outward spiral as above, but at every ring point a
 * full sweep of init_heading_candidates evenly-spaced headings is
 * tried (coarse ICP each) since there's no heading to trust.
 * (legacy alias: "position_only")
 *
 * "no_seed"                  MODE 3 — nothing known.  A grid of
 * global_search_step metres is built over every occupied map cell;
 * a full heading sweep is run at each cell.  The best candidate is
 * then refined.  ⚠ Can take 10–90 s on large maps — this is expected.
 * (legacy alias: "global")
 *
 * All three search modes refine their winning candidate with a full-
 * resolution ICP pass (vgicp_max_iterations) before locking it in.
 *
 * ── RUNTIME RE-SEED (param: "reinit_pose_topic", default "/icp/reinit_pose") ──
 * Publish a geometry_msgs/PoseStamped on this topic at any time — whether
 * the node is still initializing or already TRACKING — to force a fresh
 * initialization from that pose.  e.g. feed it a GPS fix once ICP fitness
 * starts degrading. A re-seed always runs as MODE 1
 * (seeded_position_heading): a PoseStamped already carries x, y, AND yaw,
 * so there's no ambiguity left for a heading sweep or a full grid search
 * to resolve. The node drops back into NEEDS_INIT, spirals out from the
 * new pose (up to "reinit_search_max_radius", default 15 m — deliberately
 * tighter than the general "seed_search_max_radius", since a re-seed is
 * meant as a recovery nudge, not a wide "no idea where I am" search), and
 * resumes TRACKING once a candidate converges.
 * A new re-seed message is REJECTED (dropped, not queued) if either:
 *   1. a search is currently in progress, or
 *   2. fewer than "reinit_min_interval" seconds (default 3 s) have passed
 *      since the last search finished.
 * This keeps a fast/continuous publisher (e.g. GPS streaming at several Hz)
 * from repeatedly yanking the node out of TRACKING before it ever settles.
 *
 * ── TRACKING (all modes) ──────────────────────────────────────────────────────
 * Once initialization succeeds the node enters TRACKING state:
 * 1. Try to get map→base_link from the EKF TF tree as the ICP prior.
 * 2. If TF is unavailable (EKF still cold-starting), fall back to the last
 * accepted pose.  (TF_PRIOR mode skips this fallback — original behaviour.)
 * 3. Publish PoseWithCovarianceStamped for the EKF on every good scan.
 */

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/transforms.h>
#include <pcl/PolygonMesh.h>
#include <pcl/conversions.h>

#ifdef USE_CUDA_VGICP
  #include <fast_gicp/gicp/fast_vgicp_cuda.hpp>
  using VGICPVariant = fast_gicp::FastVGICPCuda<pcl::PointXYZI, pcl::PointXYZI>;
#else
  #include <fast_gicp/gicp/fast_vgicp.hpp>
  using VGICPVariant = fast_gicp::FastVGICP<pcl::PointXYZI, pcl::PointXYZI>;
#endif

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <cmath>
#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <unordered_set>
#include <unordered_map>
#include <utility>
#include <vector>

// ─────────────────────────────────────────────────────────────────────────────
//  Enumerations & free helpers
// ─────────────────────────────────────────────────────────────────────────────

enum class InitMode {
    TF_PRIOR,                  ///< Original: always require EKF TF prior
    SEEDED_POSITION_HEADING,   ///< MODE 1: x, y, yaw known → spiral position search, narrow heading window around yaw
    SEEDED_POSITION,           ///< MODE 2: x, y known → spiral position search, heading swept at each point
    NO_SEED,                   ///< MODE 3: nothing known → grid search over whole map
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

/// Build a 3D Isometry3d from (x, y, z, yaw) for terrain integration.
static Eigen::Isometry3d makeXYZYaw(double x, double y, double z, double yaw)
{
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.translation()     = Eigen::Vector3d(x, y, z);
    T.linear()          = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
                              .toRotationMatrix();
    return T;
}

/// Enumerate the integer grid offsets forming the perimeter of a square ring
/// at Chebyshev distance `ring` from the origin (ring=0 → just the origin).
/// Used to walk outward from a seed position one ring at a time: ring 0 is
/// the seed itself, ring 1 is the 8 cells around it, ring 2 the 16 around
/// that, and so on — a simple, allocation-light spiral-search pattern.
static std::vector<std::pair<int, int>> ringOffsets(int ring)
{
    std::vector<std::pair<int, int>> offs;
    if (ring == 0) {
        offs.emplace_back(0, 0);
        return offs;
    }

    offs.reserve(8 * ring);
    for (int ix = -ring; ix <= ring; ++ix) {
        offs.emplace_back(ix, -ring);
        offs.emplace_back(ix,  ring);
    }
    for (int iy = -ring + 1; iy <= ring - 1; ++iy) {
        offs.emplace_back(-ring, iy);
        offs.emplace_back( ring, iy);
    }
    return offs;
}

/// Human-readable name for logging.
static const char * initModeName(InitMode m)
{
    switch (m) {
        case InitMode::TF_PRIOR:                return "TF_PRIOR";
        case InitMode::SEEDED_POSITION_HEADING: return "SEEDED_POSITION_HEADING";
        case InitMode::SEEDED_POSITION:         return "SEEDED_POSITION";
        case InitMode::NO_SEED:                 return "NO_SEED";
    }
    return "UNKNOWN";
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
        loadTerrainMap();
        buildVGICP();

        pub_pose_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/icp/pose", rclcpp::QoS(10));

        pub_fitness_diag_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
            "/icp/fitness", rclcpp::QoS(10));

        // SensorDataQoS (best-effort, depth 1): drop stale scans if ICP is slow.
        sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            lidar_topic_, rclcpp::SensorDataQoS(),
            std::bind(&MapOdomLocalizer::cloudCallback, this, std::placeholders::_1));

        // Lets an external node (GPS filter, operator tool, ...) force a fresh
        // initialization at any time by publishing a new seed pose.
        sub_reinit_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            reinit_pose_topic_, rclcpp::QoS(1).reliable(),
            std::bind(&MapOdomLocalizer::reinitPoseCallback, this, std::placeholders::_1));

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
        declare_parameter("base_frame",             "base_footprint");
        declare_parameter("lidar_topic",            "/ouster/points");
        declare_parameter("map_ply_path",           "");
        declare_parameter("voxel_leaf_map",         0.3);
        declare_parameter("voxel_leaf_scan",        0.3);
        declare_parameter("vgicp_resolution",       1);
        declare_parameter("vgicp_max_iterations",   64);
        declare_parameter("vgicp_max_corresp_dist", 3.0);
        declare_parameter("max_fitness_accept",     0.2);

        // Submap and cropping configs
        declare_parameter("scan_crop_radius",       35.0);
        declare_parameter("submap_radius",          100.0);
        declare_parameter("submap_update_dist",     20.0);

        // Terrain Mesh configs
        declare_parameter("terrain_ply_path",           "");
        declare_parameter("terrain_grid_resolution",    0.5);

        // ── Initialization mode ──────────────────────────────────────────────
        // "tf_prior" | "seeded_position_heading" | "seeded_position" | "no_seed"
        // (legacy: "full_pose" | "position_only" | "global" — still accepted)
        declare_parameter("init_mode", "tf_prior");

        // Seed position (used by seeded_position_heading and seeded_position)
        declare_parameter("init_x",   0.0);
        declare_parameter("init_y",   0.0);
        // Seed heading in radians — only used for seeded_position_heading
        declare_parameter("init_yaw", 0.0);

        // Heading candidates to try per position (seeded_position and no_seed)
        declare_parameter("init_heading_candidates", 16);

        // seeded_position_heading: rather than trusting init_yaw exactly,
        // try a narrow window of headings around it at every spiral
        // position (heading sensors/estimates are rarely perfect).
        // Candidates are tried centre-out (seeded yaw first, then
        // expanding ± steps), so a good seed still resolves in one ICP
        // call — the window only costs extra when the seed actually
        // needed correcting. Set to 1 to trust init_yaw exactly (the
        // original, pre-window behaviour). An odd number includes the
        // seeded yaw itself as one of the candidates.
        declare_parameter("seed_heading_candidates",   5);
        declare_parameter("seed_heading_tolerance_deg", 20.0);

        // Grid step in metres for the no_seed grid search
        declare_parameter("global_search_step", 5.0);

        // Ring spacing in metres for the seeded_position[_heading] spiral search
        declare_parameter("seed_search_step", 2.0);
        // Give up (stay in NEEDS_INIT, retry next scan) once the spiral has
        // walked out this far without a candidate beating max_fitness_accept.
        // Applies to the ORIGINALLY CONFIGURED init_mode's seed (init_x/init_y),
        // where the guess might only be roughly known.
        declare_parameter("seed_search_max_radius", 30.0);

        // Separate (and normally tighter) give-up radius used ONLY when the
        // seed came from a runtime re-seed on reinit_pose_topic. A re-seed
        // is meant as a recovery nudge — tracking was presumably fine until
        // just recently — so it doesn't need to search as wide as the
        // general startup seed above.
        declare_parameter("reinit_search_max_radius", 15.0);

        // ICP iteration budget for the *search* phase.
        // Fewer iterations = faster sweep; the best candidate is always
        // refined to vgicp_max_iterations afterwards.
        declare_parameter("init_search_max_iter", 20);

        // Override the default EKF starting pose
        declare_parameter("ekf_reset_topic", "/set_pose");

        // Publish a geometry_msgs/PoseStamped here at any time to force a
        // fresh re-initialization (always run as seeded_position_heading —
        // see the file docstring). e.g. wire a GPS filter to this topic so
        // it can recover ICP after a bad patch.
        declare_parameter("reinit_pose_topic", "/icp/reinit_pose");

        // Minimum time between ACCEPTED re-seeds. Guards against a fast /
        // continuously-publishing source (e.g. a GPS filter streaming at
        // several Hz) yanking the node back into NEEDS_INIT on every message
        // and re-running the full spiral search before it ever gets a
        // chance to settle into TRACKING. Extra messages inside the cooldown
        // window are simply dropped (the next one after the window reopens
        // still wins with its own fresh pose).
        declare_parameter("reinit_min_interval", 3.0);

        // CUDA-only: which NN search backend FastVGICPCuda should use.
        // "cpu_kdtree" | "gpu_bruteforce" | "gpu_rbf_kernel"
        // ONLY cpu_kdtree works! gpu_bruteforce is really slow and gpu_rbf_kernel needs too much memory!
        declare_parameter("cuda_nn_method", "cpu_kdtree");

        // ── Read values ──────────────────────────────────────────────────────
        map_frame_    = get_parameter("map_frame").as_string();
        base_frame_   = get_parameter("base_frame").as_string();
        lidar_topic_  = get_parameter("lidar_topic").as_string();
        map_ply_path_ = get_parameter("map_ply_path").as_string();

        voxel_leaf_map_  = get_parameter("voxel_leaf_map").as_double();
        voxel_leaf_scan_ = get_parameter("voxel_leaf_scan").as_double();
        max_fitness_     = get_parameter("max_fitness_accept").as_double();

        scan_crop_radius_   = get_parameter("scan_crop_radius").as_double();
        submap_radius_      = get_parameter("submap_radius").as_double();
        submap_update_dist_ = get_parameter("submap_update_dist").as_double();

        terrain_ply_path_        = get_parameter("terrain_ply_path").as_string();
        terrain_grid_resolution_ = get_parameter("terrain_grid_resolution").as_double();

        vgicp_resolution_ = get_parameter("vgicp_resolution").as_int();
        vgicp_max_iter_   = get_parameter("vgicp_max_iterations").as_int();
        vgicp_corr_dist_  = get_parameter("vgicp_max_corresp_dist").as_double();

        init_x_   = get_parameter("init_x").as_double();
        init_y_   = get_parameter("init_y").as_double();
        init_yaw_ = get_parameter("init_yaw").as_double();

        init_heading_candidates_ = get_parameter("init_heading_candidates").as_int();
        seed_heading_candidates_ = get_parameter("seed_heading_candidates").as_int();
        seed_heading_tolerance_  = get_parameter("seed_heading_tolerance_deg").as_double() * M_PI / 180.0;
        global_search_step_      = get_parameter("global_search_step").as_double();
        seed_search_step_        = get_parameter("seed_search_step").as_double();
        seed_search_max_radius_  = get_parameter("seed_search_max_radius").as_double();
        reinit_search_max_radius_ = get_parameter("reinit_search_max_radius").as_double();
        init_search_max_iter_    = get_parameter("init_search_max_iter").as_int();

        ekf_reset_topic_    = get_parameter("ekf_reset_topic").as_string();
        reinit_pose_topic_  = get_parameter("reinit_pose_topic").as_string();
        reinit_min_interval_ = get_parameter("reinit_min_interval").as_double();
        cuda_nn_method_     = get_parameter("cuda_nn_method").as_string();

        const std::string mode_str = get_parameter("init_mode").as_string();
        if (mode_str == "seeded_position_heading" || mode_str == "full_pose") {
            init_mode_ = InitMode::SEEDED_POSITION_HEADING;
            if (mode_str == "full_pose")
                RCLCPP_WARN(get_logger(),
                    "init_mode 'full_pose' is a deprecated alias — use 'seeded_position_heading'.");
        } else if (mode_str == "seeded_position" || mode_str == "position_only") {
            init_mode_ = InitMode::SEEDED_POSITION;
            if (mode_str == "position_only")
                RCLCPP_WARN(get_logger(),
                    "init_mode 'position_only' is a deprecated alias — use 'seeded_position'.");
        } else if (mode_str == "no_seed" || mode_str == "global") {
            init_mode_ = InitMode::NO_SEED;
            if (mode_str == "global")
                RCLCPP_WARN(get_logger(),
                    "init_mode 'global' is a deprecated alias — use 'no_seed'.");
        } else {
            if (mode_str != "tf_prior")
                RCLCPP_WARN(get_logger(),
                    "Unknown init_mode '%s' — defaulting to 'tf_prior'.", mode_str.c_str());
            init_mode_ = InitMode::TF_PRIOR;
        }

        // active_init_mode_ is what actually drives behaviour; init_mode_ is
        // just the launch-time configuration. A runtime re-seed (see
        // reinitPoseCallback) only ever changes active_init_mode_, so the
        // originally configured mode is always still recoverable/loggable.
        active_init_mode_ = init_mode_;

        // TF_PRIOR skips NEEDS_INIT entirely (original behaviour).
        if (init_mode_ == InitMode::TF_PRIOR)
            state_ = LocalizerState::TRACKING;
    }

    void logInitMode() const
    {
        switch (active_init_mode_) {
            case InitMode::TF_PRIOR:
                RCLCPP_INFO(get_logger(),
                    "[init] mode=TF_PRIOR — waiting for EKF map→base_link TF.");
                break;
            case InitMode::SEEDED_POSITION_HEADING:
                RCLCPP_INFO(get_logger(),
                    "[init] mode=SEEDED_POSITION_HEADING  x=%.2f  y=%.2f  yaw=%.3f rad  "
                    "(spiral: step=%.1fm  max_radius=%.1fm  x  %d headings within ±%.1f°)",
                    init_x_, init_y_, init_yaw_, seed_search_step_, seed_search_max_radius_,
                    seed_heading_candidates_, seed_heading_tolerance_ * 180.0 / M_PI);
                break;
            case InitMode::SEEDED_POSITION:
                RCLCPP_INFO(get_logger(),
                    "[init] mode=SEEDED_POSITION  x=%.2f  y=%.2f  "
                    "(spiral: step=%.1fm  max_radius=%.1fm  %d headings/point)",
                    init_x_, init_y_, seed_search_step_, seed_search_max_radius_,
                    init_heading_candidates_);
                break;
            case InitMode::NO_SEED:
                RCLCPP_INFO(get_logger(),
                    "[init] mode=NO_SEED  grid_step=%.1f m  headings=%d  "
                    "⚠ may take 10–90 s on first scan for large maps",
                    global_search_step_, init_heading_candidates_);
                break;
        }
        RCLCPP_INFO(get_logger(),
            "[init] Runtime re-seed: publish geometry_msgs/PoseStamped on '%s' "
            "to force a fresh initialization at any time (spiral max_radius=%.1fm, "
            "cooldown=%.1fs after each search).",
            reinit_pose_topic_.c_str(), reinit_search_max_radius_, reinit_min_interval_);
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

        global_map_ = downsample(raw_xyzi, voxel_leaf_map_);
        RCLCPP_INFO(get_logger(), "Map loaded: %zu pts (downsampled from %zu)",
            global_map_->size(), raw_xyzi->size());

        // Pre-compute bbox (needed for logging in global search)
        computeMapBBox();
    }

    void computeMapBBox()
    {
        map_min_x_ = map_min_y_ =  std::numeric_limits<float>::max();
        map_max_x_ = map_max_y_ = -std::numeric_limits<float>::max();
        for (const auto & pt : *global_map_) {
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
    //  Terrain Map loading (2.5D Elevation Grid)
    // =========================================================================

    void loadTerrainMap()
    {
        if (terrain_ply_path_.empty()) {
            RCLCPP_WARN(get_logger(), "No terrain mesh provided. Z will default to 0.0.");
            return;
        }

        // loadPLYFile handles vertex + face PLY files correctly
        pcl::PolygonMesh mesh;
        if (pcl::io::loadPLYFile(terrain_ply_path_, mesh) < 0) {
            RCLCPP_FATAL(get_logger(), "Failed to load terrain mesh: %s", terrain_ply_path_.c_str());
            rclcpp::shutdown();
            return;
        }

        // 1. Manually unpack the binary blob to bypass strict PCL type-casting
        pcl::PointCloud<pcl::PointXYZ> verts;
        verts.reserve(mesh.cloud.width * mesh.cloud.height);

        int x_off = -1, y_off = -1, z_off = -1;
        uint8_t datatype = 0; 
        for (const auto & field : mesh.cloud.fields) {
            if (field.name == "x") { x_off = field.offset; datatype = field.datatype; }
            if (field.name == "y") { y_off = field.offset; }
            if (field.name == "z") { z_off = field.offset; }
        }

        if (x_off == -1 || y_off == -1 || z_off == -1) {
            RCLCPP_FATAL(get_logger(), "Terrain mesh lacks x, y, or z fields.");
            rclcpp::shutdown();
            return;
        }

        for (size_t i = 0; i < mesh.cloud.width * mesh.cloud.height; ++i) {
            const uint8_t* pt_data = &mesh.cloud.data[i * mesh.cloud.point_step];
            pcl::PointXYZ pt;
            
            // Handle double-precision (FLOAT64) to single-precision (FLOAT32) cast
            if (datatype == pcl::PCLPointField::FLOAT64) {
                pt.x = static_cast<float>(*reinterpret_cast<const double*>(pt_data + x_off));
                pt.y = static_cast<float>(*reinterpret_cast<const double*>(pt_data + y_off));
                pt.z = static_cast<float>(*reinterpret_cast<const double*>(pt_data + z_off));
            } else {
                pt.x = *reinterpret_cast<const float*>(pt_data + x_off);
                pt.y = *reinterpret_cast<const float*>(pt_data + y_off);
                pt.z = *reinterpret_cast<const float*>(pt_data + z_off);
            }
            verts.push_back(pt);
        }

        if (verts.empty()) {
            RCLCPP_FATAL(get_logger(), "Terrain mesh has no vertices: %s", terrain_ply_path_.c_str());
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(get_logger(), "Terrain mesh: %zu vertices, %zu triangles — rasterizing…",
            verts.size(), mesh.polygons.size());

        // 2. Rasterize the faces into the 2.5D elevation grid
        const float res = static_cast<float>(terrain_grid_resolution_);

        auto cellKey = [](int32_t ix, int32_t iy) -> uint64_t {
            return (static_cast<uint64_t>(static_cast<uint32_t>(ix)) << 32)
                |  static_cast<uint64_t>(static_cast<uint32_t>(iy));
        };

        for (const auto & face : mesh.polygons) {
            if (face.vertices.size() < 3) continue;

            const auto & A = verts[face.vertices[0]];
            const auto & B = verts[face.vertices[1]];
            const auto & C = verts[face.vertices[2]];

            // 2D determinant for barycentric solve (XY plane only — DTM is a height field)
            const float det = (B.x - A.x) * (C.y - A.y) - (B.y - A.y) * (C.x - A.x);
            if (std::abs(det) < 1e-6f) continue;   // degenerate (zero-area) triangle
            const float inv_det = 1.0f / det;

            // Grid-cell range covering this triangle's XY bounding box
            const auto ix0 = static_cast<int32_t>(std::floor(std::min({A.x, B.x, C.x}) / res));
            const auto ix1 = static_cast<int32_t>(std::ceil (std::max({A.x, B.x, C.x}) / res));
            const auto iy0 = static_cast<int32_t>(std::floor(std::min({A.y, B.y, C.y}) / res));
            const auto iy1 = static_cast<int32_t>(std::ceil (std::max({A.y, B.y, C.y}) / res));

            for (int32_t ix = ix0; ix <= ix1; ++ix) {
                for (int32_t iy = iy0; iy <= iy1; ++iy) {
                    // Test the cell centre against the triangle
                    const float px = (ix + 0.5f) * res;
                    const float py = (iy + 0.5f) * res;

                    const float s = ((px - A.x) * (C.y - A.y) - (py - A.y) * (C.x - A.x)) * inv_det;
                    const float t = ((B.x - A.x) * (py - A.y) - (B.y - A.y) * (px - A.x)) * inv_det;
                    const float r = 1.0f - s - t;

                    if (s < 0.0f || t < 0.0f || r < 0.0f) continue;  // outside triangle

                    // Barycentric Z interpolation
                    elevation_grid_[cellKey(ix, iy)] = static_cast<double>(r * A.z + s * B.z + t * C.z);
                }
            }
        }

        RCLCPP_INFO(get_logger(), "Terrain grid rasterized: %zu cells at %.2f m/cell.", 
            elevation_grid_.size(), terrain_grid_resolution_);
    }

    double getElevationAt(double x, double y)
    {
        if (elevation_grid_.empty()) return 0.0;

        const double fc = x / terrain_grid_resolution_;
        const double fr = y / terrain_grid_resolution_;
        const auto c0 = static_cast<int32_t>(std::floor(fc));
        const auto r0 = static_cast<int32_t>(std::floor(fr));

        auto fetch = [&](int32_t c, int32_t r) -> double {
            const uint64_t key =
                (static_cast<uint64_t>(static_cast<uint32_t>(c)) << 32)
            |  static_cast<uint64_t>(static_cast<uint32_t>(r));
            auto it = elevation_grid_.find(key);
            if (it == elevation_grid_.end())
            {
                RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 5000, 
                    "Cell (%d, %d) is missing from the terrain grid. Treating as flat (Z=0).", c, r);
                return 0.0;
            } 
            return it->second;
        };

        const double z00 = fetch(c0,   r0);
        const double z10 = fetch(c0+1, r0);
        const double z01 = fetch(c0,   r0+1);
        const double z11 = fetch(c0+1, r0+1);

        if (std::isnan(z00) || std::isnan(z10) || std::isnan(z01) || std::isnan(z11))
            return 0.0;  // at least one corner missing — fall back

        const double tx = fc - c0, ty = fr - r0;
        return (1-tx)*(1-ty)*z00 + tx*(1-ty)*z10
            + (1-tx)*ty   *z01 + tx*ty   *z11;
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
        // FastVGICPCuda defaults to CPU_PARALLEL_KDTREE for correspondence
        // search if not set explicitly — meaning NN search would silently
        // stay CPU-bound even in the "CUDA" build. Set it explicitly.
        if (cuda_nn_method_ == "gpu_bruteforce") {
            vgicp_->setNearestNeighborSearchMethod(fast_gicp::NearestNeighborMethod::GPU_BRUTEFORCE);
            RCLCPP_INFO(get_logger(), "Matcher: cuVGICP (CUDA GPU, NN=GPU_BRUTEFORCE)");
        } else if (cuda_nn_method_ == "gpu_rbf_kernel") {
            vgicp_->setNearestNeighborSearchMethod(fast_gicp::NearestNeighborMethod::GPU_RBF_KERNEL);
            RCLCPP_INFO(get_logger(), "Matcher: cuVGICP (CUDA GPU, NN=GPU_RBF_KERNEL)");
        } else {
            if (cuda_nn_method_ != "cpu_kdtree")
                RCLCPP_WARN(get_logger(),
                    "Unknown cuda_nn_method '%s' — defaulting to 'cpu_kdtree'.",
                    cuda_nn_method_.c_str());
            vgicp_->setNearestNeighborSearchMethod(fast_gicp::NearestNeighborMethod::CPU_PARALLEL_KDTREE);
            RCLCPP_INFO(get_logger(), "Matcher: cuVGICP (CUDA GPU, NN=CPU_PARALLEL_KDTREE)");
        }
#endif
        // Set the global map by default (needed immediately if NO_SEED init mode is used)
        vgicp_->setInputTarget(global_map_);

        pub_ekf_reset_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            ekf_reset_topic_, rclcpp::QoS(1).reliable());
    }

    // =========================================================================
    //  Main scan callback
    // =========================================================================

    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        using clock = std::chrono::steady_clock;
        const auto t_start = clock::now();

        const rclcpp::Time stamp = msg->header.stamp;

        // Transform scan into base_link frame, crop, and downsample.
        auto scan_base = toBaseFrame(msg, stamp);
        if (!scan_base) return;
        const auto t_tobase = clock::now();

        auto scan_cropped = cropCloud(scan_base.value(), scan_crop_radius_);
        const auto t_crop = clock::now();

        const auto scan_ds = downsample(scan_cropped, voxel_leaf_scan_);
        const auto t_downsample = clock::now();

        // ── First-scan bootstrap ─────────────────────────────────────────────
        if (state_ == LocalizerState::NEEDS_INIT) {
            runInitialization(scan_ds, stamp);
            return;
        }

        // ── Normal tracking ──────────────────────────────────────────────────
        auto prior = getPrior(stamp);
        if (!prior) return;
        const auto t_prior = clock::now();

        // The EKF operates in 2D, so its TF prior will have Z ≈ 0. 
        // We must snap the prior to the terrain mesh before feeding it to ICP.
        double expected_z = getElevationAt(prior->translation().x(), prior->translation().y());
        prior->translation().z() = expected_z;

        // Ensure VGICP is matching against a dynamic local submap instead of the global map
        updateLocalMap(prior->translation());
        const auto t_submap = clock::now();

        // fast_gicp caches source covariances after the first setInputSource call,
        // so re-setting with the same cloud on every tick is fine (no-op if the
        // pointer matches). The call is cheap either way.
        vgicp_->setInputSource(scan_ds);
        pcl::PointCloud<pcl::PointXYZI> aligned;
        vgicp_->align(aligned, prior->matrix().cast<float>());
        const auto t_align = clock::now();

        const double fitness   = vgicp_->getFitnessScore();
        const bool   converged = vgicp_->hasConverged();

        // Published unconditionally -- see publishFitnessDiagnostics() docstring
        // for why this must happen before the accept/reject branch below.
        publishFitnessDiagnostics(fitness, stamp);

        // {
        //     auto ms = [](clock::time_point a, clock::time_point b) {
        //         return std::chrono::duration<double, std::milli>(b - a).count();
        //     };
        //     RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
        //         "[timing] toBase=%.1f crop=%.1f downsample=%.1f prior=%.1f "
        //         "submap=%.1f align=%.1f total=%.1f ms",
        //         ms(t_start, t_tobase), ms(t_tobase, t_crop), ms(t_crop, t_downsample),
        //         ms(t_downsample, t_prior), ms(t_prior, t_submap), ms(t_submap, t_align),
        //         ms(t_start, t_align));
        // }

        if (fitness > max_fitness_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "[TRACKING] fitness=%.4f — FAILED — converged=%s",
                fitness, converged ? "true" : "false");
            return;
        } else if (!converged) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "[TRACKING] fitness=%.4f — DID NOT CONVERGE", fitness);
        } else {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                "[TRACKING] fitness=%.4f", fitness);
        }

        Eigen::Isometry3d result;
        result.matrix() = vgicp_->getFinalTransformation().cast<double>();
        last_good_pose_  = result;   // always keep a fallback prior
        publishPose(result, stamp, fitness);
    }

    // =========================================================================
    //  Initialization dispatcher
    // =========================================================================

    /// RAII helper: marks init_search_in_progress_ for the duration of
    /// runInitialization() and unconditionally records the finish time on
    /// scope exit (success OR failure — every return path is covered
    /// automatically, so there's no way to forget to stamp one).
    /// Used together to gate reinitPoseCallback(): reject while a search is
    /// running, and reject for reinit_min_interval_ seconds after one ends.
    struct InitSearchGuard {
        MapOdomLocalizer * node;
        explicit InitSearchGuard(MapOdomLocalizer * n) : node(n) {
            node->init_search_in_progress_ = true;
        }
        ~InitSearchGuard() {
            node->init_search_in_progress_ = false;
            node->last_init_finish_time_   = node->get_clock()->now();
        }
    };

    /// Which spiral give-up radius applies right now — see the parameter
    /// declarations for seed_search_max_radius vs. reinit_search_max_radius.
    double activeSpiralMaxRadius() const
    {
        return came_from_reinit_ ? reinit_search_max_radius_ : seed_search_max_radius_;
    }

    void runInitialization(const pcl::PointCloud<pcl::PointXYZI>::Ptr & scan,
                           const rclcpp::Time & stamp)
    {
        // Marks init_search_in_progress_ = true for the rest of this
        // function, and records last_init_finish_time_ on return no matter
        // which path is taken below — see InitSearchGuard.
        InitSearchGuard search_guard(this);

        RCLCPP_INFO(get_logger(), "[init] Processing scan for initialization (mode=%s)…",
            initModeName(active_init_mode_));

        // Set source ONCE here — all tryAlignAt calls reuse the cached
        // source covariances without re-preprocessing.
        vgicp_->setInputSource(scan);

        // Always search against the FULL map here, never a cropped submap:
        // a stale local submap left over from a previous tracking session
        // (or an earlier runtime re-seed) may not even cover the new seed /
        // search area. TRACKING switches back to an efficient local submap
        // once finishInit() succeeds.
        vgicp_->setInputTarget(global_map_);

        switch (active_init_mode_) {

            // ── MODE 1: Seeded position + heading ─────────────────────────────
            case InitMode::SEEDED_POSITION_HEADING: {
                auto coarse = spiralSearchSeededHeading(
                    init_x_, init_y_, init_yaw_, activeSpiralMaxRadius());
                if (!coarse) return;  // warning already logged inside the spiral

                // Refine winner with full iteration budget
                auto [refined, fitness] = tryAlignAt(*coarse, vgicp_max_iter_);
                RCLCPP_INFO(get_logger(),
                    "[init] SEEDED_POSITION_HEADING refined fitness=%.4f  "
                    "pos=(%.2f, %.2f, %.2f)",
                    fitness,
                    refined.translation().x(), refined.translation().y(), refined.translation().z());
                finishInit(refined, stamp, fitness);
                return;
            }

            // ── MODE 2: Seeded position, heading unknown ──────────────────────
            case InitMode::SEEDED_POSITION: {
                auto coarse = spiralSearchHeadingSweep(init_x_, init_y_, seed_search_max_radius_);
                if (!coarse) return;  // warning already logged inside the spiral

                auto [refined, fitness] = tryAlignAt(*coarse, vgicp_max_iter_);
                RCLCPP_INFO(get_logger(),
                    "[init] SEEDED_POSITION refined fitness=%.4f  "
                    "pos=(%.2f, %.2f, %.2f)  yaw=%.3f rad",
                    fitness,
                    refined.translation().x(), refined.translation().y(), refined.translation().z(),
                    std::atan2(refined.rotation()(1, 0), refined.rotation()(0, 0)));
                finishInit(refined, stamp, fitness);
                return;
            }

            // ── MODE 3: No seed at all — grid search ──────────────────────────
            case InitMode::NO_SEED: {
                // Relies on global_map_ (which is pre-set in buildVGICP)
                auto coarse = noSeedSearch();
                if (!coarse) return;  // warning already logged inside search

                auto [refined, fitness] = tryAlignAt(*coarse, vgicp_max_iter_);
                RCLCPP_INFO(get_logger(),
                    "[init] NO_SEED refined fitness=%.4f  "
                    "pos=(%.2f, %.2f, %.2f)  yaw=%.3f rad",
                    fitness,
                    refined.translation().x(), refined.translation().y(), refined.translation().z(),
                    std::atan2(refined.rotation()(1, 0), refined.rotation()(0, 0)));
                finishInit(refined, stamp, fitness);
                return;
            }

            case InitMode::TF_PRIOR:
                // Should never reach here — state_ is set to TRACKING in
                // loadParams(), and reinitPoseCallback() never sets
                // active_init_mode_ back to TF_PRIOR.
                state_ = LocalizerState::TRACKING;
                return;
        }
    }

    /// Transition to TRACKING and publish the initial pose for the EKF.
    void finishInit(const Eigen::Isometry3d & pose, const rclcpp::Time & stamp, double fitness)
    {
        post_init_warmup_ = 5;  // use last_good_pose_ as prior for the first 5 tracking scans
        last_good_pose_ = pose;
        state_           = LocalizerState::TRACKING;

        // The search phase intentionally matched against the full global map
        // (see runInitialization) since seed/spiral candidates can land
        // arbitrarily far from any previous submap. Force a fresh, tight
        // local submap around the pose we just locked in before tracking
        // resumes, so per-scan ICP stays cheap.
        last_submap_center_ = Eigen::Vector3d(
            std::numeric_limits<double>::max(), 0.0, 0.0);
        updateLocalMap(pose.translation());

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
    //  Runtime re-seed
    // =========================================================================

    /// Accepts a fresh seed pose at any time — whether the node is still
    /// NEEDS_INIT or already TRACKING — and restarts initialization from it.
    /// Always re-seeds via SEEDED_POSITION_HEADING: a PoseStamped already
    /// carries x, y, AND yaw, so there's no ambiguity left for a heading
    /// sweep or a full grid search to resolve. Typical use: wire a GPS
    /// filter's output to this topic so it can pull ICP back onto the map
    /// once tracking fitness starts degrading.
    ///
    /// Gated two ways so a fast/continuous publisher (e.g. a GPS filter
    /// streaming at several Hz) can't repeatedly yank the node out of
    /// TRACKING before it ever settles:
    ///   1. Rejected outright while a search is currently in progress
    ///      (init_search_in_progress_, set by InitSearchGuard).
    ///   2. Rejected for reinit_min_interval_ seconds after the LAST search
    ///      actually finished (last_init_finish_time_) — not from when a
    ///      message was merely received, so a slow search can't be
    ///      interrupted the instant it completes.
    /// Either way, extra messages are simply dropped; the next one that
    /// arrives once both gates are clear wins with its own (by then more
    /// current) pose.
    ///
    /// NOTE: assumes a single-threaded executor (the ROS 2 default for this
    /// node) — which is also what makes gate #1 possible to check reliably.
    /// If this node is ever run with a multi-threaded executor, all of
    /// state_ / init_x_ / init_y_ / init_yaw_ / active_init_mode_ /
    /// came_from_reinit_ / init_search_in_progress_ / last_init_finish_time_
    /// would need a mutex, since they're also touched from cloudCallback().
    void reinitPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (init_search_in_progress_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "[reinit] Ignoring re-seed — a search is currently in progress.");
            return;
        }

        if (last_init_finish_time_) {
            const double dt = (get_clock()->now() - *last_init_finish_time_).seconds();
            if (dt < reinit_min_interval_) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                    "[reinit] Ignoring re-seed — only %.2fs since the last search finished "
                    "(reinit_min_interval=%.2fs). If you're streaming from a continuous "
                    "source (e.g. GPS), this is expected and intentional.",
                    dt, reinit_min_interval_);
                return;
            }
        }

        if (!msg->header.frame_id.empty() && msg->header.frame_id != map_frame_) {
            RCLCPP_WARN(get_logger(),
                "[reinit] Pose frame_id='%s' does not match map_frame='%s' — "
                "proceeding anyway (assuming it was already transformed into the map frame).",
                msg->header.frame_id.c_str(), map_frame_.c_str());
        }

        tf2::Quaternion q;
        tf2::fromMsg(msg->pose.orientation, q);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        init_x_   = msg->pose.position.x;
        init_y_   = msg->pose.position.y;
        init_yaw_ = yaw;

        active_init_mode_ = InitMode::SEEDED_POSITION_HEADING;
        came_from_reinit_ = true;  // sticky — uses reinit_search_max_radius_ from here on

        // Force the next init pass to re-search the full map, and force a
        // submap recentre once it succeeds (see runInitialization / finishInit).
        last_submap_center_ = Eigen::Vector3d(
            std::numeric_limits<double>::max(), 0.0, 0.0);

        const bool was_tracking = (state_ == LocalizerState::TRACKING);
        state_ = LocalizerState::NEEDS_INIT;

        RCLCPP_WARN(get_logger(),
            "[reinit] New seed received (was %s) — x=%.2f  y=%.2f  yaw=%.3f rad. "
            "Re-running SEEDED_POSITION_HEADING spiral search (max_radius=%.1fm) "
            "on the next scan.",
            was_tracking ? "TRACKING" : "NEEDS_INIT",
            init_x_, init_y_, init_yaw_, reinit_search_max_radius_);
    }

    // =========================================================================
    //  Heading sweep core  (used per-point by MODE 2, and per-cell in MODE 3)
    // =========================================================================

    /// Try init_heading_candidates_ evenly-spaced yaws at (x, y) and return
    /// whichever had the lowest fitness. Always returns a pose (as long as
    /// init_heading_candidates_ >= 1) — the caller decides whether the
    /// fitness is actually good enough.
    /// vgicp_ source must already be set before calling.
    std::pair<std::optional<Eigen::Isometry3d>, double>
    headingSweepCore(double x, double y)
    {
        double best_fitness = std::numeric_limits<double>::max();
        std::optional<Eigen::Isometry3d> best_pose;

        // Inject dynamic Z here
        const double z = getElevationAt(x, y);

        const double step = 2.0 * M_PI / init_heading_candidates_;
        for (int i = 0; i < init_heading_candidates_; ++i) {
            const double yaw = i * step;
            auto [pose, fitness] = tryAlignAt(makeXYZYaw(x, y, z, yaw), init_search_max_iter_);
            RCLCPP_DEBUG(get_logger(),
                "[init/sweep] (%.2f, %.2f) yaw=%.2f rad  fitness=%.4f", x, y, yaw, fitness);
            if (fitness < best_fitness) {
                best_fitness = fitness;
                best_pose    = pose;
            }
        }
        return {best_pose, best_fitness};
    }

    /// Builds the heading offsets tried at every spiral position in
    /// spiralSearchSeededHeading(), ordered CENTRE-OUT: [0, +step, -step,
    /// +2·step, -2·step, ...] up to ±seed_heading_tolerance_. Trying 0
    /// (the seeded yaw exactly) first means a genuinely good seed still
    /// resolves in a single ICP call per position — the window only costs
    /// extra ICP calls when the seed actually needed correcting.
    /// seed_heading_candidates_ <= 1 → just {0.0} (trust the seed exactly).
    std::vector<double> headingWindowOffsets() const
    {
        if (seed_heading_candidates_ <= 1)
            return {0.0};

        std::vector<double> offsets;
        offsets.reserve(seed_heading_candidates_);

        const bool include_center = (seed_heading_candidates_ % 2) == 1;
        const int  half           = seed_heading_candidates_ / 2;
        const double step = seed_heading_tolerance_ / std::max(1, half);

        if (include_center) offsets.push_back(0.0);
        for (int i = 1; i <= half; ++i) {
            offsets.push_back( i * step);
            offsets.push_back(-i * step);
        }
        return offsets;
    }

    // =========================================================================
    //  Spiral search around a seed position  (MODE 1 and MODE 2)
    // =========================================================================

    /// MODE 1 — x, y trusted; yaw is a SEED, not gospel. Spirals outward in
    /// position ring by ring (see ringOffsets); at every ring point, tries
    /// the narrow centre-out heading window from headingWindowOffsets()
    /// around the seeded yaw instead of trusting it exactly (GPS/compass
    /// headings are rarely perfectly accurate). Stops at the very first
    /// (position, heading) candidate that beats max_fitness_accept — this
    /// early exit is deliberate: we trust the seed enough that we just
    /// need tolerance for a few metres of position error and a few degrees
    /// of heading error, not an exhaustive search over either.
    /// `max_radius` lets the caller pick seed_search_max_radius_ (startup
    /// seed) vs. reinit_search_max_radius_ (runtime re-seed) — see
    /// activeSpiralMaxRadius().
    /// vgicp_ source must already be set, and the target must already cover
    /// the whole search area (runInitialization sets it to global_map_)
    /// before calling.
    std::optional<Eigen::Isometry3d>
    spiralSearchSeededHeading(double seed_x, double seed_y, double seed_yaw, double max_radius)
    {
        const int max_ring = std::max(0, static_cast<int>(
            std::ceil(max_radius / seed_search_step_)));
        const std::vector<double> yaw_offsets = headingWindowOffsets();

        double best_fitness = std::numeric_limits<double>::max();
        std::optional<Eigen::Isometry3d> best_pose;
        size_t calls = 0;

        for (int ring = 0; ring <= max_ring; ++ring) {
            for (const auto & [dix, diy] : ringOffsets(ring)) {
                const double x = seed_x + dix * seed_search_step_;
                const double y = seed_y + diy * seed_search_step_;
                const double z = getElevationAt(x, y);

                for (const double dyaw : yaw_offsets) {
                    auto [pose, fitness] = tryAlignAt(
                        makeXYZYaw(x, y, z, seed_yaw + dyaw), init_search_max_iter_);
                    ++calls;

                    if (fitness < best_fitness) {
                        best_fitness = fitness;
                        best_pose    = pose;
                    }
                    if (fitness <= max_fitness_) {
                        RCLCPP_INFO(get_logger(),
                            "[init/spiral] Match at ring=%d (radius=%.1fm) pos=(%.2f, %.2f) "
                            "yaw_offset=%+.1f°  after %zu ICP call(s) — fitness=%.4f",
                            ring, ring * seed_search_step_, x, y,
                            dyaw * 180.0 / M_PI, calls, fitness);
                        return best_pose;
                    }
                }
            }
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                "[init/spiral] searched ring=%d/%d (radius=%.1f/%.1fm) × "
                "%zu headings/point, %zu ICP call(s) so far, best_fitness=%.4f",
                ring, max_ring, ring * seed_search_step_, max_radius,
                yaw_offsets.size(), calls, best_fitness);
        }

        RCLCPP_WARN(get_logger(),
            "[init/spiral] Exhausted search radius=%.1fm × ±%.1f° heading window "
            "(%zu ICP calls) without reaching threshold=%.4f. Best fitness=%.4f. "
            "Check init_x/init_y/init_yaw, or increase seed_search_max_radius / "
            "seed_heading_tolerance_deg / reinit_search_max_radius. Will retry on next scan.",
            max_radius, seed_heading_tolerance_ * 180.0 / M_PI, calls, max_fitness_, best_fitness);
        return std::nullopt;
    }

    /// MODE 2 — x, y trusted, yaw unknown. Same outward spiral as above, but
    /// sweeps init_heading_candidates_ headings at every ring point (via
    /// headingSweepCore) since there's no heading to narrow things down with.
    /// Stops at the first ring point whose best heading beats max_fitness_accept.
    std::optional<Eigen::Isometry3d>
    spiralSearchHeadingSweep(double seed_x, double seed_y, double max_radius)
    {
        const int max_ring = std::max(0, static_cast<int>(
            std::ceil(max_radius / seed_search_step_)));

        double best_fitness = std::numeric_limits<double>::max();
        std::optional<Eigen::Isometry3d> best_pose;
        size_t calls = 0;

        for (int ring = 0; ring <= max_ring; ++ring) {
            for (const auto & [dix, diy] : ringOffsets(ring)) {
                const double x = seed_x + dix * seed_search_step_;
                const double y = seed_y + diy * seed_search_step_;

                auto [pose, fitness] = headingSweepCore(x, y);
                calls += static_cast<size_t>(init_heading_candidates_);

                if (fitness < best_fitness) {
                    best_fitness = fitness;
                    best_pose    = pose;
                }
                if (pose && fitness <= max_fitness_) {
                    RCLCPP_INFO(get_logger(),
                        "[init/spiral] Match at ring=%d (radius=%.1fm) pos=(%.2f, %.2f) "
                        "after %zu ICP call(s) — fitness=%.4f",
                        ring, ring * seed_search_step_, x, y, calls, fitness);
                    return best_pose;
                }
            }
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                "[init/spiral] searched ring=%d/%d (radius=%.1f/%.1fm), "
                "%zu ICP call(s) so far, best_fitness=%.4f",
                ring, max_ring, ring * seed_search_step_, max_radius,
                calls, best_fitness);
        }

        RCLCPP_WARN(get_logger(),
            "[init/spiral] Exhausted search radius=%.1fm (%zu ICP calls) without "
            "reaching threshold=%.4f. Best fitness=%.4f. Check init_x/init_y, "
            "or increase seed_search_max_radius. Will retry on next scan.",
            max_radius, calls, max_fitness_, best_fitness);
        return std::nullopt;
    }

    // =========================================================================
    //  No-seed grid search  (MODE 3)
    // =========================================================================

    /// Grid over every occupied map cell × heading sweep — used when nothing
    /// at all is known about the robot's pose.
    /// vgicp_ source must already be set before calling.
    ///
    /// Candidate positions are derived directly from the map point cloud:
    /// each cell that contains ≥1 map point becomes one candidate (at the
    /// cell centre).  Empty cells (open space, voids) are skipped, which
    /// keeps the call count proportional to the mapped area, not the bbox.
    std::optional<Eigen::Isometry3d> noSeedSearch()
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
        occupied.reserve(global_map_->size());
        for (const auto & pt : *global_map_)
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
            "[init/no_seed] %zu occupied cells × %d headings = %zu ICP calls. "
            "Map extents: %.0f × %.0f m — standing by…",
            candidates.size(), init_heading_candidates_, total,
            static_cast<double>(map_max_x_ - map_min_x_),
            static_cast<double>(map_max_y_ - map_min_y_));

        double best_fitness = std::numeric_limits<double>::max();
        std::optional<Eigen::Isometry3d> best_pose;
        const double yaw_step = 2.0 * M_PI / init_heading_candidates_;

        size_t count = 0;
        for (const auto & c : candidates) {
            // Inject dynamic Z per cell
            double z = getElevationAt(c.x, c.y);
            
            for (int h = 0; h < init_heading_candidates_; ++h) {
                auto [pose, fitness] = tryAlignAt(
                    makeXYZYaw(c.x, c.y, z, h * yaw_step),
                    init_search_max_iter_);

                if (fitness < best_fitness) {
                    best_fitness = fitness;
                    best_pose    = pose;
                }
                // Progress heartbeat every 100 calls
                if (++count % 100 == 0)
                    RCLCPP_INFO(get_logger(),
                        "[init/no_seed] %zu / %zu  best_fitness=%.4f",
                        count, total, best_fitness);
            }
        }

        RCLCPP_INFO(get_logger(),
            "[init/no_seed] Search complete — best fitness=%.4f", best_fitness);

        if (best_fitness > max_fitness_) {
            RCLCPP_WARN(get_logger(),
                "[init/no_seed] No candidate below threshold=%.4f. "
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
        // Post-init warmup: EKF hasn't propagated the reset yet, trust our pose.
        if (post_init_warmup_ > 0 && last_good_pose_) {
            --post_init_warmup_;
            return last_good_pose_;
        }

        try {
            // TimePointZero = latest available TF (includes all retroactive corrections).
            // This is the key fix: scan_stamp snapshots are always pre-correction.
            const auto tf = tf_buffer_->lookupTransform(
                map_frame_, base_frame_, stamp,
                rclcpp::Duration::from_seconds(0.05));
            return tf2::transformToEigen(tf);

        } catch (const tf2::TransformException & ex) {

            if (active_init_mode_ == InitMode::TF_PRIOR) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10'000,
                    "[TRACKING] Waiting for EKF TF: %s", ex.what());
                return std::nullopt;
            }

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

    pcl::PointCloud<pcl::PointXYZI>::Ptr cropCloud(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr & in, double max_radius)
    {
        auto out = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        out->reserve(in->size());
        const double r2 = max_radius * max_radius;
        
        for (const auto & pt : *in) {
            if ((pt.x * pt.x + pt.y * pt.y) <= r2) {
                out->push_back(pt);
            }
        }
        return out;
    }

    void updateLocalMap(const Eigen::Vector3d & current_pos)
    {
        // Check if we've moved far enough to justify updating the map
        if ((current_pos - last_submap_center_).norm() < submap_update_dist_) {
            return; 
        }

        auto local_map = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        const double r2 = submap_radius_ * submap_radius_;

        for (const auto & pt : *global_map_) {
            double dx = pt.x - current_pos.x();
            double dy = pt.y - current_pos.y();
            if ((dx*dx + dy*dy) <= r2) {
                local_map->push_back(pt);
            }
        }

        // Give the lean, local map to VGICP
        vgicp_->setInputTarget(local_map);
        last_submap_center_ = current_pos;
        
        RCLCPP_INFO(get_logger(), "[Map] Updated local submap: %zu points", local_map->size());
    }

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
        double cov = 0.01 + (0.1 * fitness);  // increase covariance for worse fits
        msg.pose.covariance[0]  = cov;  // X
        msg.pose.covariance[7]  = cov;  // Y
        msg.pose.covariance[35] = cov;  // Yaw

        // High covariance for these ignored dimensions just in case
        msg.pose.covariance[14] = 1e9;  // Z
        msg.pose.covariance[21] = 1e9;  // Roll
        msg.pose.covariance[28] = 1e9;  // Pitch

        return msg;
    }

    void publishPose(const Eigen::Isometry3d & map_to_base, const rclcpp::Time & stamp, double fitness)
    {
        pub_pose_->publish(buildPoseMsg(map_to_base, stamp, fitness));
    }

    /// Publishes fitness on every tracking scan, accepted or rejected.
    /// This is deliberately NOT gated on the fitness > max_fitness_ check --
    /// a consumer (e.g. gps_filter) needs to see fitness go bad in real time,
    /// not only see updates while ICP happens to be healthy.
    void publishFitnessDiagnostics(double fitness, const rclcpp::Time & stamp)
    {
        diagnostic_msgs::msg::DiagnosticStatus status;
        status.name        = "icp_tracking";
        status.hardware_id = "map_odom_localizer";
        status.level = (fitness <= max_fitness_)
            ? diagnostic_msgs::msg::DiagnosticStatus::OK
            : diagnostic_msgs::msg::DiagnosticStatus::ERROR;

        diagnostic_msgs::msg::KeyValue kv;
        kv.key   = "fitness_score";
        kv.value = std::to_string(fitness);
        status.values.push_back(kv);

        diagnostic_msgs::msg::DiagnosticArray diag_array;
        diag_array.header.stamp = stamp;
        diag_array.status.push_back(status);
        pub_fitness_diag_->publish(diag_array);
    }

    // =========================================================================
    //  Member variables
    // =========================================================================

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr             sub_cloud_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr           sub_reinit_pose_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_ekf_reset_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr        pub_fitness_diag_;
    std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // ── Core config ──────────────────────────────────────────────────────────
    std::string map_frame_, base_frame_, lidar_topic_, map_ply_path_;
    double      voxel_leaf_map_, voxel_leaf_scan_, vgicp_corr_dist_, max_fitness_;
    int         vgicp_resolution_, vgicp_max_iter_;
    double      scan_crop_radius_, submap_radius_, submap_update_dist_;

    // ── Terrain config ───────────────────────────────────────────────────────
    std::string terrain_ply_path_;
    double      terrain_grid_resolution_ = 0.5;
    std::unordered_map<uint64_t, double> elevation_grid_;

    // ── Init config ──────────────────────────────────────────────────────────
    InitMode init_mode_               = InitMode::TF_PRIOR;  ///< launch-time configured mode
    InitMode active_init_mode_        = InitMode::TF_PRIOR;  ///< operative mode — can change via reinitPoseCallback
    double   init_x_                  = 0.0;
    double   init_y_                  = 0.0;
    double   init_yaw_                = 0.0;
    int      init_heading_candidates_ = 16;
    int      seed_heading_candidates_ = 5;     ///< SEEDED_POSITION_HEADING: headings tried per spiral position
    double   seed_heading_tolerance_  = 20.0 * M_PI / 180.0;  ///< ± window around init_yaw (radians)
    double   global_search_step_      = 5.0;   ///< NO_SEED grid step (metres)
    double   seed_search_step_        = 2.0;   ///< SEEDED_POSITION[_HEADING] spiral ring spacing (metres)
    double   seed_search_max_radius_  = 30.0;  ///< startup-seed spiral give-up radius (metres)
    double   reinit_search_max_radius_ = 15.0; ///< runtime-re-seed spiral give-up radius (metres) — see activeSpiralMaxRadius()
    bool     came_from_reinit_        = false; ///< sticky once true — set by reinitPoseCallback
    int      init_search_max_iter_    = 20;

    std::string ekf_reset_topic_;
    std::string reinit_pose_topic_;
    double      reinit_min_interval_ = 3.0;  ///< cooldown after a search finishes before another reinit is accepted (s)
    bool        init_search_in_progress_ = false;  ///< set by InitSearchGuard for the duration of runInitialization()
    std::optional<rclcpp::Time> last_init_finish_time_;  ///< set by InitSearchGuard when a search (any outcome) ends
    std::string cuda_nn_method_ = "gpu_rbf_kernel";  ///< CUDA-only: NN search backend

    // ── Runtime state ─────────────────────────────────────────────────────────
    LocalizerState                   state_            = LocalizerState::NEEDS_INIT;
    std::optional<Eigen::Isometry3d> last_good_pose_;
    int                              post_init_warmup_ = 0;  // number of initial tracking scans to trust last_good_pose_ over TF

    // ── Map & matcher ─────────────────────────────────────────────────────────
    std::shared_ptr<VGICPVariant>        vgicp_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr global_map_;
    Eigen::Vector3d                      last_submap_center_{std::numeric_limits<double>::max(), 0.0, 0.0};
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