/**
 * map_odom_localizer.cpp (ROS 2 Version)
 *
 * Localizes a robot within a pre-built .ply map using FastVGICP.
 * * ARCHITECTURE:
 * This node relies on a Global EKF to provide the initial guess. 
 * It asks the TF tree for map --> base_link (the fused GPS/Odom prior),
 * runs the LiDAR alignment, and publishes the map --> base_link pose.
 * The Global EKF consumes this to publish the map --> odom TF.
 */

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>
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
#include <memory>
#include <string>
#include <optional>

class MapOdomLocalizer : public rclcpp::Node {
public:
    MapOdomLocalizer() : Node("map_odom_localizer") {
        loadParams();

        // This gives the background thread time to cache TF messages 
        // while the massive .ply map loads in the next step.
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        loadMap();
        buildVGICP();

        // Publishers & Subscribers
        pub_pose_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "localizer/pose", rclcpp::QoS(10));

        // Note on QoS: SensorDataQoS (keep_last(1), best_effort) is used here intentionally
        // We WANT to drop scans if ICP runs slow to ensure the localizer never falls behind real-time
        sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            lidar_topic_, rclcpp::SensorDataQoS(),
            std::bind(&MapOdomLocalizer::cloudCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "[map_odom_localizer] Ready. Waiting for EKF priors on TF...");
    }

private:
    void loadParams() {
        this->declare_parameter("map_frame", "map");
        this->declare_parameter("base_frame", "base_link");
        this->declare_parameter("lidar_topic", "/ouster/points");
        this->declare_parameter("map_ply_path", "");
        this->declare_parameter("voxel_leaf_map", 0.3);
        this->declare_parameter("voxel_leaf_scan", 0.1);
        this->declare_parameter("vgicp_resolution", 1);
        this->declare_parameter("vgicp_max_iterations", 64);
        this->declare_parameter("vgicp_max_corresp_dist", 1.5);
        this->declare_parameter("max_fitness_accept", 1.0);

        map_frame_       = this->get_parameter("map_frame").as_string();
        base_frame_      = this->get_parameter("base_frame").as_string();
        lidar_topic_     = this->get_parameter("lidar_topic").as_string();
        map_ply_path_    = this->get_parameter("map_ply_path").as_string();
        
        voxel_leaf_map_  = this->get_parameter("voxel_leaf_map").as_double();
        voxel_leaf_scan_ = this->get_parameter("voxel_leaf_scan").as_double();
        max_fitness_     = this->get_parameter("max_fitness_accept").as_double();
        
        vgicp_resolution_ = this->get_parameter("vgicp_resolution").as_int();
        vgicp_max_iter_   = this->get_parameter("vgicp_max_iterations").as_int();
        vgicp_corr_dist_  = this->get_parameter("vgicp_max_corresp_dist").as_double();
    }

    void loadMap() {
        if (map_ply_path_.empty()) {
            RCLCPP_FATAL(this->get_logger(), "map_ply_path is not set. Cannot localise.");
            rclcpp::shutdown();
            return;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr raw_xyz(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPLYFile<pcl::PointXYZ>(map_ply_path_, *raw_xyz) < 0) {
            RCLCPP_FATAL(this->get_logger(), "Failed to load PLY map: %s", map_ply_path_.c_str());
            rclcpp::shutdown();
            return;
        }

        // [FIX 8]: Guard against silently loading XYZRGB clouds as empty files
        if (raw_xyz->empty()) {
            RCLCPP_FATAL(this->get_logger(),
                "Map loaded but is empty — PLY may have XYZRGB fields. "
                "Re-save as XYZ-only or change the load type.");
            rclcpp::shutdown();
            return;
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr raw_xyzi(new pcl::PointCloud<pcl::PointXYZI>);
        raw_xyzi->reserve(raw_xyz->size());
        for (const auto& p : *raw_xyz) {
            pcl::PointXYZI pi;
            pi.x = p.x; pi.y = p.y; pi.z = p.z; pi.intensity = 0.0f;
            raw_xyzi->push_back(pi);
        }

        map_cloud_ = downsample(raw_xyzi, voxel_leaf_map_);
        RCLCPP_INFO(this->get_logger(), "Map loaded: %zu pts (downsampled)", map_cloud_->size());
    }

    void buildVGICP() {
        vgicp_ = std::make_shared<VGICPVariant>();
        vgicp_->setResolution(vgicp_resolution_);
        vgicp_->setMaxCorrespondenceDistance(vgicp_corr_dist_);
        vgicp_->setMaximumIterations(vgicp_max_iter_);
        vgicp_->setTransformationEpsilon(1e-4);
        vgicp_->setEuclideanFitnessEpsilon(1e-4);
#ifndef USE_CUDA_VGICP
        vgicp_->setNumThreads(0); 
        RCLCPP_INFO(this->get_logger(), "Matcher: FastVGICP (CPU)");
#else
        RCLCPP_INFO(this->get_logger(), "Matcher: cuVGICP (CUDA GPU)");
#endif
        vgicp_->setInputTarget(map_cloud_);
    }

    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        rclcpp::Time stamp = msg->header.stamp;

        // 1. Transform Lidar -> Base Link
        auto scan_base = toBaseFrame(msg, stamp);
        if (!scan_base) return;
        auto scan_ds = downsample(scan_base, voxel_leaf_scan_);

        // 2. Look up the prior guess from the Global EKF
        Eigen::Isometry3d initial_guess;
        try {
            auto tf_prior = tf_buffer_->lookupTransform(
                map_frame_, base_frame_, stamp, rclcpp::Duration::from_seconds(0.05));
            initial_guess = tf2::transformToEigen(tf_prior);
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
                "[map_odom_localizer] Waiting for Global EKF to publish map->base_link prior. "
                "(This is expected during initial GPS convergence): %s", ex.what());
            return;
        }

        // 3. VGICP Scan Matching
        vgicp_->setInputSource(scan_ds);
        pcl::PointCloud<pcl::PointXYZI> aligned;
        vgicp_->align(aligned, initial_guess.matrix().cast<float>());

        // Cache and print the actual fitness score for debugging
        double fitness = vgicp_->getFitnessScore();
        if (!vgicp_->hasConverged() || fitness > max_fitness_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "[map_odom_localizer] ICP failed. fitness=%.4f (threshold=%.4f). Scan dropped.", 
                fitness, max_fitness_);
            return;
        }

        Eigen::Isometry3d map_to_base;
        map_to_base.matrix() = vgicp_->getFinalTransformation().cast<double>();

        // 4. Publish absolute pose for the EKF
        publishPose(map_to_base, stamp);
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr downsample(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& in, double leaf)
    {
        auto out = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        pcl::VoxelGrid<pcl::PointXYZI> vg;
        vg.setLeafSize(leaf, leaf, leaf);
        vg.setInputCloud(in);
        vg.filter(*out);
        return out;
    }

    std::optional<pcl::PointCloud<pcl::PointXYZI>::Ptr>
    toBaseFrame(const sensor_msgs::msg::PointCloud2::SharedPtr& msg, const rclcpp::Time& stamp) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*msg, *cloud);

        if (msg->header.frame_id == base_frame_) return cloud;

        try {
            auto tf_msg = tf_buffer_->lookupTransform(
                base_frame_, msg->header.frame_id, stamp, rclcpp::Duration::from_seconds(0.1));
            Eigen::Affine3d T = tf2::transformToEigen(tf_msg);
            auto out = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
            pcl::transformPointCloud(*cloud, *out, T.matrix().cast<float>());
            return out;
        } catch (const tf2::TransformException& ex) {
            // Warn if the URDF is missing the LiDAR link
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "[map_odom_localizer] Sensor TF failed: %s. Is the static transform for '%s' -> '%s' published?",
                ex.what(), msg->header.frame_id.c_str(), base_frame_.c_str());
            return std::nullopt;
        }
    }

    void publishPose(const Eigen::Isometry3d& map_to_base, const rclcpp::Time& stamp) {
        geometry_msgs::msg::PoseWithCovarianceStamped out;
        out.header.stamp    = stamp;
        out.header.frame_id = map_frame_;
        Eigen::Quaterniond q(map_to_base.rotation());
        auto& p = out.pose.pose;
        p.position.x    = map_to_base.translation().x();
        p.position.y    = map_to_base.translation().y();
        p.position.z    = map_to_base.translation().z();
        p.orientation.x = q.x();
        p.orientation.y = q.y();
        p.orientation.z = q.z();
        p.orientation.w = q.w();

        // Populate a baseline covariance matrix for the EKF to consume
        out.pose.covariance[0] = 0.01; // X
        out.pose.covariance[7] = 0.01; // Y
        out.pose.covariance[14] = 0.01; // Z
        out.pose.covariance[21] = 0.01; // Roll
        out.pose.covariance[28] = 0.01; // Pitch
        out.pose.covariance[35] = 0.01; // Yaw

        pub_pose_->publish(out);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_;

    std::shared_ptr<tf2_ros::Buffer>               tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener>    tf_listener_;

    std::string map_frame_, base_frame_, lidar_topic_, map_ply_path_;
    double voxel_leaf_map_, voxel_leaf_scan_, vgicp_corr_dist_, max_fitness_;
    int    vgicp_resolution_, vgicp_max_iter_;

    std::shared_ptr<VGICPVariant>        vgicp_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapOdomLocalizer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}