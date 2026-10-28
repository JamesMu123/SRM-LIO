#include <mutex>
#include <vector>
#include <queue>
#include <memory>
#include <iostream>
#include <chrono>
#include <stdexcept>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <unistd.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#if SRM_HAS_LIVOX
#include <livox_ros_driver2/msg/custom_msg.hpp>
#endif

#include "utils.h"
#include "map_builder/commons.h"
#include "map_builder/map_builder.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include "tf2_ros/transform_broadcaster.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;
struct NodeConfig
{
    std::string imu_topic = "/livox/imu";
    std::string lidar_topic = "/livox/lidar";
    std::string lidar_type = "livox";
    double imu_acc_scale = 10.0;
    std::string body_frame = "body";
    std::string world_frame = "lidar";
    bool print_time_cost = false;
    std::string experiment_dir;
    std::string config_path;
};
struct StateData
{
    bool lidar_pushed = false;
    std::mutex imu_mutex;
    std::mutex lidar_mutex;
    double last_lidar_time = -1.0;
    double last_imu_time = -1.0;
    std::deque<IMUData> imu_buffer;
    std::deque<std::pair<double, pcl::PointCloud<pcl::PointXYZINormal>::Ptr>> lidar_buffer;
    nav_msgs::msg::Path path;
};

class LIONode : public rclcpp::Node
{
public:
    LIONode() : Node("lio_node")
    {
        RCLCPP_INFO(this->get_logger(), "LIO Node Started");
        loadParameters();

        m_imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(m_node_config.imu_topic, 10, std::bind(&LIONode::imuCB, this, std::placeholders::_1));
        if (m_node_config.lidar_type == "livox")
#if SRM_HAS_LIVOX
            m_lidar_sub = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(m_node_config.lidar_topic, 10, std::bind(&LIONode::lidarCB, this, std::placeholders::_1));
#else
            throw std::runtime_error("This build has no livox_ros_driver2; rebuild after installing the driver");
#endif
        else if (m_node_config.lidar_type == "pointcloud2")
            m_pointcloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(m_node_config.lidar_topic, 10, std::bind(&LIONode::pointCloudCB, this, std::placeholders::_1));
        else
            throw std::invalid_argument("lidar_type must be livox or pointcloud2");

        m_body_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("body_cloud", 10000);
        m_world_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("world_cloud", 10000);
        m_path_pub = this->create_publisher<nav_msgs::msg::Path>("lio_path", 10000);
        m_odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("lio_odom", 10000);
        m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

        m_state_data.path.poses.clear();
        m_state_data.path.header.frame_id = m_node_config.world_frame;

        m_kf = std::make_shared<IESKF>();
        m_builder = std::make_shared<MapBuilder>(m_builder_config, m_kf);
        initializeExperimentLog();
        m_timer = this->create_wall_timer(20ms, std::bind(&LIONode::timerCB, this));
    }

    ~LIONode() override
    {
        saveFinalMap();
    }

    void loadParameters()
    {
        this->declare_parameter("config_path", "");
        this->declare_parameter("experiment_dir", "");
        std::string config_path;
        this->get_parameter<std::string>("config_path", config_path);
        m_node_config.config_path = config_path;
        this->get_parameter<std::string>("experiment_dir", m_node_config.experiment_dir);

        YAML::Node config = YAML::LoadFile(config_path);
        if (config["base_config"])
        {
            const std::filesystem::path base_path = std::filesystem::path(config_path).parent_path() /
                                                    config["base_config"].as<std::string>();
            YAML::Node merged = YAML::LoadFile(base_path.string());
            for (const auto &entry : config)
            {
                const std::string key = entry.first.as<std::string>();
                if (key != "base_config")
                    merged[key] = entry.second;
            }
            config = merged;
        }
        m_effective_config = YAML::Dump(config);
        if (!config)
        {
            RCLCPP_WARN(this->get_logger(), "FAIL TO LOAD YAML FILE!");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "LOAD FROM YAML CONFIG PATH: %s", config_path.c_str());

        m_node_config.imu_topic = config["imu_topic"].as<std::string>();
        m_node_config.lidar_topic = config["lidar_topic"].as<std::string>();
        m_node_config.lidar_type = config["lidar_type"] ? config["lidar_type"].as<std::string>() : "livox";
        m_node_config.imu_acc_scale = config["imu_acc_scale"] ? config["imu_acc_scale"].as<double>() : 10.0;
        m_node_config.body_frame = config["body_frame"].as<std::string>();
        m_node_config.world_frame = config["world_frame"].as<std::string>();
        m_node_config.print_time_cost = config["print_time_cost"].as<bool>();

        m_builder_config.lidar_filter_num = config["lidar_filter_num"].as<int>();
        m_builder_config.lidar_min_range = config["lidar_min_range"].as<double>();
        m_builder_config.lidar_max_range = config["lidar_max_range"].as<double>();
        m_builder_config.scan_resolution = config["scan_resolution"].as<double>();
        m_builder_config.map_resolution = config["map_resolution"].as<double>();
        m_builder_config.cube_len = config["cube_len"].as<double>();
        m_builder_config.det_range = config["det_range"].as<double>();
        m_builder_config.move_thresh = config["move_thresh"].as<double>();
        m_builder_config.na = config["na"].as<double>();
        m_builder_config.ng = config["ng"].as<double>();
        m_builder_config.nba = config["nba"].as<double>();
        m_builder_config.nbg = config["nbg"].as<double>();

        m_builder_config.imu_init_num = config["imu_init_num"].as<int>();
        m_builder_config.normalize_imu_acc = config["normalize_imu_acc"] ? config["normalize_imu_acc"].as<bool>() : false;
        m_builder_config.near_search_num = config["near_search_num"].as<int>();
        m_builder_config.ieskf_max_iter = config["ieskf_max_iter"].as<int>();
        m_builder_config.gravity_align = config["gravity_align"].as<bool>();
        m_builder_config.esti_il = config["esti_il"].as<bool>();
        std::vector<double> t_il_vec = config["t_il"].as<std::vector<double>>();
        std::vector<double> r_il_vec = config["r_il"].as<std::vector<double>>();
        m_builder_config.t_il << t_il_vec[0], t_il_vec[1], t_il_vec[2];
        m_builder_config.r_il << r_il_vec[0], r_il_vec[1], r_il_vec[2], r_il_vec[3], r_il_vec[4], r_il_vec[5], r_il_vec[6], r_il_vec[7], r_il_vec[8];
        Eigen::JacobiSVD<M3D> extrinsic_svd(m_builder_config.r_il, Eigen::ComputeFullU | Eigen::ComputeFullV);
        M3D extrinsic_u = extrinsic_svd.matrixU();
        const M3D extrinsic_v = extrinsic_svd.matrixV();
        if ((extrinsic_u * extrinsic_v.transpose()).determinant() < 0.0)
            extrinsic_u.col(2) *= -1.0;
        const M3D projected_r_il = extrinsic_u * extrinsic_v.transpose();
        const double extrinsic_projection_error = (projected_r_il - m_builder_config.r_il).norm();
        if (extrinsic_projection_error > 0.05)
            throw std::invalid_argument("Configured r_il is too far from a valid rotation matrix");
        if (extrinsic_projection_error > 1e-10)
            RCLCPP_WARN(this->get_logger(), "Projected rounded r_il onto SO(3), correction norm: %.3e",
                        extrinsic_projection_error);
        m_builder_config.r_il = projected_r_il;
        m_builder_config.lidar_cov_inv = config["lidar_cov_inv"].as<double>();
        auto read_optional = [&config](const char *key, auto default_value) {
            return config[key] ? config[key].as<decltype(default_value)>() : default_value;
        };
        m_builder_config.enable_adaptive_observability =
            read_optional("enable_adaptive_observability", false);
        m_builder_config.enable_structure_subspace =
            read_optional("enable_structure_subspace", false);
        m_builder_config.structure_plane_min_quality =
            read_optional("structure_plane_min_quality", 0.15);
        m_builder_config.structure_line_min_quality =
            read_optional("structure_line_min_quality", 0.20);
        m_builder_config.structure_line_min_weak_gain =
            read_optional("structure_line_min_weak_gain", 0.05);
        m_builder_config.structure_line_max_neighborhoods =
            read_optional("structure_line_max_neighborhoods", 64);
        m_builder_config.structure_line_budget_scale =
            read_optional("structure_line_budget_scale", 1.0);
        m_builder_config.structure_line_min_effective_score =
            read_optional("structure_line_min_effective_score", 0.02);
        m_builder_config.srda_min_reliability = read_optional("srda_min_reliability", 0.05);
        m_builder_config.srda_inlier_threshold = read_optional("srda_inlier_threshold", 0.1);
        m_builder_config.srda_robust_iterations = read_optional("srda_robust_iterations", 5);
        m_builder_config.srda_enable_soft_gate = read_optional("srda_enable_soft_gate", true);
        m_builder_config.srda_gate_degeneracy_threshold =
            read_optional("srda_gate_degeneracy_threshold", 0.55);
        m_builder_config.srda_gate_sharpness = read_optional("srda_gate_sharpness", 12.0);
        m_builder_config.ipmm_voxel_size = read_optional("ipmm_voxel_size", 0.6);
        m_builder_config.ipmm_point_budget = read_optional("ipmm_point_budget", 500);
        m_builder_config.ipmm_information_regularization = read_optional("ipmm_information_regularization", 1e-3);
        m_builder_config.ipmm_characteristic_length = read_optional("ipmm_characteristic_length", 10.0);
        m_builder_config.ipmm_expansion_ratio = read_optional("ipmm_expansion_ratio", 0.2);
        m_builder_config.ipmm_reliability_weight = read_optional("ipmm_reliability_weight", 0.25);
        m_builder_config.ipmm_novelty_weight = read_optional("ipmm_novelty_weight", 0.15);
        m_builder_config.ipmm_direction_quota_ratio =
            read_optional("ipmm_direction_quota_ratio", 0.0);
        if (m_builder_config.srda_min_reliability <= 0.0 || m_builder_config.srda_min_reliability > 1.0 ||
            m_builder_config.srda_inlier_threshold <= 0.0 || m_builder_config.srda_robust_iterations < 1 ||
            m_builder_config.srda_gate_degeneracy_threshold < 0.0 ||
            m_builder_config.srda_gate_degeneracy_threshold > 1.0 ||
            m_builder_config.srda_gate_sharpness <= 0.0 ||
            m_builder_config.structure_plane_min_quality < 0.0 ||
            m_builder_config.structure_plane_min_quality > 1.0 ||
            m_builder_config.structure_line_min_quality < 0.0 ||
            m_builder_config.structure_line_min_quality > 1.0 ||
            m_builder_config.structure_line_min_weak_gain < 0.0 ||
            m_builder_config.structure_line_min_weak_gain > 1.0 ||
            m_builder_config.structure_line_max_neighborhoods < 0 ||
            m_builder_config.structure_line_budget_scale < 0.0 ||
            m_builder_config.structure_line_min_effective_score < 0.0 ||
            m_builder_config.structure_line_min_effective_score > 1.0 ||
            m_builder_config.ipmm_voxel_size <= 0.0 || m_builder_config.ipmm_point_budget < 0 ||
            m_builder_config.ipmm_information_regularization <= 0.0 ||
            m_builder_config.ipmm_characteristic_length <= 0.0 ||
            m_builder_config.ipmm_expansion_ratio < 0.0 || m_builder_config.ipmm_expansion_ratio > 1.0 ||
            m_builder_config.ipmm_reliability_weight < 0.0 || m_builder_config.ipmm_novelty_weight < 0.0 ||
            m_builder_config.ipmm_direction_quota_ratio < 0.0 ||
            m_builder_config.ipmm_direction_quota_ratio > 1.0 / 3.0)
            throw std::invalid_argument("Invalid SRM-LIO configuration");
    }

    void initializeExperimentLog()
    {
        if (m_node_config.experiment_dir.empty())
            return;
        std::filesystem::create_directories(m_node_config.experiment_dir);
        std::ofstream config_used(std::filesystem::path(m_node_config.experiment_dir) / "config_used.yaml");
        if (!config_used)
            throw std::runtime_error("Cannot save effective configuration");
        config_used << m_effective_config << '\n';
        m_metrics_file.open(std::filesystem::path(m_node_config.experiment_dir) / "frame_metrics.csv");
        m_trajectory_file.open(std::filesystem::path(m_node_config.experiment_dir) / "trajectory_tum.txt");
        m_diagnostics_file.open(std::filesystem::path(m_node_config.experiment_dir) / "diagnostics.log");
        if (!m_metrics_file || !m_trajectory_file || !m_diagnostics_file)
            throw std::runtime_error("Cannot create experiment output files");
        m_metrics_file << "timestamp,mode,total_ms,update_ms,map_insert_ms,effective_observations,"
                          "mean_reliability,map_points,insertion_candidates,inserted_points,rss_mb,"
                          "cov_min_eigenvalue,cov_max_eigenvalue,geometry_degeneracy,srda_gate,srda_fit_benefit,"
                          "subspace_observations,mean_subspace_confidence,line_candidates,"
                          "selected_lines,mean_line_weak_gain\n";
        m_diagnostics_file << "mode=" << "srm_lio"
                           << "\nconfig_path=" << m_node_config.config_path << "\n";
    }

    void saveFinalMap() noexcept
    {
        if (m_map_saved || m_node_config.experiment_dir.empty() || !m_builder)
            return;
        m_map_saved = true;
        try
        {
            PointVec points = m_builder->lidar_processor()->exportGlobalMap();
            if (points.empty())
            {
                RCLCPP_WARN(this->get_logger(), "Final map is empty; map.pcd was not written");
                if (m_diagnostics_file)
                    m_diagnostics_file << "MAP_SAVE_SKIPPED empty_map\n";
                return;
            }

            CloudType cloud;
            cloud.points = std::move(points);
            cloud.width = static_cast<uint32_t>(cloud.points.size());
            cloud.height = 1;
            cloud.is_dense = true;
            const std::filesystem::path output_path =
                std::filesystem::path(m_node_config.experiment_dir) / "map.pcd";
            if (pcl::io::savePCDFileBinary(output_path.string(), cloud) != 0)
                throw std::runtime_error("PCL failed to write " + output_path.string());

            std::ofstream info(std::filesystem::path(m_node_config.experiment_dir) / "map_info.txt");
            if (!info)
                throw std::runtime_error("Cannot create map_info.txt");
            info << "frame_id=" << m_node_config.world_frame << '\n'
                 << "points=" << cloud.points.size() << '\n'
                 << "format=binary_pcd\n"
                 << "scope=archived_and_active_ikdtree_points\n";
            info.close();
            if (m_diagnostics_file)
            {
                m_diagnostics_file << "MAP_SAVED path=" << output_path.string()
                                   << " points=" << cloud.points.size() << '\n';
                m_diagnostics_file.flush();
            }
            RCLCPP_INFO(this->get_logger(), "Saved final global map: %s (%zu points)",
                        output_path.c_str(), cloud.points.size());
        }
        catch (const std::exception &error)
        {
            if (m_diagnostics_file)
            {
                m_diagnostics_file << "MAP_SAVE_FAILED " << error.what() << '\n';
                m_diagnostics_file.flush();
            }
            RCLCPP_ERROR(this->get_logger(), "Failed to save final map: %s", error.what());
        }
    }

    void recordExperiment(double timestamp, double total_ms)
    {
        if (!m_metrics_file)
            return;
        const auto &metrics = m_builder->lidar_processor()->metrics();
        const char *mode = "srm_lio";
        long resident_pages = 0;
        long ignored_pages = 0;
        std::ifstream statm("/proc/self/statm");
        statm >> ignored_pages >> resident_pages;
        const double rss_mb = static_cast<double>(resident_pages) * static_cast<double>(sysconf(_SC_PAGESIZE)) /
                              (1024.0 * 1024.0);
        Eigen::SelfAdjointEigenSolver<M21D> covariance_solver(0.5 * (m_kf->P() + m_kf->P().transpose()));
        if (covariance_solver.info() != Eigen::Success)
            throw std::runtime_error("Cannot compute covariance eigenvalues for diagnostics");
        m_metrics_file << std::fixed << std::setprecision(9) << timestamp << ',' << mode << ',' << total_ms << ','
                       << metrics.update_ms << ',' << metrics.map_insert_ms << ',' << metrics.effective_observations << ','
                       << metrics.mean_reliability << ',' << metrics.map_points << ',' << metrics.insertion_candidates << ','
                       << metrics.inserted_points << ',' << rss_mb << ',' << covariance_solver.eigenvalues().minCoeff()
                       << ',' << covariance_solver.eigenvalues().maxCoeff() << ','
                       << metrics.geometry_degeneracy << ',' << metrics.srda_gate << ','
                       << metrics.srda_fit_benefit << ',' << metrics.subspace_observations << ','
                       << metrics.mean_subspace_confidence << ',' << metrics.line_candidates << ','
                       << metrics.selected_lines << ',' << metrics.mean_line_weak_gain << '\n';
        Eigen::Quaterniond q(m_kf->x().r_wi);
        m_trajectory_file << std::fixed << std::setprecision(9) << timestamp << ' ' << m_kf->x().t_wi.transpose() << ' '
                          << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w() << '\n';
        m_metrics_file.flush();
        m_trajectory_file.flush();
    }

    void imuCB(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(m_state_data.imu_mutex);
        double timestamp = Utils::getSec(msg->header);
        if (timestamp < m_state_data.last_imu_time)
        {
            RCLCPP_WARN(this->get_logger(), "IMU Message is out of order");
            std::deque<IMUData>().swap(m_state_data.imu_buffer);
        }
        m_state_data.imu_buffer.emplace_back(V3D(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z) * m_node_config.imu_acc_scale,
                                             V3D(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z),
                                             timestamp);
        m_state_data.last_imu_time = timestamp;
    }
#if SRM_HAS_LIVOX
    void lidarCB(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
    {
        CloudType::Ptr cloud = Utils::livox2PCL(msg, m_builder_config.lidar_filter_num, m_builder_config.lidar_min_range, m_builder_config.lidar_max_range);
        std::lock_guard<std::mutex> lock(m_state_data.lidar_mutex);
        double timestamp = Utils::getSec(msg->header);
        if (timestamp < m_state_data.last_lidar_time)
        {
            RCLCPP_WARN(this->get_logger(), "Lidar Message is out of order");
            std::deque<std::pair<double, pcl::PointCloud<pcl::PointXYZINormal>::Ptr>>().swap(m_state_data.lidar_buffer);
        }
        m_state_data.lidar_buffer.emplace_back(timestamp, cloud);
        m_state_data.last_lidar_time = timestamp;
    }
#endif

    void pointCloudCB(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        bool has_time = false;
        for (const auto &field : msg->fields)
            has_time = has_time || field.name == "time";
        if (!has_time)
            throw std::runtime_error("PointCloud2 input has no per-point 'time' field required for undistortion");

        sensor_msgs::PointCloud2ConstIterator<float> iter_time(*msg, "time");
        double min_time = std::numeric_limits<double>::infinity();
        double max_time = -std::numeric_limits<double>::infinity();
        for (; iter_time != iter_time.end(); ++iter_time)
        {
            if (std::isfinite(*iter_time))
            {
                min_time = std::min(min_time, static_cast<double>(*iter_time));
                max_time = std::max(max_time, static_cast<double>(*iter_time));
            }
        }
        if (!std::isfinite(min_time) || !std::isfinite(max_time) || max_time - min_time > 1.0)
            throw std::runtime_error("PointCloud2 per-point time field is invalid");

        CloudType::Ptr cloud(new CloudType);
        cloud->reserve(msg->width * msg->height / std::max(1, m_builder_config.lidar_filter_num) + 1);
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
        sensor_msgs::PointCloud2ConstIterator<float> iter_point_time(*msg, "time");
        size_t index = 0;
        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_point_time, ++index)
        {
            if (index % static_cast<size_t>(std::max(1, m_builder_config.lidar_filter_num)) != 0)
                continue;
            const double range_squared = (*iter_x) * (*iter_x) + (*iter_y) * (*iter_y) + (*iter_z) * (*iter_z);
            if (!std::isfinite(range_squared) || range_squared < m_builder_config.lidar_min_range * m_builder_config.lidar_min_range ||
                range_squared > m_builder_config.lidar_max_range * m_builder_config.lidar_max_range || !std::isfinite(*iter_point_time))
                continue;
            PointType point;
            point.x = *iter_x;
            point.y = *iter_y;
            point.z = *iter_z;
            point.intensity = 0.0f;
            point.curvature = static_cast<float>((static_cast<double>(*iter_point_time) - min_time) * 1000.0);
            cloud->push_back(point);
        }
        if (cloud->empty())
            throw std::runtime_error("PointCloud2 conversion produced an empty cloud");

        std::lock_guard<std::mutex> lock(m_state_data.lidar_mutex);
        const double timestamp = Utils::getSec(msg->header) + min_time;
        if (timestamp < m_state_data.last_lidar_time)
        {
            RCLCPP_WARN(this->get_logger(), "PointCloud2 message is out of order");
            std::deque<std::pair<double, CloudType::Ptr>>().swap(m_state_data.lidar_buffer);
        }
        m_state_data.lidar_buffer.emplace_back(timestamp, cloud);
        m_state_data.last_lidar_time = timestamp;
    }

    bool syncPackage()
    {
        if (m_state_data.imu_buffer.empty() || m_state_data.lidar_buffer.empty())
            return false;
        if (!m_state_data.lidar_pushed)
        {
            m_package.cloud = m_state_data.lidar_buffer.front().second;
            std::sort(m_package.cloud->points.begin(), m_package.cloud->points.end(), [](PointType &p1, PointType &p2)
                      { return p1.curvature < p2.curvature; });
            m_package.cloud_start_time = m_state_data.lidar_buffer.front().first;
            m_package.cloud_end_time = m_package.cloud_start_time + m_package.cloud->points.back().curvature / 1000.0;
            m_state_data.lidar_pushed = true;
        }
        if (m_state_data.last_imu_time < m_package.cloud_end_time)
            return false;

        Vec<IMUData>().swap(m_package.imus);
        while (!m_state_data.imu_buffer.empty() && m_state_data.imu_buffer.front().time < m_package.cloud_end_time)
        {
            m_package.imus.emplace_back(m_state_data.imu_buffer.front());
            m_state_data.imu_buffer.pop_front();
        }
        m_state_data.lidar_buffer.pop_front();
        m_state_data.lidar_pushed = false;
        return true;
    }

    void publishCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub, CloudType::Ptr cloud, std::string frame_id, const double &time)
    {
        if (pub->get_subscription_count() <= 0)
            return;
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud, cloud_msg);
        cloud_msg.header.frame_id = frame_id;
        cloud_msg.header.stamp = Utils::getTime(time);
        pub->publish(cloud_msg);
    }

    void publishOdometry(rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub, std::string frame_id, std::string child_frame, const double &time)
    {
        if (odom_pub->get_subscription_count() <= 0)
            return;
        nav_msgs::msg::Odometry odom;
        odom.header.frame_id = frame_id;
        odom.header.stamp = Utils::getTime(time);
        odom.child_frame_id = child_frame;
        odom.pose.pose.position.x = m_kf->x().t_wi.x();
        odom.pose.pose.position.y = m_kf->x().t_wi.y();
        odom.pose.pose.position.z = m_kf->x().t_wi.z();
        Eigen::Quaterniond q(m_kf->x().r_wi);
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();

        V3D vel = m_kf->x().r_wi.transpose() * m_kf->x().v;
        odom.twist.twist.linear.x = vel.x();
        odom.twist.twist.linear.y = vel.y();
        odom.twist.twist.linear.z = vel.z();
        odom_pub->publish(odom);
    }

    void publishPath(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub, std::string frame_id, const double &time)
    {
        if (path_pub->get_subscription_count() <= 0)
            return;
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = frame_id;
        pose.header.stamp = Utils::getTime(time);
        pose.pose.position.x = m_kf->x().t_wi.x();
        pose.pose.position.y = m_kf->x().t_wi.y();
        pose.pose.position.z = m_kf->x().t_wi.z();
        Eigen::Quaterniond q(m_kf->x().r_wi);
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();
        m_state_data.path.poses.push_back(pose);
        path_pub->publish(m_state_data.path);
    }

    void broadCastTF(std::shared_ptr<tf2_ros::TransformBroadcaster> broad_caster, std::string frame_id, std::string child_frame, const double &time)
    {
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.frame_id = frame_id;
        transformStamped.child_frame_id = child_frame;
        transformStamped.header.stamp = Utils::getTime(time);
        Eigen::Quaterniond q(m_kf->x().r_wi);
        V3D t = m_kf->x().t_wi;
        transformStamped.transform.translation.x = t.x();
        transformStamped.transform.translation.y = t.y();
        transformStamped.transform.translation.z = t.z();
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();
        broad_caster->sendTransform(transformStamped);
    }

    void timerCB()
    {
        if (!syncPackage())
            return;
        auto t1 = std::chrono::high_resolution_clock::now();
        try
        {
            m_builder->process(m_package);
        }
        catch (const std::exception &error)
        {
            if (m_diagnostics_file)
            {
                m_diagnostics_file << std::fixed << std::setprecision(9) << m_package.cloud_end_time
                                   << " FATAL " << error.what() << '\n';
                m_diagnostics_file.flush();
            }
            RCLCPP_FATAL(this->get_logger(), "SRM-LIO fatal diagnostic: %s", error.what());
            saveFinalMap();
            throw;
        }
        auto t2 = std::chrono::high_resolution_clock::now();
        const double time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000;

        if (m_node_config.print_time_cost)
        {
            RCLCPP_WARN(this->get_logger(), "Time cost: %.2f ms", time_used);
        }

        if (m_builder->status() != BuilderStatus::MAPPING)
            return;
        recordExperiment(m_package.cloud_end_time, time_used);

        broadCastTF(m_tf_broadcaster, m_node_config.world_frame, m_node_config.body_frame, m_package.cloud_end_time);

        publishOdometry(m_odom_pub, m_node_config.world_frame, m_node_config.body_frame, m_package.cloud_end_time);

        CloudType::Ptr body_cloud = m_builder->lidar_processor()->transformCloud(m_package.cloud, m_kf->x().r_il, m_kf->x().t_il);

        publishCloud(m_body_cloud_pub, body_cloud, m_node_config.body_frame, m_package.cloud_end_time);

        CloudType::Ptr world_cloud = m_builder->lidar_processor()->transformCloud(m_package.cloud, m_builder->lidar_processor()->r_wl(), m_builder->lidar_processor()->t_wl());

        publishCloud(m_world_cloud_pub, world_cloud, m_node_config.world_frame, m_package.cloud_end_time);

        publishPath(m_path_pub, m_node_config.world_frame, m_package.cloud_end_time);
    }

private:
#if SRM_HAS_LIVOX
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr m_lidar_sub;
#endif
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_pointcloud_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imu_sub;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_body_cloud_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_world_cloud_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_path_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_odom_pub;

    rclcpp::TimerBase::SharedPtr m_timer;
    StateData m_state_data;
    SyncPackage m_package;
    NodeConfig m_node_config;
    std::string m_effective_config;
    Config m_builder_config;
    std::shared_ptr<IESKF> m_kf;
    std::shared_ptr<MapBuilder> m_builder;
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
    std::ofstream m_metrics_file;
    std::ofstream m_trajectory_file;
    std::ofstream m_diagnostics_file;
    bool m_map_saved = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LIONode>());
    rclcpp::shutdown();
    return 0;
}
