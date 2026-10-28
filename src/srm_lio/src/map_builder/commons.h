#pragma once
#include <Eigen/Eigen>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

using PointType = pcl::PointXYZINormal;
using CloudType = pcl::PointCloud<PointType>;
using PointVec = std::vector<PointType, Eigen::aligned_allocator<PointType>>;

using M3D = Eigen::Matrix3d;
using V3D = Eigen::Vector3d;
using M3F = Eigen::Matrix3f;
using V3F = Eigen::Vector3f;
using M2D = Eigen::Matrix2d;
using V2D = Eigen::Vector2d;
using M2F = Eigen::Matrix2f;
using V2F = Eigen::Vector2f;
using M4D = Eigen::Matrix4d;
using V4D = Eigen::Vector4d;
using M6D = Eigen::Matrix<double, 6, 6>;
using V6D = Eigen::Matrix<double, 6, 1>;


template <typename T>
using Vec = std::vector<T>;


bool esti_plane(PointVec &points, const double &thresh, V4D &out);

float sq_dist(const PointType &p1, const PointType &p2);

struct Config
{
    int lidar_filter_num = 3;
    double lidar_min_range = 0.5;
    double lidar_max_range = 20.0;
    double scan_resolution = 0.15;
    double map_resolution = 0.3;

    double cube_len = 300;
    double det_range = 60;
    double move_thresh = 1.5;

    double na = 0.01;
    double ng = 0.01;
    double nba = 0.0001;
    double nbg = 0.0001;
    int imu_init_num = 20;
    bool normalize_imu_acc = false;
    int near_search_num = 5;
    int ieskf_max_iter = 5;
    bool gravity_align = true;
    bool esti_il = false;
    M3D r_il = M3D::Identity();
    V3D t_il = V3D::Zero();

    double lidar_cov_inv = 1000.0;
    // Scale-invariant, directional observability update. The legacy SRDA/IPMM
    // path remains available by switching this flag off.
    bool enable_adaptive_observability = false;
    // Complement reliable planes with anisotropic point-to-line constraints
    // extracted from locally linear map neighborhoods.  This never creates a
    // full isotropic point-to-point residual.
    bool enable_structure_subspace = false;
    double structure_plane_min_quality = 0.15;
    double structure_line_min_quality = 0.20;
    double structure_line_min_weak_gain = 0.05;
    // Local line fits from neighbouring scan points are strongly correlated.
    // Keep only the highest-scoring neighbourhoods under a sublinear budget
    // instead of treating every fit as independent evidence.
    int structure_line_max_neighborhoods = 64;
    double structure_line_budget_scale = 1.0;
    double structure_line_min_effective_score = 0.02;
    double srda_min_reliability = 0.05;
    double srda_inlier_threshold = 0.1;
    int srda_robust_iterations = 5;
    bool srda_enable_soft_gate = true;
    double srda_gate_degeneracy_threshold = 0.55;
    double srda_gate_sharpness = 12.0;
    double ipmm_voxel_size = 0.6;
    int ipmm_point_budget = 500;
    double ipmm_information_regularization = 1e-3;
    double ipmm_characteristic_length = 10.0;
    double ipmm_expansion_ratio = 0.2;
    double ipmm_reliability_weight = 0.25;
    double ipmm_novelty_weight = 0.15;
    double ipmm_direction_quota_ratio = 0.0;
};

struct IMUData
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    V3D acc;
    V3D gyro;
    double time;
    IMUData() = default;
    IMUData(const V3D &a, const V3D &g, double &t) : acc(a), gyro(g), time(t) {}
};

struct Pose
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double offset;
    V3D acc;
    V3D gyro;
    V3D vel;
    V3D trans;
    M3D rot;
    Pose() = default;
    Pose(double t, const V3D &a, const V3D &g, const V3D &v, const V3D &p, const M3D &r) : offset(t), acc(a), gyro(g), vel(v), trans(p), rot(r) {}
};

struct SyncPackage
{
    Vec<IMUData> imus;
    CloudType::Ptr cloud;
    double cloud_start_time = 0.0;
    double cloud_end_time = 0.0;
};
