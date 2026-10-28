#pragma once
#include "commons.h"
#include "ieskf.h"
#include "ikd_Tree.h"
#include "structure_reliability.h"
#include <cstdint>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>

struct LocalMap
{
    bool initialed = false;
    BoxPointType local_map_corner;
    Vec<BoxPointType> cub_to_rm;
};

struct LidarFrameMetrics
{
    size_t effective_observations = 0;
    size_t map_points = 0;
    size_t insertion_candidates = 0;
    size_t inserted_points = 0;
    double mean_reliability = 0.0;
    double geometry_degeneracy = 0.0;
    double srda_gate = 0.0;
    double srda_fit_benefit = 0.0;
    size_t subspace_observations = 0;
    double mean_subspace_confidence = 0.0;
    size_t line_candidates = 0;
    size_t selected_lines = 0;
    double mean_line_weak_gain = 0.0;
    double update_ms = 0.0;
    double map_insert_ms = 0.0;
};

class LidarProcessor
{
public:
    LidarProcessor(Config &config, std::shared_ptr<IESKF> kf);

    void trimCloudMap();

    void incrCloudMap();

    void initCloudMap(PointVec &point_vec);

    void process(SyncPackage &package);

    void updateLossFunc(State &state, SharedState &share_data);

    static CloudType::Ptr transformCloud(CloudType::Ptr inp, const M3D &r, const V3D &t);
    M3D r_wl() { return m_kf->x().r_wi * m_kf->x().r_il; }
    V3D t_wl() { return m_kf->x().t_wi + m_kf->x().r_wi * m_kf->x().t_il; }
    const LidarFrameMetrics &metrics() const { return m_metrics; }

    // Return the complete map built over the trajectory: points that left the
    // sliding local cube plus points that are still active in the ikd-Tree.
    PointVec exportGlobalMap();

private:
    Config m_config;
    LocalMap m_local_map;
    std::shared_ptr<IESKF> m_kf;
    std::shared_ptr<KD_TREE<PointType>> m_ikdtree;
    PointVec m_archived_map_points;
    CloudType::Ptr m_cloud_lidar;
    CloudType::Ptr m_cloud_down_lidar;
    CloudType::Ptr m_cloud_down_world;
    // Do not use std::vector<bool> here: its packed-bit proxy makes writes to
    // distinct indices race when the correspondence loop runs under OpenMP.
    std::vector<uint8_t> m_point_selected_flag;
    CloudType::Ptr m_norm_vec;
    CloudType::Ptr m_effect_cloud_lidar;
    CloudType::Ptr m_effect_norm_vec;
    std::vector<PointVec> m_nearest_points;
    std::vector<double> m_structure_reliability;
    std::vector<V3D, Eigen::aligned_allocator<V3D>> m_structure_normals;
    StructureReliability m_reliability_evaluator;
    LidarFrameMetrics m_metrics;
    pcl::VoxelGrid<PointType> m_scan_filter;
};
