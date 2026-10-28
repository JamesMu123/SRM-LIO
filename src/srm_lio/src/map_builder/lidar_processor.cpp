#include "lidar_processor.h"
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <chrono>
#ifdef MP_EN
#include <omp.h>
#endif

LidarProcessor::LidarProcessor(Config &config, std::shared_ptr<IESKF> kf)
    : m_config(config), m_kf(kf),
      m_reliability_evaluator(StructureReliability::Parameters{config.srda_inlier_threshold,
                                                               config.srda_min_reliability, 0.01,
                                                               config.srda_robust_iterations})
{
    m_ikdtree = std::make_shared<KD_TREE<PointType>>();
    m_ikdtree->set_downsample_param(m_config.map_resolution);
    m_cloud_down_lidar.reset(new CloudType);
    m_cloud_down_world.reset(new CloudType(10000, 1));
    m_norm_vec.reset(new CloudType(10000, 1));
    m_effect_cloud_lidar.reset(new CloudType(10000, 1));
    m_effect_norm_vec.reset(new CloudType(10000, 1));
    m_nearest_points.resize(10000);
    m_point_selected_flag.resize(10000, false);
    m_structure_reliability.resize(10000, 1.0);
    m_structure_normals.resize(10000, V3D::Zero());

    if (m_config.scan_resolution > 0.0)
    {
        m_scan_filter.setLeafSize(m_config.scan_resolution, m_config.scan_resolution, m_config.scan_resolution);
    }

    m_kf->setLossFunction([&](State &s, SharedState &d)
                          { updateLossFunc(s, d); });
    m_kf->setStopFunction([&](const V21D &delta) -> bool
                          { V3D rot_delta = delta.block<3, 1>(0, 0);
                            V3D t_delta = delta.block<3, 1>(3, 0);
                            return (rot_delta.norm() * 57.3 < 0.01) && (t_delta.norm() * 100 < 0.015); });
}

void LidarProcessor::trimCloudMap()
{
    m_local_map.cub_to_rm.clear();
    const State &state = m_kf->x();
    Eigen::Vector3d pos_lidar = state.t_wi + state.r_wi * state.t_il;

    if (!m_local_map.initialed)
    {
        for (int i = 0; i < 3; i++)
        {
            m_local_map.local_map_corner.vertex_min[i] = pos_lidar[i] - m_config.cube_len / 2.0;
            m_local_map.local_map_corner.vertex_max[i] = pos_lidar[i] + m_config.cube_len / 2.0;
        }
        m_local_map.initialed = true;
        return;
    }
    float dist_to_map_edge[3][2];
    bool need_move = false;
    double det_thresh = m_config.move_thresh * m_config.det_range;
    for (int i = 0; i < 3; i++)
    {
        dist_to_map_edge[i][0] = fabs(pos_lidar(i) - m_local_map.local_map_corner.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_lidar(i) - m_local_map.local_map_corner.vertex_max[i]);

        if (dist_to_map_edge[i][0] <= det_thresh || dist_to_map_edge[i][1] <= det_thresh)
            need_move = true;
    }
    if (!need_move)
        return;
    BoxPointType new_corner, temp_corner;
    new_corner = m_local_map.local_map_corner;
    float mov_dist = std::max((m_config.cube_len - 2.0 * m_config.move_thresh * m_config.det_range) * 0.5 * 0.9, double(m_config.det_range * (m_config.move_thresh - 1)));

    for (int i = 0; i < 3; i++)
    {
        temp_corner = m_local_map.local_map_corner;
        if (dist_to_map_edge[i][0] <= det_thresh)
        {
            new_corner.vertex_max[i] -= mov_dist;
            new_corner.vertex_min[i] -= mov_dist;
            temp_corner.vertex_min[i] = m_local_map.local_map_corner.vertex_max[i] - mov_dist;
            m_local_map.cub_to_rm.push_back(temp_corner);
        }
        else if (dist_to_map_edge[i][1] <= det_thresh)
        {
            new_corner.vertex_max[i] += mov_dist;
            new_corner.vertex_min[i] += mov_dist;
            temp_corner.vertex_max[i] = m_local_map.local_map_corner.vertex_min[i] + mov_dist;
            m_local_map.cub_to_rm.push_back(temp_corner);
        }
    }
    m_local_map.local_map_corner = new_corner;

    PointVec points_history;
    m_ikdtree->acquire_removed_points(points_history);
    m_archived_map_points.insert(m_archived_map_points.end(), points_history.begin(), points_history.end());

    // 删除局部地图之外的点云
    if (m_local_map.cub_to_rm.size() > 0)
        m_ikdtree->Delete_Point_Boxes(m_local_map.cub_to_rm);
    return;
}

PointVec LidarProcessor::exportGlobalMap()
{
    // Collect removals produced by the most recent local-cube shift, which
    // have not yet been acquired by the next trimCloudMap() call.
    PointVec pending_removed;
    m_ikdtree->acquire_removed_points(pending_removed);
    m_archived_map_points.insert(m_archived_map_points.end(), pending_removed.begin(), pending_removed.end());

    PointVec active_points;
    if (m_ikdtree->Root_Node != nullptr)
    {
        BoxPointType range = m_ikdtree->tree_range();
        constexpr float margin = 1.0f;
        for (int axis = 0; axis < 3; ++axis)
        {
            range.vertex_min[axis] -= margin;
            range.vertex_max[axis] += margin;
        }
        m_ikdtree->Box_Search(range, active_points);
    }

    PointVec complete_map;
    complete_map.reserve(m_archived_map_points.size() + active_points.size());
    complete_map.insert(complete_map.end(), m_archived_map_points.begin(), m_archived_map_points.end());
    complete_map.insert(complete_map.end(), active_points.begin(), active_points.end());
    return complete_map;
}

void LidarProcessor::incrCloudMap()
{
    if (m_cloud_down_lidar->empty())
        return;
    const State &state = m_kf->x();
    int size = m_cloud_down_lidar->size();
    PointVec point_to_add;
    PointVec point_no_need_downsample;
    std::vector<int> point_to_add_indices;
    std::vector<int> point_no_need_downsample_indices;
    for (int i = 0; i < size; i++)
    {
        const PointType &p = m_cloud_down_lidar->points[i];
        Eigen::Vector3d point(p.x, p.y, p.z);
        point = state.r_wi * (state.r_il * point + state.t_il) + state.t_wi;
        m_cloud_down_world->points[i].x = point(0);
        m_cloud_down_world->points[i].y = point(1);
        m_cloud_down_world->points[i].z = point(2);
        m_cloud_down_world->points[i].intensity = m_cloud_down_lidar->points[i].intensity;
        // 如果该点附近没有近邻点则需要添加到地图中
        if (m_nearest_points[i].empty())
        {
            point_to_add.push_back(m_cloud_down_world->points[i]);
            point_to_add_indices.push_back(i);
            continue;
        }

        const PointVec &points_near = m_nearest_points[i];
        bool need_add = true;
        PointType downsample_result, mid_point;
        mid_point.x = std::floor(m_cloud_down_world->points[i].x / m_config.map_resolution) * m_config.map_resolution + 0.5 * m_config.map_resolution;
        mid_point.y = std::floor(m_cloud_down_world->points[i].y / m_config.map_resolution) * m_config.map_resolution + 0.5 * m_config.map_resolution;
        mid_point.z = std::floor(m_cloud_down_world->points[i].z / m_config.map_resolution) * m_config.map_resolution + 0.5 * m_config.map_resolution;

        // 如果该点所在的voxel没有点，则直接加入地图，且不需要降采样
        if (fabs(points_near[0].x - mid_point.x) > 0.5 * m_config.map_resolution && fabs(points_near[0].y - mid_point.y) > 0.5 * m_config.map_resolution && fabs(points_near[0].z - mid_point.z) > 0.5 * m_config.map_resolution)
        {
            point_no_need_downsample.push_back(m_cloud_down_world->points[i]);
            point_no_need_downsample_indices.push_back(i);
            continue;
        }
        float dist = sq_dist(m_cloud_down_world->points[i], mid_point);

        for (int readd_i = 0; readd_i < m_config.near_search_num; readd_i++)
        {
            // 如果该点的近邻点较少，则需要加入到地图中
            if (points_near.size() < static_cast<size_t>(m_config.near_search_num))
                break;
            // 如果该点的近邻点距离voxel中心点的距离比该点距离voxel中心点更近，则不需要加入该点
            if (sq_dist(points_near[readd_i], mid_point) < dist)
            {
                need_add = false;
                break;
            }
        }
        if (need_add)
        {
            point_to_add.push_back(m_cloud_down_world->points[i]);
            point_to_add_indices.push_back(i);
        }
    }
    struct Candidate
    {
        PointType point;
        Eigen::Matrix<double, 6, 1> h = Eigen::Matrix<double, 6, 1>::Zero();
        double reliability = 0.0;
        double novelty = 0.0;
        double score = 0.0;
        int64_t voxel_x = 0;
        int64_t voxel_y = 0;
        int64_t voxel_z = 0;
        bool valid_structure = false;
        bool no_downsample = false;
    };
    struct VoxelKey
    {
        int64_t x, y, z;
        bool operator==(const VoxelKey &other) const { return x == other.x && y == other.y && z == other.z; }
    };
    struct VoxelHash
    {
        size_t operator()(const VoxelKey &key) const
        {
            size_t seed = std::hash<int64_t>{}(key.x);
            seed ^= std::hash<int64_t>{}(key.y) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            seed ^= std::hash<int64_t>{}(key.z) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            return seed;
        }
    };
    std::unordered_map<VoxelKey, Candidate, VoxelHash> voxel_candidates;
    const double voxel_size = std::max(1e-3, m_config.ipmm_voxel_size);
    auto add_candidate = [&](const PointType &point, int source_index, bool no_downsample) {
        const VoxelKey key{static_cast<int64_t>(std::floor(point.x / voxel_size)),
                           static_cast<int64_t>(std::floor(point.y / voxel_size)),
                           static_cast<int64_t>(std::floor(point.z / voxel_size))};
        Candidate candidate;
        candidate.point = point;
        candidate.no_downsample = no_downsample;
        candidate.voxel_x = key.x;
        candidate.voxel_y = key.y;
        candidate.voxel_z = key.z;
        V3D normal = V3D::Zero();
        candidate.reliability = m_structure_reliability[source_index];
        normal = m_structure_normals[source_index];
        candidate.valid_structure = normal.allFinite() && normal.norm() > 1e-9;
        const auto &neighbors = m_nearest_points[source_index];
        candidate.novelty = neighbors.empty()
                                ? 1.0
                                : std::min(1.0, std::sqrt(std::max(0.0f, sq_dist(point, neighbors.front()))) /
                                                    voxel_size);
        if (candidate.valid_structure)
        {
            normal.normalize();
            const V3D position_world(point.x, point.y, point.z);
            const V3D point_imu = state.r_wi.transpose() * (position_world - state.t_wi);
            const V3D normal_imu = state.r_wi.transpose() * normal;
            candidate.h.head<3>() = point_imu.cross(normal_imu) /
                                    m_config.ipmm_characteristic_length;
            candidate.h.tail<3>() = normal;
        }
        candidate.score = m_config.ipmm_reliability_weight * candidate.reliability +
                          m_config.ipmm_novelty_weight * candidate.novelty;
        auto existing = voxel_candidates.find(key);
        if (existing == voxel_candidates.end() || candidate.score > existing->second.score)
            voxel_candidates[key] = candidate;
    };
    for (size_t i = 0; i < point_to_add.size(); ++i)
        add_candidate(point_to_add[i], point_to_add_indices[i], false);
    for (size_t i = 0; i < point_no_need_downsample.size(); ++i)
        add_candidate(point_no_need_downsample[i], point_no_need_downsample_indices[i], true);

    std::vector<Candidate> structured;
    std::vector<Candidate> expansion;
    structured.reserve(voxel_candidates.size());
    expansion.reserve(voxel_candidates.size());
    for (const auto &entry : voxel_candidates)
        (entry.second.valid_structure ? structured : expansion).push_back(entry.second);
    auto stable_candidate_order = [](const Candidate &left, const Candidate &right) {
        if (left.score != right.score)
            return left.score > right.score;
        if (left.voxel_x != right.voxel_x)
            return left.voxel_x < right.voxel_x;
        if (left.voxel_y != right.voxel_y)
            return left.voxel_y < right.voxel_y;
        return left.voxel_z < right.voxel_z;
    };
    std::sort(structured.begin(), structured.end(), stable_candidate_order);
    std::sort(expansion.begin(), expansion.end(), stable_candidate_order);
    PointVec selected_downsample;
    PointVec selected_no_downsample;
    Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Identity() *
                                                    std::max(1e-12, m_config.ipmm_information_regularization);
    const size_t candidate_count = structured.size() + expansion.size();
    const size_t budget = std::min(candidate_count, static_cast<size_t>(std::max(0, m_config.ipmm_point_budget)));
    m_metrics.insertion_candidates = candidate_count;
    m_metrics.inserted_points = budget;
    auto insert_selected = [&](const Candidate &candidate) {
        (candidate.no_downsample ? selected_no_downsample : selected_downsample).push_back(candidate.point);
    };

    // Reserve a bounded part of the budget for genuinely new regions where no
    // plane can be estimated yet.  Such points preserve map growth but never
    // enter the Fisher matrix with an invented radial normal.
    const size_t desired_expansion = static_cast<size_t>(std::ceil(
        static_cast<double>(budget) * m_config.ipmm_expansion_ratio));
    const size_t expansion_budget = std::min(expansion.size(), desired_expansion);
    for (size_t i = 0; i < expansion_budget; ++i)
        insert_selected(expansion[i]);
    expansion.erase(expansion.begin(), expansion.begin() + expansion_budget);

    const size_t structured_budget = std::min(structured.size(), budget - expansion_budget);
    size_t selected_structured = 0;
    auto select_best = [&](int required_normal_axis) {
        const Eigen::Matrix<double, 6, 6> information_inverse = information.inverse();
        size_t best = 0;
        double best_gain = -1.0;
        for (size_t i = 0; i < structured.size(); ++i)
        {
            Eigen::Index dominant_axis = 0;
            structured[i].h.tail<3>().cwiseAbs().maxCoeff(&dominant_axis);
            if (required_normal_axis >= 0 && dominant_axis != required_normal_axis)
                continue;
            const double information_gain = std::log1p(structured[i].reliability *
                (structured[i].h.transpose() * information_inverse * structured[i].h)(0, 0));
            const double gain = information_gain +
                                m_config.ipmm_reliability_weight * structured[i].reliability +
                                m_config.ipmm_novelty_weight * structured[i].novelty;
            if (std::isfinite(gain) && gain > best_gain)
            {
                best_gain = gain;
                best = i;
            }
        }
        if (best_gain < 0.0)
            return false;
        Candidate chosen = structured[best];
        information.noalias() += chosen.reliability * chosen.h * chosen.h.transpose();
        insert_selected(chosen);
        structured.erase(structured.begin() + best);
        ++selected_structured;
        return true;
    };

    // Reserve a small part of the structured budget for each
    // unsigned principal normal direction. Reliability still controls quality
    // inside every bin; the quota only prevents one dominant plane family from
    // consuming the complete local-map budget needed by future SRDA updates.
    const size_t quota_per_axis = static_cast<size_t>(std::floor(
        static_cast<double>(structured_budget) * m_config.ipmm_direction_quota_ratio));
    for (int axis = 0; axis < 3; ++axis)
    {
        for (size_t count = 0; count < quota_per_axis && selected_structured < structured_budget; ++count)
        {
            if (!select_best(axis))
                break;
        }
    }


    while (selected_structured < structured_budget)
    {
        if (!select_best(-1))
            throw std::runtime_error("IPMM information gain became non-finite");
    }

    // If one class cannot consume its quota, deterministically fill the rest
    // from the other class so the fixed per-frame budget remains effective.
    size_t selected_count = selected_downsample.size() + selected_no_downsample.size();
    for (const Candidate &candidate : expansion)
    {
        if (selected_count >= budget)
            break;
        insert_selected(candidate);
        ++selected_count;
    }
    for (const Candidate &candidate : structured)
    {
        if (selected_count >= budget)
            break;
        insert_selected(candidate);
        ++selected_count;
    }
    Eigen::LLT<Eigen::Matrix<double, 6, 6>> information_llt(information);
    if (information_llt.info() != Eigen::Success)
        throw std::runtime_error("IPMM accumulated information matrix is not positive definite");
    m_ikdtree->Add_Points(selected_downsample, true);
    m_ikdtree->Add_Points(selected_no_downsample, false);
}

void LidarProcessor::initCloudMap(PointVec &point_vec)
{
    m_ikdtree->Build(point_vec);
}

void LidarProcessor::process(SyncPackage &package)
{
    if (m_config.scan_resolution > 0.0)
    {
        m_scan_filter.setInputCloud(package.cloud);
        m_scan_filter.filter(*m_cloud_down_lidar);
    }
    else
    {
        pcl::copyPointCloud(*package.cloud, *m_cloud_down_lidar);
    }
    const size_t required_size = m_cloud_down_lidar->size();
    if (m_cloud_down_world->size() < required_size)
    {
        m_cloud_down_world->resize(required_size);
        m_norm_vec->resize(required_size);
        m_effect_cloud_lidar->resize(required_size);
        m_effect_norm_vec->resize(required_size);
        m_nearest_points.resize(required_size);
        m_point_selected_flag.resize(required_size, false);
        m_structure_reliability.resize(required_size, 1.0);
        m_structure_normals.resize(required_size, V3D::Zero());
    }
    trimCloudMap();
    const auto update_start = std::chrono::steady_clock::now();
    m_kf->update();
    const auto insertion_start = std::chrono::steady_clock::now();
    incrCloudMap();
    const auto insertion_end = std::chrono::steady_clock::now();
    m_metrics.update_ms = std::chrono::duration<double, std::milli>(insertion_start - update_start).count();
    m_metrics.map_insert_ms = std::chrono::duration<double, std::milli>(insertion_end - insertion_start).count();
    m_metrics.map_points = static_cast<size_t>(std::max(0, m_ikdtree->validnum()));
}

void LidarProcessor::updateLossFunc(State &state, SharedState &share_data)
{
    int size = m_cloud_down_lidar->size();
    std::vector<V4D, Eigen::aligned_allocator<V4D>> baseline_planes(size, V4D::Zero());
    std::vector<V4D, Eigen::aligned_allocator<V4D>> robust_planes(size, V4D::Zero());
    std::vector<uint8_t> baseline_valid(size, 0);
    std::vector<uint8_t> robust_valid(size, 0);
    std::vector<double> fit_benefit(size, 0.0);
    std::vector<LineSubspaceResult, Eigen::aligned_allocator<LineSubspaceResult>> line_subspaces(size);
#ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
    for (int i = 0; i < size; i++)
    {
        m_structure_reliability[i] = m_config.srda_min_reliability;
        m_structure_normals[i].setZero();
        PointType &point_body = m_cloud_down_lidar->points[i];
        PointType &point_world = m_cloud_down_world->points[i];
        Eigen::Vector3d point_body_vec(point_body.x, point_body.y, point_body.z);
        Eigen::Vector3d point_world_vec = state.r_wi * (state.r_il * point_body_vec + state.t_il) + state.t_wi;
        point_world.x = point_world_vec(0);
        point_world.y = point_world_vec(1);
        point_world.z = point_world_vec(2);
        point_world.intensity = point_body.intensity;
        std::vector<float> point_sq_dist(m_config.near_search_num);
        auto &points_near = m_nearest_points[i];
        m_ikdtree->Nearest_Search(point_world, m_config.near_search_num, points_near, point_sq_dist);
        if (points_near.size() >= static_cast<size_t>(m_config.near_search_num) && point_sq_dist[m_config.near_search_num - 1] <= 5)
            m_point_selected_flag[i] = true;
        else
            m_point_selected_flag[i] = false;
        if (!m_point_selected_flag[i])
            continue;

        Eigen::Vector4d pabcd = Eigen::Vector4d::Zero();
        m_point_selected_flag[i] = false;
        double reliability = 1.0;
        bool plane_valid = false;
        if (m_config.enable_structure_subspace)
        {
            baseline_valid[i] = esti_plane(points_near, 0.1, baseline_planes[i]);
            const auto metrics = m_reliability_evaluator.evaluate(points_near);
            const auto geometry = assessLocalGeometry(
                points_near, metrics,
                m_config.structure_plane_min_quality,
                m_config.structure_line_min_quality,
                m_config.srda_inlier_threshold,
                m_config.srda_robust_iterations);
            line_subspaces[i] = geometry.line;
            if (geometry.model != LocalGeometryModel::Plane)
                continue;
            plane_valid = true;
            robust_valid[i] = true;
            pabcd = metrics.robust_plane;
            robust_planes[i] = metrics.robust_plane;
            reliability = geometry.plane_quality;
            m_structure_normals[i] = metrics.robust_plane.head<3>();
            m_structure_reliability[i] = reliability;
            fit_benefit[i] = baseline_valid[i]
                                 ? computePlaneFitBenefit(points_near, baseline_planes[i], robust_planes[i])
                                 : 1.0;
        }
        else
        {
            baseline_valid[i] = esti_plane(points_near, 0.1, baseline_planes[i]);
            const auto metrics = m_reliability_evaluator.evaluate(points_near);
            plane_valid = metrics.valid;
            robust_valid[i] = plane_valid;
            if (plane_valid)
            {
                pabcd = metrics.robust_plane;
                robust_planes[i] = metrics.robust_plane;
                reliability = metrics.reliability;
                m_structure_normals[i] = metrics.robust_plane.head<3>();
                if (baseline_valid[i])
                    fit_benefit[i] = computePlaneFitBenefit(points_near, baseline_planes[i], robust_planes[i]);
                else
                    fit_benefit[i] = 1.0;
            }
            else
                m_structure_normals[i].setZero();
            m_structure_reliability[i] = reliability;
        }
        if (plane_valid)
        {
            double pd2 = pabcd(0) * point_world_vec(0) + pabcd(1) * point_world_vec(1) + pabcd(2) * point_world_vec(2) + pabcd(3);
            double s = 1 - 0.9 * std::fabs(pd2) / std::sqrt(point_body_vec.norm());
            if (s > 0.9)
            {
                m_point_selected_flag[i] = true;
                m_norm_vec->points[i].x = pabcd(0);
                m_norm_vec->points[i].y = pabcd(1);
                m_norm_vec->points[i].z = pabcd(2);
                m_norm_vec->points[i].intensity = pd2;
                m_norm_vec->points[i].curvature = reliability;
            }
        }
    }

    m_metrics.geometry_degeneracy = 0.0;
    m_metrics.srda_gate = 0.0;
    m_metrics.srda_fit_benefit = 0.0;
    m_metrics.subspace_observations = 0;
    m_metrics.mean_subspace_confidence = 0.0;
    m_metrics.line_candidates = 0;
    m_metrics.selected_lines = 0;
    m_metrics.mean_line_weak_gain = 0.0;
    double structure_gate = 1.0;
    AdaptiveObservabilityResult adaptive_observability;
    if ((m_config.srda_enable_soft_gate) ||
        m_config.enable_structure_subspace)
    {
        M6D geometric_information = M6D::Zero();
        size_t observation_count = 0;
        for (int i = 0; i < size; ++i)
        {
            if (!m_point_selected_flag[i])
                continue;
            V3D normal(m_norm_vec->points[i].x, m_norm_vec->points[i].y, m_norm_vec->points[i].z);
            if (!normal.allFinite() || normal.norm() <= 1e-9)
                continue;
            normal.normalize();
            const PointType &point = m_cloud_down_lidar->points[i];
            const V3D point_imu = state.r_il * V3D(point.x, point.y, point.z) + state.t_il;
            const V3D normal_imu = state.r_wi.transpose() * normal;
            V6D jacobian = V6D::Zero();
            jacobian.head<3>() = point_imu.cross(normal_imu) /
                                 m_config.ipmm_characteristic_length;
            jacobian.tail<3>() = normal;
            geometric_information.noalias() += jacobian * jacobian.transpose();
            ++observation_count;
        }
        if (m_config.enable_adaptive_observability ||
            m_config.enable_structure_subspace)
        {
            adaptive_observability = computeAdaptiveObservability(geometric_information,
                                                                   observation_count);
            if (adaptive_observability.valid)
            {
                structure_gate = 0.0;
                m_metrics.geometry_degeneracy = adaptive_observability.degeneracy;
            }
        }
        if (!m_config.enable_adaptive_observability)
        {
            const auto gate = computeStructureGate6D(geometric_information, observation_count,
                                                     m_config.srda_gate_degeneracy_threshold,
                                                     m_config.srda_gate_sharpness);
            structure_gate = gate.gate;
            m_metrics.geometry_degeneracy = gate.degeneracy;
        }
    }

    double gate_sum = 0.0;
    double benefit_sum = 0.0;
    size_t gate_count = 0;
#ifdef MP_EN
#pragma omp parallel for reduction(+ : gate_sum, benefit_sum, gate_count)
#endif
    for (int i = 0; i < size; ++i)
    {
        const double plane_gate = m_config.enable_adaptive_observability
                                      ? fit_benefit[i]
                                      : (m_config.srda_enable_soft_gate
                                             ? structure_gate * fit_benefit[i]
                                             : 1.0);
        m_point_selected_flag[i] = false;
        V4D effective_plane = V4D::Zero();
        bool effective_plane_valid = false;
        if (m_config.enable_structure_subspace && robust_valid[i])
        {
            effective_plane = robust_planes[i];
            effective_plane_valid = true;
        }
        else if (!m_config.srda_enable_soft_gate)
        {
            if (robust_valid[i])
            {
                effective_plane = robust_planes[i];
                effective_plane_valid = true;
            }
        }
        else if (baseline_valid[i] && robust_valid[i])
        {
            V4D aligned_robust = robust_planes[i];
            if (baseline_planes[i].head<3>().dot(aligned_robust.head<3>()) < 0.0)
                aligned_robust = -aligned_robust;
            effective_plane = (1.0 - plane_gate) * baseline_planes[i] +
                              plane_gate * aligned_robust;
            const double normal_norm = effective_plane.head<3>().norm();
            if (normal_norm > 1e-9 && effective_plane.allFinite())
            {
                effective_plane /= normal_norm;
                effective_plane_valid = true;
            }
        }
        else if (baseline_valid[i])
        {
            effective_plane = baseline_planes[i];
            effective_plane_valid = true;
        }
        else if (!m_config.enable_adaptive_observability && robust_valid[i] && plane_gate >= 0.5)
        {
            // Only let the robust estimator rescue a plane rejected by
            // baseline once the scan is clearly in a degenerate regime.
            effective_plane = robust_planes[i];
            effective_plane_valid = true;
        }
        if (!effective_plane_valid)
            continue;

        const PointType &point_body = m_cloud_down_lidar->points[i];
        const PointType &point_world = m_cloud_down_world->points[i];
        const V3D point_body_vec(point_body.x, point_body.y, point_body.z);
        const double pd2 = effective_plane.head<3>().dot(V3D(point_world.x, point_world.y, point_world.z)) +
                           effective_plane(3);
        const double s = 1.0 - 0.9 * std::fabs(pd2) / std::sqrt(point_body_vec.norm());
        if (s <= 0.9)
            continue;

        m_point_selected_flag[i] = true;
        const double effective_reliability = m_config.enable_structure_subspace
                                                 ? m_structure_reliability[i]
                                                 : applyStructureGate(m_structure_reliability[i], plane_gate);
        m_norm_vec->points[i].x = effective_plane(0);
        m_norm_vec->points[i].y = effective_plane(1);
        m_norm_vec->points[i].z = effective_plane(2);
        m_norm_vec->points[i].intensity = pd2;
        m_norm_vec->points[i].curvature = effective_reliability;
        gate_sum += plane_gate;
        benefit_sum += fit_benefit[i];
        ++gate_count;
    }
    if (gate_count > 0)
    {
        m_metrics.srda_gate = gate_sum / static_cast<double>(gate_count);
        m_metrics.srda_fit_benefit = benefit_sum / static_cast<double>(gate_count);
    }



    int effect_feat_num = 0;
    for (int i = 0; i < size; i++)
    {
        if (!m_point_selected_flag[i])
            continue;
        m_effect_cloud_lidar->points[effect_feat_num] = m_cloud_down_lidar->points[i];
        m_effect_norm_vec->points[effect_feat_num] = m_norm_vec->points[i];
        effect_feat_num++;
    }
    double reliability_sum = 0.0;
    double reliability_normalizer = 1.0;
    if (effect_feat_num > 0 && m_config.enable_adaptive_observability)
    {
        double raw_sum = 0.0;
        for (int i = 0; i < effect_feat_num; ++i)
            raw_sum += static_cast<double>(m_effect_norm_vec->points[i].curvature);
        reliability_normalizer = std::max(1e-12, raw_sum / static_cast<double>(effect_feat_num));
    }
    share_data.H.setZero();
    share_data.b.setZero();
    Eigen::Matrix<double, 1, 12> J;
    for (int i = 0; i < effect_feat_num; i++)
    {
        J.setZero();
        const PointType &laser_p = m_effect_cloud_lidar->points[i];
        const PointType &norm_p = m_effect_norm_vec->points[i];
        Eigen::Vector3d laser_p_vec(laser_p.x, laser_p.y, laser_p.z);
        Eigen::Vector3d norm_vec(norm_p.x, norm_p.y, norm_p.z);
        Eigen::Matrix<double, 1, 3> B = -norm_vec.transpose() * state.r_wi * Sophus::SO3d::hat(state.r_il * laser_p_vec + state.t_il);
        J.block<1, 3>(0, 0) = B;
        J.block<1, 3>(0, 3) = norm_vec.transpose();
        if (m_config.esti_il)
        {
            Eigen::Matrix<double, 1, 3> C = -norm_vec.transpose() * state.r_wi * state.r_il * Sophus::SO3d::hat(laser_p_vec);
            Eigen::Matrix<double, 1, 3> D = norm_vec.transpose() * state.r_wi;
            J.block<1, 3>(0, 6) = C;
            J.block<1, 3>(0, 9) = D;
        }
        const double raw_reliability = static_cast<double>(norm_p.curvature);
        const double normalized_reliability =
            (m_config.enable_adaptive_observability)
                ? raw_reliability / reliability_normalizer
                : raw_reliability;
        const double information_weight = m_config.lidar_cov_inv * normalized_reliability;
        if (!std::isfinite(information_weight) || information_weight <= 0.0)
            throw std::runtime_error("SRDA produced invalid observation information");
        share_data.H += J.transpose() * information_weight * J;
        share_data.b += J.transpose() * information_weight * norm_p.intensity;
        reliability_sum += normalized_reliability;
    }
    m_metrics.mean_reliability = effect_feat_num > 0
                                     ? reliability_sum / static_cast<double>(effect_feat_num)
                                     : 0.0;

    size_t subspace_observations = 0;
    double subspace_confidence_sum = 0.0;
    size_t line_candidates = 0;
    size_t selected_lines = 0;
    double weak_gain_sum = 0.0;
    if (m_config.enable_structure_subspace)
    {
        struct LineAxisCandidate
        {
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            int neighborhood = -1;
            V3D axis = V3D::Zero();
            V3D laser_point = V3D::Zero();
            double residual = 0.0;
            double effective_score = 0.0;
            double weak_gain = 0.0;
        };
        struct LineNeighborhoodCandidate
        {
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            int index = -1;
            double score = 0.0;
            std::vector<LineAxisCandidate, Eigen::aligned_allocator<LineAxisCandidate>> axes;
        };
        std::vector<LineNeighborhoodCandidate,
                    Eigen::aligned_allocator<LineNeighborhoodCandidate>> candidates;
        candidates.reserve(size);
        for (int i = 0; i < size; ++i)
        {
            const auto &subspace = line_subspaces[i];
            if (!shouldUseLineConstraint(m_point_selected_flag[i], subspace))
                continue;
            ++line_candidates;
            const PointType &laser_p = m_cloud_down_lidar->points[i];
            const PointType &world_p = m_cloud_down_world->points[i];
            const V3D laser_p_vec(laser_p.x, laser_p.y, laser_p.z);
            const V3D world_p_vec(world_p.x, world_p.y, world_p.z);
            const V3D offset = world_p_vec - subspace.centroid;

            // A line supplies exactly its two transverse constraints.  Its
            // tangent (largest-eigenvalue axis) remains unconstrained, unlike
            // a point-to-point fallback that would invent information there.
            LineNeighborhoodCandidate candidate;
            candidate.index = i;
            for (int axis_index = 0; axis_index < 2; ++axis_index)
            {
                V3D axis = subspace.eigenvectors.col(axis_index);
                if (!axis.allFinite() || axis.norm() <= 1e-9)
                    continue;
                axis.normalize();
                const double residual = axis.dot(offset);
                const double variance = std::max(
                    m_config.srda_inlier_threshold * m_config.srda_inlier_threshold,
                    subspace.eigenvalues(axis_index));
                const double robust_weight = 1.0 / (1.0 + residual * residual / variance);
                const V3D point_imu = state.r_il * laser_p_vec + state.t_il;
                const V3D axis_imu = state.r_wi.transpose() * axis;
                V6D pose_jacobian = V6D::Zero();
                pose_jacobian.head<3>() = point_imu.cross(axis_imu) /
                                          m_config.ipmm_characteristic_length;
                pose_jacobian.tail<3>() = axis;
                const double weak_gain = computeWeakDirectionGain(
                    pose_jacobian, adaptive_observability);
                if (weak_gain < m_config.structure_line_min_weak_gain)
                    continue;
                const double effective_score =
                    subspace.confidence * robust_weight * weak_gain;
                if (!std::isfinite(effective_score) ||
                    effective_score < m_config.structure_line_min_effective_score)
                    continue;
                LineAxisCandidate axis_candidate;
                axis_candidate.neighborhood = i;
                axis_candidate.axis = axis;
                axis_candidate.laser_point = laser_p_vec;
                axis_candidate.residual = residual;
                axis_candidate.effective_score = effective_score;
                axis_candidate.weak_gain = weak_gain;
                candidate.axes.push_back(axis_candidate);
                candidate.score += effective_score;
            }
            if (!candidate.axes.empty())
                candidates.push_back(std::move(candidate));
        }

        std::sort(candidates.begin(), candidates.end(),
                  [](const LineNeighborhoodCandidate &left,
                     const LineNeighborhoodCandidate &right) {
                      return left.score > right.score;
                  });
        const size_t correlation_budget = static_cast<size_t>(std::ceil(
            m_config.structure_line_budget_scale *
            std::sqrt(static_cast<double>(std::max(1, effect_feat_num)))));
        const size_t hard_budget = static_cast<size_t>(
            m_config.structure_line_max_neighborhoods);
        const size_t neighborhood_budget = std::min(
            candidates.size(), std::min(correlation_budget, hard_budget));
        for (size_t candidate_index = 0;
             candidate_index < neighborhood_budget; ++candidate_index)
        {
            const auto &candidate = candidates[candidate_index];
            bool line_selected = false;
            for (const auto &axis_candidate : candidate.axes)
            {
                const V3D &axis = axis_candidate.axis;
                const V3D &laser_p_vec = axis_candidate.laser_point;
                const double axis_weight = 0.5 * axis_candidate.effective_score;
                const double information_weight = m_config.lidar_cov_inv * axis_weight;
                if (!std::isfinite(information_weight) || information_weight <= 0.0)
                    continue;
                J.setZero();
                const Eigen::Matrix<double, 1, 3> B =
                    -axis.transpose() * state.r_wi *
                    Sophus::SO3d::hat(state.r_il * laser_p_vec + state.t_il);
                J.block<1, 3>(0, 0) = B;
                J.block<1, 3>(0, 3) = axis.transpose();
                if (m_config.esti_il)
                {
                    const Eigen::Matrix<double, 1, 3> C =
                        -axis.transpose() * state.r_wi * state.r_il *
                        Sophus::SO3d::hat(laser_p_vec);
                    const Eigen::Matrix<double, 1, 3> D = axis.transpose() * state.r_wi;
                    J.block<1, 3>(0, 6) = C;
                    J.block<1, 3>(0, 9) = D;
                }
                share_data.H += J.transpose() * information_weight * J;
                share_data.b += J.transpose() * information_weight * axis_candidate.residual;
                ++subspace_observations;
                subspace_confidence_sum += axis_candidate.effective_score;
                weak_gain_sum += axis_candidate.weak_gain;
                line_selected = true;
            }
            if (line_selected)
                ++selected_lines;
        }
    }
    m_metrics.subspace_observations = subspace_observations;
    m_metrics.mean_subspace_confidence = subspace_observations > 0
                                             ? subspace_confidence_sum /
                                                   static_cast<double>(subspace_observations)
                                             : 0.0;
    m_metrics.line_candidates = line_candidates;
    m_metrics.selected_lines = selected_lines;
    m_metrics.mean_line_weak_gain = subspace_observations > 0
                                        ? weak_gain_sum /
                                              static_cast<double>(subspace_observations)
                                        : 0.0;
    m_metrics.effective_observations = static_cast<size_t>(effect_feat_num) +
                                       subspace_observations;
    if (m_metrics.effective_observations < 1)
    {
        share_data.valid = false;
        std::cerr << "NO Effective Points!" << std::endl;
        return;
    }
    share_data.valid = true;

    // Preserve the legacy observability attenuation when explicitly requested
    // without hierarchical line completion.  In the new path, the plane-only
    // eigensystem selects complementary lines; projecting the combined system
    // would suppress exactly the information those lines were chosen to add.
    if (m_config.enable_adaptive_observability &&
        !m_config.enable_structure_subspace && adaptive_observability.valid)
    {
        M12D scale = M12D::Identity();
        scale.block<3, 3>(0, 0) = M3D::Identity() * m_config.ipmm_characteristic_length;
        const M12D scale_inverse = scale.inverse();
        M12D projector = M12D::Identity();
        projector.block<6, 6>(0, 0) = adaptive_observability.projector;
        M12D normalized_information = scale_inverse.transpose() * share_data.H * scale_inverse;
        V12D normalized_gradient = scale_inverse.transpose() * share_data.b;
        normalized_information = projector * normalized_information * projector;
        normalized_gradient = projector * normalized_gradient;
        share_data.H = scale.transpose() * normalized_information * scale;
        share_data.b = scale.transpose() * normalized_gradient;
        share_data.H = 0.5 * (share_data.H + share_data.H.transpose());
    }
}

CloudType::Ptr LidarProcessor::transformCloud(CloudType::Ptr inp, const M3D &r, const V3D &t)
{
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform.block<3, 3>(0, 0) = r.cast<float>();
    transform.block<3, 1>(0, 3) = t.cast<float>();
    CloudType::Ptr ret(new CloudType);
    pcl::transformPointCloud(*inp, *ret, transform);
    return ret;
}
