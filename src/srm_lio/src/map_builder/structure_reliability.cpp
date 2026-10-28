#include "structure_reliability.h"

#include <algorithm>
#include <array>
#include <cmath>

namespace
{
double median(std::vector<double> values)
{
    if (values.empty())
        return 0.0;
    const size_t middle = values.size() / 2;
    std::nth_element(values.begin(), values.begin() + middle, values.end());
    double result = values[middle];
    if (values.size() % 2 == 0)
    {
        std::nth_element(values.begin(), values.begin() + middle - 1, values.end());
        result = 0.5 * (result + values[middle - 1]);
    }
    return result;
}

double clamp01(double value)
{
    return std::max(0.0, std::min(1.0, value));
}
} // namespace

LineSubspaceResult computeLineSubspace(const PointVec &points,
                                       double inlier_threshold,
                                       int robust_iterations)
{
    LineSubspaceResult result;
    if (points.size() < 3 || inlier_threshold <= 0.0 || robust_iterations < 1)
        return result;

    for (const auto &point : points)
        result.centroid += V3D(point.x, point.y, point.z);
    result.centroid /= static_cast<double>(points.size());

    std::vector<double> weights(points.size(), 1.0);
    for (int iteration = 0; iteration < robust_iterations; ++iteration)
    {
        double weight_sum = 0.0;
        V3D weighted_centroid = V3D::Zero();
        for (size_t i = 0; i < points.size(); ++i)
        {
            weight_sum += weights[i];
            weighted_centroid += weights[i] * V3D(points[i].x, points[i].y, points[i].z);
        }
        if (weight_sum < 3e-6)
            return result;
        result.centroid = weighted_centroid / weight_sum;

        M3D covariance = M3D::Zero();
        for (size_t i = 0; i < points.size(); ++i)
        {
            const V3D centered = V3D(points[i].x, points[i].y, points[i].z) - result.centroid;
            covariance.noalias() += weights[i] * centered * centered.transpose();
        }
        covariance /= weight_sum;
        Eigen::SelfAdjointEigenSolver<M3D> solver(covariance);
        if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite())
            return result;
        result.eigenvalues = solver.eigenvalues().cwiseMax(0.0);
        result.eigenvectors = solver.eigenvectors();

        if (iteration == 0)
        {
            const double largest = result.eigenvalues(2);
            if (largest <= 1e-12)
                return result;
            const double initial_linearity =
                (result.eigenvalues(2) - result.eigenvalues(1)) / largest;
            const double initial_planarity =
                (result.eigenvalues(1) - result.eigenvalues(0)) / largest;
            const double initial_scattering = result.eigenvalues(0) / largest;
            // Robust line refinement is allowed only for a neighbourhood that
            // is already one-dimensional. Otherwise reweighting a plane can
            // collapse its support into an artificial narrow strip.
            if (initial_linearity <= std::max(initial_planarity, initial_scattering))
                break;
        }

        const V3D tangent = result.eigenvectors.col(2);
        for (size_t i = 0; i < points.size(); ++i)
        {
            const V3D offset = V3D(points[i].x, points[i].y, points[i].z) - result.centroid;
            const double transverse_distance = (offset - tangent * tangent.dot(offset)).norm();
            const double u = transverse_distance / (4.685 * inlier_threshold);
            weights[i] = u < 1.0 ? std::pow(1.0 - u * u, 2) : 0.0;
        }
    }

    const double largest = result.eigenvalues(2);
    if (largest <= 1e-12)
        return result;
    result.linearity = clamp01((result.eigenvalues(2) - result.eigenvalues(1)) / largest);
    const double planarity = (result.eigenvalues(1) - result.eigenvalues(0)) / largest;
    const double scatteredness = result.eigenvalues(0) / largest;
    const std::array<double, 3> saliency = {scatteredness, planarity, result.linearity};
    size_t winner = 0;
    size_t runner_up = 1;
    if (saliency[runner_up] > saliency[winner])
        std::swap(winner, runner_up);
    for (size_t i = 2; i < saliency.size(); ++i)
    {
        if (saliency[i] > saliency[winner])
        {
            runner_up = winner;
            winner = i;
        }
        else if (saliency[i] > saliency[runner_up])
            runner_up = i;
    }
    result.model = static_cast<LocalGeometryModel>(winner);
    size_t inliers = 0;
    double squared_transverse_sum = 0.0;
    const V3D tangent = result.eigenvectors.col(2);
    for (const auto &point : points)
    {
        const V3D offset = V3D(point.x, point.y, point.z) - result.centroid;
        const double distance = (offset - tangent * tangent.dot(offset)).norm();
        if (distance <= inlier_threshold)
        {
            ++inliers;
            squared_transverse_sum += distance * distance;
        }
    }
    result.inlier_ratio = static_cast<double>(inliers) / static_cast<double>(points.size());
    result.transverse_uncertainty = inliers > 0
                                        ? squared_transverse_sum / static_cast<double>(inliers)
                                        : 1.0;
    const double uncertainty_score = std::exp(-result.transverse_uncertainty /
                                              (inlier_threshold * inlier_threshold));
    const double saliency_margin = clamp01(saliency[winner] - saliency[runner_up]);
    result.confidence = clamp01(result.linearity * result.inlier_ratio *
                                uncertainty_score * saliency_margin);
    result.valid = result.model == LocalGeometryModel::Line && inliers >= 3 &&
                   result.confidence > 1e-6 && result.centroid.allFinite() &&
                   result.eigenvectors.allFinite();
    return result;
}

LocalGeometryAssessment assessLocalGeometry(
    const PointVec &points,
    const StructureReliabilityResult &plane,
    double plane_min_quality,
    double line_min_quality,
    double line_inlier_threshold,
    int line_robust_iterations)
{
    LocalGeometryAssessment result;
    result.plane_quality = plane.valid ? clamp01(plane.reliability) : 0.0;
    // planarity is essential here: a line can have an extremely small
    // point-to-plane residual but lacks the second tangent direction required
    // to define a stable plane.
    if (plane.valid && plane.planarity >= plane_min_quality &&
        result.plane_quality >= plane_min_quality)
    {
        result.model = LocalGeometryModel::Plane;
        return result;
    }

    result.line = computeLineSubspace(points, line_inlier_threshold,
                                      line_robust_iterations);
    if (result.line.valid && result.line.confidence >= line_min_quality)
        result.model = LocalGeometryModel::Line;
    return result;
}

bool shouldUseLineConstraint(bool plane_constraint_valid,
                             const LineSubspaceResult &line_subspace)
{
    return !plane_constraint_valid && line_subspace.valid &&
           line_subspace.model == LocalGeometryModel::Line;
}

double computeLineAxisWeight(double confidence, double robust_weight)
{
    if (!std::isfinite(confidence) || !std::isfinite(robust_weight))
        return 0.0;
    return 0.5 * clamp01(confidence) * clamp01(robust_weight);
}

double computeWeakDirectionGain(const V6D &jacobian,
                                const AdaptiveObservabilityResult &plane_observability)
{
    if (!jacobian.allFinite())
        return 0.0;
    const double energy = jacobian.squaredNorm();
    if (energy <= 1e-12)
        return 0.0;
    if (!plane_observability.valid)
        return 1.0;
    const M6D weak_projector = M6D::Identity() -
                               plane_observability.projector * plane_observability.projector;
    return clamp01(jacobian.dot(weak_projector * jacobian) / energy);
}

StructureReliabilityResult StructureReliability::evaluate(const PointVec &points) const
{
    StructureReliabilityResult result;
    if (points.size() < 3)
        return result;

    V3D centroid = V3D::Zero();
    for (const auto &point : points)
        centroid += V3D(point.x, point.y, point.z);
    centroid /= static_cast<double>(points.size());

    M3D covariance = M3D::Zero();
    for (const auto &point : points)
    {
        const V3D centered = V3D(point.x, point.y, point.z) - centroid;
        covariance.noalias() += centered * centered.transpose();
    }
    covariance /= static_cast<double>(points.size());
    Eigen::SelfAdjointEigenSolver<M3D> solver(covariance);
    if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite())
        return result;

    result.eigenvalues = solver.eigenvalues().cwiseMax(0.0);
    result.eigenvectors = solver.eigenvectors();
    V3D initial_normal = result.eigenvectors.col(0).normalized();
    result.initial_plane << initial_normal, -initial_normal.dot(centroid);

    V3D robust_normal = initial_normal;
    V3D robust_centroid = centroid;
    for (int iteration = 0; iteration < parameters_.robust_iterations; ++iteration)
    {
        std::vector<double> absolute_residuals;
        absolute_residuals.reserve(points.size());
        for (const auto &point : points)
            absolute_residuals.push_back(std::fabs(robust_normal.dot(V3D(point.x, point.y, point.z) - robust_centroid)));
        const double scale = std::max(parameters_.robust_scale_floor, 1.4826 * median(absolute_residuals));

        double weight_sum = 0.0;
        V3D weighted_centroid = V3D::Zero();
        std::vector<double> weights(points.size(), 0.0);
        for (size_t i = 0; i < points.size(); ++i)
        {
            const double u = absolute_residuals[i] / (4.685 * scale);
            const double weight = u < 1.0 ? std::pow(1.0 - u * u, 2) : 0.0;
            weights[i] = weight;
            weight_sum += weight;
            weighted_centroid += weight * V3D(points[i].x, points[i].y, points[i].z);
        }
        if (weight_sum < 3.0 * 1e-6)
            return result;
        weighted_centroid /= weight_sum;
        M3D weighted_covariance = M3D::Zero();
        for (size_t i = 0; i < points.size(); ++i)
        {
            const V3D centered = V3D(points[i].x, points[i].y, points[i].z) - weighted_centroid;
            weighted_covariance.noalias() += weights[i] * centered * centered.transpose();
        }
        weighted_covariance /= weight_sum;
        Eigen::SelfAdjointEigenSolver<M3D> robust_solver(weighted_covariance);
        if (robust_solver.info() != Eigen::Success || !robust_solver.eigenvalues().allFinite())
            return result;
        V3D next_normal = robust_solver.eigenvectors().col(0).normalized();
        if (next_normal.dot(robust_normal) < 0.0)
            next_normal = -next_normal;
        robust_normal = next_normal;
        robust_centroid = weighted_centroid;
    }

    if (robust_normal.dot(initial_normal) < 0.0)
        robust_normal = -robust_normal;
    result.robust_plane << robust_normal, -robust_normal.dot(robust_centroid);

    size_t inliers = 0;
    double squared_residual_sum = 0.0;
    for (const auto &point : points)
    {
        const double residual = robust_normal.dot(V3D(point.x, point.y, point.z)) + result.robust_plane(3);
        if (std::fabs(residual) <= parameters_.inlier_threshold)
        {
            ++inliers;
            squared_residual_sum += residual * residual;
        }
    }
    result.inlier_ratio = static_cast<double>(inliers) / static_cast<double>(points.size());
    const double eigen_sum = std::max(1e-12, result.eigenvalues.sum());
    result.planarity = clamp01((result.eigenvalues(1) - result.eigenvalues(0)) /
                              std::max(1e-12, result.eigenvalues(2)));
    result.roughness = clamp01(result.eigenvalues(0) / eigen_sum);
    result.normal_consistency = clamp01(std::fabs(initial_normal.dot(robust_normal)));
    result.plane_uncertainty = inliers > 0 ? squared_residual_sum / static_cast<double>(inliers) : 1.0;
    const double uncertainty_score = std::exp(-result.plane_uncertainty /
                                              std::max(1e-12, parameters_.inlier_threshold * parameters_.inlier_threshold));
    const double roughness_score = clamp01(1.0 - 3.0 * result.roughness);
    result.reliability = clamp01(result.planarity * roughness_score * result.normal_consistency *
                                 result.inlier_ratio * uncertainty_score);
    result.reliability = std::max(parameters_.min_reliability, result.reliability);
    result.valid = result.robust_plane.allFinite() && std::isfinite(result.reliability) && inliers >= 3;
    return result;
}

StructureGateResult computeStructureGate(const M3D &normal_information,
                                         size_t normal_count,
                                         double degeneracy_threshold,
                                         double sharpness)
{
    StructureGateResult result;
    if (normal_count < 3 || !normal_information.allFinite())
        return result;

    const M3D normalized_information = normal_information / static_cast<double>(normal_count);
    Eigen::SelfAdjointEigenSolver<M3D> solver(normalized_information);
    if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite())
        return result;

    const double trace = std::max(1e-12, solver.eigenvalues().sum());
    result.richness = clamp01(3.0 * std::max(0.0, solver.eigenvalues().minCoeff()) / trace);
    result.degeneracy = 1.0 - result.richness;
    const double argument = std::clamp(sharpness * (result.degeneracy - degeneracy_threshold), -60.0, 60.0);
    result.gate = 1.0 / (1.0 + std::exp(-argument));
    return result;
}

StructureGateResult computeStructureGate6D(const M6D &information,
                                           size_t observation_count,
                                           double degeneracy_threshold,
                                           double sharpness)
{
    StructureGateResult result;
    if (observation_count < 6 || !information.allFinite())
        return result;

    const M6D normalized_information = information / static_cast<double>(observation_count);
    Eigen::SelfAdjointEigenSolver<M6D> solver(normalized_information);
    if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite())
        return result;

    const double trace = std::max(1e-12, solver.eigenvalues().sum());
    result.richness = clamp01(6.0 * std::max(0.0, solver.eigenvalues().minCoeff()) / trace);
    result.degeneracy = 1.0 - result.richness;
    const double argument = std::clamp(sharpness * (result.degeneracy - degeneracy_threshold), -60.0, 60.0);
    result.gate = 1.0 / (1.0 + std::exp(-argument));
    return result;
}

AdaptiveObservabilityResult computeAdaptiveObservability(const M6D &information,
                                                          size_t observation_count)
{
    AdaptiveObservabilityResult result;
    if (observation_count < 6 || !information.allFinite())
        return result;

    const M6D normalized = 0.5 * (information + information.transpose()) /
                           static_cast<double>(observation_count);
    Eigen::SelfAdjointEigenSolver<M6D> solver(normalized);
    if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite())
        return result;

    const V6D eigenvalues = solver.eigenvalues().cwiseMax(0.0);
    const double median = std::max(1e-12, 0.5 * (eigenvalues(2) + eigenvalues(3)));
    V6D raw_gains;
    for (Eigen::Index i = 0; i < raw_gains.size(); ++i)
        raw_gains(i) = eigenvalues(i) / (eigenvalues(i) + median);
    const double normalization = std::max(1e-12, raw_gains.maxCoeff());
    result.gains = (raw_gains / normalization).cwiseMax(0.0).cwiseMin(1.0);
    // This operator is applied on both sides of the normal matrix.  Use the
    // square root so each eigendirection receives the requested information
    // gain once, rather than accidentally squaring the attenuation.
    result.projector = solver.eigenvectors() * result.gains.cwiseSqrt().asDiagonal() *
                       solver.eigenvectors().transpose();
    result.projector = 0.5 * (result.projector + result.projector.transpose());
    result.degeneracy = 1.0 - result.gains.minCoeff();
    result.effective_rank = result.gains.sum();
    result.valid = result.projector.allFinite() && result.gains.allFinite();
    return result;
}

double combineStructureGates(double primary_gate, double enhancement_gate)
{
    const double primary = clamp01(primary_gate);
    const double enhancement = clamp01(enhancement_gate);
    // Rotation may strengthen an already-supported degeneracy decision, but it
    // must not independently turn SRDA on in a scene with rich plane normals.
    return clamp01(primary + (1.0 - primary) * enhancement * primary);
}

double computePlaneFitBenefit(const PointVec &points,
                              const V4D &baseline_plane,
                              const V4D &robust_plane)
{
    if (points.empty() || !baseline_plane.allFinite() || !robust_plane.allFinite())
        return 0.0;

    const double baseline_normal_norm = baseline_plane.head<3>().norm();
    const double robust_normal_norm = robust_plane.head<3>().norm();
    if (baseline_normal_norm <= 1e-12 || robust_normal_norm <= 1e-12)
        return 0.0;

    double baseline_squared_error = 0.0;
    double robust_squared_error = 0.0;
    for (const auto &point : points)
    {
        const V3D position(point.x, point.y, point.z);
        const double baseline_residual =
            (baseline_plane.head<3>().dot(position) + baseline_plane(3)) / baseline_normal_norm;
        const double robust_residual =
            (robust_plane.head<3>().dot(position) + robust_plane(3)) / robust_normal_norm;
        baseline_squared_error += baseline_residual * baseline_residual;
        robust_squared_error += robust_residual * robust_residual;
    }
    if (baseline_squared_error <= 1e-12)
        return 0.0;

    const double relative_improvement =
        (baseline_squared_error - robust_squared_error) / baseline_squared_error;
    // The square root keeps the response soft while allowing a clear, measured
    // fit improvement to make meaningful use of the environment-level gate.
    return std::sqrt(clamp01(relative_improvement));
}

double applyStructureGate(double reliability, double gate)
{
    return clamp01(1.0 - clamp01(gate) * (1.0 - clamp01(reliability)));
}
