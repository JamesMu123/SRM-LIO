#include "structure_reliability.h"

#include <cmath>
#include <iostream>
#include <sophus/so3.hpp>

int main()
{
    PointVec planar;
    for (int x = -5; x <= 5; ++x)
        for (int y = -5; y <= 5; ++y)
        {
            PointType point;
            point.x = 0.1f * x;
            point.y = 0.1f * y;
            point.z = 0.001f * ((x + y) % 3);
            planar.push_back(point);
        }
    StructureReliability evaluator;
    const auto clean = evaluator.evaluate(planar);
    if (!clean.valid || !clean.robust_plane.allFinite() || clean.reliability < 0.5 || clean.planarity < 0.5)
    {
        std::cerr << "clean plane reliability test failed\n";
        return 1;
    }

    PointType outlier;
    outlier.x = 0.0f;
    outlier.y = 0.0f;
    outlier.z = 2.0f;
    planar.push_back(outlier);
    const auto contaminated = evaluator.evaluate(planar);
    if (!contaminated.valid || std::fabs(contaminated.robust_plane.head<3>().dot(V3D::UnitZ())) < 0.99 ||
        contaminated.inlier_ratio >= 1.0)
    {
        std::cerr << "robust plane outlier test failed\n";
        return 2;
    }

    const double weight = 1000.0 * contaminated.reliability;
    Eigen::Matrix<double, 1, 12> jacobian = Eigen::Matrix<double, 1, 12>::Random();
    const double residual = 0.07;
    const auto direct_h = jacobian.transpose() * weight * jacobian;
    const auto white_h = (std::sqrt(weight) * jacobian).transpose() * (std::sqrt(weight) * jacobian);
    const auto direct_b = jacobian.transpose() * weight * residual;
    const auto white_b = (std::sqrt(weight) * jacobian).transpose() * (std::sqrt(weight) * residual);
    if (!direct_h.isApprox(white_h, 1e-10) || !direct_b.isApprox(white_b, 1e-10))
    {
        std::cerr << "residual/Jacobian whitening consistency test failed\n";
        return 3;
    }
    Eigen::LLT<Eigen::Matrix<double, 12, 12>> llt(direct_h + Eigen::Matrix<double, 12, 12>::Identity());
    if (llt.info() != Eigen::Success)
    {
        std::cerr << "positive-definiteness test failed\n";
        return 4;
    }

    const M3D rotation = Eigen::AngleAxisd(0.3, V3D(0.2, -0.4, 0.7).normalized()).toRotationMatrix();
    const V3D point(1.2, -0.5, 2.0);
    const V3D translation(0.3, 0.1, -0.2);
    const V3D normal = V3D(0.4, 0.2, 0.8).normalized();
    Eigen::Matrix<double, 1, 6> analytic;
    analytic.head<3>() = -normal.transpose() * rotation * Sophus::SO3d::hat(point);
    analytic.tail<3>() = normal.transpose();
    Eigen::Matrix<double, 1, 6> numeric;
    const double epsilon = 1e-7;
    auto point_plane_residual = [&](const Eigen::Matrix<double, 6, 1> &delta) {
        return normal.dot(rotation * Sophus::SO3d::exp(delta.head<3>()).matrix() * point +
                          translation + delta.tail<3>());
    };
    for (int column = 0; column < 6; ++column)
    {
        Eigen::Matrix<double, 6, 1> plus = Eigen::Matrix<double, 6, 1>::Zero();
        Eigen::Matrix<double, 6, 1> minus = Eigen::Matrix<double, 6, 1>::Zero();
        plus(column) = epsilon;
        minus(column) = -epsilon;
        numeric(column) = (point_plane_residual(plus) - point_plane_residual(minus)) / (2.0 * epsilon);
    }
    if (!analytic.isApprox(numeric, 1e-7))
    {
        std::cerr << "numerical point-to-plane Jacobian test failed\n";
        return 5;
    }

    const M3D rich_information = M3D::Identity() * 100.0;
    const auto rich_gate = computeStructureGate(rich_information, 300, 0.55, 12.0);
    const M3D degenerate_information = V3D::UnitX() * V3D::UnitX().transpose() * 300.0;
    const auto degenerate_gate = computeStructureGate(degenerate_information, 300, 0.55, 12.0);
    if (rich_gate.richness < 0.99 || rich_gate.gate >= 0.01 ||
        degenerate_gate.degeneracy < 0.99 || degenerate_gate.gate <= 0.99)
    {
        std::cerr << "soft structure gate response test failed\n";
        return 6;
    }
    const double raw_reliability = 0.2;
    if (applyStructureGate(raw_reliability, 0.0) != 1.0 ||
        std::fabs(applyStructureGate(raw_reliability, 1.0) - raw_reliability) > 1e-12)
    {
        std::cerr << "soft structure gate blending test failed\n";
        return 7;
    }
    const M6D rich_information_6d = M6D::Identity() * 100.0;
    const auto rich_gate_6d = computeStructureGate6D(rich_information_6d, 600, 0.55, 12.0);
    M6D degenerate_information_6d = M6D::Zero();
    degenerate_information_6d(0, 0) = 600.0;
    const auto degenerate_gate_6d = computeStructureGate6D(degenerate_information_6d, 600, 0.55, 12.0);
    if (rich_gate_6d.richness < 0.99 || rich_gate_6d.gate >= 0.01 ||
        degenerate_gate_6d.degeneracy < 0.99 || degenerate_gate_6d.gate <= 0.99)
    {
        std::cerr << "6D structure gate response test failed\n";
        return 8;
    }
    if (combineStructureGates(0.0, 1.0) != 0.0 ||
        std::fabs(combineStructureGates(0.1, 1.0) - 0.19) > 1e-12 ||
        std::fabs(combineStructureGates(0.4, 0.5) - 0.52) > 1e-12 ||
        combineStructureGates(1.0, 0.0) != 1.0)
    {
        std::cerr << "conditioned structure gate fusion test failed\n";
        return 9;
    }
    PointVec benefit_points(3);
    benefit_points[0].x = 0.0f; benefit_points[0].y = 0.0f; benefit_points[0].z = 0.0f;
    benefit_points[1].x = 1.0f; benefit_points[1].y = 0.0f; benefit_points[1].z = 0.0f;
    benefit_points[2].x = 0.0f; benefit_points[2].y = 1.0f; benefit_points[2].z = 0.0f;
    V4D poor_plane(0.0, 0.0, 1.0, 1.0);
    V4D exact_plane(0.0, 0.0, 1.0, 0.0);
    if (std::fabs(computePlaneFitBenefit(benefit_points, poor_plane, exact_plane) - 1.0) > 1e-12 ||
        computePlaneFitBenefit(benefit_points, exact_plane, exact_plane) != 0.0)
    {
        std::cerr << "plane fit benefit test failed\n";
        return 10;
    }
    M6D isotropic = M6D::Identity() * 100.0;
    const auto isotropic_observability = computeAdaptiveObservability(isotropic, 100);
    if (!isotropic_observability.valid ||
        (isotropic_observability.gains.array() - 1.0).abs().maxCoeff() > 1e-12 ||
        isotropic_observability.degeneracy > 1e-12)
    {
        std::cerr << "isotropic adaptive observability test failed\n";
        return 11;
    }
    M6D directional = M6D::Identity() * 100.0;
    directional(0, 0) = 1e-6;
    const auto directional_observability = computeAdaptiveObservability(directional, 100);
    const auto scaled_observability = computeAdaptiveObservability(directional * 1000.0, 100);
    if (!directional_observability.valid || directional_observability.gains(0) > 1e-5 ||
        directional_observability.gains.tail<5>().minCoeff() < 0.99 ||
        (directional_observability.projector - scaled_observability.projector).norm() > 1e-10)
    {
        std::cerr << "directional adaptive observability test failed\n";
        return 12;
    }
    PointVec linear;
    for (int x = -10; x <= 10; ++x)
    {
        PointType point;
        point.x = 0.1f * x;
        point.y = 0.001f * (x % 2);
        point.z = 0.001f * (x % 3);
        linear.push_back(point);
    }
    const auto line_subspace = computeLineSubspace(linear);
    if (!line_subspace.valid || line_subspace.model != LocalGeometryModel::Line ||
        line_subspace.confidence < 0.9 ||
        std::fabs(line_subspace.eigenvectors.col(2).dot(V3D::UnitX())) < 0.99)
    {
        std::cerr << "linear subspace detection test failed\n";
        return 13;
    }
    planar.pop_back();
    const auto plane_subspace = computeLineSubspace(planar);
    if (plane_subspace.valid || plane_subspace.model != LocalGeometryModel::Plane)
    {
        std::cerr << "plane-to-line rejection test failed\n";
        return 14;
    }
    if (shouldUseLineConstraint(true, line_subspace) ||
        !shouldUseLineConstraint(false, line_subspace) ||
        shouldUseLineConstraint(false, plane_subspace))
    {
        std::cerr << "line fallback arbitration test failed\n";
        return 15;
    }
    const double axis_weight = computeLineAxisWeight(0.8, 1.0);
    if (std::fabs(2.0 * axis_weight - 0.8) > 1e-12 ||
        computeLineAxisWeight(2.0, 2.0) != 0.5)
    {
        std::cerr << "line fallback weight normalization test failed\n";
        return 16;
    }
    const auto plane_geometry = assessLocalGeometry(planar, evaluator.evaluate(planar),
                                                    0.15, 0.20, 0.1, 3);
    const auto line_geometry = assessLocalGeometry(linear, evaluator.evaluate(linear),
                                                   0.15, 0.20, 0.1, 3);
    if (plane_geometry.model != LocalGeometryModel::Plane ||
        line_geometry.model != LocalGeometryModel::Line)
    {
        std::cerr << "hierarchical plane-first arbitration test failed\n";
        return 17;
    }
    V6D weak_axis = V6D::Zero();
    weak_axis(0) = 1.0;
    V6D strong_axis = V6D::Zero();
    strong_axis(1) = 1.0;
    if (computeWeakDirectionGain(weak_axis, directional_observability) < 0.99 ||
        computeWeakDirectionGain(strong_axis, directional_observability) > 0.01)
    {
        std::cerr << "weak-direction line gain test failed\n";
        return 18;
    }
    return 0;
}
