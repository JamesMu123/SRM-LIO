#pragma once

#include "commons.h"

struct StructureReliabilityResult
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    bool valid = false;
    V4D initial_plane = V4D::Zero();
    V4D robust_plane = V4D::Zero();
    V3D eigenvalues = V3D::Zero(); // ascending
    M3D eigenvectors = M3D::Identity();
    double planarity = 0.0;
    double roughness = 1.0;
    double normal_consistency = 0.0;
    double inlier_ratio = 0.0;
    double plane_uncertainty = 1.0;
    double reliability = 0.0;
};

class StructureReliability
{
public:
    struct Parameters
    {
        double inlier_threshold = 0.1;
        double min_reliability = 0.05;
        double robust_scale_floor = 0.01;
        int robust_iterations = 5;
    };

    StructureReliability() = default;
    explicit StructureReliability(const Parameters &parameters) : parameters_(parameters) {}
    StructureReliabilityResult evaluate(const PointVec &points) const;

private:
    Parameters parameters_;
};

struct StructureGateResult
{
    double richness = 0.0;
    double degeneracy = 1.0;
    double gate = 1.0;
};

enum class LocalGeometryModel : uint8_t
{
    Scattered,
    Plane,
    Line
};

struct LineSubspaceResult
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    bool valid = false;
    V3D centroid = V3D::Zero();
    M3D eigenvectors = M3D::Identity(); // ascending eigenvalue order
    V3D eigenvalues = V3D::Zero();
    double confidence = 0.0;
    double linearity = 0.0;
    double inlier_ratio = 0.0;
    double transverse_uncertainty = 1.0;
    LocalGeometryModel model = LocalGeometryModel::Scattered;
};

struct LocalGeometryAssessment
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    LocalGeometryModel model = LocalGeometryModel::Scattered;
    double plane_quality = 0.0;
    LineSubspaceResult line;
};

struct AdaptiveObservabilityResult;

// Fit a robust one-dimensional subspace.  This is evaluated only after a
// neighbourhood has failed the reliable-plane test.
LineSubspaceResult computeLineSubspace(const PointVec &points,
                                       double inlier_threshold = 0.1,
                                       int robust_iterations = 3);

// Hierarchical arbitration: a plane with genuine two-dimensional support has
// priority; a line may be used only when that plane is not reliable.
LocalGeometryAssessment assessLocalGeometry(
    const PointVec &points,
    const StructureReliabilityResult &plane,
    double plane_min_quality,
    double line_min_quality,
    double line_inlier_threshold,
    int line_robust_iterations);

// Plane and line models are mutually exclusive for each neighbourhood.
bool shouldUseLineConstraint(bool plane_constraint_valid,
                             const LineSubspaceResult &line_subspace);

// A line contributes two transverse scalar residuals. Split the evidence
// between them so one neighbourhood cannot count as two independent points.
double computeLineAxisWeight(double confidence, double robust_weight);

// Fraction of a candidate constraint's normalized information that falls in
// weak eigendirections of the plane-only system.  An invalid plane system
// returns one so line-only initialization remains possible.
double computeWeakDirectionGain(const V6D &jacobian,
                                const AdaptiveObservabilityResult &plane_observability);

struct AdaptiveObservabilityResult
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    bool valid = false;
    M6D projector = M6D::Identity();
    V6D gains = V6D::Ones();
    double degeneracy = 0.0;
    double effective_rank = 6.0;
};

// Compute a scale-invariant continuous projector from a dimensionless pose
// information matrix. No scene- or dataset-specific threshold is used.
AdaptiveObservabilityResult computeAdaptiveObservability(const M6D &information,
                                                          size_t observation_count);

StructureGateResult computeStructureGate(const M3D &normal_information,
                                         size_t normal_count,
                                         double degeneracy_threshold,
                                         double sharpness);

StructureGateResult computeStructureGate6D(const M6D &information,
                                           size_t observation_count,
                                           double degeneracy_threshold,
                                           double sharpness);

double combineStructureGates(double primary_gate, double enhancement_gate);

double computePlaneFitBenefit(const PointVec &points,
                              const V4D &baseline_plane,
                              const V4D &robust_plane);

double applyStructureGate(double reliability, double gate);
