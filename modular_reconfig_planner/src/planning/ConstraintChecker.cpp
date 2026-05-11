#include "planning/ConstraintChecker.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

namespace mrr {

static double positivePart(double v) { return v < 0.0 ? 0.0 : v; }

static bool isTerrainLikeObstacle(const Obstacle& obs) {
    const std::string t = normalizeToken(obs.type);
    return t == "ROUGH_BLOCK" || t == "STEP" || t == "PLATFORM" || t == "FLOOR";
}

ConstraintReport ConstraintChecker::check(
    const RobotGraph& currentGraph,
    const Configuration& candidate,
    const EnvironmentState& env
) const {
    ConstraintReport r;

    r.width = candidate.estimatedWidth();
    r.length = candidate.estimatedLength();
    r.height = candidate.estimatedHeight();
    r.requiredMinClearance = env.requiredMinClearance;
    r.requiredStabilityMargin = std::max(candidate.minStabilityMargin, env.requiredStabilityMargin);

    r.currentGraphConnected = currentGraph.isConnected();
    r.targetGraphConnected = candidate.allowDisconnectedGraph || candidate.graph.isConnected();
    r.jointLimitsOk = checkJointLimits(candidate);
    r.dimensionLimitsOk = checkDimensionLimits(candidate, env, r);
    r.geometryOk = checkGeometry(candidate, env, r);
    r.stabilityMargin = computeStabilityMargin(candidate);
    r.stable = r.stabilityMargin >= r.requiredStabilityMargin;
    r.minClearance = estimateMinClearance(candidate, env);
    r.collisionFree = r.minClearance >= env.requiredMinClearance;

    r.values["width"] = r.width;
    r.values["length"] = r.length;
    r.values["height"] = r.height;
    r.values["stability_margin"] = r.stabilityMargin;
    r.values["minimum_clearance"] = r.minClearance;

    if (!r.currentGraphConnected) r.violations.push_back("current graph is disconnected");
    if (!r.targetGraphConnected) r.violations.push_back("target graph is disconnected");
    if (!r.jointLimitsOk) r.violations.push_back("joint angle limit is violated");
    if (!r.dimensionLimitsOk) r.violations.push_back("configuration exceeds width/length/height limit");
    if (!r.geometryOk) r.violations.push_back("configuration is not passable for the detected environment");
    if (!r.stable) r.violations.push_back("stability margin is below the required value");
    if (!r.collisionFree) r.violations.push_back("minimum clearance to obstacle is too small");

    r.valid = r.currentGraphConnected && r.targetGraphConnected && r.jointLimitsOk &&
              r.dimensionLimitsOk && r.geometryOk && r.stable && r.collisionFree;

    if (!r.valid) {
        std::ostringstream oss;
        for (std::size_t i = 0; i < r.violations.size(); ++i) {
            if (i) oss << "; ";
            oss << r.violations[i];
        }
        r.reason = oss.str();
    }
    return r;
}

bool ConstraintChecker::checkJointLimits(const Configuration& candidate) const {
    for (const auto& t : candidate.targets) {
        for (double q : t.targetJointAngles) {
            if (std::abs(q) > candidate.maxJointAngle + 1e-9) return false;
        }
    }
    return true;
}

bool ConstraintChecker::checkDimensionLimits(const Configuration& candidate, const EnvironmentState& env, ConstraintReport& report) const {
    const double allowedWidth = std::min(candidate.maxWidth, env.maxAllowedWidth);
    const double allowedLength = std::min(candidate.maxLength, env.maxAllowedLength);
    const double allowedHeight = std::min(candidate.maxHeightLimit, env.maxAllowedHeight);
    report.values["allowed_width"] = allowedWidth;
    report.values["allowed_length"] = allowedLength;
    report.values["allowed_height"] = allowedHeight;
    return report.width <= allowedWidth && report.length <= allowedLength && report.height <= allowedHeight;
}

bool ConstraintChecker::checkGeometry(const Configuration& candidate, const EnvironmentState& env, ConstraintReport& report) const {
    switch (env.type) {
        case EnvironmentType::NarrowPassage:
            report.values["corridor_width"] = env.corridorWidth;
            return report.width <= env.corridorWidth;
        case EnvironmentType::Step:
            report.values["step_height"] = env.stepHeight;
            return report.height >= env.stepHeight + candidate.moduleDimensions.z * 0.5;
        case EnvironmentType::Gap:
            report.values["gap_width"] = env.gapWidth;
            return report.length >= env.gapWidth + 2.0 * candidate.moduleDimensions.x;
        case EnvironmentType::RoughSurface:
            report.values["roughness"] = env.roughness;
            return report.width >= candidate.moduleDimensions.y;
        default:
            return true;
    }
}

double ConstraintChecker::computeStabilityMargin(const Configuration& candidate) const {
    if (candidate.targets.empty()) return 0.0;

    double minZ = std::numeric_limits<double>::infinity();
    for (const auto& t : candidate.targets) minZ = std::min(minZ, t.targetPose.position.z);

    std::vector<Vec3> support;
    Vec3 com{0.0, 0.0, 0.0};
    for (const auto& t : candidate.targets) {
        com = com + t.targetPose.position;
        if (t.targetPose.position.z <= minZ + candidate.moduleDimensions.z * 1.2) support.push_back(t.targetPose.position);
    }
    com = com / static_cast<double>(candidate.targets.size());
    if (support.empty()) return candidate.stabilityEstimate;

    double minX = std::numeric_limits<double>::infinity();
    double maxX = -std::numeric_limits<double>::infinity();
    double minY = std::numeric_limits<double>::infinity();
    double maxY = -std::numeric_limits<double>::infinity();

    for (const auto& p : support) {
        minX = std::min(minX, p.x - candidate.moduleDimensions.x / 2.0);
        maxX = std::max(maxX, p.x + candidate.moduleDimensions.x / 2.0);
        minY = std::min(minY, p.y - candidate.moduleDimensions.y / 2.0);
        maxY = std::max(maxY, p.y + candidate.moduleDimensions.y / 2.0);
    }

    const double mx = std::min(com.x - minX, maxX - com.x);
    const double my = std::min(com.y - minY, maxY - com.y);
    const double geometricMargin = std::max(0.0, std::min(mx, my));
    return std::max(geometricMargin, candidate.stabilityEstimate);
}

double ConstraintChecker::estimateMinClearance(const Configuration& candidate, const EnvironmentState& env) const {
    if (env.obstacles.empty()) return 999.0;

    double minClearance = 999.0;
    const Vec3 halfModule = candidate.moduleDimensions * 0.5;

    for (const auto& t : candidate.targets) {
        const Vec3 p = t.targetPose.position;
        for (const auto& obs : env.obstacles) {
            if (isTerrainLikeObstacle(obs)) continue;
            const Vec3 halfObs = obs.size * 0.5;
            const double dx = positivePart(std::abs(p.x - obs.position.x) - halfObs.x - halfModule.x);
            const double dy = positivePart(std::abs(p.y - obs.position.y) - halfObs.y - halfModule.y);
            const double dz = positivePart(std::abs(p.z - obs.position.z) - halfObs.z - halfModule.z);
            const double d = std::sqrt(dx*dx + dy*dy + dz*dz);
            minClearance = std::min(minClearance, d);
        }
    }
    return minClearance;
}

} // namespace mrr
