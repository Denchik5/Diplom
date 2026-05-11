#pragma once

#include <map>
#include <string>
#include <vector>
#include "core/Configuration.h"
#include "core/EnvironmentState.h"

namespace mrr {

struct ConstraintReport {
    bool valid = true;
    bool currentGraphConnected = true;
    bool targetGraphConnected = true;
    bool jointLimitsOk = true;
    bool collisionFree = true;
    bool stable = true;
    bool geometryOk = true;
    bool dimensionLimitsOk = true;

    double stabilityMargin = 0.0;
    double requiredStabilityMargin = 0.0;
    double minClearance = 999.0;
    double requiredMinClearance = 0.005;
    double width = 0.0;
    double length = 0.0;
    double height = 0.0;

    std::vector<std::string> violations;
    std::map<std::string, double> values;
    std::string reason;
};

class ConstraintChecker {
public:
    ConstraintReport check(
        const RobotGraph& currentGraph,
        const Configuration& candidate,
        const EnvironmentState& env
    ) const;

private:
    bool checkJointLimits(const Configuration& candidate) const;
    bool checkGeometry(const Configuration& candidate, const EnvironmentState& env, ConstraintReport& report) const;
    bool checkDimensionLimits(const Configuration& candidate, const EnvironmentState& env, ConstraintReport& report) const;
    double computeStabilityMargin(const Configuration& candidate) const;
    double estimateMinClearance(const Configuration& candidate, const EnvironmentState& env) const;
};

} // namespace mrr
