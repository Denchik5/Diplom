#include "planning/CostFunction.h"
#include <nlohmann/json.hpp>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>

using json = nlohmann::json;

namespace mrr {

CostFunction::CostFunction(const CostWeights& weights) : weights_(weights) {}

CostComponents CostFunction::evaluate(
    const RobotGraph& current,
    const Configuration& target,
    const EnvironmentState& env,
    const ConstraintReport& report
) const {
    CostComponents c;

    double totalDistance = 0.0;
    double maxModuleDistance = 0.0;
    double totalAngle = 0.0;

    for (const auto& m : current.modules) {
        const ModuleTarget* t = target.findTarget(m.id);
        if (!t) continue;

        const double d = distance(m.pose.position, t->targetPose.position);
        totalDistance += d;
        maxModuleDistance = std::max(maxModuleDistance, d);

        const std::size_t n = std::min(m.jointAngles.size(), t->targetJointAngles.size());
        for (std::size_t i = 0; i < n; ++i) totalAngle += std::abs(m.jointAngles[i] - t->targetJointAngles[i]);
    }

    const int connectionChanges = current.edgeEditDistance(target.graph);
    c.C = static_cast<double>(connectionChanges);

    if (totalDistance < 1e-9 && totalAngle < 1e-9 && connectionChanges == 0) {
        c.T = 0.0;
        c.E = 0.0;
    } else {
        c.T = 0.5 + 0.8 * maxModuleDistance + 0.25 * totalAngle + 0.15 * connectionChanges;
        c.E = 6.0 * totalDistance + 1.5 * totalAngle + 2.0 * connectionChanges;
    }

    const double safeClearance = std::max(0.20, env.requiredMinClearance * 4.0);
    if (report.minClearance >= safeClearance) c.R = 0.0;
    else if (report.minClearance <= env.requiredMinClearance) c.R = 1.0;
    else c.R = (safeClearance - report.minClearance) / (safeClearance - env.requiredMinClearance);
    c.R = std::clamp(c.R, 0.0, 1.0);

    const double requiredStability = std::max(target.minStabilityMargin, env.requiredStabilityMargin);
    if (requiredStability <= 1e-9) c.S = 0.0;
    else c.S = std::clamp((requiredStability - report.stabilityMargin) / requiredStability, 0.0, 1.0);

    c.J = weights_.wT * c.T + weights_.wE * c.E + weights_.wC * c.C + weights_.wR * c.R + weights_.wS * c.S;
    return c;
}

CostWeights loadCostWeights(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("Cannot open weights file: " + path);
    json j;
    in >> j;

    CostWeights w;
    if (j.contains("weights")) {
        const auto& jw = j.at("weights");
        w.wT = jw.value("wT", jw.value("time", w.wT));
        w.wE = jw.value("wE", jw.value("energy", w.wE));
        w.wC = jw.value("wC", jw.value("connection_changes", w.wC));
        w.wR = jw.value("wR", jw.value("collision_risk", w.wR));
        w.wS = jw.value("wS", jw.value("stability_penalty", w.wS));
    }
    return w;
}

} // namespace mrr
