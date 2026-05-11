#include "planning/ReconfigurationPlanner.h"
#include <algorithm>
#include <iostream>
#include <limits>
#include <map>
#include <queue>
#include <set>

namespace mrr {

ReconfigurationPlanner::ReconfigurationPlanner(
    const ConfigurationLibrary& library,
    const ConstraintChecker& checker,
    const CostFunction& costFunction
) : library_(library), checker_(checker), costFunction_(costFunction) {
    transitionGraph_["WIDE_STABLE"] = {"LINE", "SNAKE", "CLIMB", "BRIDGE", "MANIPULATOR"};
    transitionGraph_["LINE"] = {"WIDE_STABLE", "SNAKE", "BRIDGE", "CLIMB"};
    transitionGraph_["SNAKE"] = {"WIDE_STABLE", "LINE", "CLIMB", "BRIDGE"};
    transitionGraph_["CLIMB"] = {"SNAKE", "WIDE_STABLE", "BRIDGE"};
    transitionGraph_["BRIDGE"] = {"LINE", "WIDE_STABLE", "CLIMB", "MANIPULATOR"};
    transitionGraph_["MANIPULATOR"] = {"WIDE_STABLE", "BRIDGE"};
}

PlanningResult ReconfigurationPlanner::plan(
    const RobotGraph& currentGraph,
    const std::string& currentConfigurationName,
    const EnvironmentState& env
) const {
    PlanningResult best;
    best.initialConfiguration = currentConfigurationName;

    const EnvironmentType detected = classifier_.classify(env);
    auto candidates = library_.getCandidates(detected);

    double bestJ = std::numeric_limits<double>::infinity();
    bool sawCandidate = false;
    std::vector<std::string> rejectedReasons;

    std::cout << "Detected environment: " << toString(detected)
              << ", preferred configuration: " << preferredConfigurationFor(detected) << "\n";

    for (const auto& candidate : candidates) {
        sawCandidate = true;
        auto report = checker_.check(currentGraph, candidate, env);
        if (!report.valid) {
            std::cout << "  Candidate " << candidate.name << " rejected: " << report.reason << "\n";
            rejectedReasons.push_back(candidate.name + ": " + report.reason);
            continue;
        }

        auto cost = costFunction_.evaluate(currentGraph, candidate, env, report);
        std::cout << "  Candidate " << candidate.name << " accepted, J=" << cost.J << "\n";
        if (cost.J < bestJ) {
            bestJ = cost.J;
            best.success = true;
            best.constraintsValid = true;
            best.reason = "ok";
            best.targetConfiguration = candidate.name;
            best.cost = cost;
            best.numberOfConnectionChanges = static_cast<int>(cost.C);
            best.minimumClearance = report.minClearance;
            best.stabilityMargin = report.stabilityMargin;
            best.constraintViolations.clear();
        }
    }

    if (!best.success) {
        best.constraintsValid = false;
        best.reason = "No valid target configuration found for environment " + toString(detected);
        if (!sawCandidate) best.reason += "; configuration library returned no candidates";
        for (const auto& reason : rejectedReasons) best.constraintViolations.push_back(reason);
        return best;
    }

    best.sequence = buildTransitionSequence(currentConfigurationName, best.targetConfiguration);
    if (best.sequence.empty()) best.sequence = {best.targetConfiguration};

    const int transitionCount = (currentConfigurationName == best.targetConfiguration)
        ? 0
        : std::max(1, static_cast<int>(best.sequence.size()) - 1);
    best.estimatedStepCount = transitionCount * 20;
    return best;
}

std::vector<std::string> ReconfigurationPlanner::buildTransitionSequence(
    const std::string& current,
    const std::string& target
) const {
    if (current == target) return {target};

    std::queue<std::string> q;
    std::set<std::string> visited;
    std::map<std::string, std::string> parent;

    q.push(current);
    visited.insert(current);

    while (!q.empty()) {
        const auto u = q.front();
        q.pop();
        auto it = transitionGraph_.find(u);
        if (it == transitionGraph_.end()) continue;

        for (const auto& v : it->second) {
            if (visited.count(v)) continue;
            visited.insert(v);
            parent[v] = u;

            if (v == target) {
                std::vector<std::string> path;
                std::string x = target;
                while (true) {
                    path.push_back(x);
                    if (x == current) break;
                    x = parent[x];
                }
                std::reverse(path.begin(), path.end());
                return path;
            }
            q.push(v);
        }
    }
    return {current, target};
}

} // namespace mrr
