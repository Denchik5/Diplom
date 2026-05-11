#pragma once

#include <string>
#include <vector>
#include "core/Enums.h"

namespace mrr {

struct CostComponents {
    double T = 0.0;
    double E = 0.0;
    double C = 0.0;
    double R = 0.0;
    double S = 0.0;
    double J = 0.0;
};

struct Metrics {
    std::string scenarioName;
    EnvironmentType environmentType = EnvironmentType::Unknown;
    std::string selectedConfiguration;
    bool success = false;
    double reconfigurationTime = 0.0;
    int numberOfSteps = 0;
    int numberOfConnectionChanges = 0;
    double estimatedEnergy = 0.0;
    double collisionRisk = 0.0;
    double stabilityPenalty = 0.0;
    double minimumClearance = 999.0;
    double totalCost = 0.0;
};

struct ReconfigurationPlan {
    bool success = false;
    bool constraintsValid = false;
    std::string reason;
    std::string initialConfiguration;
    std::string targetConfiguration;
    std::vector<std::string> sequence;
    std::vector<std::string> constraintViolations;
    CostComponents cost;

    int estimatedStepCount = 0;
    int numberOfConnectionChanges = 0;
    double minimumClearance = 999.0;
    double stabilityMargin = 0.0;
};

using PlanningResult = ReconfigurationPlan;

} // namespace mrr
