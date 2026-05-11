#pragma once

#include <map>
#include <string>
#include <vector>
#include "core/ReconfigurationPlan.h"
#include "planning/ConfigurationLibrary.h"
#include "planning/ConstraintChecker.h"
#include "planning/CostFunction.h"
#include "planning/EnvironmentClassifier.h"

namespace mrr {

class ReconfigurationPlanner {
public:
    ReconfigurationPlanner(
        const ConfigurationLibrary& library,
        const ConstraintChecker& checker,
        const CostFunction& costFunction
    );

    PlanningResult plan(
        const RobotGraph& currentGraph,
        const std::string& currentConfigurationName,
        const EnvironmentState& env
    ) const;

private:
    const ConfigurationLibrary& library_;
    const ConstraintChecker& checker_;
    const CostFunction& costFunction_;
    EnvironmentClassifier classifier_;

    std::map<std::string, std::vector<std::string>> transitionGraph_;

    std::vector<std::string> buildTransitionSequence(
        const std::string& current,
        const std::string& target
    ) const;
};

} // namespace mrr
