#pragma once

#include <string>
#include "core/Configuration.h"
#include "core/EnvironmentState.h"
#include "core/ReconfigurationPlan.h"
#include "core/RobotGraph.h"
#include "planning/ConstraintChecker.h"

namespace mrr {

struct CostWeights {
    double wT = 0.30;
    double wE = 0.15;
    double wC = 0.15;
    double wR = 0.25;
    double wS = 0.15;
};

class CostFunction {
public:
    CostFunction() = default;
    explicit CostFunction(const CostWeights& weights);

    CostComponents evaluate(
        const RobotGraph& current,
        const Configuration& target,
        const EnvironmentState& env,
        const ConstraintReport& report
    ) const;

    const CostWeights& weights() const { return weights_; }

private:
    CostWeights weights_;
};

CostWeights loadCostWeights(const std::string& path);

} // namespace mrr
