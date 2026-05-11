#pragma once

#include "core/EnvironmentState.h"

namespace mrr {

class EnvironmentClassifier {
public:
    EnvironmentType classify(const EnvironmentState& env) const;
};

} // namespace mrr
