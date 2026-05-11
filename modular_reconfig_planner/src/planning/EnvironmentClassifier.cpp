#include "planning/EnvironmentClassifier.h"

namespace mrr {

EnvironmentType EnvironmentClassifier::classify(const EnvironmentState& env) const {
    if (env.targetReached) return EnvironmentType::TargetZone;
    if (env.type != EnvironmentType::Unknown) return env.type;
    if (env.gapWidth > 0.10) return EnvironmentType::Gap;
    if (env.stepHeight > 0.08) return EnvironmentType::Step;
    if (env.corridorWidth < 0.55) return EnvironmentType::NarrowPassage;
    if (env.roughness > 0.03) return EnvironmentType::RoughSurface;
    return EnvironmentType::Flat;
}

} // namespace mrr
