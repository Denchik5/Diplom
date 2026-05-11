#pragma once

#include <string>
#include <vector>
#include "core/Enums.h"
#include "core/MathTypes.h"

namespace mrr {

struct Obstacle {
    std::string id;
    std::string type;
    Vec3 position;
    Vec3 size;
};

struct TargetZone {
    Vec3 center;
    double radius = 0.5;
};

class EnvironmentState {
public:
    std::string scenarioId;
    EnvironmentType type = EnvironmentType::Unknown;
    std::vector<Obstacle> obstacles;
    TargetZone targetZone;

    double corridorWidth = 999.0;
    double stepHeight = 0.0;
    double gapWidth = 0.0;
    double roughness = 0.0;
    bool targetReached = false;

    double maxAllowedWidth = 999.0;
    double maxAllowedLength = 999.0;
    double maxAllowedHeight = 999.0;
    double requiredMinClearance = 0.005;
    double requiredStabilityMargin = 0.02;
};

} // namespace mrr
