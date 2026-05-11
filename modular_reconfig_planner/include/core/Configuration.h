#pragma once

#include <string>
#include <vector>
#include "core/EnvironmentState.h"
#include "core/RobotGraph.h"

namespace mrr {

struct ModuleTarget {
    int moduleId = -1;
    Pose3D targetPose;
    std::vector<double> targetJointAngles;
};

class Configuration {
public:
    std::string name;
    std::string description;
    int moduleCount = 0;

    Vec3 moduleDimensions{0.22, 0.18, 0.10}; // length, width, height, meters
    RobotGraph graph;
    std::vector<ModuleTarget> targets;

    double maxWidth = 999.0;
    double maxLength = 999.0;
    double maxHeightLimit = 999.0;
    double minStabilityMargin = 0.02;
    double stabilityEstimate = 0.05;
    double maxJointAngle = 1.57;
    bool allowDisconnectedGraph = false;

    std::vector<EnvironmentType> applicableTypes;

    const ModuleTarget* findTarget(int moduleId) const;
    double estimatedWidth() const;
    double estimatedLength() const;
    double estimatedHeight() const;
    double moduleBoundingRadius() const;
};

} // namespace mrr
