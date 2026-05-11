#pragma once

#include <string>
#include <vector>
#include "core/Enums.h"
#include "core/MathTypes.h"

namespace mrr {

struct Connector {
    std::string name;
    bool available = true;
    int connectedModuleId = -1;
};

struct JointLimit {
    double minAngle = -1.57;
    double maxAngle =  1.57;
};

struct ModuleState {
    int id = -1;
    Pose3D pose;
    std::vector<double> jointAngles;
    ModuleStatus status = ModuleStatus::Connected;
};

class Module {
public:
    int id = -1;
    Pose3D pose;
    std::vector<int> neighbors;
    std::vector<Connector> connectors;
    std::vector<double> jointAngles;
    std::vector<JointLimit> jointLimits;
    ModuleStatus status = ModuleStatus::Connected;

    Module() = default;
    explicit Module(int id_);

    bool isJointStateValid() const;
    ModuleState stateSnapshot() const;
};

} // namespace mrr
