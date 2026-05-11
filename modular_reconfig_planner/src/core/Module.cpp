#include "core/Module.h"

namespace mrr {

Module::Module(int id_) : id(id_) {
    connectors = {
        {"FRONT", true, -1}, {"BACK", true, -1},
        {"LEFT", true, -1},  {"RIGHT", true, -1},
        {"TOP", true, -1},   {"BOTTOM", true, -1}
    };
    jointAngles = {0.0, 0.0};
    jointLimits = {{-1.57, 1.57}, {-1.57, 1.57}};
}

bool Module::isJointStateValid() const {
    if (jointAngles.size() != jointLimits.size()) return false;
    for (std::size_t i = 0; i < jointAngles.size(); ++i) {
        if (jointAngles[i] < jointLimits[i].minAngle || jointAngles[i] > jointLimits[i].maxAngle) {
            return false;
        }
    }
    return true;
}

ModuleState Module::stateSnapshot() const {
    return ModuleState{id, pose, jointAngles, status};
}

} // namespace mrr
