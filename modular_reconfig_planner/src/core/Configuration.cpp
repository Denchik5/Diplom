#include "core/Configuration.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace mrr {

const ModuleTarget* Configuration::findTarget(int moduleId) const {
    for (const auto& t : targets) if (t.moduleId == moduleId) return &t;
    return nullptr;
}

double Configuration::estimatedWidth() const {
    if (targets.empty()) return 0.0;
    double minY = std::numeric_limits<double>::infinity();
    double maxY = -std::numeric_limits<double>::infinity();
    for (const auto& t : targets) {
        minY = std::min(minY, t.targetPose.position.y - moduleDimensions.y / 2.0);
        maxY = std::max(maxY, t.targetPose.position.y + moduleDimensions.y / 2.0);
    }
    return maxY - minY;
}

double Configuration::estimatedLength() const {
    if (targets.empty()) return 0.0;
    double minX = std::numeric_limits<double>::infinity();
    double maxX = -std::numeric_limits<double>::infinity();
    for (const auto& t : targets) {
        minX = std::min(minX, t.targetPose.position.x - moduleDimensions.x / 2.0);
        maxX = std::max(maxX, t.targetPose.position.x + moduleDimensions.x / 2.0);
    }
    return maxX - minX;
}

double Configuration::estimatedHeight() const {
    if (targets.empty()) return 0.0;
    double maxZ = -std::numeric_limits<double>::infinity();
    for (const auto& t : targets) {
        maxZ = std::max(maxZ, t.targetPose.position.z + moduleDimensions.z / 2.0);
    }
    return maxZ;
}

double Configuration::moduleBoundingRadius() const {
    return 0.5 * std::sqrt(
        moduleDimensions.x * moduleDimensions.x +
        moduleDimensions.y * moduleDimensions.y +
        moduleDimensions.z * moduleDimensions.z
    );
}

} // namespace mrr
