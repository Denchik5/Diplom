#pragma once

#include <string>
#include "core/Configuration.h"
#include "core/EnvironmentState.h"
#include "core/MathTypes.h"
#include "core/ReconfigurationPlan.h"
#include "core/RobotGraph.h"
#include "planning/ConfigurationLibrary.h"

namespace mrr {

class ISimInterface {
public:
    virtual ~ISimInterface() = default;

    virtual bool connect() = 0;
    virtual void disconnect() = 0;
    virtual void startSimulation() = 0;
    virtual void stopSimulation() = 0;
    virtual void step() = 0;
    virtual void stepSimulation() = 0;

    virtual Pose3D getModulePose(int moduleId) const = 0;
    virtual void setModulePose(int moduleId, const Pose3D& pose) = 0;
    virtual void moveModuleInterpolated(int moduleId, const Pose3D& startPose, const Pose3D& targetPose, int steps) = 0;
    virtual void applyConfiguration(const Configuration& configuration) = 0;

    virtual RobotGraph getRobotGraph() const = 0;
    virtual EnvironmentState getEnvironmentState() const = 0;
    virtual bool hasNextEnvironment() const = 0;
    virtual void advanceEnvironment() = 0;

    virtual void executePlan(const ReconfigurationPlan& plan, const ConfigurationLibrary& library) = 0;
    virtual std::string currentConfigurationName() const = 0;
};

} // namespace mrr
