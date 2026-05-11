#pragma once

#include <nlohmann/json.hpp>
#include <string>
#include <vector>
#include "sim/ISimInterface.h"

namespace mrr {

class MockCoppeliaSimInterface : public ISimInterface {
public:
    MockCoppeliaSimInterface(const std::string& scenarioPath, const ConfigurationLibrary& library);

    bool connect() override;
    void disconnect() override;
    void startSimulation() override;
    void stopSimulation() override;
    void step() override;
    void stepSimulation() override;

    Pose3D getModulePose(int moduleId) const override;
    void setModulePose(int moduleId, const Pose3D& pose) override;
    void moveModuleInterpolated(int moduleId, const Pose3D& startPose, const Pose3D& targetPose, int steps) override;
    void applyConfiguration(const Configuration& configuration) override;

    RobotGraph getRobotGraph() const override;
    EnvironmentState getEnvironmentState() const override;
    bool hasNextEnvironment() const override;
    void advanceEnvironment() override;

    void executePlan(const ReconfigurationPlan& plan, const ConfigurationLibrary& library) override;
    std::string currentConfigurationName() const override;

private:
    void loadScenario(const std::string& path);
    static EnvironmentState parseEnvironmentObject(const nlohmann::json& j);

    std::vector<EnvironmentState> environments_;
    std::size_t envIndex_ = 0;
    RobotGraph graph_;
    std::string currentConfig_ = "WIDE_STABLE";
    double simulationTime_ = 0.0;
};

} // namespace mrr
