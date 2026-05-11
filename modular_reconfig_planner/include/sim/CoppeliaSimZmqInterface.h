#pragma once

#include <nlohmann/json.hpp>
#include <memory>
#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include "sim/ISimInterface.h"

class RemoteAPIClient;

namespace mrr {

struct RemoteApiClientDeleter {
    void operator()(RemoteAPIClient* ptr) const;
};

struct MotionSettings {
    std::string motionMode = "rigid_visual_reconfiguration";
    int interpolationSteps = 180;
    int stepPauseMs = 0;
    bool useSmoothStep = true;
    bool useSafeLift = false;
    double safeLiftHeight = 0.08;
    bool updateConnectors = true;
    bool disableModuleDynamics = true;
    bool disableModuleRespondable = true;
    bool moveRobotRootBetweenSegments = true;
    double rootAdvancePerSegment = 0.65;
    double connectorRadius = 0.018;
    int connectorCount = 16;
};

inline MotionSettings defaultMotionSettings() {
    return MotionSettings{};
}

inline MotionSettings loadMotionSettings(const std::string& path) {
    MotionSettings s;
    if (path.empty()) return s;

    std::ifstream in(path);
    if (!in) {
        std::cerr << "Warning: motion settings file not found: " << path
                  << ". Built-in demo motion settings will be used.\n";
        return s;
    }

    nlohmann::json j;
    in >> j;
    s.motionMode = j.value("motion_mode", s.motionMode);
    s.interpolationSteps = j.value("interpolation_steps", s.interpolationSteps);
    s.stepPauseMs = j.value("step_pause_ms", s.stepPauseMs);
    s.useSmoothStep = j.value("use_smooth_step", s.useSmoothStep);
    s.useSafeLift = j.value("use_safe_lift", s.useSafeLift);
    s.safeLiftHeight = j.value("safe_lift_height", s.safeLiftHeight);
    s.updateConnectors = j.value("update_connectors", s.updateConnectors);
    s.disableModuleDynamics = j.value("disable_module_dynamics", s.disableModuleDynamics);
    s.disableModuleRespondable = j.value("disable_module_respondable", s.disableModuleRespondable);
    s.moveRobotRootBetweenSegments = j.value("move_robot_root_between_segments", s.moveRobotRootBetweenSegments);
    s.rootAdvancePerSegment = j.value("root_advance_per_segment", s.rootAdvancePerSegment);
    s.connectorRadius = j.value("connector_radius", s.connectorRadius);
    s.connectorCount = j.value("connector_count", s.connectorCount);
    return s;
}

class CoppeliaSimZmqInterface : public ISimInterface {
public:
    CoppeliaSimZmqInterface(
        const ConfigurationLibrary& library,
        std::string scenarioPath,
        MotionSettings settings = defaultMotionSettings()
    );
    ~CoppeliaSimZmqInterface() override;

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

    void validateScene(bool requireDemoObjects = true);
    void prepareDemoMode();
    void applyConfigurationLocal(const Configuration& configuration);
    void moveRobotRoot(const Vec3& delta, int steps);
    void moveRobotRootToNextSegment();
    void runTestMotion(const ConfigurationLibrary& library);
    const MotionSettings& motionSettings() const { return settings_; }

    RobotGraph getRobotGraph() const override;
    EnvironmentState getEnvironmentState() const override;
    bool hasNextEnvironment() const override;
    void advanceEnvironment() override;

    void executePlan(const ReconfigurationPlan& plan, const ConfigurationLibrary& library) override;
    std::string currentConfigurationName() const override;

private:
    void loadScenario(const std::string& path);
    static EnvironmentState parseEnvironmentObject(const nlohmann::json& j);

    void cacheSceneHandles();
    int getCachedModuleHandle(int moduleId) const;
    std::vector<Pose3D> readModulePosesLocal() const;
    void setModulePoseLocal(int moduleId, const Pose3D& pose);
    int interpolationStepsFor(const std::vector<Pose3D>& start, const std::vector<Pose3D>& target) const;
    void interpolateAllModules(
        const std::vector<Pose3D>& start,
        const std::vector<Pose3D>& target,
        const RobotGraph& connectorGraph,
        int steps
    );
    void applyJointTargets(const Configuration& configuration);
    void updateVisualConnectors(const RobotGraph& graph);
    void ensureConnectorObjects(int requiredCount);
    void hideUnusedConnectors(std::size_t usedCount);
    void sleepAfterStep() const;
    static double smoothStep(double alpha);
    static Vec3 connectorEulerFromDelta(const Vec3& delta);

    const ConfigurationLibrary& library_;
    std::string scenarioPath_;
    MotionSettings settings_;
    std::vector<EnvironmentState> environments_;
    std::size_t envIndex_ = 0;
    mutable RobotGraph graphCache_;
    std::string currentConfig_ = "WIDE_STABLE";
    int moduleCount_ = 8;

    std::unique_ptr<RemoteAPIClient, RemoteApiClientDeleter> client_;
    int robotHandle_ = -1;
    int connectorsRootHandle_ = -1;
    std::vector<int> moduleHandles_;
    std::vector<int> connectorHandles_;
    std::vector<double> connectorLengths_;
};

} // namespace mrr
