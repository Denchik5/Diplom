#include "sim/MockCoppeliaSimInterface.h"
#include <algorithm>
#include <fstream>
#include <iostream>
#include <stdexcept>

using json = nlohmann::json;

namespace mrr {

static Vec3 vec3FromJsonLocal(const json& a) {
    return Vec3(a.at(0).get<double>(), a.at(1).get<double>(), a.at(2).get<double>());
}

MockCoppeliaSimInterface::MockCoppeliaSimInterface(
    const std::string& scenarioPath,
    const ConfigurationLibrary& library
) {
    loadScenario(scenarioPath);
    const auto& initial = library.getByName("WIDE_STABLE");
    graph_ = initial.graph;
    currentConfig_ = "WIDE_STABLE";
}

bool MockCoppeliaSimInterface::connect() {
    std::cout << "Mock interface connected. Offline mode does not require CoppeliaSim.\n";
    return true;
}

void MockCoppeliaSimInterface::disconnect() {
    std::cout << "Mock interface disconnected.\n";
}

void MockCoppeliaSimInterface::startSimulation() {
    simulationTime_ = 0.0;
    std::cout << "Mock simulation started.\n";
}

void MockCoppeliaSimInterface::stopSimulation() {
    std::cout << "Mock simulation stopped at t=" << simulationTime_ << " s.\n";
}

void MockCoppeliaSimInterface::step() {
    simulationTime_ += 0.05;
}

void MockCoppeliaSimInterface::stepSimulation() {
    step();
}

Pose3D MockCoppeliaSimInterface::getModulePose(int moduleId) const {
    const Module* m = graph_.findModule(moduleId);
    if (!m) throw std::runtime_error("Unknown module id in mock interface: " + std::to_string(moduleId));
    return m->pose;
}

void MockCoppeliaSimInterface::setModulePose(int moduleId, const Pose3D& pose) {
    Module* m = graph_.findModule(moduleId);
    if (!m) throw std::runtime_error("Unknown module id in mock interface: " + std::to_string(moduleId));
    m->pose = pose;
}

void MockCoppeliaSimInterface::moveModuleInterpolated(int moduleId, const Pose3D& startPose, const Pose3D& targetPose, int steps) {
    const int n = std::max(1, steps);
    for (int k = 1; k <= n; ++k) {
        const double alpha = static_cast<double>(k) / static_cast<double>(n);
        setModulePose(moduleId, interpolatePose(startPose, targetPose, alpha));
        stepSimulation();
    }
}

void MockCoppeliaSimInterface::applyConfiguration(const Configuration& configuration) {
    RobotGraph start = graph_;
    const int interpolationSteps = 20;

    if (currentConfig_ != configuration.name) {
        for (int k = 1; k <= interpolationSteps; ++k) {
            const double alpha = static_cast<double>(k) / interpolationSteps;
            for (auto& module : graph_.modules) {
                const Module* startModule = start.findModule(module.id);
                const ModuleTarget* target = configuration.findTarget(module.id);
                if (!startModule || !target) continue;
                module.pose = interpolatePose(startModule->pose, target->targetPose, alpha);
                module.jointAngles = target->targetJointAngles;
                module.status = ModuleStatus::Moving;
            }
            stepSimulation();
        }
    }

    graph_ = configuration.graph;
    for (auto& m : graph_.modules) m.status = ModuleStatus::Connected;
    currentConfig_ = configuration.name;
}

RobotGraph MockCoppeliaSimInterface::getRobotGraph() const {
    return graph_;
}

EnvironmentState MockCoppeliaSimInterface::getEnvironmentState() const {
    if (environments_.empty()) return EnvironmentState{};
    if (envIndex_ >= environments_.size()) return environments_.back();
    return environments_.at(envIndex_);
}

bool MockCoppeliaSimInterface::hasNextEnvironment() const {
    return envIndex_ + 1 < environments_.size();
}

void MockCoppeliaSimInterface::advanceEnvironment() {
    if (hasNextEnvironment()) ++envIndex_;
}

void MockCoppeliaSimInterface::executePlan(const ReconfigurationPlan& plan, const ConfigurationLibrary& library) {
    if (!plan.success) return;
    std::cout << "Executing sequence: ";
    for (const auto& name : plan.sequence) std::cout << name << " ";
    std::cout << "\n";

    for (const auto& configName : plan.sequence) {
        if (configName == currentConfig_) continue;
        applyConfiguration(library.getByName(configName));
    }
}

std::string MockCoppeliaSimInterface::currentConfigurationName() const {
    return currentConfig_;
}

void MockCoppeliaSimInterface::loadScenario(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("Cannot open scenario file: " + path);
    json j;
    in >> j;

    if (j.contains("sequence")) {
        for (const auto& item : j.at("sequence")) environments_.push_back(parseEnvironmentObject(item));
    } else {
        environments_.push_back(parseEnvironmentObject(j));
    }
    if (environments_.empty()) throw std::runtime_error("Scenario has no environment states: " + path);
}

EnvironmentState MockCoppeliaSimInterface::parseEnvironmentObject(const json& j) {
    EnvironmentState env;
    env.scenarioId = j.value("scenario_id", std::string{"scenario"});
    env.type = environmentTypeFromString(j.value("environment_type", std::string{"UNKNOWN"}));
    env.corridorWidth = j.value("corridor_width", 999.0);
    env.stepHeight = j.value("step_height", 0.0);
    env.gapWidth = j.value("gap_width", 0.0);
    env.roughness = j.value("roughness", 0.0);
    env.targetReached = j.value("target_reached", false);
    env.maxAllowedWidth = j.value("max_allowed_width", 999.0);
    env.maxAllowedLength = j.value("max_allowed_length", 999.0);
    env.maxAllowedHeight = j.value("max_allowed_height", 999.0);
    env.requiredMinClearance = j.value("required_min_clearance", 0.005);
    env.requiredStabilityMargin = j.value("required_stability_margin", 0.02);

    if (j.contains("target_zone")) {
        const auto& tz = j.at("target_zone");
        env.targetZone.center = vec3FromJsonLocal(tz.at("center"));
        env.targetZone.radius = tz.value("radius", 0.5);
    }

    for (const auto& jo : j.value("obstacles", json::array())) {
        Obstacle o;
        o.id = jo.value("id", std::string{"obstacle"});
        o.type = jo.value("type", std::string{"BOX"});
        o.position = vec3FromJsonLocal(jo.at("position"));
        o.size = vec3FromJsonLocal(jo.at("size"));
        env.obstacles.push_back(o);
    }
    return env;
}

} // namespace mrr
