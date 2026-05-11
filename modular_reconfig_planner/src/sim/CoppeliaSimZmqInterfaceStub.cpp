#include "sim/CoppeliaSimZmqInterface.h"
#include <algorithm>
#include <fstream>
#include <iostream>
#include <stdexcept>

using NlohmannJson = nlohmann::json;

namespace mrr {

void RemoteApiClientDeleter::operator()(RemoteAPIClient* ptr) const {
    (void)ptr;
}

namespace {

Vec3 vec3FromJsonStub(const NlohmannJson& a) {
    return Vec3(a.at(0).get<double>(), a.at(1).get<double>(), a.at(2).get<double>());
}

} // namespace

CoppeliaSimZmqInterface::CoppeliaSimZmqInterface(
    const ConfigurationLibrary& library,
    std::string scenarioPath,
    MotionSettings settings
)
    : library_(library), scenarioPath_(std::move(scenarioPath)), settings_(std::move(settings)) {
    loadScenario(scenarioPath_);
    graphCache_ = library_.getByName("WIDE_STABLE").graph;
}

CoppeliaSimZmqInterface::~CoppeliaSimZmqInterface() = default;

bool CoppeliaSimZmqInterface::connect() {
    std::cerr << "planner_coppelia was built without the real CoppeliaSim ZeroMQ Remote API client.\n"
              << "Reconfigure with -DUSE_COPPELIASIM_ZMQ=ON after copying CoppeliaSim/programming/zmqRemoteApi to external/zmqRemoteApi.\n";
    return false;
}

void CoppeliaSimZmqInterface::disconnect() {}
void CoppeliaSimZmqInterface::startSimulation() {}
void CoppeliaSimZmqInterface::stopSimulation() {}
void CoppeliaSimZmqInterface::step() {}
void CoppeliaSimZmqInterface::stepSimulation() { step(); }

Pose3D CoppeliaSimZmqInterface::getModulePose(int moduleId) const {
    const Module* m = graphCache_.findModule(moduleId);
    if (!m) throw std::runtime_error("Unknown module id: " + std::to_string(moduleId));
    return m->pose;
}

void CoppeliaSimZmqInterface::setModulePose(int moduleId, const Pose3D& pose) {
    Module* m = graphCache_.findModule(moduleId);
    if (!m) throw std::runtime_error("Unknown module id: " + std::to_string(moduleId));
    m->pose = pose;
}

void CoppeliaSimZmqInterface::moveModuleInterpolated(int moduleId, const Pose3D&, const Pose3D& targetPose, int) {
    setModulePose(moduleId, targetPose);
}

void CoppeliaSimZmqInterface::applyConfiguration(const Configuration& configuration) {
    applyConfigurationLocal(configuration);
}

void CoppeliaSimZmqInterface::validateScene(bool) {}
void CoppeliaSimZmqInterface::prepareDemoMode() {}

void CoppeliaSimZmqInterface::applyConfigurationLocal(const Configuration& configuration) {
    graphCache_ = configuration.graph;
    currentConfig_ = configuration.name;
}

void CoppeliaSimZmqInterface::moveRobotRoot(const Vec3&, int) {}
void CoppeliaSimZmqInterface::moveRobotRootToNextSegment() {}

void CoppeliaSimZmqInterface::runTestMotion(const ConfigurationLibrary& library) {
    applyConfigurationLocal(library.getByName("WIDE_STABLE"));
    applyConfigurationLocal(library.getByName("LINE"));
    applyConfigurationLocal(library.getByName("WIDE_STABLE"));
}

RobotGraph CoppeliaSimZmqInterface::getRobotGraph() const { return graphCache_; }

EnvironmentState CoppeliaSimZmqInterface::getEnvironmentState() const {
    if (environments_.empty()) return EnvironmentState{};
    if (envIndex_ >= environments_.size()) return environments_.back();
    return environments_.at(envIndex_);
}

bool CoppeliaSimZmqInterface::hasNextEnvironment() const { return envIndex_ + 1 < environments_.size(); }
void CoppeliaSimZmqInterface::advanceEnvironment() { if (hasNextEnvironment()) ++envIndex_; }

void CoppeliaSimZmqInterface::executePlan(const ReconfigurationPlan& plan, const ConfigurationLibrary& library) {
    if (!plan.success) return;
    for (const auto& configName : plan.sequence) applyConfigurationLocal(library.getByName(configName));
}

std::string CoppeliaSimZmqInterface::currentConfigurationName() const { return currentConfig_; }

void CoppeliaSimZmqInterface::loadScenario(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("Cannot open scenario file: " + path);
    NlohmannJson j;
    in >> j;
    if (j.contains("sequence")) {
        for (const auto& item : j.at("sequence")) environments_.push_back(parseEnvironmentObject(item));
    } else {
        environments_.push_back(parseEnvironmentObject(j));
    }
}

EnvironmentState CoppeliaSimZmqInterface::parseEnvironmentObject(const NlohmannJson& j) {
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
        env.targetZone.center = vec3FromJsonStub(tz.at("center"));
        env.targetZone.radius = tz.value("radius", 0.5);
    }
    for (const auto& jo : j.value("obstacles", NlohmannJson::array())) {
        Obstacle o;
        o.id = jo.value("id", std::string{"obstacle"});
        o.type = jo.value("type", std::string{"BOX"});
        o.position = vec3FromJsonStub(jo.at("position"));
        o.size = vec3FromJsonStub(jo.at("size"));
        env.obstacles.push_back(o);
    }
    return env;
}

void CoppeliaSimZmqInterface::cacheSceneHandles() {}
int CoppeliaSimZmqInterface::getCachedModuleHandle(int moduleId) const { return moduleId; }
std::vector<Pose3D> CoppeliaSimZmqInterface::readModulePosesLocal() const { return {}; }
void CoppeliaSimZmqInterface::setModulePoseLocal(int moduleId, const Pose3D& pose) { setModulePose(moduleId, pose); }
int CoppeliaSimZmqInterface::interpolationStepsFor(const std::vector<Pose3D>&, const std::vector<Pose3D>&) const { return settings_.interpolationSteps; }
void CoppeliaSimZmqInterface::interpolateAllModules(const std::vector<Pose3D>&, const std::vector<Pose3D>&, const RobotGraph&, int) {}
void CoppeliaSimZmqInterface::applyJointTargets(const Configuration&) {}
void CoppeliaSimZmqInterface::updateVisualConnectors(const RobotGraph&) {}
void CoppeliaSimZmqInterface::ensureConnectorObjects(int) {}
void CoppeliaSimZmqInterface::hideUnusedConnectors(std::size_t) {}
void CoppeliaSimZmqInterface::sleepAfterStep() const {}
double CoppeliaSimZmqInterface::smoothStep(double alpha) { return alpha * alpha * (3.0 - 2.0 * alpha); }
Vec3 CoppeliaSimZmqInterface::connectorEulerFromDelta(const Vec3&) { return Vec3{}; }

} // namespace mrr
