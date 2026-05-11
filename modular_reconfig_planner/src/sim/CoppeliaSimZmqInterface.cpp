#include "sim/CoppeliaSimZmqInterface.h"
#include "RemoteAPIClient.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <thread>

using NlohmannJson = nlohmann::json;

namespace mrr {

void RemoteApiClientDeleter::operator()(RemoteAPIClient* ptr) const {
    delete ptr;
}

namespace {

Vec3 vec3FromJsonZmq(const NlohmannJson& a) {
    return Vec3(a.at(0).get<double>(), a.at(1).get<double>(), a.at(2).get<double>());
}

std::string modulePath(int moduleId) {
    return "/Robot/Module_" + std::to_string(moduleId);
}

std::string connectorPath(int connectorId) {
    return "/Robot/Connectors/Connector_" + std::to_string(connectorId);
}

std::vector<double> toRemoteVector(const Vec3& v) {
    return {v.x, v.y, v.z};
}

std::vector<double> toRemotePoseVector(const Pose3D& p) {
    return {p.position.x, p.position.y, p.position.z, p.rpy.x, p.rpy.y, p.rpy.z};
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
    moduleHandles_.assign(moduleCount_, -1);
}

CoppeliaSimZmqInterface::~CoppeliaSimZmqInterface() = default;

bool CoppeliaSimZmqInterface::connect() {
    std::cout << "Connecting to CoppeliaSim ZeroMQ Remote API...\n";
    try {
        client_.reset(new RemoteAPIClient());
        auto sim = client_->getObject().sim();
        const double t = sim.getSimulationTime();
        cacheSceneHandles();
        std::cout << "Connected. Simulation time = " << t << "\n";
        return true;
    } catch (const std::exception& e) {
        std::cerr << "ZeroMQ connection failed: " << e.what() << "\n";
        client_.reset();
        return false;
    }
}

void CoppeliaSimZmqInterface::disconnect() {
    client_.reset();
    std::cout << "CoppeliaSim ZeroMQ interface disconnected.\n";
}

void CoppeliaSimZmqInterface::startSimulation() {
    if (!client_) throw std::runtime_error("CoppeliaSim interface is not connected");
    auto sim = client_->getObject().sim();
    sim.setStepping(true);
    sim.startSimulation();
}

void CoppeliaSimZmqInterface::stopSimulation() {
    if (!client_) return;
    try {
        auto sim = client_->getObject().sim();
        sim.stopSimulation();
    } catch (const std::exception& e) {
        std::cerr << "Warning: could not stop CoppeliaSim simulation cleanly: " << e.what() << "\n";
    }
}

void CoppeliaSimZmqInterface::step() {
    if (!client_) throw std::runtime_error("CoppeliaSim interface is not connected");
    auto sim = client_->getObject().sim();
    sim.step();
    sleepAfterStep();
}

void CoppeliaSimZmqInterface::stepSimulation() {
    step();
}

void CoppeliaSimZmqInterface::cacheSceneHandles() {
    if (!client_) throw std::runtime_error("CoppeliaSim interface is not connected");
    auto sim = client_->getObject().sim();

    robotHandle_ = sim.getObject("/Robot");
    moduleHandles_.assign(moduleCount_, -1);
    for (int i = 0; i < moduleCount_; ++i) {
        moduleHandles_[i] = sim.getObject(modulePath(i));
    }

    try {
        connectorsRootHandle_ = sim.getObject("/Robot/Connectors");
    } catch (...) {
        connectorsRootHandle_ = -1;
    }
}

int CoppeliaSimZmqInterface::getCachedModuleHandle(int moduleId) const {
    if (moduleId < 0 || moduleId >= moduleCount_) {
        throw std::runtime_error("Invalid module id: " + std::to_string(moduleId));
    }
    if (moduleId >= static_cast<int>(moduleHandles_.size()) || moduleHandles_[moduleId] < 0) {
        throw std::runtime_error("Module handle is not cached for /Robot/Module_" + std::to_string(moduleId));
    }
    return moduleHandles_[moduleId];
}

void CoppeliaSimZmqInterface::validateScene(bool requireDemoObjects) {
    if (!client_) throw std::runtime_error("CoppeliaSim interface is not connected");
    auto sim = client_->getObject().sim();

    std::vector<std::string> errors;
    auto requireObject = [&](const std::string& path, const std::string& hint) -> int {
        try {
            return sim.getObject(path);
        } catch (const std::exception&) {
            errors.push_back("Missing object " + path + ". " + hint);
            return -1;
        }
    };

    robotHandle_ = requireObject("/Robot", "Create a root dummy named Robot at the scene root.");
    for (int i = 0; i < moduleCount_; ++i) {
        const std::string path = modulePath(i);
        const int h = requireObject(path, "Create Module_" + std::to_string(i) + " as a direct child of /Robot.");
        if (h >= 0 && robotHandle_ >= 0) {
            try {
                const int parent = sim.getObjectParent(h);
                if (parent != robotHandle_) {
                    errors.push_back(path + " exists but is not a direct child of /Robot. Drag it under Robot in the scene tree.");
                }
            } catch (...) {
                errors.push_back("Could not verify parent of " + path + ". Make sure it is a direct child of /Robot.");
            }
        }
    }

    if (requireDemoObjects) {
        if (settings_.updateConnectors) {
            connectorsRootHandle_ = requireObject(
                "/Robot/Connectors",
                "Create a dummy /Robot/Connectors for visual connector rods."
            );
        }
        requireObject("/Obstacles", "Create a dummy /Obstacles for obstacle visualization.");
        requireObject("/TargetZone", "Create a visible /TargetZone object or dummy.");
    }

    if (!errors.empty()) {
        std::ostringstream oss;
        oss << "CoppeliaSim scene validation failed:\n";
        for (const auto& e : errors) oss << "  - " << e << "\n";
        oss << "Fix the scene according to docs/coppeliasim_stable_scene_setup.md and run again.\n";
        throw std::runtime_error(oss.str());
    }

    cacheSceneHandles();
    if (settings_.updateConnectors && connectorsRootHandle_ >= 0) {
        ensureConnectorObjects(std::max(settings_.connectorCount, 16));
    }
    std::cout << "CoppeliaSim scene validation passed.\n";
}

void CoppeliaSimZmqInterface::prepareDemoMode() {
    if (!client_) throw std::runtime_error("CoppeliaSim interface is not connected");
    auto sim = client_->getObject().sim();

    if (!settings_.disableModuleDynamics && !settings_.disableModuleRespondable) return;

    std::cout << "Preparing kinematic demo mode for modules...\n";
    for (int i = 0; i < moduleCount_; ++i) {
        const int h = getCachedModuleHandle(i);
        if (settings_.disableModuleDynamics) {
            try {
                sim.setObjectInt32Param(h, sim.shapeintparam_static, 1);
            } catch (const std::exception& e) {
                std::cerr << "Warning: could not set Module_" << i << " to static/non-dynamic via Remote API: "
                          << e.what() << ". Disable Dynamic manually in CoppeliaSim.\n";
            }
        }
        if (settings_.disableModuleRespondable) {
            try {
                sim.setObjectInt32Param(h, sim.shapeintparam_respondable, 0);
            } catch (const std::exception& e) {
                std::cerr << "Warning: could not disable Respondable for Module_" << i << " via Remote API: "
                          << e.what() << ". Disable Respondable manually in CoppeliaSim.\n";
            }
        }
    }
}

Pose3D CoppeliaSimZmqInterface::getModulePose(int moduleId) const {
    if (!client_) throw std::runtime_error("CoppeliaSim interface is not connected");
    auto sim = client_->getObject().sim();
    const int h = getCachedModuleHandle(moduleId);
    const int rel = robotHandle_ >= 0 ? robotHandle_ : sim.handle_world;
    auto p = sim.getObjectPosition(h, rel);
    auto r = sim.getObjectOrientation(h, rel);
    Pose3D pose;
    if (p.size() >= 3) pose.position = Vec3(p[0], p[1], p[2]);
    if (r.size() >= 3) pose.rpy = Vec3(r[0], r[1], r[2]);
    return pose;
}

void CoppeliaSimZmqInterface::setModulePose(int moduleId, const Pose3D& pose) {
    setModulePoseLocal(moduleId, pose);
}

void CoppeliaSimZmqInterface::setModulePoseLocal(int moduleId, const Pose3D& pose) {
    if (!client_) throw std::runtime_error("CoppeliaSim interface is not connected");
    auto sim = client_->getObject().sim();
    const int h = getCachedModuleHandle(moduleId);
    const int rel = robotHandle_ >= 0 ? robotHandle_ : sim.handle_world;

    sim.setObjectPosition(h, rel, toRemoteVector(pose.position));
    sim.setObjectOrientation(h, rel, toRemoteVector(pose.rpy));

    if (Module* m = graphCache_.findModule(moduleId)) m->pose = pose;
}

void CoppeliaSimZmqInterface::moveModuleInterpolated(
    int moduleId,
    const Pose3D& startPose,
    const Pose3D& targetPose,
    int steps
) {
    const int n = std::max(120, steps);
    for (int k = 1; k <= n; ++k) {
        double alpha = static_cast<double>(k) / static_cast<double>(n);
        if (settings_.useSmoothStep) alpha = smoothStep(alpha);
        setModulePoseLocal(moduleId, interpolatePose(startPose, targetPose, alpha));
        updateVisualConnectors(graphCache_);
        stepSimulation();
    }
}

std::vector<Pose3D> CoppeliaSimZmqInterface::readModulePosesLocal() const {
    std::vector<Pose3D> out;
    out.reserve(moduleCount_);
    for (int i = 0; i < moduleCount_; ++i) out.push_back(getModulePose(i));
    return out;
}

int CoppeliaSimZmqInterface::interpolationStepsFor(
    const std::vector<Pose3D>& start,
    const std::vector<Pose3D>& target
) const {
    double maxDistance = 0.0;
    const std::size_t n = std::min(start.size(), target.size());
    for (std::size_t i = 0; i < n; ++i) {
        maxDistance = std::max(maxDistance, distance(start[i].position, target[i].position));
    }

    int steps = std::max(120, settings_.interpolationSteps);
    if (maxDistance > 0.70) steps = std::max(steps, 240);
    else if (maxDistance > 0.45) steps = std::max(steps, 210);
    else if (maxDistance > 0.25) steps = std::max(steps, 180);
    return steps;
}

void CoppeliaSimZmqInterface::interpolateAllModules(
    const std::vector<Pose3D>& start,
    const std::vector<Pose3D>& target,
    const RobotGraph& connectorGraph,
    int steps
) {
    const int n = std::max(120, steps);
    for (int k = 1; k <= n; ++k) {
        double alpha = static_cast<double>(k) / static_cast<double>(n);
        if (settings_.useSmoothStep) alpha = smoothStep(alpha);

        for (int i = 0; i < moduleCount_; ++i) {
            setModulePoseLocal(i, interpolatePose(start.at(i), target.at(i), alpha));
        }
        updateVisualConnectors(connectorGraph);
        stepSimulation();
    }
}

void CoppeliaSimZmqInterface::applyConfiguration(const Configuration& configuration) {
    applyConfigurationLocal(configuration);
}

void CoppeliaSimZmqInterface::applyConfigurationLocal(const Configuration& configuration) {
    std::vector<Pose3D> start = readModulePosesLocal();
    std::vector<Pose3D> target(moduleCount_);
    for (int i = 0; i < moduleCount_; ++i) {
        const ModuleTarget* mt = configuration.findTarget(i);
        if (!mt) throw std::runtime_error("Target configuration " + configuration.name + " has no Module_" + std::to_string(i));
        target[i] = mt->targetPose;
    }

    const int steps = interpolationStepsFor(start, target);
    std::cout << "Applying local configuration " << currentConfig_ << " -> " << configuration.name
              << " using " << steps << " interpolation steps.\n";

    if (settings_.useSafeLift) {
        std::vector<Pose3D> liftedStart = start;
        std::vector<Pose3D> liftedTarget = target;
        for (int i = 0; i < moduleCount_; ++i) {
            const double safeZ = std::max(start[i].position.z, target[i].position.z) + settings_.safeLiftHeight;
            liftedStart[i].position.z = safeZ;
            liftedTarget[i].position.z = safeZ;
        }
        interpolateAllModules(start, liftedStart, configuration.graph, std::max(60, steps / 3));
        interpolateAllModules(liftedStart, liftedTarget, configuration.graph, steps);
        interpolateAllModules(liftedTarget, target, configuration.graph, std::max(60, steps / 3));
    } else {
        interpolateAllModules(start, target, configuration.graph, steps);
    }

    applyJointTargets(configuration);
    graphCache_ = configuration.graph;
    currentConfig_ = configuration.name;
    updateVisualConnectors(graphCache_);
}

void CoppeliaSimZmqInterface::applyJointTargets(const Configuration& configuration) {
    if (!client_) return;
    auto sim = client_->getObject().sim();
    for (int i = 0; i < moduleCount_; ++i) {
        const ModuleTarget* target = configuration.findTarget(i);
        if (!target) continue;
        for (std::size_t j = 0; j < target->targetJointAngles.size(); ++j) {
            const std::string jointPath = "/Robot/Module_" + std::to_string(i) + "/joint_" + std::to_string(j);
            try {
                const int jh = sim.getObject(jointPath);
                sim.setJointTargetPosition(jh, target->targetJointAngles[j]);
            } catch (...) {
                // Simplified SMORES-like demo modules normally do not contain internal joints.
            }
        }
    }
}

void CoppeliaSimZmqInterface::moveRobotRoot(const Vec3& delta, int steps) {
    if (!client_ || robotHandle_ < 0) return;
    auto sim = client_->getObject().sim();
    auto p0 = sim.getObjectPosition(robotHandle_, sim.handle_world);
    if (p0.size() < 3) return;

    const Vec3 start(p0[0], p0[1], p0[2]);
    const Vec3 target = start + delta;
    const int n = std::max(120, steps);
    std::cout << "Moving /Robot root by " << delta << " using " << n << " interpolation steps.\n";

    for (int k = 1; k <= n; ++k) {
        double alpha = static_cast<double>(k) / static_cast<double>(n);
        if (settings_.useSmoothStep) alpha = smoothStep(alpha);
        const Vec3 p = lerp(start, target, alpha);
        sim.setObjectPosition(robotHandle_, sim.handle_world, toRemoteVector(p));
        stepSimulation();
    }
}

void CoppeliaSimZmqInterface::moveRobotRootToNextSegment() {
    if (!settings_.moveRobotRootBetweenSegments) return;
    moveRobotRoot(Vec3(settings_.rootAdvancePerSegment, 0.0, 0.0), std::max(120, settings_.interpolationSteps / 2));
}

RobotGraph CoppeliaSimZmqInterface::getRobotGraph() const {
    RobotGraph g = graphCache_;
    for (auto& m : g.modules) {
        try {
            m.pose = getModulePose(m.id);
        } catch (const std::exception& e) {
            std::cerr << "Warning: could not read /Robot/Module_" << m.id << ": " << e.what() << "\n";
        }
    }
    return g;
}

EnvironmentState CoppeliaSimZmqInterface::getEnvironmentState() const {
    if (environments_.empty()) return EnvironmentState{};
    if (envIndex_ >= environments_.size()) return environments_.back();
    return environments_.at(envIndex_);
}

bool CoppeliaSimZmqInterface::hasNextEnvironment() const {
    return envIndex_ + 1 < environments_.size();
}

void CoppeliaSimZmqInterface::advanceEnvironment() {
    if (hasNextEnvironment()) ++envIndex_;
}

void CoppeliaSimZmqInterface::executePlan(const ReconfigurationPlan& plan, const ConfigurationLibrary& library) {
    if (!plan.success) return;
    for (const auto& configName : plan.sequence) {
        if (configName == currentConfig_) continue;
        applyConfigurationLocal(library.getByName(configName));
    }
}

void CoppeliaSimZmqInterface::runTestMotion(const ConfigurationLibrary& library) {
    std::cout << "Running test motion: WIDE_STABLE -> LINE -> WIDE_STABLE\n";
    applyConfigurationLocal(library.getByName("WIDE_STABLE"));
    applyConfigurationLocal(library.getByName("LINE"));
    applyConfigurationLocal(library.getByName("WIDE_STABLE"));
    std::cout << "Test motion finished. If modules did not jump or scatter, the scene is ready.\n";
}

std::string CoppeliaSimZmqInterface::currentConfigurationName() const {
    return currentConfig_;
}

void CoppeliaSimZmqInterface::ensureConnectorObjects(int requiredCount) {
    if (!settings_.updateConnectors || !client_ || connectorsRootHandle_ < 0) return;
    auto sim = client_->getObject().sim();

    connectorHandles_.clear();
    connectorLengths_.clear();
    for (int i = 0; i < requiredCount; ++i) {
        try {
            connectorHandles_.push_back(sim.getObject(connectorPath(i)));
            connectorLengths_.push_back(1.0);
            continue;
        } catch (...) {
            // Missing connector objects are created as non-respondable visual cylinders.
        }

        try {
            const double d = std::max(0.005, settings_.connectorRadius * 2.0);
            const int h = sim.createPrimitiveShape(sim.primitiveshape_cylinder, {d, d, 1.0}, 0);
            try { sim.setObjectAlias(h, "Connector_" + std::to_string(i), 0); } catch (...) {}
            try { sim.setObjectParent(h, connectorsRootHandle_, true); } catch (...) {}
            try { sim.setObjectInt32Param(h, sim.shapeintparam_static, 1); } catch (...) {}
            try { sim.setObjectInt32Param(h, sim.shapeintparam_respondable, 0); } catch (...) {}
            connectorHandles_.push_back(h);
            connectorLengths_.push_back(1.0);
        } catch (const std::exception& e) {
            std::cerr << "Warning: could not create visual Connector_" << i << ": " << e.what()
                      << ". Pre-create /Robot/Connectors/Connector_i manually or disable update_connectors.\n";
            break;
        }
    }
}

void CoppeliaSimZmqInterface::updateVisualConnectors(const RobotGraph& graph) {
    if (!settings_.updateConnectors || !client_ || connectorsRootHandle_ < 0) return;
    auto sim = client_->getObject().sim();

    if (connectorHandles_.size() < graph.edges.size()) {
        ensureConnectorObjects(static_cast<int>(std::max<std::size_t>(graph.edges.size(), settings_.connectorCount)));
    }

    std::size_t used = 0;
    for (const auto& e : graph.edges) {
        if (used >= connectorHandles_.size()) break;
        const Module* a = graphCache_.findModule(e.from);
        const Module* b = graphCache_.findModule(e.to);
        if (!a || !b) continue;

        const Vec3 p0 = getModulePose(e.from).position;
        const Vec3 p1 = getModulePose(e.to).position;
        const Vec3 delta = p1 - p0;
        const double len = std::max(0.001, delta.norm());
        const Vec3 mid = (p0 + p1) * 0.5;
        const Vec3 rpy = connectorEulerFromDelta(delta);
        const int h = connectorHandles_[used++];

        try {
            sim.setObjectPosition(h, connectorsRootHandle_, toRemoteVector(mid));
            sim.setObjectOrientation(h, connectorsRootHandle_, toRemoteVector(rpy));
            // Connector primitives are created with unit length along local Z.
            // sim.scaleObject is relative, so keep the last applied visual length.
            if (used - 1 < connectorLengths_.size()) {
                const double oldLen = std::max(0.001, connectorLengths_[used - 1]);
                const double ratio = len / oldLen;
                try { sim.scaleObject(h, 1.0, 1.0, ratio, 0); connectorLengths_[used - 1] = len; } catch (...) {}
            }
        } catch (const std::exception& ex) {
            std::cerr << "Warning: could not update visual connector " << (used - 1) << ": " << ex.what() << "\n";
        }
    }
    hideUnusedConnectors(used);
}

void CoppeliaSimZmqInterface::hideUnusedConnectors(std::size_t usedCount) {
    if (!client_) return;
    auto sim = client_->getObject().sim();
    for (std::size_t i = usedCount; i < connectorHandles_.size(); ++i) {
        try {
            sim.setObjectPosition(connectorHandles_[i], connectorsRootHandle_, {0.0, 0.0, -10.0});
        } catch (...) {}
    }
}

Vec3 CoppeliaSimZmqInterface::connectorEulerFromDelta(const Vec3& delta) {
    const double len = std::max(1e-9, delta.norm());
    const double yaw = std::atan2(delta.y, delta.x);
    const double xy = std::sqrt(delta.x * delta.x + delta.y * delta.y);
    const double pitch = std::atan2(xy, delta.z);
    return Vec3(0.0, pitch, yaw);
}

void CoppeliaSimZmqInterface::sleepAfterStep() const {
    if (settings_.stepPauseMs > 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(settings_.stepPauseMs));
    }
}

double CoppeliaSimZmqInterface::smoothStep(double alpha) {
    const double a = std::clamp(alpha, 0.0, 1.0);
    return a * a * (3.0 - 2.0 * a);
}

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
    if (environments_.empty()) throw std::runtime_error("Scenario has no environment states: " + path);
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
        env.targetZone.center = vec3FromJsonZmq(tz.at("center"));
        env.targetZone.radius = tz.value("radius", 0.5);
    }
    for (const auto& jo : j.value("obstacles", NlohmannJson::array())) {
        Obstacle o;
        o.id = jo.value("id", std::string{"obstacle"});
        o.type = jo.value("type", std::string{"BOX"});
        o.position = vec3FromJsonZmq(jo.at("position"));
        o.size = vec3FromJsonZmq(jo.at("size"));
        env.obstacles.push_back(o);
    }
    return env;
}

} // namespace mrr
