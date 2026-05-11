#include "planning/ConfigurationLibrary.h"
#include <nlohmann/json.hpp>
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <stdexcept>

namespace fs = std::filesystem;
using json = nlohmann::json;

namespace mrr {

static Vec3 vec3FromJson(const json& a) {
    return Vec3(a.at(0).get<double>(), a.at(1).get<double>(), a.at(2).get<double>());
}

static Vec3 dimensionsFromJson(const json& j) {
    if (j.contains("module_dimensions")) return vec3FromJson(j.at("module_dimensions"));
    if (j.contains("module_length") || j.contains("module_width") || j.contains("module_height")) {
        return Vec3(
            j.value("module_length", 0.22),
            j.value("module_width", 0.18),
            j.value("module_height", 0.10)
        );
    }
    if (j.contains("module_size")) {
        const double s = j.value("module_size", 0.22);
        return Vec3(s, s, s);
    }
    return Vec3(0.22, 0.18, 0.10);
}

void ConfigurationLibrary::loadFromDirectory(const std::string& directory) {
    configurations_.clear();
    if (!fs::exists(directory)) throw std::runtime_error("Configuration directory does not exist: " + directory);

    std::vector<fs::path> files;
    for (const auto& entry : fs::directory_iterator(directory)) {
        if (entry.is_regular_file() && entry.path().extension() == ".json") files.push_back(entry.path());
    }
    std::sort(files.begin(), files.end());
    for (const auto& path : files) loadFile(path.string());

    if (configurations_.empty()) throw std::runtime_error("No configuration JSON files found in " + directory);
}

void ConfigurationLibrary::loadFile(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("Cannot open configuration file: " + path);

    json j;
    in >> j;

    Configuration c;
    c.name = j.at("name").get<std::string>();
    c.description = j.value("description", std::string{});
    c.moduleCount = j.at("module_count").get<int>();
    c.moduleDimensions = dimensionsFromJson(j);

    for (int i = 0; i < c.moduleCount; ++i) c.graph.modules.emplace_back(i);

    for (const auto& jm : j.at("modules")) {
        ModuleTarget t;
        t.moduleId = jm.at("id").get<int>();
        t.targetPose.position = vec3FromJson(jm.at("target_position"));
        t.targetPose.rpy = vec3FromJson(jm.at("target_orientation_rpy"));
        t.targetJointAngles = jm.value("joint_targets", std::vector<double>{0.0, 0.0});
        c.targets.push_back(t);

        if (t.moduleId >= 0 && t.moduleId < static_cast<int>(c.graph.modules.size())) {
            c.graph.modules[t.moduleId].pose = t.targetPose;
            c.graph.modules[t.moduleId].jointAngles = t.targetJointAngles;
        }
    }

    for (const auto& je : j.at("edges")) {
        Edge e;
        e.from = je.at("from").get<int>();
        e.to = je.at("to").get<int>();
        e.fromConnector = je.value("from_connector", std::string{"RIGHT"});
        e.toConnector = je.value("to_connector", std::string{"LEFT"});
        c.graph.addEdge(e);
    }

    if (j.contains("constraints")) {
        const auto& jc = j.at("constraints");
        c.maxWidth = jc.value("max_width", 999.0);
        c.maxLength = jc.value("max_length", 999.0);
        c.maxHeightLimit = jc.value("max_height", 999.0);
        c.minStabilityMargin = jc.value("min_stability_margin", 0.02);
        c.stabilityEstimate = jc.value("stability_estimate", c.minStabilityMargin);
        c.maxJointAngle = jc.value("max_joint_angle_rad", 1.57);
        c.allowDisconnectedGraph = jc.value("allow_disconnected_graph", false);
    }

    for (const auto& s : j.value("applicable_environment_types", std::vector<std::string>{})) {
        c.applicableTypes.push_back(environmentTypeFromString(s));
    }

    if (c.moduleCount != 8) {
        throw std::runtime_error("Configuration " + c.name + " must contain 8 modules for this demo");
    }
    if (c.targets.size() != static_cast<std::size_t>(c.moduleCount)) {
        throw std::runtime_error("Configuration " + c.name + " has incomplete target module list");
    }

    configurations_[c.name] = c;
}

bool ConfigurationLibrary::contains(const std::string& name) const {
    return configurations_.count(name) > 0;
}

const Configuration& ConfigurationLibrary::getByName(const std::string& name) const {
    auto it = configurations_.find(name);
    if (it == configurations_.end()) throw std::runtime_error("Unknown configuration: " + name);
    return it->second;
}

std::vector<Configuration> ConfigurationLibrary::getCandidates(EnvironmentType type) const {
    std::vector<Configuration> out;
    const std::string preferred = preferredConfigurationFor(type);

    for (const auto& [name, config] : configurations_) {
        if (std::find(config.applicableTypes.begin(), config.applicableTypes.end(), type) != config.applicableTypes.end()) {
            out.push_back(config);
        }
    }

    std::sort(out.begin(), out.end(), [&](const Configuration& a, const Configuration& b) {
        if (a.name == preferred) return true;
        if (b.name == preferred) return false;
        return a.name < b.name;
    });

    if (out.empty() && configurations_.count(preferred)) out.push_back(configurations_.at(preferred));
    if (out.empty() && configurations_.count("WIDE_STABLE")) out.push_back(configurations_.at("WIDE_STABLE"));
    return out;
}

std::vector<std::string> ConfigurationLibrary::names() const {
    std::vector<std::string> out;
    for (const auto& [name, config] : configurations_) out.push_back(name);
    return out;
}

} // namespace mrr
