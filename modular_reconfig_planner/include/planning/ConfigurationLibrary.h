#pragma once

#include <map>
#include <string>
#include <vector>
#include "core/Configuration.h"

namespace mrr {

class ConfigurationLibrary {
public:
    void loadFromDirectory(const std::string& directory);
    void loadFile(const std::string& path);

    bool contains(const std::string& name) const;
    const Configuration& getByName(const std::string& name) const;
    std::vector<Configuration> getCandidates(EnvironmentType type) const;
    std::vector<std::string> names() const;

private:
    std::map<std::string, Configuration> configurations_;
};

} // namespace mrr
