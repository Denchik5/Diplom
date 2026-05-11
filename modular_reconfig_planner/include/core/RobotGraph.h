#pragma once

#include <set>
#include <string>
#include <utility>
#include <vector>
#include "core/Module.h"

namespace mrr {

struct Edge {
    int from = -1;
    int to = -1;
    std::string fromConnector;
    std::string toConnector;
};

class RobotGraph {
public:
    std::vector<Module> modules;
    std::vector<Edge> edges;

    bool isConnected() const;
    bool hasEdge(int a, int b) const;
    int edgeEditDistance(const RobotGraph& target) const;
    std::vector<int> getNeighbors(int moduleId) const;
    void addEdge(const Edge& edge);
    void clearEdges();
    const Module* findModule(int moduleId) const;
    Module* findModule(int moduleId);

private:
    static std::pair<int,int> normalizedEdge(int a, int b);
    std::set<std::pair<int,int>> edgeSet() const;
};

} // namespace mrr
