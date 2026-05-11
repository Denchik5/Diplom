#include "core/RobotGraph.h"
#include <algorithm>
#include <queue>

namespace mrr {

std::pair<int,int> RobotGraph::normalizedEdge(int a, int b) {
    return a < b ? std::make_pair(a, b) : std::make_pair(b, a);
}

std::set<std::pair<int,int>> RobotGraph::edgeSet() const {
    std::set<std::pair<int,int>> s;
    for (const auto& e : edges) s.insert(normalizedEdge(e.from, e.to));
    return s;
}

const Module* RobotGraph::findModule(int moduleId) const {
    for (const auto& m : modules) if (m.id == moduleId) return &m;
    return nullptr;
}

Module* RobotGraph::findModule(int moduleId) {
    for (auto& m : modules) if (m.id == moduleId) return &m;
    return nullptr;
}

bool RobotGraph::isConnected() const {
    if (modules.empty()) return true;

    std::set<int> moduleIds;
    for (const auto& m : modules) moduleIds.insert(m.id);

    std::queue<int> q;
    std::set<int> visited;
    q.push(modules.front().id);
    visited.insert(modules.front().id);

    while (!q.empty()) {
        int u = q.front();
        q.pop();
        for (int v : getNeighbors(u)) {
            if (moduleIds.count(v) && !visited.count(v)) {
                visited.insert(v);
                q.push(v);
            }
        }
    }
    return visited.size() == moduleIds.size();
}

bool RobotGraph::hasEdge(int a, int b) const {
    auto p = normalizedEdge(a, b);
    for (const auto& e : edges) if (normalizedEdge(e.from, e.to) == p) return true;
    return false;
}

int RobotGraph::edgeEditDistance(const RobotGraph& target) const {
    auto a = edgeSet();
    auto b = target.edgeSet();
    int distance = 0;
    for (const auto& e : a) if (!b.count(e)) ++distance;
    for (const auto& e : b) if (!a.count(e)) ++distance;
    return distance;
}

std::vector<int> RobotGraph::getNeighbors(int moduleId) const {
    std::vector<int> result;
    for (const auto& e : edges) {
        if (e.from == moduleId) result.push_back(e.to);
        else if (e.to == moduleId) result.push_back(e.from);
    }
    return result;
}

void RobotGraph::addEdge(const Edge& edge) {
    if (edge.from == edge.to || edge.from < 0 || edge.to < 0) return;
    if (!hasEdge(edge.from, edge.to)) edges.push_back(edge);

    Module* a = findModule(edge.from);
    Module* b = findModule(edge.to);
    if (a && std::find(a->neighbors.begin(), a->neighbors.end(), edge.to) == a->neighbors.end()) {
        a->neighbors.push_back(edge.to);
    }
    if (b && std::find(b->neighbors.begin(), b->neighbors.end(), edge.from) == b->neighbors.end()) {
        b->neighbors.push_back(edge.from);
    }
}

void RobotGraph::clearEdges() {
    edges.clear();
    for (auto& m : modules) m.neighbors.clear();
}

} // namespace mrr
