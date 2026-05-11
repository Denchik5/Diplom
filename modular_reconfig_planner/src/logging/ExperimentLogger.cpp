#include "logging/ExperimentLogger.h"
#include "core/Enums.h"
#include <filesystem>
#include <stdexcept>

namespace fs = std::filesystem;

namespace mrr {

ExperimentLogger::ExperimentLogger(const std::string& metricsPath, const std::string& summaryPath) {
    fs::create_directories(fs::path(metricsPath).parent_path());
    metricsOut_.open(metricsPath, std::ios::trunc);
    if (!metricsOut_) throw std::runtime_error("Cannot open CSV log: " + metricsPath);
    writeHeader(metricsOut_);

    if (!summaryPath.empty()) {
        fs::create_directories(fs::path(summaryPath).parent_path());
        const bool header = needsHeader(summaryPath);
        summaryOut_.open(summaryPath, std::ios::app);
        if (!summaryOut_) throw std::runtime_error("Cannot open summary CSV log: " + summaryPath);
        if (header) writeHeader(summaryOut_);
    }
}

ExperimentLogger::~ExperimentLogger() {
    if (metricsOut_.is_open()) metricsOut_.close();
    if (summaryOut_.is_open()) summaryOut_.close();
}

bool ExperimentLogger::needsHeader(const std::string& path) {
    return !fs::exists(path) || fs::file_size(path) == 0;
}

std::string ExperimentLogger::csvEscape(const std::string& s) {
    std::string out = "\"";
    for (char c : s) {
        if (c == '"') out += "\"\"";
        else out.push_back(c);
    }
    out += "\"";
    return out;
}

void ExperimentLogger::writeHeader(std::ostream& out) {
    out << "scenario_name,environment_type,selected_configuration,success,"
        << "reconfiguration_time,number_of_steps,number_of_connection_changes,"
        << "estimated_energy,collision_risk,stability_penalty,minimum_clearance,total_cost\n";
}

void ExperimentLogger::writeRow(std::ostream& out, const Metrics& m) {
    out << csvEscape(m.scenarioName) << ","
        << toString(m.environmentType) << ","
        << csvEscape(m.selectedConfiguration) << ","
        << (m.success ? 1 : 0) << ","
        << m.reconfigurationTime << ","
        << m.numberOfSteps << ","
        << m.numberOfConnectionChanges << ","
        << m.estimatedEnergy << ","
        << m.collisionRisk << ","
        << m.stabilityPenalty << ","
        << m.minimumClearance << ","
        << m.totalCost << "\n";
}

void ExperimentLogger::logMetrics(const Metrics& metrics) {
    writeRow(metricsOut_, metrics);
    metricsOut_.flush();
    if (summaryOut_.is_open()) {
        writeRow(summaryOut_, metrics);
        summaryOut_.flush();
    }
}

} // namespace mrr
