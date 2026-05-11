#pragma once

#include <fstream>
#include <string>
#include "core/ReconfigurationPlan.h"

namespace mrr {

class ExperimentLogger {
public:
    explicit ExperimentLogger(const std::string& metricsPath, const std::string& summaryPath = "");
    ~ExperimentLogger();

    void logMetrics(const Metrics& metrics);

private:
    static void writeHeader(std::ostream& out);
    static void writeRow(std::ostream& out, const Metrics& metrics);
    static std::string csvEscape(const std::string& s);
    static bool needsHeader(const std::string& path);

    std::ofstream metricsOut_;
    std::ofstream summaryOut_;
};

} // namespace mrr
