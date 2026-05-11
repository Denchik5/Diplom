#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>
#include "logging/ExperimentLogger.h"
#include "planning/ConfigurationLibrary.h"
#include "planning/ConstraintChecker.h"
#include "planning/CostFunction.h"
#include "planning/EnvironmentClassifier.h"
#include "planning/ReconfigurationPlanner.h"
#include "sim/MockCoppeliaSimInterface.h"

namespace fs = std::filesystem;
using namespace mrr;

static std::string firstExistingPath(const std::vector<std::string>& candidates) {
    for (const auto& c : candidates) if (fs::exists(c)) return c;
    return candidates.empty() ? std::string{} : candidates.front();
}

static std::string defaultConfigurationDir() {
    return firstExistingPath({"configurations", "../configurations", "data/configurations", "../data/configurations"});
}

static std::string defaultWeightsPath() {
    return firstExistingPath({"weights/default_cost_weights.json", "../weights/default_cost_weights.json", "data/weights/default_cost_weights.json", "../data/weights/default_cost_weights.json"});
}

static std::string scenarioStem(const std::string& scenarioPath) {
    return fs::path(scenarioPath).stem().string();
}

static int transitionCount(const ReconfigurationPlan& plan) {
    if (!plan.success || plan.initialConfiguration == plan.targetConfiguration) return 0;
    return std::max(1, static_cast<int>(plan.sequence.size()) - 1);
}

static Metrics makeMetrics(const EnvironmentState& env, const ReconfigurationPlan& plan, int interpolationStepsPerTransition) {
    Metrics m;
    m.scenarioName = env.scenarioId.empty() ? "scenario" : env.scenarioId;
    m.environmentType = env.type;
    m.selectedConfiguration = plan.targetConfiguration;
    m.success = plan.success && plan.constraintsValid;
    m.reconfigurationTime = plan.cost.T;
    m.numberOfSteps = transitionCount(plan) * interpolationStepsPerTransition;
    m.numberOfConnectionChanges = plan.numberOfConnectionChanges;
    m.estimatedEnergy = plan.cost.E;
    m.collisionRisk = plan.cost.R;
    m.stabilityPenalty = plan.cost.S;
    m.minimumClearance = plan.minimumClearance;
    m.totalCost = plan.cost.J;
    return m;
}

static void printReport(const EnvironmentState& env, const ReconfigurationPlan& plan, const std::string& metricsPath) {
    std::cout << "Scenario: " << env.scenarioId << "\n"
              << "Environment type: " << toString(env.type) << "\n"
              << "Selected configuration: " << (plan.targetConfiguration.empty() ? "none" : plan.targetConfiguration) << "\n"
              << "Constraints valid: " << (plan.constraintsValid ? "yes" : "no") << "\n"
              << "Total cost: " << plan.cost.J << "\n"
              << "Success: " << (plan.success ? "yes" : "no") << "\n"
              << "Metrics saved to: " << metricsPath << "\n\n";
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: planner_offline <scenario.json> [config_dir] [weights.json] [results_dir]\n";
        return 1;
    }

    const std::string scenarioPath = argv[1];
    const std::string configDir = argc > 2 ? argv[2] : defaultConfigurationDir();
    const std::string weightsPath = argc > 3 ? argv[3] : defaultWeightsPath();
    const std::string resultsDir = argc > 4 ? argv[4] : std::string{"results"};
    const std::string runName = scenarioStem(scenarioPath);
    const std::string metricsPath = (fs::path(resultsDir) / (runName + "_metrics.csv")).string();
    const std::string summaryPath = (fs::path(resultsDir) / "summary.csv").string();

    try {
        fs::create_directories(resultsDir);

        ConfigurationLibrary library;
        library.loadFromDirectory(configDir);

        CostWeights weights = loadCostWeights(weightsPath);
        ConstraintChecker checker;
        CostFunction costFunction(weights);
        ReconfigurationPlanner planner(library, checker, costFunction);
        MockCoppeliaSimInterface sim(scenarioPath, library);
        ExperimentLogger logger(metricsPath, summaryPath);

        if (!sim.connect()) return 2;
        sim.startSimulation();

        bool running = true;
        while (running) {
            RobotGraph graph = sim.getRobotGraph();
            EnvironmentState env = sim.getEnvironmentState();
            EnvironmentClassifier classifier;
            env.type = classifier.classify(env);

            ReconfigurationPlan plan = planner.plan(graph, sim.currentConfigurationName(), env);
            if (plan.success) sim.executePlan(plan, library);

            Metrics metrics = makeMetrics(env, plan, 20);
            logger.logMetrics(metrics);
            printReport(env, plan, metricsPath);

            if (sim.hasNextEnvironment()) sim.advanceEnvironment();
            else running = false;
        }

        sim.stopSimulation();
        sim.disconnect();
        std::cout << "Summary saved to: " << summaryPath << "\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Fatal error: " << e.what() << "\n";
        return 3;
    }
}
