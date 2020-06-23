/*
 * ---------------------------------------------------------------------------------------------------------------------
 * Author: Thomas Pintaric (thomas.pintaric@gmail.com)
 * SPDX-License-Identifier: GPL-3.0-or-later
 * ---------------------------------------------------------------------------------------------------------------------
 */

#include <iostream>
#include <memory>
#include <csignal>
#include <functional>
#include <chrono>
#include <omp.h>

#if __GNUC__ >= 8
#include <filesystem>
    namespace fs = std::filesystem;
#else
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#endif

#include <algorithm>
#include <tclap/CmdLine.h>

#include <nlohmann/json.hpp>
using json = nlohmann::json;

#define DEFAULT_SERVER_ADDRESS "0.0.0.0:50051"
#define DEFAULT_CONFIGURATION_FILENAME "racecar_simulator.json"
#define DEFAULT_SERVER_SHUTDOWN_TIMEOUT 2 // [s]

#include <racecar_simulator.h>
#include <racecar_simulator/simulation_server.h>

namespace {
    std::function<void(int)> shutdown_handler;
    void signal_handler(int signal) { shutdown_handler(signal); }
}

int main(int argc, char** argv) {

    std::shared_ptr<f1tenth::SimulationServer> simulation_server;
    TCLAP::CmdLine cmd("F1/Tenth Racecar Simulator", ' ', RACECAR_SIMULATOR_VERSION_STRING, true);
    TCLAP::ValueArg<std::string> server_address("a", "server_address", "server address/port", false, DEFAULT_SERVER_ADDRESS, "string");
    TCLAP::ValueArg<std::string> configuration("c", "configuration", ".json configuration file", false, DEFAULT_CONFIGURATION_FILENAME, "string");
    TCLAP::SwitchArg disable_lookup_tables("d", "disable_lookup_tables", "globally disable all lookup tables", false);
    TCLAP::SwitchArg run_once("o", "run_once", "run once (do not enter main loop)", false);
    TCLAP::SwitchArg verbose("v", "verbose", "enable verbose console output", false);

    try {
        cmd.add(server_address);
        cmd.add(configuration);
        cmd.add(disable_lookup_tables);
        cmd.add(run_once);
        cmd.add(verbose);
        cmd.parse(argc, argv);

        std::cout << "F1/Tenth Racecar Simulator" << std::endl;

        fs::path configuration_file = configuration.getValue();
        if(!fs::exists(configuration_file)) {
            std::ostringstream o;
            o << "could not find " << configuration_file;
            throw std::runtime_error(o.str());
        }

        json j = json::parse(std::ifstream(configuration_file));
        simulation_server = std::shared_ptr<f1tenth::SimulationServer>(
                new f1tenth::SimulationServer(j,
                                              configuration_file.parent_path(),
                                              disable_lookup_tables.getValue(),
                                              verbose.getValue()));
        
        assert(simulation_server);

        if(run_once.getValue()) {
            std::cout << "Exit without entering main loop (--run_once is set)." << std::endl;
            return 0;
        }

        std::unique_ptr<grpc::Server> grpc_server = simulation_server->create(server_address.getValue());

        if(!grpc_server) {
            throw std::runtime_error("failed to create grpc::Server instance");
        }

        std::signal(SIGINT, signal_handler);
        shutdown_handler = [&](int signal) {
            std::cout << "Server is shutting down..." << std::endl;
            const std::chrono::system_clock::time_point deadline = std::chrono::system_clock::now() +
                    std::chrono::seconds(DEFAULT_SERVER_SHUTDOWN_TIMEOUT);
            grpc_server->Shutdown(deadline);
        };

        std::cout << "Server is listening on " << server_address.getValue() << std::endl;
        grpc_server->Wait(); // enter main loop
        std::cout << "Server has shut down." << std::endl;

    } catch (TCLAP::ArgException &e) {
        std::cerr << "ERROR: " << e.error() << " for argument " << e.argId() << std::endl;
    } catch (std::exception &e) {
        std::cerr << "ERROR: " << e.what() << std::endl;
    }
    return 0;
}
