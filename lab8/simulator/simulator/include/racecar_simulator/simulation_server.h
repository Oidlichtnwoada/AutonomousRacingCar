/*
 * ---------------------------------------------------------------------------------------------------------------------
 * Author: Thomas Pintaric (thomas.pintaric@gmail.com)
 * SPDX-License-Identifier: GPL-3.0-or-later
 * ---------------------------------------------------------------------------------------------------------------------
 */

#pragma once

#include <grpc/grpc.h>
#include <grpcpp/server.h>
#include "racecar_simulator.grpc.pb.h"

#if __GNUC__ >= 8
#include <filesystem>
    namespace fs = std::filesystem;
#else
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#endif

#include <nlohmann/json.hpp>
using json = nlohmann::json;

#include <atomic>
#include <shared_mutex>
#include <map>
#include <vector>

#include <racecar_simulator/map.h>
#include <racecar_simulator/simulation.h>

namespace f1tenth {

    class SimulationServer final : public Simulator::Service {
    public:
        explicit SimulationServer(const json& j,
                                  const fs::path base_path,
                                  bool disable_all_lookup_tables = false,
                                  bool verbose = false);
        std::unique_ptr<grpc::Server> create(const std::string server_address);

        // RPCs
        virtual ::grpc::Status ListSimulationScenarios(::grpc::ServerContext* context, const ::f1tenth::Empty* request, ::f1tenth::ListSimulationScenariosResponse* response) override;
        virtual ::grpc::Status CreateSimulation(::grpc::ServerContext* context, const ::f1tenth::CreateSimulationRequest* request, ::f1tenth::CreateSimulationResponse* response) override;
        virtual ::grpc::Status DeleteSimulation(::grpc::ServerContext* context, const ::f1tenth::DeleteSimulationRequest* request, ::f1tenth::DeleteSimulationResponse* response) override;
        virtual ::grpc::Status GetSimulationState(::grpc::ServerContext* context, const ::f1tenth::GetSimulationStateRequest* request, ::f1tenth::GetSimulationStateResponse* response) override;
        virtual ::grpc::Status SimulationStep(::grpc::ServerContext* context, const ::f1tenth::SimulationStepRequest* request, ::f1tenth::SimulationStepResponse* response) override;
        virtual ::grpc::Status GetVersion(::grpc::ServerContext* context, const ::f1tenth::Empty* request, ::f1tenth::GetVersionResponse* response) override;

    private:
        struct Scenario {
            inline unsigned int number_of_vehicles() const {
                return(vehicle_definition_references_.size());
            }
            std::tuple<std::string, std::shared_ptr<Simulation::Map> > map_; // <name, map>
            std::vector<std::string> vehicle_definition_references_;
            ::f1tenth::VehicleStates initial_vehicle_states_;
            ::f1tenth::SimulationScenario_CollisionBehavior collision_behavior_;
        };

    private:
        const bool disable_all_lookup_tables_;
        bool verbose_;
        void loadMapsFromJson(const json& j, const fs::path base_path);
        void loadVehicleDefinitionsFromJson(const json& j);
        void loadScenariosFromJson(const json& j);
        std::map<std::string, std::shared_ptr<Simulation::Map> > maps_;
        std::map<std::string, std::shared_ptr<const VehicleDefinition> > vehicle_definitions_;
        std::map<std::string, std::shared_ptr<const Scenario> > scenarios_;

        typedef Simulation::Handle Handle;
        typedef Simulation::Tag Tag;
        std::atomic<Handle> next_simulation_handle_;
        mutable std::shared_mutex simulations_mutex_;
        std::map<Handle, std::shared_ptr<Simulation> > simulations_;
        std::map<Tag, Handle> simulation_tags_;

    };

} // namespace f1tenth
