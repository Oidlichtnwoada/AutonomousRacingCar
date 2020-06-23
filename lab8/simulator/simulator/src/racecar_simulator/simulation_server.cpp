/*
 * ---------------------------------------------------------------------------------------------------------------------
 * Author: Thomas Pintaric (thomas.pintaric@gmail.com)
 * SPDX-License-Identifier: GPL-3.0-or-later
 * ---------------------------------------------------------------------------------------------------------------------
 */

#include <racecar_simulator.h>
#include <racecar_simulator/simulation_server.h>
#include <grpcpp/server_builder.h>
#include <grpcpp/server_context.h>
#include <google/protobuf/util/json_util.h>
#include <string>
using namespace google::protobuf;

namespace f1tenth {

    SimulationServer::SimulationServer(const json& j,
                                       const fs::path base_path,
                                       bool disable_all_lookup_tables,
                                       bool verbose)
        : disable_all_lookup_tables_(disable_all_lookup_tables)
        , verbose_(verbose)
        , next_simulation_handle_(1UL)
    {
        loadVehicleDefinitionsFromJson(j);
        loadMapsFromJson(j, base_path);
        loadScenariosFromJson(j); // NOTE: must be called last (order is important!)
    }

    std::unique_ptr<grpc::Server> SimulationServer::create(const std::string server_address) {
        grpc::ServerBuilder builder;
        builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
        builder.RegisterService(this);
        std::unique_ptr<grpc::Server> server(builder.BuildAndStart());
        return(server);
    }

    void SimulationServer::loadMapsFromJson(const json& j, const fs::path base_path) {

        for (const auto& [_, json_map] : j.find("maps")->items()) {

            const std::string map_name = json_map.find("map_name").value();

            const float map_resolution = json_map.find("map_resolution").value().get<float>();
            const float map_origin_x = json_map.find("map_origin")->at(0).get<float>();
            const float map_origin_y = json_map.find("map_origin")->at(1).get<float>();
            const Eigen::Vector2f map_origin(map_origin_x, map_origin_y);

            const bool enable_lookup_table = (json_map.find("enable_lookup_table").value().get<bool>() and
                                              (not disable_all_lookup_tables_));
            unsigned int rays_per_lut_bin = 0;
            if(enable_lookup_table) {
                rays_per_lut_bin = json_map.find("rays_per_lut_bin").value().get<unsigned int>();
                if (rays_per_lut_bin == 0) {
                    std::cerr << "WARNING: Invalid number of rays per bin. " <<
                              "Disabling lookup table for map \"" << map_name << "\"." << std::endl;
                    // TODO: alternatively, we could just throw an exception here...
                }
            }

            const fs::path obstacles_file = (base_path / json_map.find("subdirectory").value() /
                                             fs::path(json_map.find("obstacles").value()));
            if(!fs::exists(obstacles_file)) {
                std::cerr << "WARNING: " << obstacles_file.relative_path() << " does not exist. Skipping map \"" << map_name << "\"."
                          << std::endl;
                // TODO: alternatively, we could just throw an exception here...
                continue;
            }

            const fs::path drivable_area_file = (base_path / json_map.find("subdirectory").value() /
                                                 fs::path(json_map.find("drivable_area").value()));

            if(!fs::exists(drivable_area_file)) {
                std::cerr << "WARNING: " << drivable_area_file.relative_path() << " does not exist. Skipping map \"" << map_name
                          << "\"." << std::endl;
                // TODO: alternatively, we could just throw an exception here...
                continue;
            }

            std::shared_ptr<::f1tenth::Map> map = std::shared_ptr<::f1tenth::Map>(
                    new ::f1tenth::Map(obstacles_file,
                                       map_origin,
                                       map_resolution,
                                       drivable_area_file,
                                       enable_lookup_table,
                                       rays_per_lut_bin));

            if(verbose_) {
                const auto [lower_bound_x, lower_bound_y, upper_bound_x, upper_bound_y] = map->getMetricBounds();
                std::cout << "Loaded map \"" << map_name << "\". Octree bounds: " <<
                          "[" << lower_bound_x << ", " << lower_bound_y << "]" << " - " <<
                          "[" << upper_bound_x << ", " << upper_bound_y << "], " <<
                          map->getNumberOfNodes() << " nodes." << std::endl;
            }
            maps_.insert(std::make_pair(map_name, map));
        }
    }

    void SimulationServer::loadVehicleDefinitionsFromJson(const json& j) {
        for (const auto&[key, json_vehicle] : j.find("vehicles")->items()) {
            const std::string vehicle_definition_name = json_vehicle.find("vehicle_definition_name").value();
            const auto json_vehicle_definition = json_vehicle.find("vehicle_definition");
            std::shared_ptr<VehicleDefinition> vehicle_definition(new VehicleDefinition());
            const util::Status status = util::JsonStringToMessage(
                    json_vehicle_definition->dump(), vehicle_definition.get());
            if (!status.ok()) {
                std::cerr << "WARNING: could not load vehicle definition \"" << vehicle_definition_name <<
                          "\"." << std::endl;
                // TODO: alternatively, we could just throw an exception here...
            } else {
                vehicle_definitions_.insert(std::make_pair(vehicle_definition_name, vehicle_definition));
            }
        }
    }

    void SimulationServer::loadScenariosFromJson(const json& j) {

        for (const auto&[key, json_scenario] : j.find("scenarios")->items()) {

            std::shared_ptr<Scenario> scenario(new Scenario());
            const std::string scenario_name = json_scenario.find("scenario_name").value();
            const std::string map_reference = json_scenario.find("map_reference").value();
            const auto map_it = maps_.find(map_reference);
            if(map_it == maps_.end()) {
                std::cerr << "WARNING: scenario \"" << scenario_name <<
                          "\" references an unknown map \"" << map_reference <<
                          "\". Skipping scenario." << std::endl;
                // TODO: alternatively, we could just throw an exception here...
                continue;
            }
            scenario->map_ = std::make_tuple(map_it->first, map_it->second);
            scenario->collision_behavior_ = (::f1tenth::SimulationScenario_CollisionBehavior)
                    json_scenario.find("collision_behavior").value().get<unsigned int>();

            bool success = true;
            for (const auto&[key, json_vehicle] : json_scenario.find("vehicles")->items()) {

                const std::string vehicle_definition_name = json_vehicle.find("vehicle_definition_name").value();
                scenario->vehicle_definition_references_.push_back(vehicle_definition_name);

                std::shared_ptr<VehicleState> vehicle_state(new VehicleState());

                VehicleState::Pose pose;
                {
                    const auto json_pose = json_vehicle.find("initial_state")->find("pose");
                    const util::Status status = util::JsonStringToMessage(json_pose->dump(), &pose);
                    if (!status.ok()) {
                        std::cerr << "WARNING: could not load initial pose of vehicle " <<
                                  scenario->initial_vehicle_states_.size() <<
                                  " (simulation scenario \"" << scenario_name <<
                                  "\")." << std::endl;
                        success = false;
                        // TODO: alternatively, we could just throw an exception here...
                        break;
                    } else {
                        vehicle_state->mutable_pose()->CopyFrom(pose);
                    }
                }

                VehicleState::Motion motion;
                {
                    const auto json_motion = json_vehicle.find("initial_state")->find("motion");
                    const util::Status status = util::JsonStringToMessage(json_motion->dump(), &motion);
                    if (!status.ok()) {
                        std::cerr << "WARNING: could not load initial motion of vehicle " <<
                                  scenario->initial_vehicle_states_.size() <<
                                  " (simulation scenario \"" << scenario_name <<
                                  "\")." << std::endl;
                        success = false;
                        // TODO: alternatively, we could just throw an exception here...
                        break;
                    } else {
                        vehicle_state->mutable_motion()->CopyFrom(motion);
                    }
                }
                scenario->initial_vehicle_states_.push_back(vehicle_state);
            }
            if(!success) {
                // skip this scenario
                continue;
            }
            assert(scenario->vehicle_definition_references_.size() ==
                   scenario->initial_vehicle_states_.size());

            scenarios_.insert(std::make_pair(scenario_name, scenario));
        }
    }

    ::grpc::Status SimulationServer::ListSimulationScenarios(
            ::grpc::ServerContext* context,
            const ::f1tenth::Empty* request,
            ::f1tenth::ListSimulationScenariosResponse* response) {

        for(auto const& [name, scenario] : scenarios_){
            f1tenth::SimulationScenario* entry = response->mutable_simulation_scenarios()->Add();
            entry->mutable_scenario_name()->assign(name);
            entry->set_number_of_vehicles(scenario->number_of_vehicles());
            entry->set_supports_lidar_lookup_table(std::get<1>(scenario->map_)->supportsLookupTable());
        }
        return(grpc::Status::OK);
    }

    ::grpc::Status SimulationServer::CreateSimulation(
            ::grpc::ServerContext* context,
            const ::f1tenth::CreateSimulationRequest* request,
            ::f1tenth::CreateSimulationResponse* response) {

        if((not request->simulation_tag().empty()) and
           (simulation_tags_.find(request->simulation_tag()) != simulation_tags_.end())) {
            std::ostringstream o;
            o << "tag \"" << request->simulation_tag() << "\" already exists";
            response->mutable_error()->mutable_message()->assign(o.str());
            return(grpc::Status::OK);
        }

        auto const env_it = scenarios_.find(request->scenario_name());
        if(env_it == scenarios_.end()) {
            std::ostringstream o;
            o << "unknown scenario \"" << request->scenario_name() << "\"";
            response->mutable_error()->mutable_message()->assign(o.str());
        } else {
            std::shared_ptr<const Scenario> scenario = env_it->second;

            assert(scenario->vehicle_definition_references_.size() ==
                   scenario->initial_vehicle_states_.size());

            std::shared_ptr<Simulation::Map> map = std::get<1>(scenario->map_);
            assert(map);

            const bool use_lidar_lookup_table = ((request->should_use_lidar_lookup_table() and
                                                  map->supportsLookupTable()) and (not disable_all_lookup_tables_));

            ::f1tenth::VehicleDefinitions vehicle_definitions;
            for(const std::string& key : scenario->vehicle_definition_references_) {
                assert(vehicle_definitions_.find(key) != vehicle_definitions_.end());
                vehicle_definitions.push_back(vehicle_definitions_[key]);
            }

            const Handle handle = next_simulation_handle_.fetch_add(1UL);
            std::shared_ptr<Simulation> simulation(new Simulation(
                    handle,
                    request->simulation_tag(),
                    map,
                    use_lidar_lookup_table,
                    vehicle_definitions,
                    scenario->initial_vehicle_states_,
                    scenario->collision_behavior_));
            {
                simulations_mutex_.lock(); // lock for write access
                simulations_.insert(std::make_pair(handle, simulation));
                if(not request->simulation_tag().empty()) {
                    simulation_tags_.insert(std::make_pair(request->simulation_tag(), handle));
                }
                if(verbose_) {
                    std::cout << "Created simulation (handle = " << handle;
                    if(not request->simulation_tag().empty()) {
                        std::cout << ", tag = \"" << request->simulation_tag() << "\"";
                    }
                    std::cout << ")" << std::endl;
                }

                simulation->writeStateTo(response->mutable_simulation_state());
                assert(response->simulation_state().simulation_handle() == handle);
                assert(response->simulation_state().simulation_time() == 0.0f); // should always start at t=0
                assert(response->simulation_state().uses_lidar_lookup_table() or (not use_lidar_lookup_table));
                simulations_mutex_.unlock();
            }
        }

        return(grpc::Status::OK);
    }

    ::grpc::Status SimulationServer::DeleteSimulation(::grpc::ServerContext* context,
            const ::f1tenth::DeleteSimulationRequest* request,
            ::f1tenth::DeleteSimulationResponse* response) {

        simulations_mutex_.lock(); // lock for write access

        Handle handle;
        switch(request->identifier_case()) {
            default: {
                throw std::runtime_error("invalid message field");
                break;
            }
            case(DeleteSimulationRequest::kSimulationHandle): {
                handle = request->simulation_handle();
                break;
            }
            case(DeleteSimulationRequest::kSimulationTag): {
                const Tag tag = request->simulation_tag();
                auto tag_it = simulation_tags_.find(tag);
                if(tag_it == simulation_tags_.end()) {
                    response->mutable_error()->mutable_message()->assign("invalid tag");
                    return(grpc::Status::OK);
                }
                handle = tag_it->second;
                break;
            }
        }

        auto it = simulations_.find(handle);
        if(it == simulations_.end()) {
            response->mutable_error()->mutable_message()->assign("invalid handle");
        } else {
            it->second->mutex_.lock(); // lock for write access
            if(handle != it->second->handle_) {
                throw std::runtime_error("inconsistent internal state (handle mismatch)");
            }
            const Tag tag = it->second->tag_;
            it->second->marked_for_deletion_ = true;
            response->set_deleted_simulation_handle(it->second->handle_);
            if(not it->second->tag_.empty()) {
                response->set_deleted_simulation_tag(it->second->tag_);
            }
            it->second->mutex_.unlock();
            if(verbose_) {
                std::cout << "Deleted simulation (handle = " << handle;
                if(!tag.empty()) {
                    std::cout << ", tag = \"" << tag << "\"";
                }
                std::cout << ", t = " << it->second->simulation_time_ << ")" << std::endl;
            }
            simulations_.erase(handle);
            if(!tag.empty()) {
                simulation_tags_.erase(tag);
            }
        }
        simulations_mutex_.unlock();
        return(grpc::Status::OK);
    }

    ::grpc::Status SimulationServer::GetSimulationState(::grpc::ServerContext* context,
            const ::f1tenth::GetSimulationStateRequest* request,
            ::f1tenth::GetSimulationStateResponse* response) {

        simulations_mutex_.lock_shared(); // lock for read-only access

        Handle handle;
        Tag tag;
        switch(request->identifier_case()) {
            default: {
                throw std::runtime_error("invalid message field");
                break;
            }
            case(GetSimulationStateRequest::kSimulationHandle): {
                handle = request->simulation_handle();
                break;
            }
            case(GetSimulationStateRequest::kSimulationTag): {
                tag = request->simulation_tag();
                auto tag_it = simulation_tags_.find(tag);
                if(tag_it == simulation_tags_.end()) {
                    response->mutable_error()->mutable_message()->assign("invalid tag");
                    return(grpc::Status::OK);
                }
                handle = tag_it->second;
                break;
            }
        }

        auto it = simulations_.find(handle);
        if(it == simulations_.end()) {
            response->mutable_error()->mutable_message()->assign("invalid handle");
        } else {
            it->second->writeStateTo(response->mutable_simulation_state());
            assert(response->simulation_state().simulation_handle() == it->first);
            assert(tag.empty() || tag.compare(response->simulation_state().simulation_tag()) == 0);
        }
        simulations_mutex_.unlock_shared();
        return(grpc::Status::OK);
    }

    ::grpc::Status SimulationServer::SimulationStep(::grpc::ServerContext* context,
            const ::f1tenth::SimulationStepRequest* request,
            ::f1tenth::SimulationStepResponse* response) {

        simulations_mutex_.lock_shared(); // lock for read-only access

        Handle handle;
        Tag tag;
        switch(request->identifier_case()) {
            default: {
                throw std::runtime_error("invalid message field");
                break;
            }
            case(SimulationStepRequest::kSimulationHandle): {
                handle = request->simulation_handle();
                break;
            }
            case(SimulationStepRequest::kSimulationTag): {
                tag = request->simulation_tag();
                auto tag_it = simulation_tags_.find(tag);
                if(tag_it == simulation_tags_.end()) {
                    response->mutable_error()->mutable_message()->assign("invalid tag");
                    return(grpc::Status::OK);
                }
                handle = tag_it->second;
                break;
            }
        }

        auto it = simulations_.find(handle);
        if(it == simulations_.end()) {
            response->mutable_error()->mutable_message()->assign("invalid handle");
        } else if(request->vehicle_control_input_size() != it->second->getNumberOfVehicles()) {
            std::ostringstream o;
            o << "wrong number of vehicle control inputs (expected " << it->second->getNumberOfVehicles() <<
              ", got " << request->vehicle_control_input_size() << ")";
            response->mutable_error()->mutable_message()->assign(o.str());
        } else if(request->time_step() < 0.0f) {
            std::ostringstream o;
            o << "invalid time step (t=" << request->time_step() << ")";
            response->mutable_error()->mutable_message()->assign(o.str());
        } else {

            ::f1tenth::VehicleControlInputs vehicle_control_inputs;
            for(unsigned int i=0; i<request->vehicle_control_input_size(); i++) {
                std::shared_ptr<VehicleControlInput> vehicle_control_input(
                        new VehicleControlInput(request->vehicle_control_input(i)));
                vehicle_control_inputs.push_back(vehicle_control_input);
            }
            if(request->time_step() > 0.0f) {
                it->second->step(request->time_step(), vehicle_control_inputs); // <--- main work will be done here
            }
            it->second->writeStateTo(response->mutable_simulation_state());
            assert(response->simulation_state().simulation_handle() == it->first);
            assert(tag.empty() || tag.compare(response->simulation_state().simulation_tag()) == 0);
            assert(request->time_step() == 0.0f || response->simulation_state().simulation_time() > 0.0f);
        }
        simulations_mutex_.unlock_shared();
        return(grpc::Status::OK);
    }

    ::grpc::Status SimulationServer::GetVersion(::grpc::ServerContext* context,
            const ::f1tenth::Empty* request,
            ::f1tenth::GetVersionResponse* response) {

        response->set_version_major(RACECAR_SIMULATOR_VERSION_MAJOR);
        response->set_version_minor(RACECAR_SIMULATOR_VERSION_MINOR);
        response->set_version_patch(RACECAR_SIMULATOR_VERSION_PATCH);
        return(grpc::Status::OK);
    }

} // namespace f1tenth
