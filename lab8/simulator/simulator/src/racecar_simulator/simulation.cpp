/*
 * ---------------------------------------------------------------------------------------------------------------------
 * Author: Thomas Pintaric (thomas.pintaric@gmail.com)
 * SPDX-License-Identifier: GPL-3.0-or-later
 * ---------------------------------------------------------------------------------------------------------------------
 */

#include <racecar_simulator/simulation.h>
#include <sstream>
#include <set>

namespace f1tenth {

    Simulation::Simulation(
            Handle handle,
            Tag tag,
            std::shared_ptr<Map> map,
            bool use_lidar_lookup_table,
            VehicleDefinitions vehicle_definitions,
            VehicleStates initial_vehicle_states,
            SimulationScenario_CollisionBehavior collision_behavior)
            : handle_(handle)
            , tag_(tag)
            , map_(map)
            , use_lidar_lookup_table_(use_lidar_lookup_table)
            , marked_for_deletion_(false)
            , simulation_time_(0.0f) // always start at t=0
            , collision_behavior_(collision_behavior)
    {
        if(vehicle_definitions.size() != initial_vehicle_states.size()) {
            std::ostringstream o;
            o << "number of vehicle definitions (" << vehicle_definitions.size() <<
              ") does not match the number of initial vehicle states (" <<
              initial_vehicle_states.size() << ")";
            throw std::runtime_error(o.str());
        }

        for(unsigned int i=0; i<vehicle_definitions.size(); i++) {
            vehicles_.push_back(std::shared_ptr<Vehicle>(
                    new Vehicle(*vehicle_definitions[i], *initial_vehicle_states[i])));
        }

        checkForCollisions();
        generateLaserRangeScans();
    }

    void Simulation::writeStateTo(::f1tenth::SimulationState* simulation_state) const {

        mutex_.lock_shared(); // lock for read-only access

        simulation_state->mutable_vehicle_definitions()->Clear();
        simulation_state->mutable_vehicle_states()->Clear();
        simulation_state->mutable_vehicle_definitions()->Reserve(vehicles_.size());
        simulation_state->mutable_vehicle_states()->Reserve(vehicles_.size());

        for(std::shared_ptr<const Vehicle> vehicle : vehicles_) {
            simulation_state->mutable_vehicle_definitions()->Add()->CopyFrom(vehicle->getDefinition());
            simulation_state->mutable_vehicle_states()->Add()->CopyFrom(vehicle->getState());
        }
        simulation_state->set_simulation_handle(handle_);
        if(not tag_.empty()) {
            simulation_state->set_simulation_tag(tag_);
        }
        simulation_state->set_simulation_time(simulation_time_);
        simulation_state->set_uses_lidar_lookup_table(use_lidar_lookup_table_);

        mutex_.unlock_shared();
    }

    bool Simulation::step(float time_step, VehicleControlInputs vehicle_control_inputs) {
        mutex_.lock(); // lock for write access
        if(marked_for_deletion_) {
            mutex_.unlock();
            return(false);
        }

        // vehicle states at time t_{n+1} = t_{n} + time_step
        std::vector<std::shared_ptr<VehicleState> > next_vehicle_states;
        next_vehicle_states.resize(vehicles_.size());

        assert(vehicles_.size() == vehicle_control_inputs.size());

        #ifndef DO_NOT_USE_OPENMP
        #pragma omp parallel for
        #endif
        for(unsigned int i=0; i<vehicle_control_inputs.size(); i++) {
            const Vehicle &vehicle = *vehicles_.at(i);
            if(vehicle.getState().collision().in_collision()) {
                switch (collision_behavior_) {
                    default: {
                        throw std::runtime_error("undefined collision behavior");
                        //break;
                    }
                    case (::f1tenth::SimulationScenario_CollisionBehavior_STOP_VEHICLE): {
                        next_vehicle_states.at(i) = std::shared_ptr<VehicleState>(new VehicleState(vehicle.getState()));
                        continue;
                        //break;
                    }
                }
            }

            // control input at time t_{n}
            std::shared_ptr<VehicleControlInput> control_input(
                    new VehicleControlInput(*vehicle_control_inputs.at(i)));

            // Apply proportional control to convert from "legacy" control input (longitudinal velocity, steering
            // angle position) to "standard" control input (longitudinal acceleration, steering angle velocity),
            // if necessary. This functionality mimics the behavior of the "old" F1/tenth simulator
            // [https://github.com/f1tenth/f1tenth_simulator].
            vehicle.convertLegacyControlInput(control_input.get());

            // Apply steering and acceleration constraints to the control input.
            vehicle.applyControlInputConstraints(control_input.get());

            // Compute vehicle dynamics. Result is the next vehicle state at time t_{n+1} = t_{n} + time_step.
            std::shared_ptr<VehicleState> next_state = vehicle.computeDynamics(time_step, control_input.get());

            next_vehicle_states.at(i) = next_state;
        }

        // Update simulation time
        simulation_time_ += time_step;

        // Update vehicle states
        for(unsigned int i=0; i<next_vehicle_states.size(); i++) {
            vehicles_.at(i)->updateState(next_vehicle_states.at(i));
        }

        // Check for vehicle-vehicle and vehicle-obstacle collisions
        checkForCollisions();

        // Simulate laser range scans
        generateLaserRangeScans();

        mutex_.unlock();
        return(true);
    }

    void Simulation::checkForCollisions() {

        std::set<unsigned int> vehicles_in_collision;

        // Check for vehicle-map collisions
        for(unsigned int i=0; i<vehicles_.size(); i++) {
            if(vehicles_[i]->checkCollision(*map_)) {
                // collision between vehicles i and obstacles on the map
                vehicles_in_collision.insert(i);
            }
        }
        if(vehicles_.size() > 1) {
            for (unsigned int i = 0; i < vehicles_.size() - 1; i++) {
                for (unsigned int j = (i+1); j < vehicles_.size(); j++) {
                    if(vehicles_[i]->checkCollision(*vehicles_[j])) {
                        // collision between vehicle i and vehicle j
                        vehicles_in_collision.insert(i);
                        vehicles_in_collision.insert(j);
                    }
                }
            }
        }

        for(const int i : vehicles_in_collision) {

            switch(collision_behavior_) {
                default:
                {
                    throw std::runtime_error("undefined collision behavior");
                    //break;
                }
                case(::f1tenth::SimulationScenario_CollisionBehavior_STOP_VEHICLE):
                {
                    VehicleState new_state(vehicles_.at(i)->getState());
                    new_state.mutable_collision()->set_in_collision(true);
                    new_state.mutable_motion()->set_longitudinal_velocity(0.0f);
                    new_state.mutable_motion()->set_yaw_rate(0.0f);
                    new_state.mutable_motion()->set_slip_angle(0.0f);
                    vehicles_.at(i)->updateState(new_state);
                    break;
                }
                    /*
                    case(::f1tenth::SimulationScenario_CollisionBehavior_NEW_BEHAVIOR):
                    {
                        // IMPLEMENT NEW BEHAVIOR HERE...
                        break;
                    }
                    */
            }
        }
    }

    void Simulation::generateLaserRangeScans() {

        std::set<const Vehicle*> other_vehicles;
        for(unsigned int i=0; i<vehicles_.size(); i++) {
            other_vehicles.insert(vehicles_[i].get());
        }
        #ifndef DO_NOT_USE_OPENMP
        #pragma omp parallel for
        #endif
        for(unsigned int i=0; i<vehicles_.size(); i++) {
            vehicles_[i]->generateLaserRangeScan(*map_, use_lidar_lookup_table_, other_vehicles);
        }
    }

} // namespace f1tenth
