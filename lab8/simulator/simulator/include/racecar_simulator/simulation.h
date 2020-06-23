/*
 * ---------------------------------------------------------------------------------------------------------------------
 * Author: Thomas Pintaric (thomas.pintaric@gmail.com)
 * SPDX-License-Identifier: GPL-3.0-or-later
 * ---------------------------------------------------------------------------------------------------------------------
 */

#pragma once
#include <memory>
#include <mutex>
#include <shared_mutex>
#include "racecar_simulator.grpc.pb.h"
#include <racecar_simulator/vehicle.h>
#include <racecar_simulator/map.h>

namespace f1tenth {

    class SimulationServer;

    class Simulation {

    public:
        typedef uint64_t Handle;
        typedef std::string Tag;
        typedef ::f1tenth::Map Map;
        typedef ::f1tenth::Vehicle Vehicle;

        Simulation(Handle handle,
                   Tag tag,
                   std::shared_ptr<Map> map,
                   bool use_lidar_lookup_table,
                   ::f1tenth::VehicleDefinitions vehicle_definitions,
                   ::f1tenth::VehicleStates initial_vehicle_states,
                   ::f1tenth::SimulationScenario_CollisionBehavior collision_behavior);

        bool step(float time_step, ::f1tenth::VehicleControlInputs vehicle_control_inputs);
        void writeStateTo(::f1tenth::SimulationState* simulation_state) const;

        inline float getSimulationTime() const {
            return simulation_time_;
        }
        inline unsigned int getNumberOfVehicles() const {
            return(vehicles_.size());
        }

    private:
        friend SimulationServer;
        mutable std::shared_mutex mutex_;

        bool marked_for_deletion_;
        float simulation_time_; // [s]

        const Handle handle_;
        const Tag tag_;
        const std::shared_ptr<Map> map_;
        const bool use_lidar_lookup_table_;
        std::vector<std::shared_ptr<Vehicle> > vehicles_;
        ::f1tenth::SimulationScenario_CollisionBehavior collision_behavior_;

        void checkForCollisions();
        void generateLaserRangeScans();

    private:
        Simulation(); // hide default constructor

    };

} // namespace f1tenth
