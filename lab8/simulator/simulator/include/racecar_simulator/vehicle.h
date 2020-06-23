/*
 * ---------------------------------------------------------------------------------------------------------------------
 * Author: Thomas Pintaric (thomas.pintaric@gmail.com)
 * ---------------------------------------------------------------------------------------------------------------------
 * Vehicle dynamics models are based on CommonRoad [1] by the Cyber-Physical System Group at TU Munich.
 * "Legacy" control input conversion is based on the F1/tenth Simulator [2] code from the University of Pennsylvania.
 *
 * [1] M. Althoff, M. Koschi, and S. Manzinger, 'CommonRoad: Composable Benchmarks for Motion Planning
 * on Roads,' in Proc. of the IEEE Intelligent Vehicles Symposium, 2017, pp. 719-726.
 * [2] https://github.com/f1tenth/f1tenth_simulator
 * ---------------------------------------------------------------------------------------------------------------------
 * SPDX-License-Identifier: GPL-3.0-or-later
 * ---------------------------------------------------------------------------------------------------------------------
 */

#pragma once
#include <memory>
#include <set>
#include <random>
#include "racecar_simulator.grpc.pb.h"
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <octomap/octomap.h>
#include <racecar_simulator/map.h>

namespace f1tenth {

    namespace geometry { // namespace f1tenth::geometry
        typedef boost::geometry::model::point<float, 2, boost::geometry::cs::cartesian> Point;
        typedef boost::geometry::model::linestring<Point> Linestring;
        typedef boost::geometry::model::polygon<Point> Polygon;
        typedef boost::geometry::model::multi_point<Point> Multipoint;
    } // namespace f1tenth::geometry

    typedef std::vector<std::shared_ptr<const VehicleDefinition> > VehicleDefinitions;
    typedef std::vector<std::shared_ptr<const VehicleState> > VehicleStates;
    typedef std::vector<std::shared_ptr<const VehicleControlInput> > VehicleControlInputs;

    class Vehicle {
    public:
        Vehicle(const VehicleDefinition& definition,
                const VehicleState& initial_state);

        void updateState(const VehicleState& new_state);
        void updateState(std::shared_ptr<VehicleState> new_state);

        inline const VehicleState& getState() const { return *state_; }
        inline const VehicleDefinition& getDefinition() const { return *definition_; }

        bool convertLegacyControlInput(VehicleControlInput* control_input) const;
        bool applyControlInputConstraints(VehicleControlInput* control_input) const;
        std::shared_ptr<VehicleState> computeDynamics(const float time_step, const VehicleControlInput* control_input) const;

    public:
        bool checkCollision(const Vehicle& other_vehicle) const;
        bool checkCollision(const Map& map) const;
        void generateLaserRangeScan(const Map& map,
                                    bool use_lidar_lookup_table,
                                    std::set<const Vehicle*> other_vehicles);

    private:
        bool checkCollision(const octomap::OcTree& octree) const;
        /*
        void generateLaserRangeScan(const octomap::OcTree& octree,
                                    std::set<const Vehicle*> other_vehicles);
                                    */

        const std::shared_ptr<const VehicleDefinition> definition_;
        std::shared_ptr<VehicleState> state_;

        geometry::Polygon computeBoundingPolygon() const;
        geometry::Polygon bounding_polygon_;

        std::random_device random_device_;
        std::mt19937 random_generator_;
        std::normal_distribution<float> gaussian_noise_;
        bool add_gaussian_noise_;


    private:
        Vehicle(); // hide default constructor
        Vehicle(const Vehicle&); // hide default copy constructor

    };

} // namespace f1tenth
