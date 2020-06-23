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

#include <racecar_simulator/vehicle.h>
#include <boost/numeric/odeint.hpp>
#include <boost/lambda/lambda.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace f1tenth {

    Vehicle::Vehicle(const VehicleDefinition& definition,
                     const VehicleState& initial_state)
            : definition_(new VehicleDefinition(definition))
            , state_(new VehicleState(initial_state))
            , bounding_polygon_(computeBoundingPolygon())
            , random_device_()
            , random_generator_(random_device_())
            {
                const float mu = 0.0f;
                if(not definition_->has_laser_range_scanner_parameters()) {
                    throw std::runtime_error("laser range scanner parameters are missing");
                }
                const float sigma = std::abs(definition_->laser_range_scanner_parameters().gaussian_noise_sigma());
                add_gaussian_noise_ = (sigma > 0.0f);
                gaussian_noise_.param(std::normal_distribution<float>::param_type(mu, sigma));
            }

    void Vehicle::updateState(const VehicleState& new_state) {
        state_ = std::shared_ptr<VehicleState>(new VehicleState(new_state));
        bounding_polygon_ = computeBoundingPolygon();
    }

    void Vehicle::updateState(std::shared_ptr<VehicleState> new_state) {
        state_ = new_state;
        bounding_polygon_ = computeBoundingPolygon();
    }

    bool Vehicle::applyControlInputConstraints(VehicleControlInput* control_input) const {
        assert(control_input != nullptr);
        assert(control_input->has_standard_control_input());

        bool changes_were_made = false;

        // Apply longitudinal acceleration constraints
        float longitudinal_acceleration = control_input->standard_control_input().longitudinal_acceleration();
        {
            const float switching_velocity = definition_->constraints().switching_velocity();
            const float minimum_longitudinal_velocity = (-definition_->constraints().maximum_velocity());
            const float maximum_deceleration = definition_->constraints().maximum_deceleration();
            const float current_longitudinal_velocity = state_->motion().longitudinal_velocity();

            float acceleration_threshold = definition_->constraints().maximum_acceleration();
            if (current_longitudinal_velocity > switching_velocity) {
                acceleration_threshold *= (switching_velocity / current_longitudinal_velocity);
            }

            if ((current_longitudinal_velocity <= minimum_longitudinal_velocity) and
                (longitudinal_acceleration >= 0.0f)) {
                longitudinal_acceleration = 0.0f;
            } else if (longitudinal_acceleration <= (-maximum_deceleration)) {
                longitudinal_acceleration = (-maximum_deceleration);
            } else if (longitudinal_acceleration >= acceleration_threshold) {
                longitudinal_acceleration = acceleration_threshold;
            }
        }
        if(control_input->standard_control_input().longitudinal_acceleration() !=
           longitudinal_acceleration) {
            changes_were_made = true;
        }
        control_input->mutable_standard_control_input()->
                set_longitudinal_acceleration(longitudinal_acceleration);

        // Apply steering angle velocity constraints
        float steering_angle_velocity = control_input->standard_control_input().steering_angle_velocity();
        {
            const float minimum_steering_angle = definition_->constraints().minimum_steering_angle();
            const float maximum_steering_angle = definition_->constraints().maximum_steering_angle();
            const float current_steering_angle = state_->motion().steering_angle();

            if((current_steering_angle <= minimum_steering_angle and steering_angle_velocity <= 0.0f) or
               (current_steering_angle >= maximum_steering_angle and steering_angle_velocity >= 0.0f)) {
                steering_angle_velocity = 0.0f;
            } else if(steering_angle_velocity <= minimum_steering_angle) {
                steering_angle_velocity = minimum_steering_angle;
            } else if(steering_angle_velocity >= maximum_steering_angle) {
                steering_angle_velocity = maximum_steering_angle;
            }
        }
        if(control_input->standard_control_input().steering_angle_velocity() !=
           steering_angle_velocity) {
            changes_were_made = true;
        }
        control_input->mutable_standard_control_input()->
                set_steering_angle_velocity(steering_angle_velocity);

        return changes_were_made;
    }

    bool Vehicle::convertLegacyControlInput(VehicleControlInput* control_input) const {

        assert(control_input != nullptr);
        if (control_input->has_standard_control_input()) {
            return (false); // no conversion necessary
        }

        // Apply proportional control to convert from "legacy" control input (longitudinal velocity, steering
        // angle position) to "standard" control input (longitudinal acceleration, steering angle velocity),
        // if necessary. This functionality mimics the behavior of the "old" F1/tenth simulator
        // [https://github.com/f1tenth/f1tenth_simulator]. If your application was not explicitly written for the
        // "old" simulator, you should not use "legacy" control input. Instead, use an external PID-controller
        // to generate "standard" control input.
        //
        // References:
        // [1] https://github.com/f1tenth/f1tenth_simulator/blob/7a76970595c6af34454216ed5d7ea49bc0255085/node/simulator.cpp#L501
        // [2] https://github.com/f1tenth/f1tenth_simulator/blob/7a76970595c6af34454216ed5d7ea49bc0255085/node/simulator.cpp#L486

        assert(control_input->has_legacy_control_input());

        const float longitudinal_velocity_setpoint = control_input->legacy_control_input().longitudinal_velocity();
        const float steering_angle_position_setpoint = control_input->legacy_control_input().steering_angle();
        control_input->clear_legacy_control_input();

        float steering_angle_velocity = 0.0f;
        const float steering_angle_position_error = (steering_angle_position_setpoint -
                                                     state_->motion().steering_angle());

        if (steering_angle_position_error > 0.0001f /* see [2] */) {
            steering_angle_velocity = definition_->constraints().maximum_steering_velocity();
        } else if (steering_angle_position_error < (-0.0001f) /* see [2] */) {
            steering_angle_velocity = definition_->constraints().minimum_steering_velocity();
        }

        float longitudinal_acceleration = 0.0f;
        const float longitudinal_velocity_error = (longitudinal_velocity_setpoint -
                                                   state_->motion().longitudinal_velocity());

        const float maximum_acceleration = definition_->constraints().maximum_acceleration();
        const float maximum_deceleration = definition_->constraints().maximum_deceleration();
        const float maximum_velocity = definition_->constraints().maximum_velocity();

        /* see [1] */
        if (state_->motion().longitudinal_velocity() > 0.0f) {
            // positive velocity
            if (longitudinal_velocity_error > 0.0f) {
                // error is positive, accelerate
                const float kp = 2.0f * maximum_acceleration / maximum_velocity;
                longitudinal_acceleration = kp * longitudinal_velocity_error;
            } else {
                // error is negative, decelerate
                longitudinal_acceleration = (-maximum_deceleration);
            }
        } else if (state_->motion().longitudinal_velocity() < 0) {
            // negative velocity
            if (longitudinal_velocity_error > 0.0f) {
                // error is positive, decelerate
                longitudinal_acceleration = (+maximum_deceleration);
            } else {
                // error is negative, accelerate
                const float kp = 2.0 * maximum_acceleration / maximum_velocity;
                longitudinal_acceleration = kp * longitudinal_velocity_error;
            }
        } else {
            // zero speed, accelerate
            const float kp = 2.0 * maximum_acceleration / maximum_velocity;
            longitudinal_acceleration = kp * longitudinal_velocity_error;
        }

        control_input->mutable_standard_control_input()->set_longitudinal_acceleration(longitudinal_acceleration);
        control_input->mutable_standard_control_input()->set_steering_angle_velocity(steering_angle_velocity);

        assert(control_input->has_standard_control_input());
        return (true);
    }

    std::shared_ptr<VehicleState> Vehicle::computeDynamics(const float time_step,
                                                           const VehicleControlInput* control_input) const {

        using namespace boost::numeric::odeint;
        const float current_longitudinal_velocity = state_->motion().longitudinal_velocity();

        std::shared_ptr<VehicleState> next_state(new VehicleState(*state_));

        constexpr double t0 = 0.0; // [s]
        const double t1 = (double) time_step; // [s]
        const double dt = std::min(0.001, (t1-t0) / 2.0); // [s]

        // Initial state is x(t0), which will be changed to the
        // approximate solution x(t1) at the end of integration.
        typedef std::vector<double> state_type;
        state_type x = {
                (double) state_->pose().x(),
                (double) state_->pose().y(),
                (double) state_->motion().steering_angle(),
                (double) state_->motion().longitudinal_velocity(),
                (double) state_->pose().yaw(),
                (double) state_->motion().yaw_rate(),
                (double) state_->motion().slip_angle()
        };

        const double v_delta = control_input->standard_control_input().steering_angle_velocity();
        const double ax = control_input->standard_control_input().longitudinal_acceleration();

        if(current_longitudinal_velocity < 0.1) {

            // ---------------------------------------------------------------------------------------------------------
            // Use kinematic single-track vehicle dynamics for small velocities
            // ---------------------------------------------------------------------------------------------------------

            const double wheelbase = definition_->parameters().l_f() + definition_->parameters().l_r();
            x.resize(5);

            // integrate(system, x0, t0, t1, dt) uses runge_kutta_dopri5 with standard error bounds 1E-6 for the steps.
            // https://www.boost.org/doc/libs/release/libs/numeric/odeint/doc/html/boost_numeric_odeint/odeint_in_detail/integrate_functions.html
            const size_t number_of_steps = integrate([=](const state_type &x ,state_type &dxdt, double t) {
                dxdt[0] = x[3] * std::cos(x[4]);
                dxdt[1] = x[3] * std::sin(x[4]);
                dxdt[2] = v_delta;
                dxdt[3] = ax;
                dxdt[4] = x[3] / wheelbase * std::tan(x[2]);
            }, x, t0, t1, dt);

            x.resize(7);
            x[5] = ax / wheelbase * std::tan(x[2]) + x[3] / (wheelbase * std::pow(std::cos(x[2]),2)) * v_delta; // yaw rate
            x[6] = 0.0f; // slip angle

            assert(number_of_steps >= 1);

        } else {

            // ---------------------------------------------------------------------------------------------------------
            // Single-track vehicle dynamics
            // ---------------------------------------------------------------------------------------------------------

            constexpr double g = 9.81; // gravity [m/s^2]
            const double mu = definition_->parameters().mu();
            const double C_Sf = definition_->parameters().c_sf();
            const double C_Sr = definition_->parameters().c_sr();
            const double lf = definition_->parameters().l_f();
            const double lr = definition_->parameters().l_r();
            const double h = definition_->parameters().h_cg();
            const double m = definition_->parameters().m();
            const double I = definition_->parameters().i_z();

            // integrate(system, x0, t0, t1, dt) uses runge_kutta_dopri5 with standard error bounds 1E-6 for the steps.
            // https://www.boost.org/doc/libs/release/libs/numeric/odeint/doc/html/boost_numeric_odeint/odeint_in_detail/integrate_functions.html
            const size_t number_of_steps = integrate([=](const state_type &x ,state_type &dxdt, double t) {
                dxdt[0] = x[3] * std::cos(x[6] + x[4]);
                dxdt[1] = x[3] * std::sin(x[6] + x[4]);
                dxdt[2] = v_delta;
                dxdt[3] = ax;
                dxdt[4] = x[5];
                dxdt[5] = -mu * m / (x[3] * I * (lr + lf)) * (lf * lf * C_Sf * (g * lr - ax * h) + lr * lr * C_Sr * (g * lf + ax * h)) * x[5] +
                          mu * m / (I * (lr + lf)) * (lr * C_Sr * (g * lf + ax * h) - lf * C_Sf * (g * lr - ax * h)) * x[6] +
                          mu * m / (I * (lr + lf)) * lf * C_Sf * (g * lr - ax * h) * x[2];
                dxdt[6] = (mu / (x[3] * x[3] * (lr + lf)) * (C_Sr * (g * lf + ax * h) * lr - C_Sf * (g * lr - ax * h) * lf) - 1) * x[5] -
                          mu / (x[3] * (lr + lf)) * (C_Sr * (g * lf + ax * h) + C_Sf * (g * lr - ax * h)) * x[6] +
                          mu / (x[3] * (lr + lf)) * (C_Sf * (g * lr - ax * h)) * x[2];
            }, x, t0, t1, dt);

            assert(number_of_steps >= 1);
        }

        next_state->mutable_pose()->set_x(x[0]);
        next_state->mutable_pose()->set_y(x[1]);
        next_state->mutable_motion()->set_steering_angle(x[2]);
        next_state->mutable_motion()->set_longitudinal_velocity(x[3]);
        next_state->mutable_pose()->set_yaw(x[4]);
        next_state->mutable_motion()->set_yaw_rate(x[5]);
        next_state->mutable_motion()->set_slip_angle(x[6]);
        return(next_state);
    }

    bool Vehicle::checkCollision(const Vehicle& other_vehicle) const {
        const bool bounding_polygons_intersect =
                boost::geometry::intersects(bounding_polygon_, other_vehicle.bounding_polygon_);
        return bounding_polygons_intersect;
    }

    bool Vehicle::checkCollision(const octomap::OcTree& octree) const {
        const size_t number_of_points = bounding_polygon_.outer().size();
        for(size_t i=0; i<number_of_points; i++) {
            const size_t j = (i+1) % number_of_points;
            const auto& point_i = bounding_polygon_.outer().at(i);
            const auto& point_j = bounding_polygon_.outer().at(j);

            auto p0x = point_i.get<0>();
            auto p0y = point_i.get<1>();
            auto p1x = point_j.get<0>();
            auto p1y = point_j.get<1>();

            octomap::KeyRay keys;
            octree.computeRayKeys(octomap::point3d(p0x,p0y,0.0),
                                  octomap::point3d(p1x,p1y,0.0),
                                  keys);

            for(const octomap::OcTreeKey& key : keys) {
                const octomap::OcTreeNode* node = octree.search(key);
                if(node != nullptr) {
                    const bool node_is_occupied = octree.isNodeOccupied(node);
                    if(node_is_occupied) {
                        return true; // we have a collision
                    }
                }
            }
        }
        return false;
    }

    bool Vehicle::checkCollision(const Map& map) const {
        return checkCollision(*map.getOctree());
    }

    geometry::Polygon Vehicle::computeBoundingPolygon() const {

        const float vehicle_length = definition_->parameters().l(); // vehicle length [m]
        const float vehicle_width = definition_->parameters().w();  // vehicle width [m]
        const float x = state_->pose().x();
        const float y = state_->pose().y();
        const float yaw = state_->pose().yaw();

        // NOTE: origin (0,0) is the center of the rear axle
        const auto T = Eigen::Translation2f(x,y) * Eigen::Rotation2Df(yaw);
        const Eigen::Vector2f p0 = T * Eigen::Vector2f(0.0f, -vehicle_width/2.0f);
        const Eigen::Vector2f p1 = T * Eigen::Vector2f(0.0f, +vehicle_width/2.0f);
        const Eigen::Vector2f p2 = T * Eigen::Vector2f(vehicle_length, +vehicle_width/2.0f);
        const Eigen::Vector2f p3 = T * Eigen::Vector2f(vehicle_length, -vehicle_width/2.0f);

        geometry::Polygon polygon { { {p0.x(), p0.y()},
                                      {p1.x(), p1.y()},
                                      {p2.x(), p2.y()},
                                      {p3.x(), p3.y()} } };
        return(polygon);
    }

    void Vehicle::generateLaserRangeScan(const Map& map,
                                         bool use_lidar_lookup_table,
                                         std::set<const Vehicle*> other_vehicles) {

        use_lidar_lookup_table = (use_lidar_lookup_table and map.supportsLookupTable());
        VehicleState_LaserRangeScan* laser_range_scan = state_->mutable_laser_range_scan();

        if(not laser_range_scan->has_parameters()) {
            laser_range_scan->mutable_parameters()->CopyFrom(
                    definition_->laser_range_scanner_parameters());
        }
        laser_range_scan->clear_ranges();

        const float distance_from_rear_axle = laser_range_scan->parameters().distance_from_rear_axle();

        const auto T = Eigen::Translation2f(state_->pose().x(), state_->pose().y()) *
                       Eigen::Rotation2Df(state_->pose().yaw());

        const Eigen::Vector2f ray_center =
                T * Eigen::Vector2f(distance_from_rear_axle, 0.0f);

        const float minimum_range = laser_range_scan->parameters().minimum_range();
        if(minimum_range > 0.0f) {
            // minimum_range > 0 is not currently implemented.
            throw std::runtime_error("minimum_range must be zero");
        }

        const float maximum_range = laser_range_scan->parameters().maximum_range();
        const float minimum_ray_angle = laser_range_scan->parameters().minimum_angle();
        const float ray_angle_increment = laser_range_scan->parameters().angular_increment();

        std::vector<float> ray_lengths;
        ray_lengths.resize(laser_range_scan->parameters().number_of_rays());

        #ifndef DO_NOT_USE_OPENMP
        #pragma omp parallel for
        #endif
        for(unsigned int n=0; n < laser_range_scan->parameters().number_of_rays(); n++) {

            const float ray_angle = minimum_ray_angle + (float) n * ray_angle_increment;
            const Eigen::Vector2f ray_direction =
                    Eigen::Rotation2Df(state_->pose().yaw()) *
                    Eigen::Rotation2Df(ray_angle) *
                    Eigen::Vector2f(1.0f, 0.0f);

            // compute vehicle-to-map ray distances

            auto [got_a_hit, ray_length, ray_endpoint] = \
                use_lidar_lookup_table ?
                map.lookupRayLength(ray_center, state_->pose().yaw() + ray_angle, maximum_range) :
                map.castRay(ray_center, ray_direction, maximum_range);

            assert((not map.supportsLookupTable()) or got_a_hit);

            // check vehicle-to-vehicle ray distances

            const geometry::Linestring ray { {ray_center.x(), ray_center.y()},
                                             {ray_endpoint.x(), ray_endpoint.y()} };

            for(const Vehicle* other_vehicle : other_vehicles) {
                if(other_vehicle == this) {
                    continue;
                }

                if(boost::geometry::intersects(ray, other_vehicle->bounding_polygon_)) {
                    geometry::Multipoint points_of_intersection;
                    boost::geometry::intersection(ray, other_vehicle->bounding_polygon_, points_of_intersection);
                    for(const geometry::Point& point : points_of_intersection) {
                        const float length = (Eigen::Vector2f(point.get<0>(), point.get<1>()) - ray_center).norm();
                        ray_length = std::min(ray_length, length);
                    }
                }
            }

            if(add_gaussian_noise_) {
                ray_length += gaussian_noise_(random_generator_);
            }

            if(ray_length < minimum_range) {
                ray_length = minimum_range;
            }

            if(ray_length > maximum_range) {
                ray_length = maximum_range;
            }

            ray_lengths[n] = ray_length;
        }

        auto ranges = state_->mutable_laser_range_scan()->mutable_ranges();
        ranges->Clear();
        ranges->Add(ray_lengths.begin(), ray_lengths.end());
    }

} // namespace f1tenth
