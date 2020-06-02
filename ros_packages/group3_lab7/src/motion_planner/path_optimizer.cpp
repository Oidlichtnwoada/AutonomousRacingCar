/* -------------------------------------------------------
 * VU Autonomous Racing Cars (2020S) - TU Wien
 * -------------------------------------------------------
 * Team 3: Adelmann, Brantner, Lukitsch, Pintaric
 * -------------------------------------------------------
 */

#include <motion_planner/path_optimizer.h>
#include <algorithm>

#define OPTIM_ENABLE_EIGEN_WRAPPERS
#include <optim.hpp>

using namespace optim;

namespace motion_planner {

    struct OptimizationProblem {

        OptimizationProblem(Path<float> path, boost::shared_ptr<const OccupancyGrid> occupancy_grid)
                : path_(path)
                , normals_(ApproximateNormals<float>(path_))
                , occupancy_grid_(occupancy_grid)
                , path_length_(LinearPathLength<float>(path_))
        {
            assert(path_.size() >= 2);
            initial_distance_from_nearest_obstacle_.resize(path_.size());
            average_distance_from_nearest_obstacle_ = 0.0f;
            for(int i=0; i<path_.size(); i++) {
                const Eigen::Vector2f& waypoint = path_.at(i);
                const float distance_from_nearest_obstacle =
                        occupancy_grid_->interpolatedDistanceFromNearestObstacle(waypoint);
                initial_distance_from_nearest_obstacle_[i] = distance_from_nearest_obstacle;
                average_distance_from_nearest_obstacle_ += distance_from_nearest_obstacle;
            }
            average_distance_from_nearest_obstacle_ /= (float) path_.size();
        }

        Path<float> applyToPath(const optim::Vec_t& x) const {
            assert(path_.size() - 2 == x.size());
            Path<float> path(path_.size());
            path.front() = path_.front();
            for(int i=1; i < path_.size()-1; i++) {
                path[i] = path_[i] + normals_[i] * x[i-1];
            }
            path.back() = path_.back();
            return(path);
        }

        double cost(const optim::Vec_t& x) const {
            assert(x.size() == path_.size()-2);

            Path<float> modified_path = applyToPath(x);
            const float modified_path_length = LinearPathLength<float>(modified_path);

            const float avg_path_length_increase =
                    (modified_path_length - path_length_) / (float) modified_path.size();

            float sum_of_weighted_distance_improvements = 0.0f;

            for(int i=0; i<modified_path.size(); i++) {

                if(modified_path[i](0) < 0 ||
                   modified_path[i](0) >= occupancy_grid_->rows() ||
                   modified_path[i](1) < 0 ||
                   modified_path[i](1) >= occupancy_grid_->cols() ||
                   occupancy_grid_->isGridCellOccupied(modified_path[i].cast<int>())) {

                    //return std::numeric_limits<double>::max();
                    return(1E6); // return some large value, but not std::numeric_limits<double>::max()
                }

                const float distance_from_nearest_obstacle =
                        occupancy_grid_->interpolatedDistanceFromNearestObstacle(modified_path[i]);

                float distance_improvement =
                        (distance_from_nearest_obstacle - initial_distance_from_nearest_obstacle_[i]);

                if(distance_improvement < 0.0f) {
                    // assign penalty
                    constexpr float kappa = 1.0f; // <--- parameter!
                    sum_of_weighted_distance_improvements += distance_improvement * kappa;

                } else {

                    constexpr float tau = 0.5f; // <--- parameter!
                    const float weighted_distance_improvement =
                            (2.0f * (1.0f / (1.0f + std::exp(-distance_improvement * tau))) - 1.0f);
                    sum_of_weighted_distance_improvements += weighted_distance_improvement;
                }
            }

            constexpr float rho = 0.5f; // <--- parameter!
            const float cost = (-sum_of_weighted_distance_improvements / (float) modified_path.size()) +
                               (avg_path_length_increase * rho);
            return(cost);
        }

        Path<float> path_;
        Path<float> normals_;
        boost::shared_ptr<const OccupancyGrid> occupancy_grid_;
        float path_length_;
        float average_distance_from_nearest_obstacle_;
        std::vector<float> initial_distance_from_nearest_obstacle_;
    };

    double optimization_fn(const Vec_t& vals_inp, Vec_t* grad_out, void* opt_data)
    {
        assert(opt_data != nullptr);
        const OptimizationProblem &problem = *((OptimizationProblem*) opt_data);
        double cost = (double) problem.cost(vals_inp);

        // numerical gradient computation
        if(grad_out != nullptr) {
            const float epsilon = 0.5;
            Vec_t v = vals_inp;
            for(int j=0; j<vals_inp.size(); j++) {
                v[j] = vals_inp[j] + epsilon;
                const float cost_plus_epsilon = problem.cost(v);
                v[j] = vals_inp[j] - epsilon;
                const float cost_minus_epsilon = problem.cost(v);
                v[j] = vals_inp[j];
                (*grad_out)[j] = (cost_plus_epsilon - cost_minus_epsilon) / (2.0 * epsilon);
            }
        }
        return(cost);
    }
}



namespace motion_planner {


    PathOptimizer::Options::Options()
        : number_of_resampled_path_segments_(default_number_of_resampled_path_segments)
        , number_of_optimization_iterations_(default_number_of_optimization_iterations)
        , enable_path_optimization_(default_enable_path_optimization)
        , maximum_waypoint_translation_along_normal_(default_maximum_waypoint_translation_along_normal)
        , path_optimization_step_size_(default_path_optimization_step_size)
        {}

    PathOptimizer::Options::Options(const Configuration &configuration)
            : number_of_resampled_path_segments_(configuration.number_of_resampled_path_segments)
            , number_of_optimization_iterations_(configuration.number_of_optimization_iterations)
            , enable_path_optimization_(configuration.enable_path_optimization)
            , maximum_waypoint_translation_along_normal_(configuration.maximum_waypoint_translation_along_normal)
            , path_optimization_step_size_(configuration.path_optimization_step_size)
    {}

    PathOptimizer::PathOptimizer(Options options) : options_(options) {}


    std::tuple<bool, PathOptimizer::FloatingPointGridPath>
    PathOptimizer::optimizePath(
            const GridPath &grid_path,
            boost::shared_ptr<OccupancyGrid> occupancy_grid) {

        assert(occupancy_grid);

        if (grid_path.size() < 2) {
            FloatingPointGridPath empty_path;
            return {false, empty_path};
        }

        const Path<float> path = ConvertPath<int,float>(grid_path);
        const float linear_path_length = LinearPathLength<float>(path);

        const unsigned int number_of_points_on_resampled_path =
                (options_.number_of_resampled_path_segments_ + 1);

        const std::deque<Eigen::Vector2f> resampled_path = UniformLinearInterpolation<float>(
                path, number_of_points_on_resampled_path, linear_path_length);

        if(!options_.enable_path_optimization_ ||
           options_.number_of_optimization_iterations_ == 0 ||
           options_.maximum_waypoint_translation_along_normal_ == 0.0f) {
            return {true, resampled_path};
        }

        if(not occupancy_grid->hasDistances()) {
            occupancy_grid->computeDistanceTransform();
        }

        OptimizationProblem problem(resampled_path, occupancy_grid);
        optim::Vec_t x = optim::Vec_t::Zero(resampled_path.size()-2);

        optim::algo_settings_t settings;
        settings.vals_bound = true;
        const float delta = std::abs(options_.maximum_waypoint_translation_along_normal_);
        settings.lower_bounds = optim::Vec_t::Ones(resampled_path.size()-2) * (-delta);
        settings.upper_bounds = optim::Vec_t::Ones(resampled_path.size()-2) * (+delta);
        settings.gd_method = 6;
        settings.gd_settings.step_size = options_.path_optimization_step_size_;
        settings.iter_max = options_.number_of_optimization_iterations_;
        const bool success = optim::gd(x, optimization_fn, &problem, settings);
        const Path<float> optimized_path = problem.applyToPath(x);

        return {true, optimized_path};
    }

} // namespace motion_planner