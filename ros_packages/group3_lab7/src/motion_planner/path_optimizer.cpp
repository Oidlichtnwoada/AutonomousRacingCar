/* -------------------------------------------------------
 * VU Autonomous Racing Cars (2020S) - TU Wien
 * -------------------------------------------------------
 * Team 3: Adelmann, Brantner, Lukitsch, Pintaric
 * -------------------------------------------------------
 */

#include <motion_planner/path_optimizer.h>
#include <algorithm>

namespace motion_planner {


    PathOptimizer::Options::Options()
            : path_segment_length_(default_path_segment_length) {}

    PathOptimizer::Options::Options(const Configuration &configuration)
            : path_segment_length_(configuration.path_segment_length) {}

    PathOptimizer::PathOptimizer(Options options) : options_(options) {}


    std::tuple<bool, PathOptimizer::GridPath>
    PathOptimizer::optimizePath(
            const GridPath &grid_path,
            const OccupancyGrid &occupancy_grid) {

        if (grid_path.size() < 2) {
            GridPath empty_path;
            return {false, empty_path};
        }

        const Path<float> path = ConvertPath<float, int>(grid_path);
        const float linear_path_length = LinearPathLength<float>(path);

        constexpr unsigned int minimum_number_of_points_required = 5;
        const unsigned int number_of_points_on_resampled_path = std::max<unsigned int>(
                (unsigned int) std::ceil(linear_path_length / options_.path_segment_length_),
                minimum_number_of_points_required);

        const std::deque<Eigen::Vector2f> resampled_path = UniformLinearInterpolation<float>(
                path, number_of_points_on_resampled_path, linear_path_length);

        assert(resampled_path.size() >= minimum_number_of_points_required);
        return {true, ConvertPath<int, float>(resampled_path)};
    }

} // namespace motion_planner