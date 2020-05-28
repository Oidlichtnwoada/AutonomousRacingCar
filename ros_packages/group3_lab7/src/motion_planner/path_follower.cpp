/* -------------------------------------------------------
 * VU Autonomous Racing Cars (2020S) - TU Wien
 * -------------------------------------------------------
 * Team 3: Adelmann, Brantner, Lukitsch, Pintaric
 * -------------------------------------------------------
 */

#include <motion_planner/path_follower.h>
#include <motion_planner/path_interpolation.h>
#include <algorithm>

PathFollower::Options::Options()
        : path_segment_length_(default_path_segment_length)
{}

PathFollower::Options::Options(const Configuration& configuration)
        : path_segment_length_(configuration.path_segment_length)
{}

PathFollower::PathFollower(Options options) : options_(options) {}


std::tuple<bool, PathFollower::Path>
PathFollower::refinePath(const Path& path) {

    if(path.size() < 2) {
        Path empty_path;
        return {false, empty_path};
    }

    const float linear_path_length = LinearCurveLength<float>(path);

    constexpr unsigned int minimum_number_of_points_required = 5;
    const unsigned int number_of_points_on_resampled_path = std::max<unsigned int>(
            (unsigned int) std::ceil(linear_path_length / options_.path_segment_length_),
            minimum_number_of_points_required);

    const std::deque<Eigen::Vector2f> resampled_path = UniformLinearInterpolation<float>(
            path, number_of_points_on_resampled_path, linear_path_length);

    assert(resampled_path.size() >= minimum_number_of_points_required);
    return {true, resampled_path};
}