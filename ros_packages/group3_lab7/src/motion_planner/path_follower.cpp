/* -------------------------------------------------------
 * VU Autonomous Racing Cars (2020S) - TU Wien
 * -------------------------------------------------------
 * Team 3: Adelmann, Brantner, Lukitsch, Pintaric
 * -------------------------------------------------------
 */

#include <motion_planner/path_follower.h>
#include <motion_planner/path_interpolation.h>
#include <boost/math/interpolators/cubic_b_spline.hpp>
#include <algorithm>

PathFollower::Options::Options()
        : cubic_spline_points_(default_cubic_spline_points)
        , interpolated_points_on_path_(default_interpolated_points_on_path)
{}

PathFollower::Options::Options(const Configuration& configuration)
        : cubic_spline_points_(configuration.cubic_spline_points)
        , interpolated_points_on_path_(configuration.interpolated_points_on_path)
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
    //const unsigned int number_of_points_on_resampled_path = std::max<unsigned int>(
    //        (unsigned int) std::ceil(linear_path_length / options_.path_segment_length_),
    //        minimum_number_of_points_required);

    const unsigned int number_of_points_on_resampled_path = std::max<unsigned int>(
            options_.cubic_spline_points_,
            minimum_number_of_points_required);

    const std::vector<Eigen::VectorXf> resampled_path = UniformLinearInterpolation<float, float>(
            path, number_of_points_on_resampled_path, linear_path_length);

    assert(resampled_path.size() >= minimum_number_of_points_required);

    std::vector<float> x(resampled_path.size());
    std::vector<float> y(resampled_path.size());
    for(unsigned int i=0; i<resampled_path.size(); i++) {
        x[i] = resampled_path[i](0);
        y[i] = resampled_path[i](1);
    }

    const float t_step = 1.0 / (float) (options_.interpolated_points_on_path_-1);
    std::vector<float> t(options_.interpolated_points_on_path_);
    std::generate(t.begin(), t.end(), [n = 0, &t_step]() mutable { return (float)(n++) * t_step; });

    const float step = 1.0 / (float) (resampled_path.size()-1);
    boost::math::cubic_b_spline<float> spline_x(x.data(), x.size(), 0.0f /* start time */, step);
    boost::math::cubic_b_spline<float> spline_y(y.data(), y.size(), 0.0f /* start time */, step);

    Path interpolated_path;

    for(unsigned int i=0; i<t.size(); i++) {
        if(t[i] >= 1.0) {
            break;
        }
        interpolated_path.push_back(Eigen::Vector2f(spline_x(t[i]), spline_y(t[i])));
    }

    return {true, interpolated_path};
}