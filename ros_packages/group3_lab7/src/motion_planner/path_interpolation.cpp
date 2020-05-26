/* -------------------------------------------------------
 * VU Autonomous Racing Cars (2020S) - TU Wien
 * -------------------------------------------------------
 * Team 3: Adelmann, Brantner, Lukitsch, Pintaric
 * -------------------------------------------------------
 */

#include <motion_planner/path_interpolation.h>

float LinearCurveLength(std::deque<Eigen::Vector2f> const &curve) {
    float sum = 0.0;
    for(unsigned int j=1; j < curve.size(); j++) {
        sum += (curve[j]-curve[j-1]).norm();
    }
    return(sum);
}

// Adapted from:
// https://stackoverflow.com/questions/43726836/resample-curve-into-even-length-segments-using-c

std::deque<Eigen::Vector2f> UniformLinearInterpolation(
        const std::deque<Eigen::Vector2f> &curve,
        std::size_t target_size,
        const float known_linear_curve_length) {

    std::deque<Eigen::Vector2f> interpolated_curve;
    if(curve.size() < 2 || target_size < 2) {
        // degenerate input curve or invalid target size
        return interpolated_curve;
    }
    const float total_length = (known_linear_curve_length > 0.0f) ?
            known_linear_curve_length : LinearCurveLength(curve);

    const float segment_length = total_length / (float)(target_size - 1);

    auto start = 0;
    auto finish = start + 1;
    float src_segment_offset = 0.0f;
    float src_segment_length =  (curve[start] - curve[finish]).norm();

    interpolated_curve.push_back(curve[start]);

    for(size_t i=1; i<target_size-1; i++) {
        const float next_offset = segment_length * i;
        while(src_segment_offset + src_segment_length < next_offset) {
            src_segment_offset += src_segment_length;
            start = finish++;
            src_segment_length = (curve[start] - curve[finish]).norm();
        }
        const float part_offset = next_offset - src_segment_offset;
        const float part_ratio = part_offset / src_segment_length;
        interpolated_curve.push_back(Eigen::Vector2f(
                curve[start](0) +  part_ratio * (curve[finish](0) - curve[start](0)),
                curve[start](1) +  part_ratio * (curve[finish](1) - curve[start](1))));
    }
    interpolated_curve.push_back(curve.back());
    return interpolated_curve;
}