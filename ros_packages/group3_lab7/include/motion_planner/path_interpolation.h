/* -------------------------------------------------------
 * VU Autonomous Racing Cars (2020S) - TU Wien
 * -------------------------------------------------------
 * Team 3: Adelmann, Brantner, Lukitsch, Pintaric
 * -------------------------------------------------------
 */

#pragma once
#include <deque>
#include <Eigen/Dense>
#include <Eigen/Geometry>

template<typename T>
T LinearCurveLength(std::deque<Eigen::Matrix<T, 2, 1> > const &curve) {
    T sum = 0.0;
    for(unsigned int j=1; j < curve.size(); j++) {
        sum += (curve[j]-curve[j-1]).norm();
    }
    return(sum);
}

template<typename T, typename U>
std::vector<Eigen::Matrix<T, Eigen::Dynamic, 1> > UniformLinearInterpolation(
        const std::deque<Eigen::Matrix<U, 2, 1> > &curve,
        std::size_t target_size,
        const T known_linear_curve_length = 0) {

    std::vector<Eigen::Matrix<T, Eigen::Dynamic, 1> > interpolated_curve;
    if(curve.size() < 2 || target_size < 2) {
        // degenerate input curve or invalid target size
        return interpolated_curve;
    }
    const T total_length = (known_linear_curve_length > 0.0f) ?
            known_linear_curve_length : LinearCurveLength<U>(curve);

    const T segment_length = total_length / (T)(target_size - 1);

    auto start = 0;
    auto finish = start + 1;
    T src_segment_offset = 0.0f;
    T src_segment_length =  (curve[start] - curve[finish]).norm();

    interpolated_curve.push_back(curve[start].template cast<T>());

    for(size_t i=1; i<target_size-1; i++) {
        const T next_offset = segment_length * i;
        while(src_segment_offset + src_segment_length < next_offset) {
            src_segment_offset += src_segment_length;
            start = finish++;
            src_segment_length = (curve[start] - curve[finish]).norm();
        }
        const T part_offset = next_offset - src_segment_offset;
        const T part_ratio = part_offset / src_segment_length;
        interpolated_curve.push_back(Eigen::Matrix<T, 2, 1>(
                curve[start](0) +  part_ratio * (curve[finish](0) - curve[start](0)),
                curve[start](1) +  part_ratio * (curve[finish](1) - curve[start](1))));
    }
    interpolated_curve.push_back(curve.back().template cast<T>());
    return interpolated_curve;
}