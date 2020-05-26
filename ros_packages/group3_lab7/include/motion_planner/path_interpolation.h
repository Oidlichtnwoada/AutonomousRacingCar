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

float LinearCurveLength(const std::deque<Eigen::Vector2f> &curve);

std::deque<Eigen::Vector2f> UniformLinearInterpolation(
        const std::deque<Eigen::Vector2f> &curve,
        std::size_t number_of_points,
        const float known_linear_curve_length = 0.0f);
