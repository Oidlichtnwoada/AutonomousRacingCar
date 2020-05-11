#pragma once

#include <scan_matcher/correspond.h>
#define EPSILON 0.00001

struct Transform {
    float x_disp, y_disp, theta_rot;
    Transform();
    Transform(float x_disp, float y_disp, float theta_rot);
    bool operator==(const Transform&) const;
    bool operator!=(const Transform&) const;
    Transform operator+(const Transform&) const;
    Point apply(const Point) const;
    Eigen::Matrix3f getMatrix() const;
    Transform inverse() const;
};

std::vector<Point> transformPoints(const std::vector<Point>& points, const Transform& t);
Transform estimateTransformation(const SimpleCorrespondences& correspondences);