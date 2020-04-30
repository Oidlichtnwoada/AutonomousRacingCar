#pragma once

#include <vector>
#include <Eigen/Geometry>
#include <cmath>
#include <Eigen/Dense>
#include <geometry_msgs/Point.h>

struct Point {
    float r, theta;
    Point();
    Point(float range, float angle);
    float distToPoint(const Point* pt2) const;
    float distToPoint2(const Point* pt2) const;
    float radialGap(const Point* pt2) const;
    float getX() const;
    float getY() const;
    bool operator<(const Point& p) const;
    bool operator>(const Point& p) const;
    void wrapTheta();
    void rotate(float phi);
    void translate(float x, float y);
    Eigen::Vector2f getVector() const;
    geometry_msgs::Point getPoint() const;
};

struct JumpTableEntry {
    int up_bigger;
    int up_smaller;
    int down_bigger;
    int down_smaller;
};

typedef std::vector<JumpTableEntry> JumpTable;
typedef std::vector<Point> Points;
JumpTable computeJumpTable(const Points& points);

struct SimpleCorrespondence {
    Eigen::Vector2f p_t0;
    Eigen::Vector2f p_t1;
};
typedef std::vector<SimpleCorrespondence> SimpleCorrespondences;

SimpleCorrespondences findCorrespondences(const Points& pts_t0,
                                          const Points& trans_pts_t1,
                                          const JumpTable& jump_table, /* computed from pts_t0 */
                                          const float max_correspondence_dist);
