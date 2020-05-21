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

class SimpleCorrespondence {
public:
    SimpleCorrespondence(Eigen::Vector2f p0,
                         Eigen::Vector2f p0_second_best,
                         Eigen::Vector2f p1,
                         unsigned int idx_p0,
                         unsigned int idx_p1,
                         float distance = (-1.f));

    const Eigen::Vector2f& p_t0() const;
    const Eigen::Vector2f& p_t1() const;
    const Eigen::Vector2f& nn() const;
    unsigned int idx_p0() const;
    unsigned int idx_p1() const;
    float distance() const;

protected:
    Eigen::Vector2f p_t0_;
    Eigen::Vector2f p_t1_;
    Eigen::Vector2f nn_;
    unsigned int idx_p0_;
    unsigned int idx_p1_;
    float distance_;
};

typedef std::vector<SimpleCorrespondence> SimpleCorrespondences;

SimpleCorrespondences findCorrespondences(const Points& pts_t0,
                                          const Points& trans_pts_t1,
                                          const JumpTable& jump_table, /* computed from pts_t0 */
                                          float max_correspondence_dist = std::numeric_limits<float>::max(),
                                          float inlier_ratio = 0.9f);
