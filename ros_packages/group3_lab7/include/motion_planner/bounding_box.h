/* -------------------------------------------------------
 * VU Autonomous Racing Cars (2020S) - TU Wien
 * -------------------------------------------------------
 * Team 3: Adelmann, Brantner, Lukitsch, Pintaric
 * -------------------------------------------------------
 */

#pragma once
#include <Eigen/Core>
#include <Eigen/Dense>
#include <deque>
#include <limits>

namespace motion_planner {

    class BoundingBox {
    public:
        BoundingBox();
        bool isValid() const;
        float computeArea() const;
        float computeOverlap(const BoundingBox &other_bounding_box) const;

        template<typename T>
        void addPoint(const Eigen::Matrix<T, 2, 1> &point) {
            lower_bounds_(0) = std::min<float>(lower_bounds_(0), (float) point(0));
            lower_bounds_(1) = std::min<float>(lower_bounds_(1), (float) point(1));
            upper_bounds_(0) = std::max<float>(upper_bounds_(0), (float) point(0));
            upper_bounds_(1) = std::max<float>(upper_bounds_(1), (float) point(1));
        }

        template<typename T>
        static BoundingBox fromPath(const std::deque<Eigen::Matrix<T, 2, 1> > &path) {
            BoundingBox bb;
            for (auto const &point : path) {
                bb.addPoint<T>(point);
            }
            return (bb);
        }

    protected:
        Eigen::Vector2f lower_bounds_;
        Eigen::Vector2f upper_bounds_;

    };
} // namespace motion_planner
