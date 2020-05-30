/* -------------------------------------------------------
 * VU Autonomous Racing Cars (2020S) - TU Wien
 * -------------------------------------------------------
 * Team 3: Adelmann, Brantner, Lukitsch, Pintaric
 * -------------------------------------------------------
 */

#include <motion_planner/bounding_box.h>

namespace motion_planner {

    BoundingBox::BoundingBox()
            : lower_bounds_(std::numeric_limits<float>::max(), std::numeric_limits<float>::max()),
              upper_bounds_(std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest()) {}

    bool BoundingBox::isValid() const {
        return (upper_bounds_(0) > lower_bounds_(0)) &&
               (upper_bounds_(1) > lower_bounds_(1));
    }

    float BoundingBox::computeArea() const {
        const Eigen::Vector2f extents = upper_bounds_ - lower_bounds_;
        return (extents(0) * extents(1));
    }

    float BoundingBox::computeOverlap(const BoundingBox &other_bounding_box) const {
        const float row_overlap = std::min(upper_bounds_(0), other_bounding_box.upper_bounds_(0)) -
                                  std::max(lower_bounds_(0), other_bounding_box.lower_bounds_(0));
        const float col_overlap = std::min(upper_bounds_(1), other_bounding_box.upper_bounds_(1)) -
                                  std::max(lower_bounds_(1), other_bounding_box.lower_bounds_(1));
        const float overlapping_area = (row_overlap * col_overlap);

        const float overlap_ratio = (2.0f * overlapping_area) / (computeArea() + other_bounding_box.computeArea());
        return (overlap_ratio);
    }
}