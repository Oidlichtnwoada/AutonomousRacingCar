/* -------------------------------------------------------
 * VU Autonomous Racing Cars (2020S) - TU Wien
 * -------------------------------------------------------
 * Team 3: Adelmann, Brantner, Lukitsch, Pintaric
 * -------------------------------------------------------
 */

#include <motion_planner/occupancy_grid.h>

namespace motion_planner {

    OccupancyGrid::OccupancyGrid(int rows,
                                 int cols,
                                 int pixel_format,
                                 cv::Scalar initial_value,
                                 DistanceMetric distance_metric)
            : distance_metric_(distance_metric)
    {
        occupancy_ = cv::Mat(rows, cols, pixel_format, initial_value);
    }

    OccupancyGrid::OccupancyGrid(const cv::Mat &occupancy, DistanceMetric distance_metric)
            : distance_metric_(distance_metric)
    {
        occupancy_ = occupancy;
    }

    OccupancyGrid::OccupancyGrid(const OccupancyGrid &other_grid)
        : distance_metric_(other_grid.distance_metric_)
    {
        assert(not other_grid.occupancy_.empty());
        assert(other_grid.occupancy_.rows * other_grid.occupancy_.cols > 0);

        occupancy_.create(other_grid.occupancy_.rows,
                          other_grid.occupancy_.cols,
                          other_grid.occupancy_.type());
        other_grid.occupancy_.copyTo(occupancy_);
        assert(occupancy_.rows == other_grid.occupancy_.rows &&
               occupancy_.cols == other_grid.occupancy_.cols);

        if(other_grid.distances_.rows * other_grid.distances_.cols > 0) {
            distances_.create(other_grid.distances_.rows,
                              other_grid.distances_.cols,
                              other_grid.distances_.type());
            other_grid.distances_.copyTo(distances_);
            assert(not distances_.empty());
            assert(distances_.rows == other_grid.distances_.rows &&
                   distances_.cols == other_grid.distances_.cols);
        }
    }

#if 0
    void OccupancyGrid::copyMeTo(OccupancyGrid &other_grid) const {

        assert(not occupancy_.empty());
        other_grid.occupancy_.create(occupancy_.rows, occupancy_.cols, occupancy_.type());
        occupancy_.copyTo(other_grid.occupancy_);
        assert(not other_grid.occupancy_.empty());

        if(!distances_.empty()) {
            other_grid.distances_.create(distances_.rows, distances_.cols, distances_.type());
            distances_.copyTo(other_grid.distances_);
            assert(not other_grid.distances_.empty());
        }
    }
#endif

    float OccupancyGrid::interpolatedDistanceFromNearestObstacle(Eigen::Vector2f position) const {
        assert(!distances_.empty());
        cv::Matx<float, 1, 1> pixel;
        cv::getRectSubPix(distances_, cv::Size(1,1), cv::Point2f(position(1), position(0)), pixel);
        return(pixel(0,0));
    }

    // This is a modified version of
    // Bresenham's line algorithm
    //
    // Source: https://rosettacode.org/wiki/Bitmap/Bresenham%27s_line_algorithm#C

    bool OccupancyGrid::tracePath(
            const cv::Vec2i start,
            const cv::Vec2i destination,
            cv::Vec2i &end,
            const int max_expansion_distance) const {
        int x0 = start(0);
        int y0 = start(1);
        const int &x1 = destination(0);
        const int &y1 = destination(1);
        int expansion_distance = 0;

        const int dx = std::abs(x1 - x0);
        const int sx = x0 < x1 ? 1 : -1;
        const int dy = std::abs(y1 - y0);
        const int sy = y0 < y1 ? 1 : -1;
        int err = (dx > dy ? dx : -dy) / 2, e2;

        for (;;) {
            if (isGridCellOccupied(x0, y0)) {
                return (false);
            }

            if (x0 == x1 && y0 == y1) {
                break;
            }
            e2 = err;
            if (e2 > -dx) {
                if (isGridCellOccupied(x0 + sx, y0)) {
                    end = cv::Vec2i(x0, y0);
                    return (false);
                }
                expansion_distance += sx;
                if (expansion_distance >= max_expansion_distance) {
                    break;
                }
                err -= dy;
                x0 += sx;
            }
            if (e2 < dy) {
                if (isGridCellOccupied(x0, y0 + sy)) {
                    end = cv::Vec2i(x0, y0);
                    return (false);
                }
                expansion_distance += sy;
                if (expansion_distance >= max_expansion_distance) {
                    break;
                }
                err += dx;
                y0 += sy;
            }
            if (expansion_distance >= max_expansion_distance) {
                break;
            }
        }
        end = cv::Vec2i(x0, y0);
        return (true);
    }

    OccupancyGrid &OccupancyGrid::expand(float width_in_pixels) {

        assert(!occupancy_.empty());

        unsigned int structuring_element_width = static_cast<unsigned int>(std::ceil(width_in_pixels)) + 1;
        if (structuring_element_width % 2 == 0) {
            structuring_element_width++;
        }

        if (GRID_CELL_IS_FREE < GRID_CELL_IS_OCCUPIED) {
            cv::dilate(occupancy_, occupancy_ /* in-place */ , cv::getStructuringElement(
                    cv::MORPH_ELLIPSE, cv::Size(structuring_element_width, structuring_element_width)));
        } else {
            cv::erode(occupancy_, occupancy_ /* in-place */ , cv::getStructuringElement(
                    cv::MORPH_ELLIPSE, cv::Size(structuring_element_width, structuring_element_width)));
        }
        return (*this);
    }

    nav_msgs::GridCells OccupancyGrid::convertToGridCellsMessage(
            const cv::Vec2i grid_center,
            const float meters_per_pixel,
            const std::string frame_id) const {

        nav_msgs::GridCells grid_msg;
        grid_msg.header.stamp = ros::Time::now();
        grid_msg.header.frame_id = frame_id;

        //const float meters_per_pixel = map_->info.resolution;
        grid_msg.cell_height = meters_per_pixel;
        grid_msg.cell_width = meters_per_pixel;

        for (int row = 0; row < occupancy_.rows; row++) {
            for (int col = 0; col < occupancy_.cols; col++) {
                if (occupancy_.at<uint8_t>(row, col) == GRID_CELL_IS_FREE) {
                    continue;
                }

                // Coordinate system conventions:
                // +x (forward) <--> laser frame row <--> grid col (+v)
                // +y (left)    <--> laser frame col <--> grid row (+u)

                geometry_msgs::Point p;
                p.x = (float) (col - grid_center(1)) * meters_per_pixel;
                p.y = (float) (row - grid_center(0)) * meters_per_pixel;
                p.z = 0.0;
                grid_msg.cells.push_back(p);
            }
        }
        return (grid_msg);
    }

    bool OccupancyGrid::hasDistances() const {
        return (not distances_.empty());
    }
    bool OccupancyGrid::hasOccupancy() const {
        return (not occupancy_.empty());
    }

    const cv::Mat &OccupancyGrid::distances() const {
        return distances_;
    }
    const cv::Mat &OccupancyGrid::occupancy() const {
        return occupancy_;
    }


    bool OccupancyGrid::computeDistanceTransform() {

        static_assert(GRID_CELL_IS_OCCUPIED == 0 && GRID_CELL_IS_FREE == 255, "Invalid GRID_CELL constants.");

        if(occupancy_.empty()) {
            return(false);
        }

        // Calculate the distance to the closest blocked cell for each cell of the grid

        const cv::Scalar zero(0.0);
        distances_ = cv::Mat(occupancy_.cols,
                             occupancy_.rows,
                             (distance_metric_ == L1_DISTANCE_METRIC) ? CV_8UC1 : CV_32F,
                             zero);

        switch (distance_metric_) {
            default:
            case (L1_DISTANCE_METRIC): {
                cv::distanceTransform(
                        occupancy_,
                        distances_,
                        cv::DIST_L1,
                        3, // (*)
                        CV_8U);

                // (*) for DIST_L1, a 3x3 mask gives the same result as 5x5 or any larger
                //     See: https://docs.opencv.org/master/d7/d1b/group__imgproc__misc.html

                break;
            }
            case (L2_DISTANCE_METRIC_3x3_KERNEL): {
                cv::distanceTransform(
                        occupancy_,
                        distances_,
                        cv::DIST_L2,
                        3,
                        CV_32F);
                break;
            }
            case (L2_DISTANCE_METRIC_5x5_KERNEL): {
                cv::distanceTransform(
                        occupancy_,
                        distances_,
                        cv::DIST_L2,
                        5,
                        CV_32F);
                break;
            }
        }

        return(not distances_.empty());
    }
} // namespace motion_planner