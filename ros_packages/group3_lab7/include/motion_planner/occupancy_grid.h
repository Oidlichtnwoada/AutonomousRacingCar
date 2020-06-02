/* -------------------------------------------------------
 * VU Autonomous Racing Cars (2020S) - TU Wien
 * -------------------------------------------------------
 * Team 3: Adelmann, Brantner, Lukitsch, Pintaric
 * -------------------------------------------------------
 */

#pragma once
#include <opencv2/opencv.hpp>
#include <thread>
#include <boost/shared_ptr.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <ros/ros.h>
#include <nav_msgs/GridCells.h>

#define GRID_CELL_IS_FREE      255
#define GRID_CELL_IS_OCCUPIED    0

static_assert(GRID_CELL_IS_FREE     >= 0 && GRID_CELL_IS_FREE     <= 255);
static_assert(GRID_CELL_IS_OCCUPIED >= 0 && GRID_CELL_IS_OCCUPIED <= 255);
static_assert(GRID_CELL_IS_FREE != GRID_CELL_IS_OCCUPIED);

namespace motion_planner {

    class OccupancyGrid {

    public:

        enum DistanceMetric {
            L1_DISTANCE_METRIC = 0,
            L2_DISTANCE_METRIC_3x3_KERNEL = 1,
            L2_DISTANCE_METRIC_5x5_KERNEL = 2
        };
        static constexpr DistanceMetric DEFAULT_DISTANCE_METRIC = L2_DISTANCE_METRIC_3x3_KERNEL;

        OccupancyGrid(int rows, int cols, int pixel_format, cv::Scalar initial_value,
                DistanceMetric distance_metric = DEFAULT_DISTANCE_METRIC);

        OccupancyGrid(const OccupancyGrid &other_grid);

        OccupancyGrid(const cv::Mat& occupancy, DistanceMetric distance_metric = DEFAULT_DISTANCE_METRIC);

#if 0
        virtual void copyMeTo(OccupancyGrid &other_grid) const;
#endif

        inline int rows() const {
            return (occupancy_.rows);
        }
        inline int cols() const {
            return (occupancy_.cols);
        }

        inline bool setGridCellToOccupied(int row, int col) {
            assert(not occupancy_.empty());
            occupancy_.at<uint8_t>(row, col) = GRID_CELL_IS_OCCUPIED;
        }
        inline bool isGridCellOccupied(int row, int col) const {
            assert(not occupancy_.empty());
            return (occupancy_.at<uint8_t>(row, col) == GRID_CELL_IS_OCCUPIED);
        }
        inline bool isGridCellOccupied(Eigen::Vector2i position) const {
            assert(not occupancy_.empty());
            return (occupancy_.at<uint8_t>(position(0), position(1)) == GRID_CELL_IS_OCCUPIED);
        }

        inline float distanceFromNearestObstacle(int row, int col) const {
            assert(not distances_.empty());
            return (distances_.at<float>(row, col));
        }
        inline float distanceFromNearestObstacle(Eigen::Vector2i position) const {
            assert(not distances_.empty());
            return (distances_.at<float>(position(0), position(1)));
        }

        // pixels at non-integer coordinates are retrieved using bilinear interpolation:
        float interpolatedDistanceFromNearestObstacle(Eigen::Vector2f position) const;

        bool tracePath(
                const cv::Vec2i start,
                const cv::Vec2i destination,
                cv::Vec2i &end,
                const int max_expansion_distance) const;

        OccupancyGrid &expand(float width_in_pixels);

        inline OccupancyGrid &applyThresholdAndInvert() {
            cv::threshold(occupancy_, occupancy_ /* in-place */, 1, 255, cv::THRESH_BINARY);
            cv::bitwise_not(occupancy_, occupancy_);
        }

        nav_msgs::GridCells convertToGridCellsMessage(
                const cv::Vec2i grid_center,
                const float meters_per_pixel,
                const std::string frame_id) const;

        bool computeDistanceTransform();

        bool hasDistances() const;
        bool hasOccupancy() const;

        const cv::Mat &distances() const;
        const cv::Mat &occupancy() const;

        inline void clearOccupancy() {
            occupancy_.setTo(cv::Scalar(GRID_CELL_IS_FREE)); // clear the grid
        }

    protected:
        DistanceMetric distance_metric_;
        cv::Mat distances_;
        cv::Mat occupancy_;

    private:
        OccupancyGrid(); // hide default constructor

    };
} // namespace motion_planner