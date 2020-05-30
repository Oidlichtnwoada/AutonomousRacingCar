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

#include <ros/ros.h>
#include <nav_msgs/GridCells.h>

#define GRID_CELL_IS_FREE      255
#define GRID_CELL_IS_OCCUPIED    0

static_assert(GRID_CELL_IS_FREE     >= 0 && GRID_CELL_IS_FREE     <= 255);
static_assert(GRID_CELL_IS_OCCUPIED >= 0 && GRID_CELL_IS_OCCUPIED <= 255);
static_assert(GRID_CELL_IS_FREE != GRID_CELL_IS_OCCUPIED);

namespace motion_planner {

    class OccupancyGrid : public cv::Mat {

    public:

        enum DistanceMetric {
            L1_DISTANCE_METRIC = 0,
            L2_DISTANCE_METRIC_3x3_KERNEL = 1,
            L2_DISTANCE_METRIC_5x5_KERNEL = 2
        };

        OccupancyGrid(DistanceMetric distance_metric = L2_DISTANCE_METRIC_3x3_KERNEL);

        OccupancyGrid(const cv::Mat &, DistanceMetric distance_metric = L2_DISTANCE_METRIC_3x3_KERNEL);

        virtual void copyMeTo(OccupancyGrid &other_grid) const;


        inline bool isGridCellOccupied(int row, int col) const {
            return (this->at<uint8_t>(row, col) == GRID_CELL_IS_OCCUPIED);
        }

        bool tracePath(
                const cv::Vec2i start,
                const cv::Vec2i destination,
                cv::Vec2i &end,
                const int max_expansion_distance) const;

        OccupancyGrid &expand(float width_in_pixels);

        nav_msgs::GridCells convertToGridCellsMessage(
                const cv::Vec2i grid_center,
                const float meters_per_pixel,
                const std::string frame_id) const;

        void computeDistanceTransform();

        bool hasDistances() const;

        const cv::Mat &distances() const;

    protected:
        DistanceMetric distance_metric_;
        cv::Mat distances_;

    };
}