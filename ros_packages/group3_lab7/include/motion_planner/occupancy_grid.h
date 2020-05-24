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

class OccupancyGrid : public cv::Mat {

public:

    OccupancyGrid();
    OccupancyGrid(const cv::Mat&);

    inline bool isGridCellOccupied(int row, int col) const {
        return (this->at<uint8_t>(row, col) == GRID_CELL_IS_OCCUPIED);
    }
    bool expandPath(
            const cv::Vec2i start,
            const cv::Vec2i destination,
            cv::Vec2i& end,
            const int max_expansion_distance) const;

    OccupancyGrid& expand(float vehicle_width_in_pixels);

    nav_msgs::GridCells convertToGridCellsMessage(
            const cv::Vec2i grid_center,
            const float meters_per_pixel,
            const std::string frame_id) const;
};

#if 0
inline bool IsGridCellOccupied(
        int row,
        int col,
        const cv::Mat& occupancy_grid) {
    return(occupancy_grid.at<uint8_t>(row,col) == GRID_CELL_IS_OCCUPIED);
}

bool ExpandPath(
        const cv::Vec2i start,
        const cv::Vec2i destination,
        cv::Vec2i& end,
        const int max_expansion_distance,
        const cv::Mat& occupancy_grid);

cv::Mat& ExpandOccupancyGrid(
        cv::Mat& grid,
        float vehicle_width_in_pixels);

nav_msgs::GridCells ConvertToGridCellsMessage(
        cv::Mat& grid,
        const cv::Vec2i grid_center,
        const float meters_per_pixel,
        const std::string frame_id);
#endif