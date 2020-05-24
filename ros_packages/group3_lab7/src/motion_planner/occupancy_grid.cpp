/* -------------------------------------------------------
 * VU Autonomous Racing Cars (2020S) - TU Wien
 * -------------------------------------------------------
 * Team 3: Adelmann, Brantner, Lukitsch, Pintaric
 * -------------------------------------------------------
 */

#include <motion_planner/occupancy_grid.h>

OccupancyGrid::OccupancyGrid() {}

OccupancyGrid::OccupancyGrid(const cv::Mat& m) :
    cv::Mat(m)
{}

// This is a modified version of
// Bresenham's line algorithm
//
// Source: https://rosettacode.org/wiki/Bitmap/Bresenham%27s_line_algorithm#C

bool OccupancyGrid::expandPath(
        const cv::Vec2i start,
        const cv::Vec2i destination,
        cv::Vec2i& end,
        const int max_expansion_distance) const
{
    int x0 = start(0);
    int y0 = start(1);
    const int &x1 = destination(0);
    const int &y1 = destination(1);
    int expansion_distance = 0;

    const int dx = std::abs(x1-x0);
    const int sx = x0<x1 ? 1 : -1;
    const int dy = std::abs(y1-y0);
    const int sy = y0<y1 ? 1 : -1;
    int err = (dx>dy ? dx : -dy)/2, e2;

    for(;;){
        if (isGridCellOccupied(x0,y0)) {
            return(false);
        }

        if (x0==x1 && y0==y1) {
            break;
        }
        e2 = err;
        if (e2 >-dx) {
            if (isGridCellOccupied(x0 + sx, y0)) {
                end = cv::Vec2i(x0,y0);
                return(false);
            }
            expansion_distance += sx;
            if (expansion_distance >= max_expansion_distance) {
                break;
            }
            err -= dy; x0 += sx;
        }
        if (e2 < dy) {
            if (isGridCellOccupied(x0, y0 + sy)) {
                end = cv::Vec2i(x0,y0);
                return(false);
            }
            expansion_distance += sy;
            if (expansion_distance >= max_expansion_distance) {
                break;
            }
            err += dx; y0 += sy;
        }
        if (expansion_distance >= max_expansion_distance) {
            break;
        }
    }
    end = cv::Vec2i(x0,y0);
    return(true);
}

OccupancyGrid& OccupancyGrid::expand(float vehicle_width_in_pixels) {
    unsigned int structuring_element_width = static_cast<unsigned int>(std::ceil(vehicle_width_in_pixels)) + 1;
    if (structuring_element_width % 2 == 0) {
        structuring_element_width++;
    }
    if(GRID_CELL_IS_FREE < GRID_CELL_IS_OCCUPIED) {
        cv::dilate(*this, *this /* in-place */ , cv::getStructuringElement(
                cv::MORPH_ELLIPSE, cv::Size(structuring_element_width, structuring_element_width)));
    } else {
        cv::erode(*this, *this /* in-place */ , cv::getStructuringElement(
                cv::MORPH_ELLIPSE, cv::Size(structuring_element_width, structuring_element_width)));
    }
    return(*this);
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

    for(int row=0; row<this->rows; row++) {
        for(int col=0; col<this->cols; col++) {
            if(this->at<uint8_t>(row,col) == GRID_CELL_IS_FREE) {
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
    return(grid_msg);
}