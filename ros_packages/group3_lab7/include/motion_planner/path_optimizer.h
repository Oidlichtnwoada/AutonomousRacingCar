/* -------------------------------------------------------
 * VU Autonomous Racing Cars (2020S) - TU Wien
 * -------------------------------------------------------
 * Team 3: Adelmann, Brantner, Lukitsch, Pintaric
 * -------------------------------------------------------
 */

#pragma once
#include <motion_planner/path.h>
#include <motion_planner/occupancy_grid.h>
#include <group3_lab7/motion_planner_Config.h>

namespace motion_planner {


    class PathOptimizer {
    public:
        typedef motion_planner::GridPath GridPath;
        typedef group3_lab7::motion_planner_Config Configuration;

        struct Options {
            Options();
            Options(const Configuration &);
            float path_segment_length_;
            static constexpr float default_path_segment_length = 0.2;
        };

        PathOptimizer(Options options);

        std::tuple<bool, GridPath> optimizePath(const GridPath &grid_path, const OccupancyGrid &occupancy_grid);

    protected:
        Options options_;

    private:
        PathOptimizer();

    };
} // namespace motion_planner