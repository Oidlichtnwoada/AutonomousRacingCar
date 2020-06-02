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
#include <boost/shared_ptr.hpp>

namespace motion_planner {


    class PathOptimizer {
    public:
        typedef motion_planner::GridPath GridPath;
        typedef motion_planner::Path<float> FloatingPointGridPath;
        typedef group3_lab7::motion_planner_Config Configuration;

        struct Options {
            Options();
            Options(const Configuration &);
            bool enable_path_optimization_;
            int number_of_resampled_path_segments_;
            int number_of_optimization_iterations_;
            float maximum_waypoint_translation_along_normal_;
            float path_optimization_step_size_;
            static constexpr bool default_enable_path_optimization = true;
            static constexpr int default_number_of_resampled_path_segments = 20;
            static constexpr int default_number_of_optimization_iterations = 100;
            static constexpr float default_maximum_waypoint_translation_along_normal = 5.0f;
            static constexpr float default_path_optimization_step_size = 0.5f;
        };

        PathOptimizer(Options options);

        std::tuple<bool, FloatingPointGridPath>
        optimizePath(const GridPath &grid_path,
                     boost::shared_ptr<OccupancyGrid> occupancy_grid);

    protected:
        Options options_;

    private:
        PathOptimizer();

    };

} // namespace motion_planner