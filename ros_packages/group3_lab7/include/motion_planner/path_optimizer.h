/* -------------------------------------------------------
 * VU Autonomous Racing Cars (2020S) - TU Wien
 * -------------------------------------------------------
 * Team 3: Adelmann, Brantner, Lukitsch, Pintaric
 * -------------------------------------------------------
 */

#pragma once
#include <motion_planner/path_planner.h>
#include <motion_planner/occupancy_grid.h>
#include <group3_lab7/motion_planner_Config.h>

std::deque<Eigen::Vector2f> TransformPath(const std::deque<Eigen::Vector2i>& grid_path, const Eigen::Affine3f& T_grid_to_map);
std::deque<Eigen::Vector2i> TransformPath(const std::deque<Eigen::Vector2f>& path, const Eigen::Affine3f& T_map_to_grid);

template<typename T, typename U>
std::deque<Eigen::Matrix<T, 2, 1> > ConvertPath(const std::deque<Eigen::Matrix<U, 2, 1> >& input_path) {
    typename std::deque<Eigen::Matrix<T, 2, 1> > output_path;
    for(const Eigen::Matrix<U, 2, 1>& p : input_path) {
        output_path.push_back(p.template cast<T>());
    }
    return(output_path);

}

class PathOptimizer {
public:
    typedef std::deque<Eigen::Vector2f> Path;
    typedef std::deque<Eigen::Vector2i> GridPath;
    typedef group3_lab7::motion_planner_Config Configuration;

    struct Options {
        Options();
        Options(const Configuration&);

        float path_segment_length_;
        static constexpr float default_path_segment_length = 0.2;
    };

    PathOptimizer(Options options);

    std::tuple<bool, GridPath> optimizePath(const GridPath& grid_path, const OccupancyGrid& occupancy_grid);

protected:
    Options options_;

private:
    PathOptimizer();

};