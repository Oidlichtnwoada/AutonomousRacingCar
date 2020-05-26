/* -------------------------------------------------------
 * VU Autonomous Racing Cars (2020S) - TU Wien
 * -------------------------------------------------------
 * Team 3: Adelmann, Brantner, Lukitsch, Pintaric
 * -------------------------------------------------------
 */

#pragma once
#include <motion_planner/path_planner.h>
#include <group3_lab7/motion_planner_Config.h>

class PathFollower {
public:
    typedef std::deque<Eigen::Vector2f> Path;
    typedef std::deque<Eigen::Vector2i> GridPath;
    typedef group3_lab7::motion_planner_Config Configuration;

    struct Options {
        Options();
        Options(const Configuration&);

        int cubic_spline_points_;
        int interpolated_points_on_path_;
        static constexpr int default_cubic_spline_points = 10;
        static constexpr int default_interpolated_points_on_path = 30;
    };

    PathFollower(Options options);

    std::tuple<bool, Path> refinePath(const Path& path);

protected:
    Options options_;

private:
    PathFollower();

};