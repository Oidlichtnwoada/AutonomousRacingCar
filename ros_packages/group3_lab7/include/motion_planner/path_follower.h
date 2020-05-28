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

        float path_segment_length_;
        static constexpr float default_path_segment_length = 0.2;
    };

    PathFollower(Options options);

    std::tuple<bool, Path> refinePath(const Path& path);

protected:
    Options options_;

private:
    PathFollower();

};