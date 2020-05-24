/* -------------------------------------------------------
 * VU Autonomous Racing Cars (2020S) - TU Wien
 * -------------------------------------------------------
 * Team 3: Adelmann, Brantner, Lukitsch, Pintaric
 * -------------------------------------------------------
 */

#pragma once
#include <motion_planner/Tree.h>
#include <motion_planner/occupancy_grid.h>
#include <deque>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <random>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <boost/shared_ptr.hpp>
#include <future>

class PathPlanner {

public:
    PathPlanner();

public:
    typedef int T;
    static constexpr bool USE_L1_DISTANCE_METRIC = true;
    typedef motion_planner::Tree<T, USE_L1_DISTANCE_METRIC> Tree;
    typedef std::deque<Eigen::Vector2f> Path;
    typedef std::deque<Eigen::Vector2i> GridPath;

    enum Algorithm { RRT, RRT_STAR, INFORMED_RRT_STAR };

    struct Options {
        Algorithm algorithm;
    };

    std::tuple<bool /* success */, Tree /* full tree */, Path /* best path */, GridPath>
    run(Eigen::Vector2f goal_in_map_frame,
        const OccupancyGrid& occupancy_grid,
        const cv::Vec2i& occupancy_grid_center,
        const Eigen::Affine3f& T_grid_to_map,
        const Options options = {.algorithm = INFORMED_RRT_STAR},
        boost::shared_ptr<ros::Publisher> marker_publisher = nullptr);

protected:
    std::mt19937 random_generator_;
    static constexpr std::mt19937::result_type random_seed_ = 9876543210UL; // fixed seed for reproducibility
    std::future<void> marker_publishing_task_;

    typedef Tree::Node Node;
    typedef Tree::KDTree KDTree;
    typedef Tree::KNNResultSet KNNResultSet;
    typedef Tree::UniformDistribution UniformDistribution;

    std::vector<visualization_msgs::Marker>
    generateMarkerMessages(
            const Tree tree,
            const Path path,
            const Eigen::Vector2f goal_in_map_frame,
            const Eigen::Affine3f& T_grid_to_map,
            const Eigen::Vector2i grid_size,
            const Eigen::Affine3f T_sampling_domain_to_grid,
            const Eigen::Vector2f sampling_domain_extents, // [major axis length, minor axis length]
            const std::string map_frame = "map"
            ) const;
};

PathPlanner::Path GridToMap(const PathPlanner::GridPath& grid_path, const Eigen::Affine3f T_grid_to_map);
PathPlanner::GridPath MapToGrid(const PathPlanner::Path& path, const Eigen::Affine3f T_map_to_grid);
