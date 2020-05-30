/* -------------------------------------------------------
 * VU Autonomous Racing Cars (2020S) - TU Wien
 * -------------------------------------------------------
 * Team 3: Adelmann, Brantner, Lukitsch, Pintaric
 * -------------------------------------------------------
 */

#pragma once
#include <motion_planner/tree.h>
#include <motion_planner/path.h>
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

#include <group3_lab7/motion_planner_Config.h>

namespace motion_planner {

    class PathPlanner {

    public:
        PathPlanner();

        static constexpr bool USE_L1_DISTANCE_METRIC = false; // <-- don't change this!
        typedef motion_planner::Tree<float, USE_L1_DISTANCE_METRIC> Tree;
        typedef motion_planner::MapPath MapPath;
        typedef motion_planner::GridPath GridPath;
        typedef group3_lab7::motion_planner_Config Configuration;

        enum Algorithm {
            RRT = 0, RRT_STAR = 1, INFORMED_RRT_STAR = 2
        };

        struct Options {
            Options();
            Options(const Configuration &);
            Algorithm algorithm_;
            bool reward_temporal_coherence_;
            int number_of_random_samples_;
            int goal_proximity_threshold_; // in grid coordinates (pixels)
            int size_of_k_neighborhood_;   // a.k.a. just "k", only used for RRT* and Informed-RRT*
            int maximum_branch_expansion_; // in grid coordinates (pixels)
            bool generate_marker_messages_;

            static constexpr Algorithm default_algorithm = INFORMED_RRT_STAR;
            static constexpr float default_reward_temporal_coherence = true;
#if !defined(NDEBUG)
            static constexpr int default_number_of_random_samples = 2000; // DEBUG
#else
            static constexpr int default_number_of_random_samples = 8000; // RELEASE
#endif
            static constexpr int default_goal_proximity_threshold = 2;
            static constexpr int default_size_of_k_neighborhood = 10;
            static constexpr int default_maximum_branch_expansion = 5;
            static constexpr bool default_generate_marker_messages = false;

        };

        std::tuple<
                bool /* success */,
                Tree /* full tree */,
                MapPath  /* best path (map coordinates) */,
                GridPath /* best path (grid coordinates) */>
        run(Eigen::Vector2f goal_in_map_frame,
            const OccupancyGrid &occupancy_grid,
            const cv::Vec2i &occupancy_grid_center,
            const Eigen::Affine3f &T_grid_to_map,
            GridPath seeded_solution,
            const Options options = Options(),
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
                const MapPath map_path,
                const Eigen::Vector2f goal_in_map_frame,
                const Eigen::Affine3f &T_grid_to_map,
                const Eigen::Vector2i grid_size,
                const Eigen::Affine3f T_sampling_domain_to_grid,
                const Eigen::Vector2f sampling_domain_extents, // [major axis length, minor axis length]
                const std::string map_frame = "map"
        ) const;
    };

} // namespace motion_planner