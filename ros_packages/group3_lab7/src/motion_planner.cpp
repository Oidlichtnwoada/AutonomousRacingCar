/* -------------------------------------------------------
 * VU Autonomous Racing Cars (2020S) - TU Wien
 * -------------------------------------------------------
 * Team 3: Adelmann, Brantner, Lukitsch, Pintaric
 * -------------------------------------------------------
 */

#include <iostream>
#include <string>
#include <cmath>
#include <thread>
#include <atomic>
#include <mutex>

#include <ros/ros.h>

#include <std_msgs/Float32.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GridCells.h>
#include <visualization_msgs/Marker.h>
#include <std_srvs/Empty.h>

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <boost/range/adaptor/indexed.hpp>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/mat.hpp>

#include <motion_planner/occupancy_grid.h>
#include <motion_planner/path_planner.h>
#include <motion_planner/path_optimizer.h>

#include <dynamic_reconfigure/server.h>
#include <group3_lab7/motion_planner_Config.h>

//#define LIMIT_DYNAMIC_OCCUPANCY_GRID_TO_FORWARD_DIRECTION // <-- this is just an optimization

#define SUBSCRIBER_MESSAGE_QUEUE_SIZE 1000
#define GRID_PUBLISHER_MESSAGE_QUEUE_SIZE 10
#define MARKER_PUBLISHER_MESSAGE_QUEUE_SIZE 10
#define PATH_PUBLISHER_MESSAGE_QUEUE_SIZE 10
#define STEERING_ANGLE_PUBLISHER_MESSAGE_QUEUE_SIZE 5
#define COMMANDED_HEADING_PUBLISHER_MESSAGE_QUEUE_SIZE 5
#define COMMANDED_GOAL_PUBLISHER_MESSAGE_QUEUE_SIZE 5

static_assert(SUBSCRIBER_MESSAGE_QUEUE_SIZE > 0);
static_assert(GRID_PUBLISHER_MESSAGE_QUEUE_SIZE > 0);
static_assert(MARKER_PUBLISHER_MESSAGE_QUEUE_SIZE > 0);

#define TOPIC_SCAN "/scan"
#define TOPIC_ODOM "/odom"
#if defined(NDEBUG)
#define TOPIC_GOAL "/motion_planner/goal" // Release
#else
#define TOPIC_GOAL "/path_publisher/goal"     // Debug <-- TODO: remove this
//#define TOPIC_GOAL "/move_base_simple/goal" // Debug <-- TODO: remove this
#endif
#define TOPIC_MAP  "/map"
#define TOPIC_DYNAMIC_OCCUPANCY_GRID   "/motion_planner/dynamic_occupancy_grid"
#define TOPIC_STATIC_OCCUPANCY_GRID    "/motion_planner/static_occupancy_grid"
#define TOPIC_RRT_VISUALIZATION        "/motion_planner/rrt_visualization" // TODO: rename this to "markers"
#define TOPIC_PATH_TO_GOAL             "/motion_planner/path_to_goal"
#define TOPIC_COMMANDED_STEERING_ANGLE "/motion_planner/steering_angle"
#define TOPIC_COMMANDED_HEADING        "/motion_planner/commanded_heading"
#define TOPIC_COMMANDED_GOAL           "/motion_planner/commanded_goal"

#define FRAME_MAP       "map"
#define FRAME_BASE_LINK "base_link"
#define FRAME_LASER     "laser"

#define DEFAULT_VEHICLE_WHEELBASE 0.3302 // units: m (see f110_simulator/params.yaml)
#define DEFAULT_VEHICLE_WIDTH     0.2032 // units: m (see f110_simulator/params.yaml)
#define DEFAULT_MAX_LASER_SCAN_DISTANCE 5.0 // units: m
#define DEFAULT_DYNAMIC_OCCUPANCY_GRID_UPDATE_FREQUENCY 60 // units: Hz
#define DEFAULT_GOAL_UPDATE_FREQUENCY 60 // units: Hz
#define DEFAULT_STEERING_LOOKAHEAD_DISTANCE 1.0 // units: meters

static_assert(DEFAULT_VEHICLE_WHEELBASE > 0);
static_assert(DEFAULT_VEHICLE_WIDTH > 0);
static_assert(DEFAULT_MAX_LASER_SCAN_DISTANCE > 0);
static_assert(DEFAULT_DYNAMIC_OCCUPANCY_GRID_UPDATE_FREQUENCY >= 0);

namespace motion_planner {

    class MotionPlanner {

    public:
        MotionPlanner();

        ~MotionPlanner();

        typedef boost::shared_ptr<sensor_msgs::LaserScan const> LaserScanMessage;
        typedef boost::shared_ptr<geometry_msgs::PoseStamped const> PoseStampedMessage;
        typedef boost::shared_ptr<nav_msgs::Odometry const> OdometryMessage;
        typedef boost::shared_ptr<nav_msgs::Path const> PathMessage;
        typedef boost::shared_ptr<nav_msgs::OccupancyGrid const> OccupancyGridMessage;
        typedef group3_lab7::motion_planner_Config Configuration;

        std::mutex configuration_mutex_;
        std::atomic<bool> has_valid_configuration_;
        Configuration configuration_;
        std::atomic<float> scan_dt_threshold_;
        std::atomic<float> goal_dt_threshold_;
        std::atomic<float> steering_lookahead_distance_;

    private:
        float vehicle_wheelbase_; // units: m
        float vehicle_width_; // units: m
        float max_laser_scan_distance_; // units: m

        void laserScanSubscriberCallback(LaserScanMessage);

        void odomSubscriberCallback(OdometryMessage);

        void goalSubscriberCallback(PoseStampedMessage);

        void mapSubscriberCallback(OccupancyGridMessage);

        bool debugServiceCallback(std_srvs::Empty::Request &, std_srvs::Empty::Response &);

        ros::NodeHandle node_handle_;

        boost::shared_ptr<ros::Subscriber> laser_scan_subscriber_;
        boost::shared_ptr<ros::Subscriber> odometry_subscriber_;
        boost::shared_ptr<ros::Subscriber> goal_subscriber_;
        boost::shared_ptr<ros::Subscriber> map_subscriber_;
        boost::shared_ptr<ros::ServiceServer> debug_service_;
        boost::shared_ptr<ros::Publisher> dynamic_occupancy_grid_publisher_;
        boost::shared_ptr<ros::Publisher> static_occupancy_grid_publisher_;
        boost::shared_ptr<ros::Publisher> rrt_visualization_publisher_;
        boost::shared_ptr<ros::Publisher> path_to_goal_publisher_;
        boost::shared_ptr<ros::Publisher> commanded_steering_angle_publisher_;
        boost::shared_ptr<ros::Publisher> commanded_heading_publisher_;
        boost::shared_ptr<ros::Publisher> commanded_goal_publisher_;
        std::mutex path_to_goal_publisher_mutex_;

        tf2_ros::Buffer tf_buffer_;
        boost::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        ros::Time last_scan_timestamp_;
        ros::Time last_goal_timestamp_;

        // =================================================================================================================
        // Occupancy grids (static and dynamic)
        // =================================================================================================================

        MotionPlanner::OccupancyGridMessage map_; // copied from "/map"

        std::mutex occupancy_grid_mutex_;
        boost::shared_ptr<OccupancyGrid> dynamic_occupancy_grid_;
        boost::shared_ptr<OccupancyGrid> static_occupancy_grid_; // just a thin wrapper around map_->data.data()
        cv::Vec2i static_occupancy_grid_center_;  // TODO: add this to class OccupancyGrid
        cv::Vec2i dynamic_occupancy_grid_center_; // TODO: add this to class OccupancyGrid

        // Affine transformations from grid coordinates to the "map" and "laser" frames
        Eigen::Affine3f T_grid_to_laser_frame_;      // dynamic_occupancy_grid (pixels) to laser frame (meters)
        Eigen::Affine3f T_grid_to_map_frame_;        // dynamic_occupancy_grid (pixels) to map frame (meters)
        Eigen::Affine3f T_laser_to_grid_frame_;
        Eigen::Affine3f T_map_to_grid_frame_;
        Eigen::Affine3f T_map_to_base_link_frame_;
        Eigen::Affine3f T_base_link_to_map_frame_;

        // =================================================================================================================
        // Path planning algorithms (RRT/RRT*/Informed-RRT*)
        // =================================================================================================================

        PathPlanner path_planner_;
        std::atomic<bool> should_exit_path_planner_thread_;
        boost::shared_ptr<std::thread> path_planner_thread_;

        void runPathPlanner();

        std::mutex odometry_mutex_;
        nav_msgs::Odometry current_odometry_;
        std::mutex goal_mutex_;
        geometry_msgs::PoseStamped current_goal_;

        ros::Time path_planner_odometry_timestamp_;
        ros::Time path_planner_goal_timestamp_;

        std::atomic<bool> should_plan_path_;

        std::mutex last_path_mutex_;
        PathPlanner::MapPath last_shortest_path_; // previously found solution (in map frame)

#if !defined(NDEBUG)
        unsigned long long laser_scan_messages_received_;
        unsigned long long odometry_messages_received_;
        unsigned long long goal_messages_received_;
        unsigned long long map_messages_received_;
#endif

        std::mutex path_to_goal_mutex_;
        PathPlanner::MapPath path_to_goal_; // most recent (optimized) path to goal (in map frame)

        std::future<void> path_to_goal_publishing_task_;

        void publishCommandedPathToGoal(const PathPlanner::MapPath &path) const;
        bool computeAndPublishCommandedSteeringAngle(
                const PathPlanner::MapPath &path,
                const nav_msgs::Odometry &odometry) const;

    };

    MotionPlanner::~MotionPlanner() {
        should_exit_path_planner_thread_.store(true);
        path_planner_thread_->join();
    }

    MotionPlanner::MotionPlanner()
            : node_handle_(ros::NodeHandle())
            , laser_scan_subscriber_(new ros::Subscriber(node_handle_.subscribe(TOPIC_SCAN, SUBSCRIBER_MESSAGE_QUEUE_SIZE, &MotionPlanner::laserScanSubscriberCallback, this))), odometry_subscriber_(new ros::Subscriber(node_handle_.subscribe(TOPIC_ODOM, SUBSCRIBER_MESSAGE_QUEUE_SIZE, &MotionPlanner::odomSubscriberCallback, this)))
            , goal_subscriber_(new ros::Subscriber(node_handle_.subscribe(TOPIC_GOAL, SUBSCRIBER_MESSAGE_QUEUE_SIZE, &MotionPlanner::goalSubscriberCallback, this))), map_subscriber_(new ros::Subscriber(node_handle_.subscribe(TOPIC_MAP, SUBSCRIBER_MESSAGE_QUEUE_SIZE, &MotionPlanner::mapSubscriberCallback, this)))
            , tf_listener_(new tf2_ros::TransformListener(tf_buffer_, node_handle_, true)), debug_service_(new ros::ServiceServer(node_handle_.advertiseService("debug", &MotionPlanner::debugServiceCallback, this)))
            , static_occupancy_grid_publisher_(new ros::Publisher(node_handle_.advertise<nav_msgs::GridCells>(TOPIC_STATIC_OCCUPANCY_GRID, GRID_PUBLISHER_MESSAGE_QUEUE_SIZE)))
            , dynamic_occupancy_grid_publisher_(new ros::Publisher(node_handle_.advertise<nav_msgs::GridCells>(TOPIC_DYNAMIC_OCCUPANCY_GRID, GRID_PUBLISHER_MESSAGE_QUEUE_SIZE)))
            , rrt_visualization_publisher_(new ros::Publisher(node_handle_.advertise<visualization_msgs::Marker>(TOPIC_RRT_VISUALIZATION, MARKER_PUBLISHER_MESSAGE_QUEUE_SIZE)))
            , path_to_goal_publisher_(new ros::Publisher(node_handle_.advertise<nav_msgs::Path>(TOPIC_PATH_TO_GOAL, PATH_PUBLISHER_MESSAGE_QUEUE_SIZE)))
            , commanded_steering_angle_publisher_(new ros::Publisher(node_handle_.advertise<std_msgs::Float32>(TOPIC_COMMANDED_STEERING_ANGLE, STEERING_ANGLE_PUBLISHER_MESSAGE_QUEUE_SIZE)))
            , commanded_heading_publisher_(new ros::Publisher(node_handle_.advertise<geometry_msgs::PoseStamped>(TOPIC_COMMANDED_HEADING, COMMANDED_HEADING_PUBLISHER_MESSAGE_QUEUE_SIZE)))
            , commanded_goal_publisher_(new ros::Publisher(node_handle_.advertise<geometry_msgs::PointStamped>(TOPIC_COMMANDED_GOAL, COMMANDED_GOAL_PUBLISHER_MESSAGE_QUEUE_SIZE)))
            , has_valid_configuration_(false), should_exit_path_planner_thread_(false)
#if !defined(NDEBUG)
            , laser_scan_messages_received_(0)
            , odometry_messages_received_(0)
            , goal_messages_received_(0)
            , map_messages_received_(0)
#endif


    {
        should_plan_path_.store(false);
        node_handle_.param<float>("vehicle_wheelbase", vehicle_wheelbase_, DEFAULT_VEHICLE_WHEELBASE);
        node_handle_.param<float>("vehicle_width", vehicle_width_, DEFAULT_VEHICLE_WIDTH);
        node_handle_.param<float>("max_laser_scan_distance", max_laser_scan_distance_, DEFAULT_MAX_LASER_SCAN_DISTANCE);

        auto check_value = [](float f, std::string s) {
            if (f <= 0.0f) {
                std::ostringstream o;
                o << "Requirement violated: (" << s << " > 0)";
                ROS_ERROR("%s", o.str().c_str());
                throw (std::runtime_error(o.str()));
            }
        };
        check_value(vehicle_wheelbase_, "vehicle_wheelbase");
        check_value(vehicle_width_, "vehicle_width_");
        check_value(max_laser_scan_distance_, "max_laser_scan_distance");

        scan_dt_threshold_.store(1.0f / DEFAULT_DYNAMIC_OCCUPANCY_GRID_UPDATE_FREQUENCY);
        goal_dt_threshold_.store(1.0f / DEFAULT_GOAL_UPDATE_FREQUENCY);
        steering_lookahead_distance_.store(DEFAULT_STEERING_LOOKAHEAD_DISTANCE);

        should_exit_path_planner_thread_.store(false);
        path_planner_thread_ = boost::shared_ptr<std::thread>(new std::thread(&MotionPlanner::runPathPlanner, this));
    }

    bool MotionPlanner::debugServiceCallback(std_srvs::Empty::Request &request,
                                             std_srvs::Empty::Response &response) {

        // Trigger with "rosservice call /debug"

        std::lock_guard<std::mutex> scoped_grid_lock(occupancy_grid_mutex_);
        std::lock_guard<std::mutex> scoped_path_lock(last_path_mutex_);
        std::cout << "MotionPlanner::debugServiceCallback()" << std::endl;
        if(not dynamic_occupancy_grid_->hasDistances()) {
            dynamic_occupancy_grid_->computeDistanceTransform();
        }
        cv::imwrite("/tmp/occupancy_grid_distances.png", dynamic_occupancy_grid_->distances());
        cv::imwrite("/tmp/occupancy_grid.png", dynamic_occupancy_grid_->occupancy());
        {
            std::ofstream o("/tmp/map_path.json");
            o << std::setw(4) << PathToJson<float>(last_shortest_path_) << std::endl;
        }
        {
            GridPath gridPath = TransformPath<float,int>(last_shortest_path_, T_map_to_grid_frame_);
            std::ofstream o("/tmp/grid_path.json");
            o << std::setw(4) << PathToJson<int>(gridPath) << std::endl;
        }
        return (true);

    }

    void MotionPlanner::laserScanSubscriberCallback(MotionPlanner::LaserScanMessage scan_msg) {

#if !defined(NDEBUG)
        laser_scan_messages_received_++;
#endif

        static unsigned int messages_skipped = 0; // for debugging only

        std::unique_lock<std::mutex> lock(occupancy_grid_mutex_, std::try_to_lock);
        if (!lock.owns_lock()) {
            messages_skipped++;
            return;
        }

        nav_msgs::Odometry current_odometry;
        {
            std::lock_guard<std::mutex> scoped_lock(odometry_mutex_);
            current_odometry = current_odometry_;
        }

        if ((current_odometry.header.stamp.isZero()) ||
            (not tf_buffer_.canTransform(FRAME_BASE_LINK, scan_msg->header.frame_id, ros::Time(0)))) {
            messages_skipped++;
            return;
        }

        // Enforce max. update frequency
        const bool has_previous_timestamp = not last_scan_timestamp_.isZero();
        const ros::Time &current_timestamp = scan_msg->header.stamp;
        if (!has_previous_timestamp) {
            last_scan_timestamp_ = current_timestamp;
            return;
        }

        const float dt = current_timestamp.toSec() - last_scan_timestamp_.toSec();
        if (dt < scan_dt_threshold_.load()) {
            return;
        }
        last_scan_timestamp_ = current_timestamp;

        // Obtain laser-->base_link transform (NOTE: this is a static transform)
        Eigen::Isometry3f T_laser_to_base_link = tf2::transformToEigen(tf_buffer_.lookupTransform(
                FRAME_BASE_LINK, FRAME_LASER, ros::Time(0)).transform).cast<float>();

        auto poseToEigen = [](const geometry_msgs::Pose &pose) -> Eigen::Isometry3f {
            return Eigen::Isometry3f(Eigen::Translation3f(pose.position.x, pose.position.y, pose.position.z)
                                     * Eigen::Quaternionf(pose.orientation.w, pose.orientation.x, pose.orientation.y,
                                                          pose.orientation.z));
        };

        Eigen::Isometry3f T_base_link_to_map = poseToEigen(current_odometry.pose.pose);

        T_base_link_to_map_frame_ = T_base_link_to_map.matrix();
        T_map_to_base_link_frame_ = T_base_link_to_map_frame_.inverse();

        const float pixels_per_meter = 1.0f / map_->info.resolution;
        const float meters_per_pixel = map_->info.resolution;

        // =================================================================================================================
        // Compute coordinate frame transformation matrices
        // =================================================================================================================

        const Eigen::Vector3f dynamic_occupancy_grid_pixel_center((float) dynamic_occupancy_grid_center_(0),
                                                                  (float) dynamic_occupancy_grid_center_(1),
                                                                  0.0f);

        Eigen::Affine3f A = Eigen::Affine3f::Identity();

        A.pretranslate((-1.0f) * Eigen::Vector3f((float) dynamic_occupancy_grid_center_(0),
                                                 (float) dynamic_occupancy_grid_center_(1),
                                                 0.0f));

        A.prescale(meters_per_pixel); // pixels to meters

        A.prescale(Eigen::Vector3f(-1.0, 1.0, 1.0)); // <--- NOTE!
        A.prerotate(Eigen::AngleAxisf(-M_PI_2, Eigen::Vector3f::UnitZ())); // <--- NOTE!

        T_grid_to_laser_frame_ = A.matrix();
        T_laser_to_grid_frame_ = T_grid_to_laser_frame_.inverse();

        //A = T_laser_to_map.matrix() * A.matrix();
        A = T_base_link_to_map.matrix() * T_laser_to_base_link.matrix() * A.matrix();

        T_grid_to_map_frame_ = A.matrix();
        T_map_to_grid_frame_ = T_grid_to_map_frame_.inverse();

        // =================================================================================================================

        dynamic_occupancy_grid_->clearOccupancy(); // set every cell to GRID_CELL_IS_FREE

        const float angle_min = scan_msg->angle_min;
        const float angle_increment = scan_msg->angle_increment;
        for (auto const &element : scan_msg->ranges | boost::adaptors::indexed()) {
            const float range = element.value();
            const float angle = angle_min + (float) element.index() * angle_increment;

#ifdef LIMIT_DYNAMIC_OCCUPANCY_GRID_TO_FORWARD_DIRECTION
            if(angle < -M_PI_2 || angle > M_PI_2) {
                continue;
            }
#endif

            if (std::isfinite(range) && range <= max_laser_scan_distance_) {

                const float angle_in_degrees = angle * 180.0 / M_PI;

                // polar to cartesian coordinates [x,y] in the laser frame
                // +x vehicle forward direction
                // +y pointing left
                const float x = range * std::cos(angle); // units: meters
#ifdef LIMIT_DYNAMIC_OCCUPANCY_GRID_TO_FORWARD_DIRECTION
                assert(x >= 0.0);
#endif
                const float y = range * std::sin(angle); // units: meters

                // Coordinate system conventions:
                //
                // [x,y] are cartesian coordinates in the laser frame
                // [u,v] are the pixel coordinates in the dynamic occupancy grid
                //
                // +u ... grid row    (+y in the laser frame)
                // +v ... grid column (+x in the laser frame)
                //
                // Laser frame:      Grid frame:
                // ------------      -----------
                //
                //        (+x)
                //           ^        0 ----> (+x) = (+v)
                //           |        |
                // (+y) <----0        v
                //                    (+y) = (+u)

                const int u = static_cast<int>(y * pixels_per_meter) + dynamic_occupancy_grid_center_(0);
                const int v = static_cast<int>(x * pixels_per_meter) + dynamic_occupancy_grid_center_(1);

                const int center_row = dynamic_occupancy_grid_center_(0);
                const int center_col = dynamic_occupancy_grid_center_(1);
                const int u_max = dynamic_occupancy_grid_->rows() - 1;
                const int v_max = dynamic_occupancy_grid_->cols() - 1;

                assert(u >= 0 && v >= 0 && u < dynamic_occupancy_grid_->rows() && v < dynamic_occupancy_grid_->cols());
                dynamic_occupancy_grid_->setGridCellToOccupied(u,v); // set the cell to GRID_CELL_IS_OCCUPIED
            }
        }

        // Expand the dynamic occupancy grid
        const float vehicle_width_in_pixels = vehicle_width_ * pixels_per_meter;

        float grid_expansion_width = vehicle_width_in_pixels;
        if (has_valid_configuration_.load()) {
            std::lock_guard<std::mutex> scoped_lock(configuration_mutex_);
            grid_expansion_width += static_cast<float>(configuration_.extra_occupancy_grid_dilation);
            assert(grid_expansion_width >= 0);
        }
        dynamic_occupancy_grid_->expand(grid_expansion_width);
        dynamic_occupancy_grid_publisher_->publish(dynamic_occupancy_grid_->convertToGridCellsMessage(
                dynamic_occupancy_grid_center_, meters_per_pixel, FRAME_LASER));
        should_plan_path_.store(true);
    }

    void MotionPlanner::odomSubscriberCallback(MotionPlanner::OdometryMessage odom_msg) {

#if !defined(NDEBUG)
        odometry_messages_received_++;
#endif

        {
            std::lock_guard<std::mutex> scoped_lock(odometry_mutex_);
            current_odometry_ = *odom_msg;
            assert(not current_odometry_.header.stamp.isZero());
        }
        {
            std::lock_guard<std::mutex> scoped_lock(path_to_goal_mutex_);
            if (path_to_goal_.empty()) {
                return;
            }
            computeAndPublishCommandedSteeringAngle(path_to_goal_, *odom_msg);
        }
        // NOTE: should_plan_path_.store(true) should NOT be used here!
    }

    void MotionPlanner::goalSubscriberCallback(MotionPlanner::PoseStampedMessage goal_msg) {

#if !defined(NDEBUG)
        goal_messages_received_++;
#endif

        // Enforce max. update frequency
        const bool has_previous_timestamp = not last_goal_timestamp_.isZero();
        const ros::Time &current_timestamp = goal_msg->header.stamp;
        if (!has_previous_timestamp) {
            last_goal_timestamp_ = current_timestamp;
            return;
        }

        const float dt = current_timestamp.toSec() - last_goal_timestamp_.toSec();
        if (dt < goal_dt_threshold_.load()) {
            return;
        }
        last_goal_timestamp_ = current_timestamp;

        {
            std::lock_guard<std::mutex> scoped_lock(goal_mutex_);
            current_goal_ = *goal_msg;
            assert(not current_goal_.header.stamp.isZero());
            //ROS_INFO("Received new goal: [%f, %f]", current_goal_.pose.position.x, current_goal_.pose.position.y);
            should_plan_path_.store(true);
        }
    }

    void MotionPlanner::mapSubscriberCallback(MotionPlanner::OccupancyGridMessage map_msg) {

#if !defined(NDEBUG)
        map_messages_received_++;
#endif

        std::lock_guard<std::mutex> scoped_lock(occupancy_grid_mutex_);

        map_ = map_msg;
        const nav_msgs::MapMetaData &info = map_msg->info;

        const float pixels_per_meter = 1.0f / map_->info.resolution;
        const float meters_per_pixel = map_->info.resolution;

        static_occupancy_grid_center_ = cv::Vec2i(
                static_cast<int>(-map_->info.origin.position.y * pixels_per_meter),  // rows <--> y
                static_cast<int>(-map_->info.origin.position.x * pixels_per_meter)); // cols <--> x

        static_occupancy_grid_ = boost::shared_ptr<OccupancyGrid>(new OccupancyGrid(
                cv::Mat(info.height, info.width, CV_8UC1, (void *) map_->data.data()))); // just a thin wrapper around map_->data.data()
        static_occupancy_grid_->applyThresholdAndInvert();

        // Publish static occupancy grid (for RViz)
        static_occupancy_grid_publisher_->publish(static_occupancy_grid_->convertToGridCellsMessage(
                static_occupancy_grid_center_, meters_per_pixel, FRAME_MAP));

        if (info.origin.orientation.x != 0.0 || info.origin.orientation.y != 0.0 || info.origin.orientation.z != 0.0) {
            ROS_ERROR("Handling non-zero map rotations is not implemented.");
            throw (std::runtime_error("Handling non-zero map rotations is not implemented."));
        }

        if ((info.height * info.width) > (1000 * 1000)) {
            std::ostringstream o;
            o << "Map dimensions (" << info.height << "x" << info.width <<
              ") are quite large. Consider reducing the resolution.";
            ROS_WARN("%s", o.str().c_str());
        }

        // Create dynamic occupancy grid
        const float laser_scan_diameter_in_pixels = 2.0f * max_laser_scan_distance_ * pixels_per_meter;
        unsigned int dynamic_occupancy_grid_cols =
                static_cast<unsigned int>(std::ceil(laser_scan_diameter_in_pixels)) + 2;
        if (dynamic_occupancy_grid_cols % 2 == 0) {
            dynamic_occupancy_grid_cols++;
        }

        // Coordinate system conventions:
        // +x (forward) <--> laser frame row <--> grid col (+v)
        // +y (left)    <--> laser frame col <--> grid row (+u)

        const int center_col = (dynamic_occupancy_grid_cols - 1) / 2;

#ifdef LIMIT_DYNAMIC_OCCUPANCY_GRID_TO_FORWARD_DIRECTION
        const int center_row = 0;
        const unsigned int dynamic_occupancy_grid_rows = ((dynamic_occupancy_grid_cols- 1 ) / 2) + 1;
#else
        const int center_row = center_col;
        const unsigned int dynamic_occupancy_grid_rows = dynamic_occupancy_grid_cols;
#endif
        dynamic_occupancy_grid_center_ = cv::Vec2i(center_col,
                                                   center_row); // NOTE: grid is transposed w.r.t. vehicle's local coordinate system
        const cv::Scalar grid_cell_is_free(GRID_CELL_IS_FREE);
        dynamic_occupancy_grid_ = boost::shared_ptr<OccupancyGrid>(new OccupancyGrid(
                dynamic_occupancy_grid_cols, dynamic_occupancy_grid_rows, CV_8UC1, grid_cell_is_free));

        should_plan_path_.store(true);
    }

    void MotionPlanner::runPathPlanner() {

        ROS_INFO("Path planner is starting...");
        static unsigned int number_of_computations = 0; // for debugging only

        ros::Rate sampling_rate(1000); // 1000 hz (chosen to match the /odom update frequency)
        while (!should_exit_path_planner_thread_.load() && !ros::isShuttingDown()) {

            bool expected = true;
            if (has_valid_configuration_.load() &&
                should_plan_path_.compare_exchange_strong(expected, false)) {

                bool should_generate_marker_messages = true;
                bool should_seed_last_solution = false;
                PathPlanner::Options path_planner_options;
                PathOptimizer::Options path_follower_options;
                {
                    std::lock_guard<std::mutex> scoped_lock(configuration_mutex_);
                    path_planner_options = PathPlanner::Options(configuration_);
                    path_follower_options = PathOptimizer::Options(configuration_);
                    should_generate_marker_messages = configuration_.generate_marker_messages;
                    should_seed_last_solution = configuration_.seed_last_solution;
                }

                geometry_msgs::PoseStamped goal;
                nav_msgs::Odometry odometry;
                bool already_near_goal = false;
                {
                    {
                        std::lock_guard<std::mutex> scoped_lock(odometry_mutex_);
                        odometry = current_odometry_; // make a local copy of the current odometry
                    }
                    {
                        std::lock_guard<std::mutex> scoped_lock(goal_mutex_);
                        goal = current_goal_; // make a local copy of the current goal
                    }
                }

                if (goal.header.stamp.isZero()) {
                    // no goal has been set
                    continue;
                }

                if (goal.header.stamp == path_planner_goal_timestamp_ &&
                    odometry.header.stamp == path_planner_odometry_timestamp_) {
                    // neither the goal nor the odometry has changed since the last update
                    continue;
                }

                path_planner_odometry_timestamp_ = odometry.header.stamp;
                path_planner_goal_timestamp_ = goal.header.stamp;

                const Eigen::Vector2f odom_to_goal(goal.pose.position.x - odometry.pose.pose.position.x,
                                                   goal.pose.position.y - odometry.pose.pose.position.y);

                // NOTE: goal_proximity_threshold (in map coordinates, units: meters) is used only for performance
                // reasons, nothing else. We don't want to repeatedly trigger the planner if we are already at
                // (or near) the goal.

                /*
                const float goal_proximity_threshold = ... ??? ...;

                if(odom_to_goal.norm() < goal_proximity_threshold) {
                    // we are already close to the goal
                    continue;
                }
                */

                boost::shared_ptr<OccupancyGrid> occupancy_grid;
                cv::Vec2i occupancy_grid_center;
                Eigen::Affine3f T_grid_to_map;
                {
                    std::lock_guard<std::mutex> scoped_lock(occupancy_grid_mutex_);

                    if(not dynamic_occupancy_grid_) {
                        continue;
                    }
                    assert(dynamic_occupancy_grid_->hasOccupancy());
                    occupancy_grid = boost::shared_ptr<OccupancyGrid>(new OccupancyGrid(*dynamic_occupancy_grid_));
                    assert(occupancy_grid->hasOccupancy());

                    // NOTE: don't use OpenCV's .copyTo()
                    occupancy_grid_center = dynamic_occupancy_grid_center_;
                    T_grid_to_map = T_grid_to_map_frame_;
                }

                number_of_computations++;

                // TODO: throttle marker message generation to some maximum frequency
                //       (by setting should_generate_marker_messages=false if some delta_t has not yet elapsed).

                boost::shared_ptr<ros::Publisher> marker_publisher =
                        should_generate_marker_messages ? rrt_visualization_publisher_ : nullptr;

                PathPlanner::GridPath seeded_solution;
                if (should_seed_last_solution) {
                    std::lock_guard<std::mutex> scoped_lock(last_path_mutex_);
                    for (int i = 0; i < last_shortest_path_.size(); i++) {
                        const Eigen::Vector2f old_waypoint_in_map_frame = last_shortest_path_[i];

                        const Eigen::Vector3f old_waypoint_in_new_grid_frame =
                                T_map_to_grid_frame_ *
                                Eigen::Vector3f(old_waypoint_in_map_frame(0),
                                                old_waypoint_in_map_frame(1), 0.0f);
                        seeded_solution.push_back(Eigen::Vector2i(old_waypoint_in_new_grid_frame(0),
                                                                  old_waypoint_in_new_grid_frame(1)));
                    }
                }

                auto[success, tree, path, grid_path] = path_planner_.run(
                        Eigen::Vector2f(goal.pose.position.x, goal.pose.position.y),
                        occupancy_grid,
                        occupancy_grid_center,
                        T_grid_to_map,
                        seeded_solution,
                        path_planner_options,
                        marker_publisher);

                if (!success) {
                    continue;
                } else {

                    std::lock_guard<std::mutex> scoped_lock(last_path_mutex_);
                    last_shortest_path_ = path;
                }

                if (not(path_to_goal_publishing_task_.valid() && path_to_goal_publishing_task_.wait_for(
                        std::chrono::nanoseconds(0)) != std::future_status::ready)) {

                    assert(occupancy_grid->hasOccupancy());
                    path_to_goal_publishing_task_ = std::async(
                            std::launch::async, [&, grid_path, occupancy_grid]() {
                                const auto[success, optimized_grid_path] = PathOptimizer(
                                        path_follower_options).optimizePath(
                                        grid_path, occupancy_grid);
                                if (success) {
                                    PathPlanner::MapPath optimized_path =
                                            TransformPath<float,float>(optimized_grid_path, T_grid_to_map);
                                    {
                                        std::lock_guard<std::mutex> scoped_lock(path_to_goal_mutex_);
                                        path_to_goal_ = optimized_path;
                                        //recomputePathToGoalKDTree();
                                        //computeAndPublishCommandedGoal(path_to_goal_, path_to_goal_kdtree_, odometry);
                                        //computeAndPublishCommandedGoal(path_to_goal_, odometry);
                                        computeAndPublishCommandedSteeringAngle(path_to_goal_, odometry);
                                    }
                                    {
                                        std::lock_guard<std::mutex> scoped_lock(path_to_goal_publisher_mutex_);
                                        publishCommandedPathToGoal(optimized_path);
                                    }
                                }
                            });
                }

            } else {
                sampling_rate.sleep();
            }
        }
        ROS_INFO("Path planner is shutting down...");
    }

    bool MotionPlanner::computeAndPublishCommandedSteeringAngle(
            const PathPlanner::MapPath &path,
            const nav_msgs::Odometry &odometry) const {

        if(path_to_goal_.empty()) {
            return(false);
        }

        // The car's current position in the "map" frame
        const Eigen::Vector2f car_position(odometry.pose.pose.position.x, odometry.pose.pose.position.y);
        const float steering_lookahead_distance = steering_lookahead_distance_.load();

        // TODO: resample path to goal?

        int index_of_closest_point_on_path = (-1);
        {
            float shortest_distance = std::numeric_limits<float>::max();
            for (int i = 0; i < path_to_goal_.size(); i++) {
                const float distance = Eigen::Vector2f(car_position - path_to_goal_[i]).norm();
                if (distance < shortest_distance) {
                    shortest_distance = distance;
                    index_of_closest_point_on_path = i;
                }
            }
        }
        assert(index_of_closest_point_on_path >= 0);

        Eigen::Vector2f goal_position = path_to_goal_.back();
        {
            for (int i = index_of_closest_point_on_path; i < path_to_goal_.size(); i++) {
                const float distance_to_waypoint = (car_position - path_to_goal_[i]).norm();
                if (distance_to_waypoint > steering_lookahead_distance) {
                    if(i>0) {
                        const float previous_distance_to_waypoint = (car_position - path_to_goal_[i-1]).norm();

                        const float error_1 = std::abs(distance_to_waypoint - steering_lookahead_distance);
                        const float error_0 = std::abs(steering_lookahead_distance - previous_distance_to_waypoint);

                        const float weight_1 = error_0 / (error_0 + error_1);
                        const float weight_0 = (1.0f - weight_1);

                        goal_position = (weight_1 * path_to_goal_[i]) + (weight_0 * path_to_goal_[i-1]);

                    } else {
                        goal_position = path_to_goal_[i];
                    }
                    break;
                }
            }
        }

        // Publish the goal
        {
            geometry_msgs::PointStamped goal_msg;
            goal_msg.header.stamp = ros::Time::now();
            goal_msg.header.frame_id = FRAME_MAP;
            goal_msg.point.x = goal_position(0);
            goal_msg.point.y = goal_position(1);
            goal_msg.point.z = 0.0f;
            commanded_goal_publisher_->publish(goal_msg);
        }

        // =============================================================================================================
        // Pure Pursuit control law
        // =============================================================================================================

        const Eigen::Vector3f goal_position_in_base_link_frame =
                T_map_to_base_link_frame_ * Eigen::Vector3f(goal_position(0), goal_position(1), 0.0);

        auto cartesian_to_polar = [](const Eigen::Vector2f& point) -> std::tuple<float, float> {
            float theta = std::atan2(point(1),point(0));
            float rho = point.norm();
            return {theta, rho};
        };

        auto [theta, l_d] = cartesian_to_polar(Eigen::Vector2f(goal_position_in_base_link_frame(0),
                                                               goal_position_in_base_link_frame(1)));
        if(theta > M_PI) {
            theta -= M_2_PI; // ensure that theta stays in the range [-pi,pi]
        }
        assert(theta >= -M_PI && theta <= +M_PI);

        const float L = vehicle_wheelbase_;
        float delta = std::atan((2.0 * L * std::sin(theta)) / l_d);

        std_msgs::Float32 steering_angle_message;
        steering_angle_message.data = delta; // phi ... commanded steering angle (in radians)
        commanded_steering_angle_publisher_->publish(steering_angle_message);
        return (true);
    }

    void MotionPlanner::publishCommandedPathToGoal(const PathPlanner::MapPath &path) const {
        nav_msgs::Path path_msg;
        path_msg.header.frame_id = FRAME_MAP;
        path_msg.header.stamp = ros::Time::now();

        for (int i = 0; i < path.size(); i++) {
            const Eigen::Vector2f &waypoint_position = path[i];
            geometry_msgs::PoseStamped waypoint;
            waypoint.header.stamp = path_msg.header.stamp;
            waypoint.header.frame_id = path_msg.header.frame_id;
            waypoint.pose.position.x = waypoint_position(0);
            waypoint.pose.position.y = waypoint_position(1);
            waypoint.pose.position.z = 0.0f;
            waypoint.pose.orientation.x = 0.0;
            waypoint.pose.orientation.y = 0.0;
            waypoint.pose.orientation.z = 0.0;
            waypoint.pose.orientation.w = 1.0;
            path_msg.poses.push_back(waypoint);
        }
        if (path_msg.poses.size() >= 3) {
            for (int i = 0; i < path_msg.poses.size(); i++) {
                geometry_msgs::PoseStamped &waypoint = path_msg.poses[i];
                const int j_prev = (i > 0) ? (i - 1) : (i);
                const int j_next = (i < (path_msg.poses.size() - 1)) ? (i + 1) : (i);
                const float x0 = path_msg.poses[j_prev].pose.position.x;
                const float y0 = path_msg.poses[j_prev].pose.position.y;
                const float x1 = path_msg.poses[j_next].pose.position.x;
                const float y1 = path_msg.poses[j_next].pose.position.y;
                const Eigen::Vector2f path_direction(x1 - x0, y1 - y0);
                const float theta = std::atan2(path_direction(1), path_direction(0));
                const Eigen::Quaternionf q(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));
                waypoint.pose.orientation.x = q.x();
                waypoint.pose.orientation.y = q.y();
                waypoint.pose.orientation.z = q.z();
                waypoint.pose.orientation.w = q.w();
            }
        }

        path_to_goal_publisher_->publish(path_msg);
    }
} // namespace motion_planner

using namespace motion_planner;

void DynamicReconfigureCallback(
        group3_lab7::motion_planner_Config &config,
        uint32_t level,
        boost::shared_ptr<MotionPlanner> motion_planner) {

    std::lock_guard<std::mutex> scoped_lock(motion_planner->configuration_mutex_);
    motion_planner->configuration_ = config;
    motion_planner->scan_dt_threshold_.store(1.0f / config.maximum_occupancy_grid_update_frequency);
    motion_planner->goal_dt_threshold_.store(1.0f / config.maximum_goal_update_frequency);
    motion_planner->steering_lookahead_distance_.store(config.lookahead_distance);
    motion_planner->has_valid_configuration_.store(true);
}

int main(int argc, char **argv) {
#if !defined(NDEBUG)
    std::cerr << "WARNING: Debug build." << std::endl;
#endif
    try {
        ros::init(argc, argv, "motion_planner");
        boost::shared_ptr<MotionPlanner> motion_planner(new MotionPlanner());

        dynamic_reconfigure::Server<group3_lab7::motion_planner_Config> dynamic_reconfigure_server;
        dynamic_reconfigure::Server<group3_lab7::motion_planner_Config>::CallbackType dynamic_reconfigure_callback;
        dynamic_reconfigure_callback = boost::bind(&DynamicReconfigureCallback, _1, _2, motion_planner);
        dynamic_reconfigure_server.setCallback(dynamic_reconfigure_callback);

        ros::spin();
        motion_planner.reset();
    } catch(std::exception &exception) {
        std::cerr << "ERROR: " << exception.what() << std::endl;
    }
    return 0;
}
