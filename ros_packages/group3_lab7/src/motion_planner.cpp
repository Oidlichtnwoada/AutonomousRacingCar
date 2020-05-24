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

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
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

#define LIMIT_DYNAMIC_OCCUPANCY_GRID_TO_FORWARD_DIRECTION // <-- this is just an optimization

#define SUBSCRIBER_MESSAGE_QUEUE_SIZE 1000
#define GRID_PUBLISHER_MESSAGE_QUEUE_SIZE 10
#define MARKER_PUBLISHER_MESSAGE_QUEUE_SIZE 10

static_assert(SUBSCRIBER_MESSAGE_QUEUE_SIZE > 0);
static_assert(GRID_PUBLISHER_MESSAGE_QUEUE_SIZE > 0);
static_assert(MARKER_PUBLISHER_MESSAGE_QUEUE_SIZE > 0);

#define TOPIC_SCAN "/scan"
#define TOPIC_ODOM "/odom"
#if defined(NDEBUG)
#define TOPIC_GOAL "/motion_planner/goal" // Release
#else
#define TOPIC_GOAL "/follow_me/goal"
//#define TOPIC_GOAL "/move_base_simple/goal" // Debug
#endif
#define TOPIC_MAP  "/map"
#define TOPIC_DYNAMIC_OCCUPANCY_GRID "/motion_planner/dynamic_occupancy_grid"
#define TOPIC_STATIC_OCCUPANCY_GRID  "/motion_planner/static_occupancy_grid"
#define TOPIC_RRT_VISUALIZATION      "/motion_planner/rrt_visualization"
#define FRAME_MAP       "map"
#define FRAME_BASE_LINK "base_link"
#define FRAME_LASER     "laser"

#define DEFAULT_VEHICLE_WHEELBASE 0.3302 // units: m (see f110_simulator/params.yaml)
#define DEFAULT_VEHICLE_WIDTH     0.2032 // units: m (see f110_simulator/params.yaml)
#define DEFAULT_MAX_LASER_SCAN_DISTANCE 5.0 // units: m
#define DEFAULT_DYNAMIC_OCCUPANCY_GRID_UPDATE_FREQUENCY 60 // units: Hz
#define DEFAULT_GOAL_UPDATE_FREQUENCY 60 // units: Hz

static_assert(DEFAULT_VEHICLE_WHEELBASE > 0);
static_assert(DEFAULT_VEHICLE_WIDTH > 0);
static_assert(DEFAULT_MAX_LASER_SCAN_DISTANCE > 0);
static_assert(DEFAULT_DYNAMIC_OCCUPANCY_GRID_UPDATE_FREQUENCY >= 0);

#undef ENABLE_DYNAMIC_OCCUPANCY_GRID_DISTANCE_TRANSFORM // <-- not enabled by default!

class MotionPlanner {

public:
    MotionPlanner();
    ~MotionPlanner();
    typedef boost::shared_ptr<sensor_msgs::LaserScan const> LaserScanMessage;
    typedef boost::shared_ptr<geometry_msgs::PoseStamped const> PoseStampedMessage;
    typedef boost::shared_ptr<nav_msgs::Odometry const> OdometryMessage;
    typedef boost::shared_ptr<nav_msgs::OccupancyGrid const> OccupancyGridMessage;

private:
    float vehicle_wheelbase_; // units: m
    float vehicle_width_; // units: m
    float max_laser_scan_distance_; // units: m

    void laserScanSubscriberCallback(LaserScanMessage);
    void odomSubscriberCallback(OdometryMessage);
    void goalSubscriberCallback(PoseStampedMessage);
    void mapSubscriberCallback(OccupancyGridMessage);
    bool debugServiceCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
    ros::NodeHandle node_handle_;

    boost::shared_ptr<ros::Subscriber> laser_scan_subscriber_;
    boost::shared_ptr<ros::Subscriber> odometry_subscriber_;
    boost::shared_ptr<ros::Subscriber> goal_subscriber_;
    boost::shared_ptr<ros::Subscriber> map_subscriber_;
    boost::shared_ptr<ros::ServiceServer> debug_service_;
    boost::shared_ptr<ros::Publisher> dynamic_occupancy_grid_publisher_;
    boost::shared_ptr<ros::Publisher> static_occupancy_grid_publisher_;
    boost::shared_ptr<ros::Publisher> rrt_visualization_publisher_;

    tf2_ros::Buffer tf_buffer_;
    boost::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    ros::Time last_scan_timestamp_;
    float scan_dt_threshold_;

    ros::Time last_goal_timestamp_;
    float goal_dt_threshold_;

    // =================================================================================================================
    // Occupancy grids (static and dynamic)
    // =================================================================================================================

    MotionPlanner::OccupancyGridMessage map_; // copied from "/map"

    std::mutex occupancy_grid_mutex_;
    OccupancyGrid static_occupancy_grid_; // just a thin wrapper around map_->data.data()
    OccupancyGrid dynamic_occupancy_grid_;
#ifdef ENABLE_DYNAMIC_OCCUPANCY_GRID_DISTANCE_TRANSFORM
    cv::Mat dynamic_occupancy_grid_distances_;
#endif

    cv::Vec2i static_occupancy_grid_center_;
    cv::Vec2i dynamic_occupancy_grid_center_;

    // Affine transformations from grid coordinates to the "map" and "laser" frames
    Eigen::Affine3f T_dynamic_oc_pixels_to_laser_frame_;      // dynamic_occupancy_grid (pixels) to laser frame (meters)
    Eigen::Affine3f T_dynamic_oc_pixels_to_map_frame_;        // dynamic_occupancy_grid (pixels) to map frame (meters)

    // =================================================================================================================
    // Path planning algorithms (RRT/RRT*/Informed-RRT*)
    // =================================================================================================================

    PathPlanner path_planner_;
    boost::shared_ptr<std::thread> path_planner_thread_;
    void runPathPlanner();

    std::mutex odometry_mutex_;
    nav_msgs::Odometry current_odometry_;
    std::mutex goal_mutex_;
    geometry_msgs::PoseStamped current_goal_;

    ros::Time path_planner_odometry_timestamp_;
    ros::Time path_planner_goal_timestamp_;

    std::atomic<bool> should_plan_path_;

#if !defined(NDEBUG)
    unsigned long long laser_scan_messages_received_;
    unsigned long long odometry_messages_received_;
    unsigned long long goal_messages_received_;
    unsigned long long map_messages_received_;
#endif
};

MotionPlanner::~MotionPlanner() {
    path_planner_thread_->join();
}

MotionPlanner::MotionPlanner()
        : node_handle_(ros::NodeHandle())
        , laser_scan_subscriber_(new ros::Subscriber(node_handle_.subscribe(TOPIC_SCAN, SUBSCRIBER_MESSAGE_QUEUE_SIZE, &MotionPlanner::laserScanSubscriberCallback, this)))
        , odometry_subscriber_(new ros::Subscriber(node_handle_.subscribe(TOPIC_ODOM, SUBSCRIBER_MESSAGE_QUEUE_SIZE, &MotionPlanner::odomSubscriberCallback, this)))
        , goal_subscriber_(new ros::Subscriber(node_handle_.subscribe(TOPIC_GOAL, SUBSCRIBER_MESSAGE_QUEUE_SIZE, &MotionPlanner::goalSubscriberCallback, this)))
        , map_subscriber_(new ros::Subscriber(node_handle_.subscribe(TOPIC_MAP, SUBSCRIBER_MESSAGE_QUEUE_SIZE, &MotionPlanner::mapSubscriberCallback, this)))
        , tf_listener_(new tf2_ros::TransformListener(tf_buffer_, node_handle_, true))
        , debug_service_(new ros::ServiceServer(node_handle_.advertiseService("debug", &MotionPlanner::debugServiceCallback, this)))
        , static_occupancy_grid_publisher_(new ros::Publisher(node_handle_.advertise<nav_msgs::GridCells>(TOPIC_STATIC_OCCUPANCY_GRID, GRID_PUBLISHER_MESSAGE_QUEUE_SIZE)))
        , dynamic_occupancy_grid_publisher_(new ros::Publisher(node_handle_.advertise<nav_msgs::GridCells>(TOPIC_DYNAMIC_OCCUPANCY_GRID, GRID_PUBLISHER_MESSAGE_QUEUE_SIZE)))
        , rrt_visualization_publisher_(new ros::Publisher(node_handle_.advertise<visualization_msgs::Marker>(TOPIC_RRT_VISUALIZATION, MARKER_PUBLISHER_MESSAGE_QUEUE_SIZE)))
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

    float dynamic_occupancy_grid_update_frequency; // units: Hz
    float goal_update_frequency; // units: Hz

    node_handle_.param<float>("dynamic_occupancy_grid_update_frequency", dynamic_occupancy_grid_update_frequency, DEFAULT_DYNAMIC_OCCUPANCY_GRID_UPDATE_FREQUENCY);
    node_handle_.param<float>("goal_update_frequency", goal_update_frequency, DEFAULT_GOAL_UPDATE_FREQUENCY);

    auto check_value = [](float f, std::string s) {
        if(f <= 0.0f) {
            std::ostringstream o;
            o << "Requirement violated: (" << s << " > 0)";
            ROS_ERROR("%s", o.str().c_str());
            throw(std::runtime_error(o.str()));
        }
    };
    check_value(vehicle_wheelbase_, "vehicle_wheelbase");
    check_value(vehicle_width_, "vehicle_width_");
    check_value(max_laser_scan_distance_, "max_laser_scan_distance");
    check_value(dynamic_occupancy_grid_update_frequency, "dynamic_occupancy_grid_update_frequency");

    scan_dt_threshold_ = 1.0f / dynamic_occupancy_grid_update_frequency;
    goal_dt_threshold_ = 1.0f / goal_update_frequency;

    path_planner_thread_ = boost::shared_ptr<std::thread>(new std::thread(&MotionPlanner::runPathPlanner, this));
}

bool MotionPlanner::debugServiceCallback(std_srvs::Empty::Request& request,
                                         std_srvs::Empty::Response& response) {

    std::lock_guard<std::mutex> scoped_lock(occupancy_grid_mutex_);
    // Trigger with "rosservice call /debug"
    std::cout << "MotionPlanner::debugServiceCallback()" << std::endl;
    cv::imwrite("/tmp/static_occupancy_grid.png", static_occupancy_grid_);
    cv::imwrite("/tmp/dynamic_occupancy_grid.png", dynamic_occupancy_grid_);
#ifdef ENABLE_DYNAMIC_OCCUPANCY_GRID_DISTANCE_TRANSFORM
    cv::imwrite("/tmp/dynamic_occupancy_grid_distances.png", dynamic_occupancy_grid_distances_);
#endif
    return(true);
}

void MotionPlanner::laserScanSubscriberCallback(MotionPlanner::LaserScanMessage scan_msg) {

#if !defined(NDEBUG)
    laser_scan_messages_received_++;
#endif

    static unsigned int messages_skipped = 0; // for debugging only

    std::unique_lock<std::mutex> lock(occupancy_grid_mutex_, std::try_to_lock);
    if(!lock.owns_lock()){
        messages_skipped++;
        return;
    }

    nav_msgs::Odometry current_odometry;
    {
        std::lock_guard<std::mutex> scoped_lock(odometry_mutex_);
        current_odometry = current_odometry_;
    }

    if( (current_odometry.header.stamp.isZero()) ||
        (not tf_buffer_.canTransform(FRAME_BASE_LINK, scan_msg->header.frame_id,ros::Time(0))) ) {
        messages_skipped++;
        return;
    }

    // Enforce max. update frequency
    const bool has_previous_timestamp = not last_scan_timestamp_.isZero();
    const ros::Time& current_timestamp = scan_msg->header.stamp;
    if(!has_previous_timestamp) {
        last_scan_timestamp_ = current_timestamp;
        return;
    }

    const float dt = current_timestamp.toSec() - last_scan_timestamp_.toSec();
    if(dt < scan_dt_threshold_) {
        return;
    }
    last_scan_timestamp_ = current_timestamp;

    // Obtain laser-->base_link transform
    Eigen::Isometry3f T_laser_to_base_link = tf2::transformToEigen(tf_buffer_.lookupTransform(
            FRAME_BASE_LINK, FRAME_LASER, ros::Time(0)).transform).cast<float>();

    auto poseToEigen = [](const geometry_msgs::Pose& pose) -> Eigen::Isometry3f {
        return Eigen::Isometry3f(Eigen::Translation3f(pose.position.x, pose.position.y, pose.position.z)
                                 * Eigen::Quaternionf(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z));
    };

    Eigen::Isometry3f T_base_link_to_map = poseToEigen(current_odometry.pose.pose);


    // Obtain laser-->map transform
    Eigen::Isometry3f T_laser_to_map = tf2::transformToEigen(tf_buffer_.lookupTransform(
            FRAME_MAP, scan_msg->header.frame_id, ros::Time(0)).transform).cast<float>();

    const float pixels_per_meter = 1.0f / map_->info.resolution;
    const float meters_per_pixel = map_->info.resolution;

    // =================================================================================================================
    // Compute coordinate frame transformation matrices
    // =================================================================================================================

    const Eigen::Vector3f dynamic_occupancy_grid_pixel_center((float)dynamic_occupancy_grid_center_(0),
                                                              (float)dynamic_occupancy_grid_center_(1),
                                                              0.0f);

    Eigen::Affine3f A = Eigen::Affine3f::Identity();

    A.pretranslate((-1.0f) * Eigen::Vector3f((float) dynamic_occupancy_grid_center_(0),
                                   (float) dynamic_occupancy_grid_center_(1),
                                   0.0f));

    A.prescale(meters_per_pixel); // pixels to meters

    A.prescale(Eigen::Vector3f(-1.0, 1.0, 1.0)); // <--- NOTE!
    A.prerotate(Eigen::AngleAxisf(-M_PI_2, Eigen::Vector3f::UnitZ())); // <--- NOTE!

    T_dynamic_oc_pixels_to_laser_frame_ = A.matrix();

    //A = T_laser_to_map.matrix() * A.matrix();
    A = T_base_link_to_map.matrix() * T_laser_to_base_link.matrix() * A.matrix();

    T_dynamic_oc_pixels_to_map_frame_ = A.matrix();

    // =================================================================================================================

    dynamic_occupancy_grid_.setTo(cv::Scalar(GRID_CELL_IS_FREE)); // clear the grid

    const float angle_min = scan_msg->angle_min;
    const float angle_increment = scan_msg->angle_increment;
    for(auto const& element : scan_msg->ranges | boost::adaptors::indexed()) {
        const float range = element.value();
        const float angle = angle_min + (float) element.index() * angle_increment;

#ifdef LIMIT_DYNAMIC_OCCUPANCY_GRID_TO_FORWARD_DIRECTION
        if(angle < -M_PI_2 || angle > M_PI_2) {
            continue;
        }
#endif

        if(std::isfinite(range) && range <= max_laser_scan_distance_) {

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
            const int u_max = dynamic_occupancy_grid_.rows-1;
            const int v_max = dynamic_occupancy_grid_.cols-1;

            assert(u >= 0 && v >= 0 && u < dynamic_occupancy_grid_.rows && v < dynamic_occupancy_grid_.cols);
            dynamic_occupancy_grid_.at<uint8_t>(u,v) = GRID_CELL_IS_OCCUPIED;
        }
    }

    // Expand the dynamic occupancy grid
    const float vehicle_width_in_pixels = vehicle_width_ * pixels_per_meter;
    dynamic_occupancy_grid_.expand(vehicle_width_in_pixels);
    dynamic_occupancy_grid_publisher_->publish(dynamic_occupancy_grid_.convertToGridCellsMessage(
            dynamic_occupancy_grid_center_, meters_per_pixel, FRAME_LASER));

#ifdef ENABLE_DYNAMIC_OCCUPANCY_GRID_DISTANCE_TRANSFORM

    static_assert(GRID_CELL_IS_OCCUPIED == 0 && GRID_CELL_IS_FREE == 255, "Invalid GRID_CELL constants.");

    // Calculate the distance to the closest blocked cell for each cell of the grid

    // TODO: Do we want a 3x3 or a 5x5 mask for DIST_L2?
    // NOTE: for DIST_L1, a 3x3 mask gives the same result as 5x5 or any larger
    //       See: https://docs.opencv.org/master/d7/d1b/group__imgproc__misc.html
    cv::distanceTransform(
            dynamic_occupancy_grid_,
            dynamic_occupancy_grid_distances_,
            USE_L1_DISTANCE_METRIC ? cv::DIST_L1 : cv::DIST_L2,
            USE_L1_DISTANCE_METRIC ? 3 : 5,
            USE_L1_DISTANCE_METRIC ? CV_8U : CV_32F);
#endif

    should_plan_path_.store(true);
}

void MotionPlanner::odomSubscriberCallback(MotionPlanner::OdometryMessage odom_msg) {

    #if !defined(NDEBUG)
    odometry_messages_received_++;
    #endif

    std::lock_guard<std::mutex> scoped_lock(odometry_mutex_);
    current_odometry_ = *odom_msg;
    assert(not current_odometry_.header.stamp.isZero());

    // NOTE: should_plan_path_.store(true) should NOT be used here!
}

void MotionPlanner::goalSubscriberCallback(MotionPlanner::PoseStampedMessage goal_msg) {

#if !defined(NDEBUG)
    goal_messages_received_++;
#endif

    // Enforce max. update frequency
    const bool has_previous_timestamp = not last_goal_timestamp_.isZero();
    const ros::Time& current_timestamp = goal_msg->header.stamp;
    if(!has_previous_timestamp) {
        last_goal_timestamp_ = current_timestamp;
        return;
    }

    const float dt = current_timestamp.toSec() - last_goal_timestamp_.toSec();
    if(dt < goal_dt_threshold_) {
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
    const nav_msgs::MapMetaData& info = map_msg->info;

    const float pixels_per_meter = 1.0f / map_->info.resolution;
    const float meters_per_pixel = map_->info.resolution;

    static_occupancy_grid_center_ = cv::Vec2i(
            static_cast<int>(-map_->info.origin.position.y * pixels_per_meter),  // rows <--> y
            static_cast<int>(-map_->info.origin.position.x * pixels_per_meter)); // cols <--> x
    static_occupancy_grid_ = OccupancyGrid(cv::Mat(info.height,
            info.width, CV_8UC1, (void*) map_->data.data())); // just a thin wrapper around map_->data.data()

    cv::threshold(static_occupancy_grid_, static_occupancy_grid_ /* in-place */, 1, 255, cv::THRESH_BINARY);
    cv::bitwise_not(static_occupancy_grid_, static_occupancy_grid_);

    // Publish static occupancy grid (for RViz)
    static_occupancy_grid_publisher_->publish(static_occupancy_grid_.convertToGridCellsMessage(
            static_occupancy_grid_center_, meters_per_pixel, FRAME_MAP));

    if(info.origin.orientation.x != 0.0 || info.origin.orientation.y != 0.0 || info.origin.orientation.z != 0.0) {
        ROS_ERROR("Handling non-zero map rotations is not implemented.");
        throw(std::runtime_error("Handling non-zero map rotations is not implemented."));
    }

    if((info.height * info.width) > (1000 * 1000)) {
        std::ostringstream o;
        o << "Map dimensions (" << info.height << "x" << info.width <<
          ") are quite large. Consider reducing the resolution.";
        ROS_WARN("%s",o.str().c_str());
    }

    // Create dynamic occupancy grid
    const float laser_scan_diameter_in_pixels = 2.0f * max_laser_scan_distance_ * pixels_per_meter;
    unsigned int dynamic_occupancy_grid_cols = static_cast<unsigned int>(std::ceil(laser_scan_diameter_in_pixels)) + 2;
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
    dynamic_occupancy_grid_center_ = cv::Vec2i(center_col, center_row); // NOTE: grid is transposed w.r.t. vehicle's local coordinate system
    const cv::Scalar grid_cell_is_free(GRID_CELL_IS_FREE);
    dynamic_occupancy_grid_ = cv::Mat(dynamic_occupancy_grid_cols, dynamic_occupancy_grid_rows, CV_8UC1, grid_cell_is_free);

#ifdef ENABLE_DYNAMIC_OCCUPANCY_GRID_DISTANCE_TRANSFORM
    const cv::Scalar zero(0.0);
    dynamic_occupancy_grid_distances_ =
            cv::Mat(dynamic_occupancy_grid_cols,
                    dynamic_occupancy_grid_rows,
                    USE_L1_DISTANCE_METRIC ? CV_8UC1 : CV_32F,
                    zero);
#endif

    should_plan_path_.store(true);
}

void MotionPlanner::runPathPlanner() {

    // TODO: expose options to dynamic_reconfigure
    PathPlanner::Options options = {.algorithm = PathPlanner::INFORMED_RRT_STAR};

    // NOTE: goal_proximity_threshold is used only for performance reasons, nothing else.
    // We don't want to repeatedly trigger the planner if we are already at (or near) the goal.
    constexpr float goal_proximity_threshold = 0.1; // units: meters // TODO: this is an arbitrary choice for testing.

    ROS_INFO("Path planner is starting...");
    static unsigned int number_of_computations = 0; // for debugging only

    ros::Rate sampling_rate(1000); // 1000 hz (chosen to match the /odom update frequency)
    while (!ros::isShuttingDown()) {
        bool expected = true;
        if(should_plan_path_.compare_exchange_strong(expected, false)) {

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

            if(goal.header.stamp.isZero()) {
                // no goal has been set
                continue;
            }

            if(goal.header.stamp == path_planner_goal_timestamp_ &&
               odometry.header.stamp == path_planner_odometry_timestamp_) {
                // neither the goal nor the odometry has changed since the last update
                continue;
            }

            path_planner_odometry_timestamp_ = odometry.header.stamp;
            path_planner_goal_timestamp_ = goal.header.stamp;

            const Eigen::Vector2f odom_to_goal(goal.pose.position.x - odometry.pose.pose.position.x,
                                               goal.pose.position.y - odometry.pose.pose.position.y);

            if(odom_to_goal.norm() < goal_proximity_threshold) {
                // we are already close to the goal
                continue;
            }

            cv::Mat occupancy_grid;
            cv::Vec2i occupancy_grid_center;
            Eigen::Affine3f T_grid_to_map;
            {
                std::lock_guard<std::mutex> scoped_lock(occupancy_grid_mutex_);
                dynamic_occupancy_grid_.copyTo(occupancy_grid); // make a local copy of the occupancy grid
                occupancy_grid_center = dynamic_occupancy_grid_center_;
                T_grid_to_map = T_dynamic_oc_pixels_to_map_frame_;
            }

            number_of_computations++;
            bool should_generate_marker_messages = true;
            // TODO: throttle marker message generation to some maximum frequency
            //       (by setting should_generate_marker_messages=false if some delta_t has not yet elapsed).

            boost::shared_ptr<ros::Publisher> marker_publisher =
                    should_generate_marker_messages ? rrt_visualization_publisher_ : nullptr;


            auto [success, tree, path] = path_planner_.run(
                    Eigen::Vector2f(goal.pose.position.x, goal.pose.position.y),
                    occupancy_grid,
                    occupancy_grid_center,
                    T_grid_to_map,
                    options,
                    marker_publisher);
        } else {
            sampling_rate.sleep();
        }
    }

    ROS_INFO("Path planner is shutting down...");
}

int main(int argc, char **argv) {
#if !defined(NDEBUG)
    std::cerr << "WARNING: Debug build." << std::endl;
#endif
    try {
        ros::init(argc, argv, "motion_planner");
        boost::shared_ptr<MotionPlanner> motion_planner(new MotionPlanner());
        ros::spin();
        motion_planner.reset();
    } catch(std::exception &exception) {
        std::cerr << "ERROR: " << exception.what() << std::endl;
    }
    return 0;
}
