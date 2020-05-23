/* -------------------------------------------------------
 * VU Autonomous Racing Cars (2020S) - TU Wien
 * -------------------------------------------------------
 * Team 3: Adelmann, Brantner, Lukitsch, Pintaric
 * -------------------------------------------------------
 */

#include <iostream>
#include <string>
#include <cmath>
#include <random>
#include <thread>
#include <atomic>
#include <mutex>
#include <future>

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
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/mat.hpp>

#include <nanoflann/nanoflann.hpp> // https://github.com/jlblancoc/nanoflann

#include <motion_planner/Tree.h>

#define LIMIT_DYNAMIC_OCCUPANCY_GRID_TO_FORWARD_DIRECTION // <-- this is just an optimization

#define SUBSCRIBER_MESSAGE_QUEUE_SIZE 1000
#define GRID_PUBLISHER_MESSAGE_QUEUE_SIZE 10
#define MARKER_PUBLISHER_MESSAGE_QUEUE_SIZE 10

static_assert(SUBSCRIBER_MESSAGE_QUEUE_SIZE > 0);
static_assert(GRID_PUBLISHER_MESSAGE_QUEUE_SIZE > 0);
static_assert(MARKER_PUBLISHER_MESSAGE_QUEUE_SIZE > 0);

#define TOPIC_SCAN "/scan"
#define TOPIC_ODOM "/odom"
#define TOPIC_GOAL "/move_base_simple/goal"
#define TOPIC_MAP  "/map"
#define TOPIC_DYNAMIC_OCCUPANCY_GRID "/motion_planner/dynamic_occupancy_grid"
#define TOPIC_STATIC_OCCUPANCY_GRID  "/motion_planner/static_occupancy_grid"
#define TOPIC_RRT_VISUALIZATION      "/motion_planner/rrt_visualization"
#define FRAME_MAP   "map"
#define FRAME_LASER "laser"

#define DEFAULT_VEHICLE_WHEELBASE 0.3302 // units: m (see f110_simulator/params.yaml)
#define DEFAULT_VEHICLE_WIDTH     0.2032 // units: m (see f110_simulator/params.yaml)
#define DEFAULT_MAX_LASER_SCAN_DISTANCE 5.0 // units: m
#define DEFAULT_DYNAMIC_OCCUPANCY_GRID_UPDATE_FREQUENCY 20 // units: Hz

static_assert(DEFAULT_VEHICLE_WHEELBASE > 0);
static_assert(DEFAULT_VEHICLE_WIDTH > 0);
static_assert(DEFAULT_MAX_LASER_SCAN_DISTANCE > 0);
static_assert(DEFAULT_DYNAMIC_OCCUPANCY_GRID_UPDATE_FREQUENCY >= 0);

#define GRID_CELL_IS_FREE      255
#define GRID_CELL_IS_OCCUPIED    0

// Alternative:
//#define GRID_CELL_IS_FREE        0
//#define GRID_CELL_IS_OCCUPIED  255

static_assert(GRID_CELL_IS_FREE >= 0 && GRID_CELL_IS_FREE <= 255);
static_assert(GRID_CELL_IS_OCCUPIED >= 0 && GRID_CELL_IS_OCCUPIED <= 255);
static_assert(GRID_CELL_IS_FREE != GRID_CELL_IS_OCCUPIED);

#undef ENABLE_DYNAMIC_OCCUPANCY_GRID_DISTANCE_TRANSFORM // <-- not enabled by default!

constexpr bool USE_L1_DISTANCE_METRIC = true;
// L1 has proven to work well enough and is faster than L2,
// so there is really no good reason to switch back to L2.

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
    float dynamic_occupancy_grid_update_frequency_; // units: Hz

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

    // =================================================================================================================
    // Occupancy grids (static and dynamic)
    // =================================================================================================================

    MotionPlanner::OccupancyGridMessage map_; // copied from "/map"

    std::mutex occupancy_grid_mutex_;
    cv::Mat static_occupancy_grid_; // just a thin wrapper around map_->data.data()
    cv::Mat dynamic_occupancy_grid_;
#ifdef ENABLE_DYNAMIC_OCCUPANCY_GRID_DISTANCE_TRANSFORM
    cv::Mat dynamic_occupancy_grid_distances_;
#endif

    cv::Vec2i static_occupancy_grid_center_;
    cv::Vec2i dynamic_occupancy_grid_center_;

    // Affine transformations from grid coordinates to the "map" and "laser" frames
    Eigen::Affine3f T_dynamic_oc_pixels_to_laser_frame_;      // dynamic_occupancy_grid (pixels) to laser frame (meters)
    Eigen::Affine3f T_dynamic_oc_pixels_to_map_frame_;        // dynamic_occupancy_grid (pixels) to map frame (meters)

    /* inline */ bool isGridCellOccupied(int row, int col) const;

    void expandOccupancyGrid(cv::Mat& grid);

    nav_msgs::GridCells convertToGridCellsMessage(
            cv::Mat& grid, const cv::Vec2i grid_center, const std::string frame_id) const;

    // =================================================================================================================
    // Path planning algorithms (RRT/RRT*)
    // =================================================================================================================

    boost::shared_ptr<std::thread> path_planner_thread_;
    void runPathPlanner();

    template<typename T, bool USE_L1_DISTANCE_METRIC, bool USE_RRT_STAR = false>
    std::tuple<
            bool, // success
            motion_planner::Tree<T, USE_L1_DISTANCE_METRIC>,
            std::deque<Eigen::Vector2f>,
            std::vector<visualization_msgs::Marker> >
    runRRT(const nav_msgs::Odometry& odometry,
           const geometry_msgs::PoseStamped& goal,
           bool generate_marker_messages = false);

    std::mutex odometry_mutex_;
    geometry_msgs::PoseStamped current_goal_;
    nav_msgs::Odometry current_odometry_;

    std::atomic<bool> should_plan_path_;

    std::mt19937 random_generator_;
    static constexpr std::mt19937::result_type random_seed_ = 9876543210UL; // fixed seed for reproducibility

    bool expandPath(const cv::Vec2i start, const cv::Vec2i destination, cv::Vec2i& end, const int max_expansion_distance) const;

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
        , tf_listener_(new tf2_ros::TransformListener(tf_buffer_, true))
        , debug_service_(new ros::ServiceServer(node_handle_.advertiseService("debug", &MotionPlanner::debugServiceCallback, this)))
        , static_occupancy_grid_publisher_(new ros::Publisher(node_handle_.advertise<nav_msgs::GridCells>(TOPIC_STATIC_OCCUPANCY_GRID, GRID_PUBLISHER_MESSAGE_QUEUE_SIZE)))
        , dynamic_occupancy_grid_publisher_(new ros::Publisher(node_handle_.advertise<nav_msgs::GridCells>(TOPIC_DYNAMIC_OCCUPANCY_GRID, GRID_PUBLISHER_MESSAGE_QUEUE_SIZE)))
        , random_generator_(std::mt19937(MotionPlanner::random_seed_))
        , rrt_visualization_publisher_(new ros::Publisher(node_handle_.advertise<visualization_msgs::Marker>(TOPIC_RRT_VISUALIZATION, MARKER_PUBLISHER_MESSAGE_QUEUE_SIZE)))
{
    should_plan_path_.store(false);
    node_handle_.param<float>("vehicle_wheelbase", vehicle_wheelbase_, DEFAULT_VEHICLE_WHEELBASE);
    node_handle_.param<float>("vehicle_width", vehicle_width_, DEFAULT_VEHICLE_WIDTH);
    node_handle_.param<float>("max_laser_scan_distance", max_laser_scan_distance_, DEFAULT_MAX_LASER_SCAN_DISTANCE);
    node_handle_.param<float>("dynamic_occupancy_grid_update_frequency", dynamic_occupancy_grid_update_frequency_, DEFAULT_DYNAMIC_OCCUPANCY_GRID_UPDATE_FREQUENCY);

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
    check_value(dynamic_occupancy_grid_update_frequency_, "dynamic_occupancy_grid_update_frequency");

    scan_dt_threshold_ = 1.0f / dynamic_occupancy_grid_update_frequency_;

    path_planner_thread_ = boost::shared_ptr<std::thread>(new std::thread(&MotionPlanner::runPathPlanner, this));
}

bool MotionPlanner::debugServiceCallback(std_srvs::Empty::Request& request,
                                         std_srvs::Empty::Response& response) {

    std::lock_guard<std::mutex> scoped_lock(occupancy_grid_mutex_);
    // Trigger with "rosservice call /debug"
    std::cout << "MotionPlanner::debugServiceCallback()" << std::endl;
    cv::imwrite("/tmp/dynamic_occupancy_grid.png", dynamic_occupancy_grid_);
#ifdef ENABLE_DYNAMIC_OCCUPANCY_GRID_DISTANCE_TRANSFORM
    cv::imwrite("/tmp/dynamic_occupancy_grid_distances.png", dynamic_occupancy_grid_distances_);
#endif
    return(true);
}

void MotionPlanner::laserScanSubscriberCallback(MotionPlanner::LaserScanMessage scan_msg) {

    static unsigned int messages_skipped = 0; // for debugging only

    std::unique_lock<std::mutex> lock(occupancy_grid_mutex_, std::try_to_lock);
    if(!lock.owns_lock()){
        messages_skipped++;
        return;
    }

    if(!tf_buffer_.canTransform(FRAME_MAP, scan_msg->header.frame_id,ros::Time(0))) {
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

    A = T_laser_to_map.matrix() * A.matrix();

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
    expandOccupancyGrid(dynamic_occupancy_grid_);
    dynamic_occupancy_grid_publisher_->publish(convertToGridCellsMessage(
            dynamic_occupancy_grid_, dynamic_occupancy_grid_center_, FRAME_LASER));

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
    std::lock_guard<std::mutex> scoped_lock(odometry_mutex_);
    current_odometry_ = *odom_msg;
    // TODO: compute conditions for triggering: should_plan_path_.store(true);
    //       (based either on elapsed time or driven distance)
}

void MotionPlanner::goalSubscriberCallback(MotionPlanner::PoseStampedMessage goal_msg) {
    std::lock_guard<std::mutex> scoped_lock(odometry_mutex_);
    current_goal_ = *goal_msg;
    ROS_INFO("Received new goal: [%f, %f]", current_goal_.pose.position.x, current_goal_.pose.position.y);
    should_plan_path_.store(true);
}

void MotionPlanner::mapSubscriberCallback(MotionPlanner::OccupancyGridMessage map_msg) {

    std::lock_guard<std::mutex> scoped_lock(occupancy_grid_mutex_);

    map_ = map_msg;
    const nav_msgs::MapMetaData& info = map_msg->info;

    const float pixels_per_meter = 1.0f / map_->info.resolution;
    static_occupancy_grid_center_ = cv::Vec2i(
            static_cast<int>(-map_->info.origin.position.y * pixels_per_meter),  // rows <--> y
            static_cast<int>(-map_->info.origin.position.x * pixels_per_meter)); // cols <--> x
    cv::Mat static_occupancy_grid_ = cv::Mat(info.height,
            info.width, CV_8UC1, (void*) map_->data.data()); // just a thin wrapper around map_->data.data()

    cv::threshold(static_occupancy_grid_, static_occupancy_grid_ /* in-place */, 1, 255, cv::THRESH_BINARY);
    cv::bitwise_not(static_occupancy_grid_, static_occupancy_grid_);

    // Publish static occupancy grid (for RViz)
    static_occupancy_grid_publisher_->publish(convertToGridCellsMessage(
            static_occupancy_grid_, static_occupancy_grid_center_, FRAME_MAP));

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

inline bool MotionPlanner::isGridCellOccupied(int row, int col) const {
    return(dynamic_occupancy_grid_.at<uint8_t>(row,col) == GRID_CELL_IS_OCCUPIED);
}

void MotionPlanner::expandOccupancyGrid(cv::Mat& grid) {
    const float pixels_per_meter = 1.0 / map_->info.resolution;
    const float vehicle_width_in_pixels = vehicle_width_ * pixels_per_meter;
    unsigned int structuring_element_width = static_cast<unsigned int>(std::ceil(vehicle_width_in_pixels)) + 1;
    if (structuring_element_width % 2 == 0) {
        structuring_element_width++;
    }
    if(GRID_CELL_IS_FREE < GRID_CELL_IS_OCCUPIED) {
        cv::dilate(grid, grid /* in-place */ , cv::getStructuringElement(
                cv::MORPH_ELLIPSE, cv::Size(structuring_element_width, structuring_element_width)));
    } else {
        cv::erode(grid, grid /* in-place */ , cv::getStructuringElement(
                cv::MORPH_ELLIPSE, cv::Size(structuring_element_width, structuring_element_width)));
    }
}

nav_msgs::GridCells MotionPlanner::convertToGridCellsMessage(
        cv::Mat& grid, const cv::Vec2i grid_center, const std::string frame_id) const {

    nav_msgs::GridCells grid_msg;
    grid_msg.header.stamp = ros::Time::now();
    grid_msg.header.frame_id = frame_id;

    const float meters_per_pixel = map_->info.resolution;
    grid_msg.cell_height = meters_per_pixel;
    grid_msg.cell_width = meters_per_pixel;

    for(int row=0; row<grid.rows; row++) {
        for(int col=0; col<grid.cols; col++) {
            if(grid.at<uint8_t>(row,col) == GRID_CELL_IS_FREE) {
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

void MotionPlanner::runPathPlanner() {

    const bool use_rrt_star = true; // TODO: expose this as a ROS node parameter

    // NOTE: goal_proximity_threshold is used only for performance reasons, nothing else.
    // We don't want to repeatedly trigger the planner if we are already at (or near) the goal.
    constexpr float goal_proximity_threshold = 0.1; // units: meters // TODO: this is an arbitrary choice for testing.

    std::future<void> async_marker_publishing_task;

    ROS_INFO("Path planner is starting...");

    static unsigned int number_of_computations = 0; // for debugging only

    ros::Rate sampling_rate(1000); // 1000 hz (chosen to match the /odom update frequency)
    while (!ros::isShuttingDown()) {
        bool expected = true;
        if(should_plan_path_.compare_exchange_strong(expected, false)) {

            std::lock_guard<std::mutex> scoped_lock(occupancy_grid_mutex_);
            geometry_msgs::PoseStamped goal;
            nav_msgs::Odometry odometry;
            bool already_near_goal = false;
            {
                std::lock_guard<std::mutex> scoped_lock(odometry_mutex_);
                goal = current_goal_;
                odometry = current_odometry_;
            }

            if(goal.header.stamp.isZero()) {
                // no goal has been set
                continue;
            }

            const Eigen::Vector2f odom_to_goal(goal.pose.position.x - odometry.pose.pose.position.x,
                                               goal.pose.position.y - odometry.pose.pose.position.y);

            if(odom_to_goal.squaredNorm() < (goal_proximity_threshold * goal_proximity_threshold)) {
                already_near_goal = true;
            }

            if(!already_near_goal) {
                number_of_computations++;

                bool should_generate_marker_messages = true;

                // TODO: throttle marker message generation to some maximum frequency
                //       (by setting should_generate_marker_messages=false if some delta_t has not yet elapsed).

#if 0
                // EXAMPLE:

                if(number_of_computations % 20 != 0) {
                    // generate message every 20th computation
                    should_generate_marker_messages = false;
                    }
#endif

                if(should_generate_marker_messages &&
                   async_marker_publishing_task.valid() &&
                   async_marker_publishing_task.wait_for(
                           std::chrono::nanoseconds(0)) != std::future_status::ready) {
                    should_generate_marker_messages = false;
                }

                // TODO: Choose between RRT and alternative algorithms (RRT*, ...)

                auto [success, tree, path, marker_messages] =
                    use_rrt_star ?
                    runRRT<int, USE_L1_DISTANCE_METRIC, true > (odometry, goal, should_generate_marker_messages) :
                    runRRT<int, USE_L1_DISTANCE_METRIC, false> (odometry, goal, should_generate_marker_messages);

                if(!marker_messages.empty()) {
                    async_marker_publishing_task = std::async(
                            std::launch::async,
                            [](const std::vector<visualization_msgs::Marker>& marker_messages,
                                          boost::shared_ptr<ros::Publisher> publisher) {
                                for(auto message : marker_messages) {
                                    publisher->publish(message);
                                } }, marker_messages, rrt_visualization_publisher_);
                }
            }

        } else {
            sampling_rate.sleep();
        }
    }

    ROS_INFO("Path planner is shutting down...");
}

// This is a modified version of
// Bresenham's line algorithm
//
// Source: https://rosettacode.org/wiki/Bitmap/Bresenham%27s_line_algorithm#C

bool MotionPlanner::expandPath(
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
            if (isGridCellOccupied(x0 + sx,y0)) {
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

template<typename T, bool USE_L1_DISTANCE_METRIC, bool USE_RRT_STAR>
std::tuple<
        bool,
        motion_planner::Tree<T, USE_L1_DISTANCE_METRIC>,
        std::deque<Eigen::Vector2f>,
        std::vector<visualization_msgs::Marker> >
MotionPlanner::runRRT(const nav_msgs::Odometry& odometry,
                      const geometry_msgs::PoseStamped& goal,
                      bool generate_marker_messages) {

    auto distance_between_nodes = [](int row0, int col0, int row1, int col1) -> int {
        const Eigen::Vector2i v(row0 - row1, col0 - col1);
        return (USE_L1_DISTANCE_METRIC ? v.lpNorm<1>() : v.squaredNorm());
    };

#ifndef NDEBUG
    unsigned int number_of_relinked_nodes = 0; // for debugging only
#endif

    constexpr size_t kdtree_max_leaf_size = 10; // TODO: Is this a good choice?

#if !defined(NDEBUG)
    constexpr unsigned int maximum_rrt_samples = 2000;  // TODO: expose this as a ROS node parameter
#else
    constexpr unsigned int maximum_rrt_samples = 4000;  // TODO: expose this as a ROS node parameter
#endif

    constexpr int maximum_rrt_expansion_distance = 40; // TODO: expose this as a ROS node parameter
    constexpr float rrt_goal_proximity_threshold = 2;   // TODO: expose this as a ROS node parameter

    // RRT* only:
    constexpr unsigned int rrt_star_nn_k = 10; // size of the neighborhood for reconnection inspection
    constexpr bool USE_INFORMED_RRT_STAR = true; // TODO: expose this as a ROS node parameter

    assert(maximum_rrt_samples > 0);
    assert(maximum_rrt_expansion_distance > 1);
    assert(rrt_goal_proximity_threshold > 0);

    using Tree = typename motion_planner::Tree<T, USE_L1_DISTANCE_METRIC>;
    using Node = typename Tree::Node;
    using KDTree = typename Tree::KDTree;
    using KNNResultSet = typename Tree::KNNResultSet;
    using UniformDistribution = typename Tree::UniformDistribution;
    Tree tree;
    KDTree index(2, tree, nanoflann::KDTreeSingleIndexAdaptorParams(kdtree_max_leaf_size));

    bool path_to_goal_found = false;
    int length_of_best_path_to_goal = std::numeric_limits<int>::max();

    // Goal
    assert(goal.header.frame_id.compare(FRAME_MAP) == 0); // we expect a goal in the "map" frame

    const auto goal_in_grid_frame = T_dynamic_oc_pixels_to_map_frame_.inverse() *
            Eigen::Vector3f(goal.pose.position.x, goal.pose.position.y, 0.0f);
    const T goal_row = goal_in_grid_frame(0);
    const T goal_col = goal_in_grid_frame(1);

    // check if goal lies outside grid bounds
    if(goal_row < 0 ||
       goal_row >= dynamic_occupancy_grid_.rows ||
       goal_col < 0 ||
       goal_col >= dynamic_occupancy_grid_.cols) {

        // goal lies outside grid bounds

        // TODO: Is there a better way to handle such a case?
        std::deque<Eigen::Vector2f> path;
        std::vector<visualization_msgs::Marker> marker_messages;
        return {false, tree, path, marker_messages};
    }

    // Generate a uniform distribution across the entire grid (default for RRT, RRT* and
    // Informed-RRT* prior to finding a path to the goal).
    UniformDistribution uniform_row_distribution(static_cast<T>(0), static_cast<T>(dynamic_occupancy_grid_.rows - 1));
    UniformDistribution uniform_col_distribution(static_cast<T>(0), static_cast<T>(dynamic_occupancy_grid_.cols - 1));

    // Root node is the origin in the laser frame (= center of the dynamic occupancy grid)
    const T root_row = dynamic_occupancy_grid_center_(0);
    const T root_col = dynamic_occupancy_grid_center_(1);

    // For Informed-RRT* only:
    // Generate a uniform distribution across the best-fit (**) bounding rectangle (*) around the
    // ellipsoidal heuristic sampling domain described in [Gammell et al., 2014].
    //
    // (*)  this is our original contribution
    // (**) in the hyperellipsoid-aligned frame

    int heuristic_sampling_domain_major_axis_length = 0; // unknown at this point (for Informed-RRT* only)
    int heuristic_sampling_domain_minor_axis_length = 0; // unknown at this point (for Informed-RRT* only)
    UniformDistribution distribution_along_major_axis; // unknown at this point (for Informed-RRT* only)
    UniformDistribution distribution_along_minor_axis; // unknown at this point (for Informed-RRT* only)

    const Eigen::Vector2f root_to_goal(goal_row - root_row, goal_col - root_col); // can be pre-computed

    // For Informed-RRT* only:
    // Compute the sampling domain transformation ("hyperellipsoid-aligned frame" --> "grid frame"),
    // consisting of a rotation by [theta] and a translation by [root_row, root_row].
    const float theta = std::atan2(root_to_goal(1), root_to_goal(0));
    Eigen::Affine3f T_sampling_domain_to_grid_frame = Eigen::Affine3f::Identity(); // unknown at this point (for Informed-RRT* only)

    // For Informed-RRT* only:
    // The following lambda-function returns a valid(*) sample from the heuristic sampling domain.
    // (*) guaranteed to lie inside the bounds of the grid frame
    auto sample_from_heuristic_sampling_domain = [&]() -> std::tuple<int,int> {
        Eigen::Vector3f sample_in_grid_frame;
        do {
            // (x,y) are coordinates in the "hyperellipsoid-aligned frame"
            const T x = distribution_along_major_axis(random_generator_);
            const T y = distribution_along_minor_axis(random_generator_);
            // transform coordinates from "hyperellipsoid-aligned frame" to grid frame
            sample_in_grid_frame = T_sampling_domain_to_grid_frame * Eigen::Vector3f(x, y, 0.0f);

        } while(sample_in_grid_frame(0) < 0 ||
                sample_in_grid_frame(0) >= dynamic_occupancy_grid_.rows ||
                sample_in_grid_frame(1) < 0 ||
                sample_in_grid_frame(1) >= dynamic_occupancy_grid_.cols);

        return {static_cast<T>(sample_in_grid_frame(0)),
                static_cast<T>(sample_in_grid_frame(1))};
    };

    auto sample_from_entire_grid = [&]() -> std::tuple<int,int> {
        return { uniform_row_distribution(random_generator_),
                 uniform_col_distribution(random_generator_) };
    };

    const float shortest_linear_path_to_goal = root_to_goal.norm(); // L2 norm
    const float upper_bound_L1_distance_to_goal = shortest_linear_path_to_goal * std::sqrt(2.0); // upper bound for the L1 norm
    const float lower_bound_L1_distance_to_goal = shortest_linear_path_to_goal; // lower bound for the L1 norm

    tree.nodes_.push_back(Node(root_row,root_col,0)); // insert the root node
    index.addPoints(tree.nodes_.size() - 1, tree.nodes_.size() - 1); // update kd-tree

    // =================================================================================================================
    // Sampling loop
    // =================================================================================================================
    while(tree.nodes_.size() < maximum_rrt_samples) {

        auto [random_row, random_col] = (USE_INFORMED_RRT_STAR && path_to_goal_found) ?
                                        sample_from_heuristic_sampling_domain():
                                        sample_from_entire_grid();

        if(isGridCellOccupied(random_row, random_col)) {
            // grid cell is occupied
            continue;
        }

        // Run a knn-search (k=1 for RRT, k>1 for RRT*)
        constexpr size_t k = USE_RRT_STAR ? rrt_star_nn_k : 1;
        size_t nn_indices[k];
        T nn_distances[k];
        KNNResultSet nn_results(k);
        nn_results.init(nn_indices, nn_distances);
        T query_position[2] = { random_row, random_col };
        const bool neighbors_found = index.findNeighbors(nn_results, query_position, nanoflann::SearchParams(10));

        size_t nn_index = nn_indices[0]; // for RRT (k=1)
        T nn_distance = nn_distances[0]; // for RRT (k=1)

        // RRT* only: inspect the k-neighborhood, find minimum-cost link
        if(neighbors_found && USE_RRT_STAR) {

            std::vector<Node*> k_neighborhood;
            for(unsigned int i=0; i<k; i++) {
                const size_t j = nn_indices[i];
                assert(j < tree.nodes_.size());
                k_neighborhood.push_back(&tree.nodes_[j]);
            }
            const unsigned int argmin_cost = std::distance(k_neighborhood.begin(), std::min_element(
                    k_neighborhood.begin(), k_neighborhood.end(), [](Node* node0, Node* node1) {
                        return (node0->path_length_ < node1->path_length_); }));

            nn_index = nn_indices[argmin_cost];
            nn_distance = nn_distances[argmin_cost];
        }

        // The nearest node is our new parent
        const size_t parent_node_index = nn_index;
        const Node& parent_node = tree.nodes_[parent_node_index];
        const T parent_row = static_cast<T>(parent_node.position_(0));
        const T parent_col = static_cast<T>(parent_node.position_(1));

        assert(not isGridCellOccupied(parent_row, parent_col));

        // Expand the path from the parent to the leaf, check if any obstacles are in the way...
        cv::Vec2i leaf_position(0,0);
        const bool expansion_has_no_obstacles =
                expandPath(cv::Vec2i(parent_row, parent_col),
                           cv::Vec2i(random_row, random_col),
                           leaf_position,
                           maximum_rrt_expansion_distance);

        if(!expansion_has_no_obstacles) {
            continue;
        }

        const T leaf_row = leaf_position(0);
        const T leaf_col = leaf_position(1);

        assert(not isGridCellOccupied(leaf_row, leaf_col));

        const auto parent_to_leaf_distance = distance_between_nodes(parent_row, parent_col, leaf_row, leaf_col);
        const auto accumulated_path_length_to_leaf = parent_node.path_length_ + parent_to_leaf_distance;

        tree.nodes_.push_back(Node(leaf_row, leaf_col, parent_node_index, accumulated_path_length_to_leaf)); // insert new leaf
        const Node& leaf_node = tree.nodes_.back();
        const size_t leaf_node_index = tree.nodes_.size() - 1;

        if(neighbors_found && USE_RRT_STAR) {

            for(unsigned int i=0; i<k; i++) {
                const size_t j = nn_indices[i];
                if(j==nn_index) {
                    continue; // we are only interested in the (k-1)-neighborhood
                }
                assert(j < tree.nodes_.size());
                Node& node = tree.nodes_[j];

                const auto node_to_leaf_distance = distance_between_nodes(
                        node.position_(0), node.position_(1), leaf_row, leaf_col);

                if(accumulated_path_length_to_leaf + node_to_leaf_distance < node.path_length_) {
                    // re-link node to new parent (which is our leaf)

                    // Check if obstacles are in the way...
                    cv::Vec2i dummy_position;
                    const bool expansion_has_no_obstacles =
                            expandPath(cv::Vec2i(leaf_row, leaf_col),
                                       cv::Vec2i(node.position_(0), node.position_(1)),
                                       dummy_position,
                                       std::numeric_limits<int>::max());

                    if(!expansion_has_no_obstacles) {
                        continue;
                    }

                    node.path_length_ = accumulated_path_length_to_leaf + node_to_leaf_distance;
                    node.parent_ = leaf_node_index;

                    #ifndef NDEBUG
                    number_of_relinked_nodes++; // for debugging only
                    #endif
                }
            }
        }
        index.addPoints(tree.nodes_.size() - 1, tree.nodes_.size() - 1); // update kd-tree

        // Check if we are within reach of the goal
        const auto leaf_to_goal_distance = distance_between_nodes(
                goal_row, goal_col, leaf_node.position_(0), leaf_node.position_(1));

        if(leaf_to_goal_distance <= rrt_goal_proximity_threshold) {
            path_to_goal_found = true;
            const int length_of_new_path_to_goal = leaf_node.path_length_ + leaf_to_goal_distance;
            length_of_best_path_to_goal = std::min<int>(length_of_best_path_to_goal, length_of_new_path_to_goal);

            if(USE_INFORMED_RRT_STAR) {
                // Uniform distribution across the best-fit bounding rectangle (!) of the elliptical heuristic
                // sampling domain described in [Gammell et al., 2014].

                // In the paper: sqrt(c_{best}^2 - c_{min}^2), where ...
                // c_{min}  ... linear shortest distance to goal (theoretical minimum cost)
                // c_{best} ... accumulated length of best path to goal (current best cost)

                // NOTE: If we are using the L1 distance metric, this is obviously just an
                //       approximation. Since the resulting L1 minor_axis_length
                //       is an upper bound for the L2 minor_axis_length, this is ok.

                const float accumulated_length_of_best_path_to_goal = static_cast<float>(length_of_best_path_to_goal);

                // Use the variable naming scheme from [Gammell et al., 2014]...
                const float& c_min = USE_L1_DISTANCE_METRIC ? lower_bound_L1_distance_to_goal : shortest_linear_path_to_goal;
                const float& c_best = accumulated_length_of_best_path_to_goal;
                heuristic_sampling_domain_major_axis_length = static_cast<int>(c_best);
                heuristic_sampling_domain_minor_axis_length = static_cast<int>(std::sqrt((c_best * c_best) - (c_min * c_min)));
                distribution_along_major_axis = UniformDistribution(static_cast<T>(0), static_cast<T>(heuristic_sampling_domain_major_axis_length));
                distribution_along_minor_axis = UniformDistribution(static_cast<T>(0), static_cast<T>(heuristic_sampling_domain_minor_axis_length));

                T_sampling_domain_to_grid_frame = Eigen::Affine3f::Identity();
                T_sampling_domain_to_grid_frame.pretranslate(Eigen::Vector3f(0, -static_cast<float>(heuristic_sampling_domain_minor_axis_length) / 2.0, 0.0f));
                T_sampling_domain_to_grid_frame.prerotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));
                T_sampling_domain_to_grid_frame.pretranslate(Eigen::Vector3f(root_row, root_col, 0.0f));
            }
        }
    }

    // NOTE: If we did not found a path to the goal (i.e. path_to_goal_found==false), there are two possibilities:
    //       Return with an error, or choose the closest leaf in our tree and construct the path backwards from that.
    //       Let's go with the second option (which is what we would do anyways if path_to_goal_found==true).

    // Trace the path back from the goal to the parent node

    std::deque<Eigen::Vector2f> path; // path in the "map" coordinate frame
    std::deque<Node> nodes_on_path;   // nodes on path (in the grid coordinate frame)

    // Run a knn-search (k=1) to find the closest node to our goal.
    const size_t k = 1;
    size_t nn_index;
    T nn_distance;
    KNNResultSet nn_results(k);
    nn_results.init(&nn_index, &nn_distance);
    T query_position[2] = { goal_row, goal_col };
    index.findNeighbors(nn_results, query_position, nanoflann::SearchParams(10));
    assert(nn_index < tree.nodes_.size());

    const auto closest_leaf_to_goal_distance = distance_between_nodes(
            goal_row, goal_col, tree.nodes_[nn_index].position_(0), tree.nodes_[nn_index].position_(1));

    const auto accumulated_path_length_to_goal =
            tree.nodes_[nn_index].path_length_ + closest_leaf_to_goal_distance;

    nodes_on_path.push_front(Node(goal_row, goal_col, nn_index, accumulated_path_length_to_goal)); // insert the goal as the last node in the path

    while(true) {
        const size_t parent_node_index = nodes_on_path.front().parent_;
        nodes_on_path.push_front(tree.nodes_[parent_node_index]);

        const T node_row = nodes_on_path.front().position_(0);
        const T node_col = nodes_on_path.front().position_(1);

        const auto node_in_map_frame =
                T_dynamic_oc_pixels_to_map_frame_ *
                Eigen::Vector3f(node_row, node_col, 0.0f);

        path.push_front(Eigen::Vector2f(node_in_map_frame(0), node_in_map_frame(1)));
        if(parent_node_index==0) {
            break;
        }
    }

    int marker_message_id = 0;
    std::vector<visualization_msgs::Marker> marker_messages;

#define SHOW_FULL_TREE
#if defined(SHOW_FULL_TREE)
    if(generate_marker_messages) {

        visualization_msgs::Marker marker;
        marker.header.frame_id = FRAME_MAP;
        marker.header.stamp = ros::Time::now();

        // Set the namespace and id for this marker. This serves to create a unique ID.
        // Any marker sent with the same namespace and id will overwrite the old one.
        marker.ns = "rrt";
        marker.id = marker_message_id++;

        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 1.0 / 100.0f;
        marker.scale.y = 0.0; // must be zero for LINE_LIST
        marker.scale.z = 0.0; // must be zero for LINE_LIST

        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 0.33f;

        marker.lifetime = ros::Duration();

        for(unsigned int j=0; j<tree.nodes_.size(); j++) {

            const size_t i = tree.nodes_[j].parent_;
            if(i==0) {
                continue;
            }

            const auto &vertex0 = tree.nodes_[i].position_;
            const auto &vertex1 = tree.nodes_[j].position_;

            const auto vertex0_in_map_frame =
                    T_dynamic_oc_pixels_to_map_frame_ *
                    Eigen::Vector3f(vertex0(0), vertex0(1), 0.0f);

            const auto vertex1_in_map_frame =
                    T_dynamic_oc_pixels_to_map_frame_ *
                    Eigen::Vector3f(vertex1(0), vertex1(1), 0.0f);

            geometry_msgs::Point p0;
            p0.x = vertex0_in_map_frame(0);
            p0.y = vertex0_in_map_frame(1);
            p0.z = 0.0;

            geometry_msgs::Point p1;
            p1.x = vertex1_in_map_frame(0);
            p1.y = vertex1_in_map_frame(1);
            p1.z = 0.0;

            marker.points.push_back(p0);
            marker.points.push_back(p1);
        }

        marker_messages.push_back(marker);
    }
#endif

#define SHOW_HEURISTIC_SAMPLING_DOMAIN_BOUNDS
#if defined(SHOW_HEURISTIC_SAMPLING_DOMAIN_BOUNDS)
    if(generate_marker_messages) {

        visualization_msgs::Marker marker;
        marker.header.frame_id = FRAME_MAP;
        marker.header.stamp = ros::Time::now();
        marker.ns = "rrt";
        marker.id = marker_message_id++;

        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 1.0 / 100.0f;
        marker.scale.y = 0.0; // must be zero for LINE_LIST
        marker.scale.z = 0.0; // must be zero for LINE_LIST

        marker.color.r = 255.0f / 255.0f;
        marker.color.g = 0.0f;
        marker.color.b = 200.0f / 255.0f;
        marker.color.a = 1.0;

        marker.lifetime = ros::Duration();

        std::vector<Eigen::Vector3f> vertices;

        // vertices are initially in the "hyperellipsoid aligned frame".
        vertices.push_back(Eigen::Vector3f(0.0f, 0.0f, 0.0f));
        vertices.push_back(Eigen::Vector3f(heuristic_sampling_domain_major_axis_length, 0.0f, 0.0f));
        vertices.push_back(Eigen::Vector3f(heuristic_sampling_domain_major_axis_length, heuristic_sampling_domain_minor_axis_length, 0.0f));
        vertices.push_back(Eigen::Vector3f(0.0f, heuristic_sampling_domain_minor_axis_length, 0.0f));

        std::vector<geometry_msgs::Point> points;

        for(auto &vertex : vertices) {

            // transform from "hyperellipsoid aligned frame" to "grid frame".
            const auto p_in_grid_frame = T_sampling_domain_to_grid_frame * vertex;

            // transform from "grid frame" to "map frame".
            const auto p_in_map_frame = T_dynamic_oc_pixels_to_map_frame_ * p_in_grid_frame;
            geometry_msgs::Point p;
            p.x = p_in_map_frame(0);
            p.y = p_in_map_frame(1);
            p.z = 0.0f;
            points.push_back(p);
        }

        for(int i=0; i<points.size(); i++) {
            marker.points.push_back(points[i]);
            marker.points.push_back(points[(i+1) % points.size()]);
        }
        marker_messages.push_back(marker);
    }
#endif

#define SHOW_OCCUPANCY_GRID_OUTLINE
#if defined(SHOW_OCCUPANCY_GRID_OUTLINE)
    if(generate_marker_messages) {

        visualization_msgs::Marker marker;
        marker.header.frame_id = FRAME_MAP;
        marker.header.stamp = ros::Time::now();
        marker.ns = "rrt";
        marker.id = marker_message_id++;

        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 1.0 / 100.0f;
        marker.scale.y = 0.0; // must be zero for LINE_LIST
        marker.scale.z = 0.0; // must be zero for LINE_LIST

        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 1.0f;
        marker.color.a = 1.0;

        marker.lifetime = ros::Duration();

        // vertices are initially in the "grid frame".
        std::vector<Eigen::Vector3f> vertices;

        vertices.push_back(Eigen::Vector3f(0.0f, 0.0f, 0.0f));
        vertices.push_back(Eigen::Vector3f(dynamic_occupancy_grid_.rows-1, 0.0f, 0.0f));
        vertices.push_back(Eigen::Vector3f(dynamic_occupancy_grid_.rows-1, dynamic_occupancy_grid_.cols-1, 0.0f));
        vertices.push_back(Eigen::Vector3f(0.0f, dynamic_occupancy_grid_.cols-1, 0.0f));

        std::vector<geometry_msgs::Point> points;

        for(auto &vertex : vertices) {

            // transform from "grid frame" to "map frame".
            const auto p_in_map_frame = T_dynamic_oc_pixels_to_map_frame_ * vertex;
            geometry_msgs::Point p;
            p.x = p_in_map_frame(0);
            p.y = p_in_map_frame(1);
            p.z = 0.0f;
            points.push_back(p);
        }

        for(int i=0; i<points.size(); i++) {
            marker.points.push_back(points[i]);
            marker.points.push_back(points[(i+1) % points.size()]);
        }
        marker_messages.push_back(marker);
    }
#endif

#define SHOW_PATH_TO_GOAL
#if defined(SHOW_PATH_TO_GOAL)
    if(generate_marker_messages) {

        visualization_msgs::Marker marker;
        marker.header.frame_id = FRAME_MAP;
        marker.header.stamp = ros::Time::now();
        marker.ns = "rrt";
        marker.id = marker_message_id++;

        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 4.0 / 100.0f;
        marker.scale.y = 0.0; // must be zero for LINE_LIST
        marker.scale.z = 0.0; // must be zero for LINE_LIST

        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;

        marker.lifetime = ros::Duration();

        assert(path.size() > 1); // must include at least the root and the goal
        for (unsigned int j = 1; j < path.size(); j++) {

            const auto &v0_in_map_frame = path[j-1];
            const auto &v1_in_map_frame = path[j];

            geometry_msgs::Point p0;
            p0.x = v0_in_map_frame(0); p0.y = v0_in_map_frame(1); p0.z = 0.0;
            marker.points.push_back(p0);

            geometry_msgs::Point p1;
            p1.x = v1_in_map_frame(0); p1.y = v1_in_map_frame(1); p1.z = 0.0;
            marker.points.push_back(p1);
        }
        marker_messages.push_back(marker);
    }
#endif

//#define SHOW_BLOCKED_GRID_CELLS // Don't enable this, except for debugging
                                  // (nav_msgs::GridCells does the same job, but is more efficient)

#if defined(SHOW_BLOCKED_GRID_CELLS)
    if(generate_marker_messages) {

        visualization_msgs::Marker marker;
        marker.header.frame_id = FRAME_MAP;
        marker.header.stamp = ros::Time::now();
        marker.ns = "rrt";
        marker.id = marker_message_id++;

        marker.type = visualization_msgs::Marker::SPHERE_LIST;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = map_->info.resolution;
        marker.scale.y = map_->info.resolution;
        marker.scale.z = map_->info.resolution;

        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;

        marker.lifetime = ros::Duration();

        for(int row=0; row<dynamic_occupancy_grid_.rows; row++) {
            for(int col=0; col<dynamic_occupancy_grid_.cols; col++) {
                if(isGridCellOccupied(row, col)) {
                    const auto point_in_map_frame =
                            T_dynamic_oc_pixels_to_map_frame_ *
                            Eigen::Vector3f(row, col, 0.0f);

                    geometry_msgs::Point p;
                    p.x = point_in_map_frame(0);
                    p.y = point_in_map_frame(1);
                    p.z = 0.0;

                    marker.points.push_back(p);
                }
            }
        }
        marker_messages.push_back(marker);
    }
#endif

#define SHOW_GOAL
#if defined(SHOW_GOAL)
    if(generate_marker_messages) {

        visualization_msgs::Marker marker;
        marker.header.frame_id = FRAME_MAP;
        marker.header.stamp = ros::Time::now();
        marker.ns = "rrt";
        marker.id = marker_message_id++;

        marker.type = visualization_msgs::Marker::SPHERE_LIST;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;

        marker.color.r = 1.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;

        marker.lifetime = ros::Duration();

        geometry_msgs::Point p;
        p.x = goal.pose.position.x;
        p.y = goal.pose.position.y;
        p.z = 0.0;

        marker.points.push_back(p);
        marker_messages.push_back(marker);
    }
#endif

#if 0 // Don't enable this, except for debugging...

    std::cout << "PATH (GRID): ";
    for(unsigned int j=0; j<nodes_on_path.size(); j++) {
        if(j>0) {
            std::cout << " --> ";
        }
        std::cout << "[" << nodes_on_path[j].position_(0) << ", " << nodes_on_path[j].position_(1) << "]";
    }
    std::cout << std::endl;

    std::cout << "PATH (MAP): ";
    for(unsigned int j=0; j<path.size(); j++) {
        if(j>0) {
            std::cout << " --> ";
        }
        std::cout << "[" << path[j](0) << ", " << path[j](1) << "]";
    }
    std::cout << std::endl;

#endif

    return {true, tree, path, marker_messages};
}

int main(int argc, char **argv) {
#if !defined(NDEBUG)
    std::cerr << "WARNING: Debug build." << std::endl;
#endif
    ros::init(argc, argv, "motion_planner");
    boost::shared_ptr<MotionPlanner> motion_planner(new MotionPlanner());
    ros::spin();
    motion_planner.reset();
    return 0;
}
