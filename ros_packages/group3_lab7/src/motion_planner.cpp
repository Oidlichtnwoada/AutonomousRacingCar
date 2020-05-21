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

#define LIMIT_DYNAMIC_OCCUPANCY_GRID_TO_FORWARD_DIRECTION

#define SUBSCRIBER_MESSAGE_QUEUE_SIZE 1000
#define GRID_PUBLISHER_MESSAGE_QUEUE_SIZE 10
#define MARKER_PUBLISHER_MESSAGE_QUEUE_SIZE 10

#define TOPIC_SCAN "/scan"
#define TOPIC_ODOM "/odom"
#define TOPIC_GOAL "/move_base_simple/goal"
#define TOPIC_MAP  "/map"
#define TOPIC_DYNAMIC_OCCUPANCY_GRID "/motion_planner/dynamic_occupancy_grid"
#define TOPIC_STATIC_OCCUPANCY_GRID  "/motion_planner/static_occupancy_grid"
#define TOPIC_RRT_VISUALIZATION      "/motion_planner/rrt_visualization"
#define FRAME_MAP  "map"

#define DEFAULT_VEHICLE_WHEELBASE 0.3302 // units: m (see f110_simulator/params.yaml)
#define DEFAULT_VEHICLE_WIDTH     0.2032 // units: m (see f110_simulator/params.yaml)
#define DEFAULT_MAX_LASER_SCAN_DISTANCE 5.0 // units: m
#define DEFAULT_DYNAMIC_OCCUPANCY_GRID_UPDATE_FREQUENCY 20 // units: Hz

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

    // Map
    MotionPlanner::OccupancyGridMessage map_;

    std::mutex occupancy_grid_mutex_;
    cv::Mat static_occupancy_grid_; // just a thin wrapper around map_->data.data()
    cv::Mat dynamic_occupancy_grid_;
    cv::Vec2i static_occupancy_grid_center_;
    cv::Vec2i dynamic_occupancy_grid_center_;

    // Affine transformations from dynamic_occupancy_grid pixel coordinates [u,v] to other frames
    //
    // Eigen::Vector3f v_laser = T_dynamic_oc_pixels_to_laser_frame_ * Eigen::Vector3f(u, v, 0);
    // [v_laser(0),  v_laser(1)] are the cartesian coordinates in the laser frame (units: meters)

    Eigen::Affine3f T_dynamic_oc_pixels_to_laser_frame_;

    // Eigen::Vector3f v_map = T_dynamic_oc_pixels_to_map_frame_ * Eigen::Vector3f(u, v, 0);
    // [v_map(0),  v_map(1)] are the cartesian coordinates in the map frame (units: meters)

    Eigen::Affine3f T_dynamic_oc_pixels_to_map_frame_;

    // Eigen::Vector3f v_static_oc = T_dynamic_oc_pixels_to_static_oc_pixels_ * Eigen::Vector3f(u, v, 0);
    // [v_static_oc(0),  v_static_oc(1)] are the pixel coordinates [u',v'] in the static occupancy grid

    Eigen::Affine3f T_dynamic_oc_pixels_to_static_oc_pixels_;

    void dilateOccupancyGrid(cv::Mat& grid);

    nav_msgs::GridCells convertToGridCellsMessage(
            cv::Mat& grid, const cv::Vec2i grid_center, const std::string frame_id) const;

    // =================================================================================================================
    // RRT
    // =================================================================================================================

    boost::shared_ptr<std::thread> path_planner_thread_;
    void runPathPlanner();

    template<typename T, bool USE_L1_DISTANCE_METRIC>
    std::tuple<motion_planner::Tree<T, USE_L1_DISTANCE_METRIC>, std::deque<Eigen::Vector2f> >
    runRRT(const nav_msgs::Odometry& odometry,
           const geometry_msgs::PoseStamped& goal,
           visualization_msgs::Marker* vis_msg = nullptr);

    std::mutex odometry_mutex_;
    geometry_msgs::PoseStamped current_goal_;
    nav_msgs::Odometry current_odometry_;

    std::atomic<bool> should_plan_path_;

    std::mt19937 random_generator_;
    static constexpr std::mt19937::result_type random_seed_ = 1234UL; // fixed seed for reproducibility
    //std::uniform_real_distribution<float> x_distribution_;
    //std::uniform_real_distribution<float> y_distribution_;



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

    // Trigger with "rosservice call /debug"
    std::cout << "MotionPlanner::debugServiceCallback()" << std::endl;
    cv::imwrite("/tmp/static_occupancy_grid.png", static_occupancy_grid_);
    cv::imwrite("/tmp/dynamic_occupancy_grid.png", dynamic_occupancy_grid_);
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

    T_dynamic_oc_pixels_to_laser_frame_ = A.matrix();

    A = T_laser_to_map.matrix() * A.matrix();

    T_dynamic_oc_pixels_to_map_frame_ = A.matrix();

    A.prescale(pixels_per_meter); // meters to pixels
    A.pretranslate(Eigen::Vector3f((float) static_occupancy_grid_center_(0),
                                   (float) static_occupancy_grid_center_(1),
                                   0.0f));

    T_dynamic_oc_pixels_to_static_oc_pixels_ = A.matrix();

    // =================================================================================================================

    dynamic_occupancy_grid_.setTo(cv::Scalar(0.0)); // clear the grid

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
            assert(x > 0.0);
#endif
            const float y = range * std::sin(angle); // units: meters

            // [u,v] are the pixel coordinates in the dynamic occupancy grid
            // u ... row    (y in the laser frame)
            // v ... column (x in the laser frame)
            const int u = static_cast<int>(y * pixels_per_meter) + dynamic_occupancy_grid_center_(0);
            const int v = static_cast<int>(x * pixels_per_meter) + dynamic_occupancy_grid_center_(1);

            const int center_row = dynamic_occupancy_grid_center_(0);
            const int center_col = dynamic_occupancy_grid_center_(1);
            const int u_max = dynamic_occupancy_grid_.rows-1;
            const int v_max = dynamic_occupancy_grid_.cols-1;

            assert(u >= 0 && v >= 0 && u < dynamic_occupancy_grid_.rows && v < dynamic_occupancy_grid_.cols);
            dynamic_occupancy_grid_.at<uint8_t>(u,v) = 255;
        }
    }

    // Dilate the dynamic occupancy grid
    dilateOccupancyGrid(dynamic_occupancy_grid_);
    dynamic_occupancy_grid_publisher_->publish(convertToGridCellsMessage(
            dynamic_occupancy_grid_, dynamic_occupancy_grid_center_, "laser"));

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

    if(!static_occupancy_grid_.empty()) {
        ROS_ERROR("Overwriting the occupancy grid is untested and may lead to undefined behavior.");
        //throw(std::runtime_error("Overwriting the occupancy grid is untested and may lead to undefined behavior."));
    }

    map_ = map_msg;
    const nav_msgs::MapMetaData& info = map_msg->info;

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

    // Create static occupancy grid
    const float pixels_per_meter = 1.0f / map_->info.resolution;
    static_occupancy_grid_center_ = cv::Vec2i(
            static_cast<int>(-map_->info.origin.position.y * pixels_per_meter),  // rows <--> y
            static_cast<int>(-map_->info.origin.position.x * pixels_per_meter)); // cols <--> x
    static_occupancy_grid_ = cv::Mat(info.height, info.width, CV_8UC1, (void*) map_->data.data());

    // Binarize static occupancy grid (unoccupied: 0, occupied: 255).
    cv::threshold(static_occupancy_grid_, static_occupancy_grid_ /* in-place */, 1.0, 255.0, cv::THRESH_BINARY);

    // Dilate static occupancy grid
    dilateOccupancyGrid(static_occupancy_grid_);

    // Publish static occupancy grid (for RViz)
    static_occupancy_grid_publisher_->publish(convertToGridCellsMessage(
            static_occupancy_grid_, static_occupancy_grid_center_, "map"));

    // Create dynamic occupancy grid
    const float laser_scan_diameter_in_pixels = 2.0f * max_laser_scan_distance_ * pixels_per_meter;
    unsigned int dynamic_occupancy_grid_cols = static_cast<unsigned int>(std::ceil(laser_scan_diameter_in_pixels)) + 2;
    if (dynamic_occupancy_grid_cols % 2 == 0) {
        dynamic_occupancy_grid_cols++;
    }

    // Coordinate system:
    // +x (forward) <--> laser frame row <--> grid col
    // +y (left)    <--> laser frame col <--> grid row

    const int center_col = (dynamic_occupancy_grid_cols - 1) / 2;

#ifdef LIMIT_DYNAMIC_OCCUPANCY_GRID_TO_FORWARD_DIRECTION
    const int center_row = 0;
    const unsigned int dynamic_occupancy_grid_rows = ((dynamic_occupancy_grid_cols- 1 ) / 2) + 1;
#else
    const int center_row = center_col;
    const unsigned int dynamic_occupancy_grid_rows = dynamic_occupancy_grid_cols;
#endif
    dynamic_occupancy_grid_center_ = cv::Vec2i(center_col, center_row); // NOTE: grid is transposed w.r.t. vehicle's local coordinate system
    const cv::Scalar unoccupied_cell(0.0);
    dynamic_occupancy_grid_ = cv::Mat(dynamic_occupancy_grid_cols, dynamic_occupancy_grid_rows, CV_8UC1, unoccupied_cell);
    should_plan_path_.store(true);
}

void MotionPlanner::dilateOccupancyGrid(cv::Mat& occupancy_grid) {
    const float pixels_per_meter = 1.0 / map_->info.resolution;
    const float vehicle_width_in_pixels = vehicle_width_ * pixels_per_meter;
    unsigned int structuring_element_width = static_cast<unsigned int>(std::ceil(vehicle_width_in_pixels)) + 1;
    if (structuring_element_width % 2 == 0) {
        structuring_element_width++;
    }
    cv::dilate(occupancy_grid, occupancy_grid /* in-place */ , cv::getStructuringElement(
            cv::MORPH_ELLIPSE, cv::Size(structuring_element_width, structuring_element_width)));
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
            if(grid.at<uint8_t>(row,col) == 0) {
                continue;
            }

            // Coordinate system:
            // +x (forward) <--> laser frame row <--> grid col
            // +y (left)    <--> laser frame col <--> grid row

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

    constexpr bool USE_L1_DISTANCE_METRIC = true;
    constexpr float goal_proximity_threshold = 0.2; // units: meters // TODO: this is an arbitrary choice for testing...

    ROS_INFO("Path planner is starting...");

    static unsigned int number_of_computations = 0; // for debugging only

    ros::Rate sampling_rate(1000); // 1000 hz
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
                // TODO: Choose between RRT and alternative algorithms (RRT*, ...) here...
                auto [tree, path] = runRRT<int, USE_L1_DISTANCE_METRIC>(odometry, goal);
            }

        } else {
            sampling_rate.sleep();
        }
    }

    ROS_INFO("Path planner is shutting down...");
}

bool ExpandPath(
        const cv::Vec2i start,
        const cv::Vec2i destination,
        cv::Vec2i& end,
        float max_expansion_distance,
        const cv::Mat& grid) {

    int x0 = start(0);
    int y0 = start(1);
    int x1 = destination(0);
    int y1 = destination(1);
    int expansion_distance = 0;

    // Bresenham algorithm
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy, e2; /* error value e_xy */
    while (true) {
        if (expansion_distance >= max_expansion_distance) {
            end = cv::Vec2i(x0,y0);
            return true;
        }
        if (grid.at<unsigned>(x0, y0) > 0) {
            end = cv::Vec2i(x0,y0);
            return false;
        }
        if (x0 == x1 && y0 == y1) break;
        e2 = 2 * err;
        if (e2 > dy) {
            err += dy;
            x0 += sx;
            expansion_distance += sx;
        } /* e_xy+e_x > 0 */
        if (e2 < dx) {
            err += dx;
            y0 += sy;
            expansion_distance += sy;
        } /* e_xy+e_y < 0 */
    }
    end = cv::Vec2i(x0,y0);
    return true;
}

template<typename T, bool USE_L1_DISTANCE_METRIC>
std::tuple<motion_planner::Tree<T, USE_L1_DISTANCE_METRIC>, std::deque<Eigen::Vector2f> >
MotionPlanner::runRRT(const nav_msgs::Odometry& odometry,
                      const geometry_msgs::PoseStamped& goal,
                      visualization_msgs::Marker* vis_msg) {

    constexpr size_t kdtree_max_leaf_size = 10; // TODO: Is this a good choice?

    constexpr unsigned int maximum_rrt_samples = 2000;  // TODO: expose this as a ROS node parameter
    constexpr float maximum_rrt_expansion_distance = 2; // TODO: expose this as a ROS node parameter
    constexpr float rrt_goal_proximity_threshold = 2;   // TODO: expose this as a ROS node parameter

    using Tree = typename motion_planner::Tree<T, USE_L1_DISTANCE_METRIC>;
    using Node = typename Tree::Node;
    using KDTree = typename Tree::KDTree;
    using KNNResultSet = typename Tree::KNNResultSet;
    using UniformDistribution = typename Tree::UniformDistribution;
    Tree tree;
    KDTree index(2, tree, nanoflann::KDTreeSingleIndexAdaptorParams(kdtree_max_leaf_size));

    // Goal
    assert(goal.header.frame_id.compare("map") == 0); // we expect a goal in the "map" frame
    const auto goal_in_grid_frame = T_dynamic_oc_pixels_to_map_frame_.inverse() *
            Eigen::Vector3f(goal.pose.position.x, goal.pose.position.y, 0.0f);
    const T goal_row = goal_in_grid_frame(0); // <---- CHECK ROW/COL!!!!!
    const T goal_col = goal_in_grid_frame(1); // <---- CHECK ROW/COL!!!!!

    // Randomly distribute samples across the entire grid
    UniformDistribution uniform_row_distribution(static_cast<T>(0), static_cast<T>(dynamic_occupancy_grid_.rows - 1));
    UniformDistribution uniform_col_distribution(static_cast<T>(0), static_cast<T>(dynamic_occupancy_grid_.cols - 1));

    // Root node is the origin in the laser frame (= center of the dynamic occupancy grid)
    const T root_row = dynamic_occupancy_grid_center_(0);
    const T root_col = dynamic_occupancy_grid_center_(1);
    tree.nodes_.push_back(Node(root_row,root_col,0)); // insert the root node
    index.addPoints(tree.nodes_.size() - 1, tree.nodes_.size() - 1); // update kd-tree

    for(unsigned int n=0; n<maximum_rrt_samples; n++) {
        const T random_row = uniform_row_distribution(random_generator_);
        const T random_col = uniform_col_distribution(random_generator_);

        // run a knn-search (k=1)
        const size_t k = 1;
        size_t nn_index;
        T nn_distance;
        KNNResultSet nn_results(k);
        nn_results.init(&nn_index, &nn_distance);
        T query_position[2] = { random_row, random_col };
        index.findNeighbors(nn_results, query_position, nanoflann::SearchParams(10));
        assert(nn_index < tree.nodes_.size());

        // The nearest node is our new parent
        const size_t parent_node_index = nn_index;
        const Node& parent_node = tree.nodes_[parent_node_index];
        const T parent_row = static_cast<T>(parent_node.position_(0));
        const T parent_col = static_cast<T>(parent_node.position_(1));

        cv::Vec2i leaf_position;
        const bool expansion_has_no_obstacles =
                ExpandPath(cv::Vec2i(parent_row, parent_col),
                           cv::Vec2i(random_row, random_col),
                           leaf_position,
                           maximum_rrt_expansion_distance,
                           dynamic_occupancy_grid_);

        if(expansion_has_no_obstacles) {
            const T leaf_row = leaf_position(0);
            const T leaf_col = leaf_position(1);
            tree.nodes_.push_back(Node(leaf_row, leaf_col, parent_node_index)); // insert new leaf
            index.addPoints(tree.nodes_.size() - 1, tree.nodes_.size() - 1); // update kd-tree
            const Node& leaf_node = tree.nodes_.back();

            // Check if we are within reach of the goal
            Eigen::Vector2i parent_to_leaf(goal_row - leaf_node.position_(0),
                                           goal_col - leaf_node.position_(1));
            const auto distance_from_goal =
                    USE_L1_DISTANCE_METRIC ?
                    parent_to_leaf.lpNorm<1>() :
                    parent_to_leaf.squaredNorm();

            if(distance_from_goal <= rrt_goal_proximity_threshold) {
                // we reached the goal
                break;
            }
        }
    }

    // NOTE: Here, we could re-check if we reached the goal (within some proximity threshold) or not.
    //       If not, there are two possibilities: Return with an error, or choose the closest leaf in our
    //       tree and construct the path backwards from that. Let's go with the second option...

    // Trace the path back from the goal to the parent node

    std::deque<Eigen::Vector2f> path; // path in the "map" coordinate frame
    std::deque<Node> nodes_on_path;   // nodes on path (in the grid coordinate frame)

    // Run a knn-search (k=1), to find the closest node to our goal
    const size_t k = 1;
    size_t nn_index;
    T nn_distance;
    KNNResultSet nn_results(k);
    nn_results.init(&nn_index, &nn_distance);
    T query_position[2] = { goal_row, goal_col };
    index.findNeighbors(nn_results, query_position, nanoflann::SearchParams(10));
    assert(nn_index < tree.nodes_.size());
    nodes_on_path.push_front(Node(goal_row, goal_col, nn_index)); // insert the goal as the last node in the path

    while(nodes_on_path.front().parent_ != 0) {
        const size_t parent_node_index = nodes_on_path.front().parent_;
        nodes_on_path.push_front(tree.nodes_[parent_node_index]);

        const T node_row = nodes_on_path.front().position_(0);
        const T node_col = nodes_on_path.front().position_(1);

        const auto node_in_map_frame =
                T_dynamic_oc_pixels_to_map_frame_ *
                Eigen::Vector3f(node_row, node_col, 0.0f);

        path.push_front(Eigen::Vector2f(node_in_map_frame(0), node_in_map_frame(1)));
    }

    if(vis_msg != nullptr) {
        // ...
    }


#if 0 // Debug checks...

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

    return {tree, path};
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
