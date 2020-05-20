/* -------------------------------------------------------
 * VU Autonomous Racing Cars (2020S) - TU Wien
 * -------------------------------------------------------
 * Team 3: Adelmann, Brantner, Lukitsch, Pintaric
 * -------------------------------------------------------
 */

#include <iostream>
#include <string>
#include <cmath>

#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/GridCells.h>
#include <std_srvs/Empty.h>

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <boost/range/adaptor/indexed.hpp>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/mat.hpp>

#include <motion_planner/rrt.h>

#define SUBSCRIBER_MESSAGE_QUEUE_SIZE 1000
#define GRID_PUBLISHER_MESSAGE_QUEUE_SIZE 10

#define TOPIC_SCAN "/scan"
#define TOPIC_ODOM "/odom"
#define TOPIC_GOAL "/mode_base_simple/goal"
#define TOPIC_MAP  "/map"
#define TOPIC_DYNAMIC_OCCUPANCY_GRID "/motion_planner/dynamic_occupancy_grid"
#define TOPIC_STATIC_OCCUPANCY_GRID  "/motion_planner/static_occupancy_grid"
#define FRAME_MAP  "map"

#define DEFAULT_VEHICLE_WHEELBASE 0.3302 // units: m (see f110_simulator/params.yaml)
#define DEFAULT_VEHICLE_WIDTH     0.2032 // units: m (see f110_simulator/params.yaml)
#define DEFAULT_MAX_LASER_SCAN_DISTANCE 5.0 // units: m
#define DEFAULT_DYNAMIC_OCCUPANCY_GRID_UPDATE_FREQUENCY 20 // units: Hz

class MotionPlanner {

public:
    MotionPlanner();
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

    tf2_ros::Buffer tf_buffer_;
    boost::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    ros::Time last_scan_timestamp_;
    float scan_dt_threshold_;

    // Map
    MotionPlanner::OccupancyGridMessage map_;
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
};

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
{
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
        if(std::isfinite(range) && range <= max_laser_scan_distance_) {

            // 4 polar to cartesian coordinates [x,y]
            const float y = range * std::cos(angle); // units: meters
            const float x = range * std::sin(angle); // units: meters

            // [u,v] are the pixel coordinates in the dynamic occupancy grid
            const unsigned int u = static_cast<unsigned int>(x * pixels_per_meter) + dynamic_occupancy_grid_center_(0);
            const unsigned int v = static_cast<unsigned int>(y * pixels_per_meter) + dynamic_occupancy_grid_center_(1);

            assert(u >= 0 && v >= 0 && u < dynamic_occupancy_grid_.rows && v < dynamic_occupancy_grid_.cols);
            dynamic_occupancy_grid_.at<uint8_t>(u,v) = 255;
        }
    }

    // Dilate the dynamic occupancy grid
    dilateOccupancyGrid(dynamic_occupancy_grid_);
    dynamic_occupancy_grid_publisher_->publish(convertToGridCellsMessage(
            dynamic_occupancy_grid_, dynamic_occupancy_grid_center_, "laser"));
}

void MotionPlanner::odomSubscriberCallback(MotionPlanner::OdometryMessage odom_msg) {
    // ...
}

void MotionPlanner::goalSubscriberCallback(MotionPlanner::PoseStampedMessage goal_msg) {
    // ...
}

void MotionPlanner::mapSubscriberCallback(MotionPlanner::OccupancyGridMessage map_msg) {

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
            static_cast<int>(-map_->info.origin.position.x * pixels_per_meter),
            static_cast<int>(-map_->info.origin.position.y * pixels_per_meter));
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
    unsigned int dynamic_occupancy_grid_width = static_cast<unsigned int>(std::ceil(laser_scan_diameter_in_pixels)) + 2;
    if (dynamic_occupancy_grid_width % 2 == 0) {
        dynamic_occupancy_grid_width++;
    }
    dynamic_occupancy_grid_center_ = cv::Vec2i((dynamic_occupancy_grid_width - 1) / 2, (dynamic_occupancy_grid_width - 1) / 2);
    const cv::Scalar unoccupied_cell(0.0);
    dynamic_occupancy_grid_ = cv::Mat(dynamic_occupancy_grid_width, dynamic_occupancy_grid_width, CV_8UC1, unoccupied_cell);
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
            geometry_msgs::Point p;
            p.x = (float) (col - grid_center(0)) * meters_per_pixel;
            p.y = (float) (row - grid_center(1)) * meters_per_pixel;
            p.z = 0.0;
            grid_msg.cells.push_back(p);
        }
    }
    return(grid_msg);
}


int main(int argc, char **argv) {
#if !defined(NDEBUG)
    std::cerr << "WARNING: Debug build." << std::endl;
#endif
    ros::init(argc, argv, "motion_planner");
    MotionPlanner motion_planner;
    ros::spin();
    return 0;
}