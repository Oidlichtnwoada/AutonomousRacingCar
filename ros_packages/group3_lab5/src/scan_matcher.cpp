#include <sstream>
#include <string>
#include <cmath>

#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PoseStamped.h>

#include <scan_matcher/correspond.h>
#include <scan_matcher/transform.h>
#include <scan_matcher/visualization.h>

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <unsupported/Eigen/EulerAngles>

#include <iostream>
#include <future>
#include <mutex>

#include <boost/range/adaptor/indexed.hpp>

using namespace std;

#define SUBSCRIBER_MESSAGE_QUEUE_SIZE 1000
#define PUBLISHER_MESSAGE_QUEUE_SIZE    10

#define TOPIC_JOY     "/joy"
#define TOPIC_SCAN    "/scan"
#define TOPIC_RVIZ    "/scan_match_debug"
#define GT_POSE_TOPIC "/gt_pose"
#define FRAME_POINTS  "laser"
#define RANGE_LIMIT   10.0

#define DEFAULT_MAX_NUMBER_OF_ITERATIONS 20
#define DEFAULT_INLIER_RATIO 0.9
#define DEFAULT_MAX_CORRESPONDENCE_DIST 0.0
#define POSE_RESET_JOY_BUTTON_INDEX 10

std_msgs::ColorRGBA toColor(float r, float g, float b, float a = 1.0) {
    std_msgs::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;
    return(color);
}

class ScanMatcher {

public:
    ScanMatcher();
    typedef boost::shared_ptr<sensor_msgs::Joy const> JoyMessage;
    typedef boost::shared_ptr<sensor_msgs::LaserScan const> LaserScanMessage;
    typedef boost::shared_ptr<geometry_msgs::PoseStamped const> PoseStampedMessage;
    typedef std::vector<::Point> Points;

private:
    static Points convertToPoints(LaserScanMessage);
    void matchPointSets(const ros::Time&, const Points&, const Points&);
    void joySubscriberCallback(JoyMessage);
    void laserScanSubscriberCallback(LaserScanMessage);
    void groundTruthPoseCallback(PoseStampedMessage);

    ros::NodeHandle node_handle_;

    boost::shared_ptr<ros::Subscriber> gamepad_subscriber_;
    boost::shared_ptr<ros::Subscriber> laser_scan_subscriber_;
    boost::shared_ptr<ros::Subscriber> gt_pose_subscriber_;
    boost::shared_ptr<ros::Publisher> marker_pub_;

    boost::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    tf2_ros::Buffer tf_buffer_;
    boost::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    boost::shared_ptr<PointVisualizer> points_viz_;
    boost::shared_ptr<CorrespondenceVisualizer> corr_viz_;

    Eigen::Affine3d global_tf_;
    LaserScanMessage previous_scan_msg_;
    Points previous_scan_points_;
    std::future<void> async_matching_task_;

    ros::Time first_scan_timestamp_; // only for debugging

    std::mutex tf_broadcaster_mutex_;
    std::mutex global_tf_mutex_;

    std::atomic<bool> signal_pose_reset_;
    int max_number_of_iterations_;

    unsigned int scan_msg_frames_dropped_;    // only for debugging
    unsigned int scan_msg_frames_processed_;  // only for debugging

    float inlier_ratio_;
    float max_correspondence_dist_;
};

ScanMatcher::ScanMatcher()
    : node_handle_(ros::NodeHandle())
    , gamepad_subscriber_(new ros::Subscriber(node_handle_.subscribe(TOPIC_JOY, SUBSCRIBER_MESSAGE_QUEUE_SIZE, &ScanMatcher::joySubscriberCallback, this)))
    , laser_scan_subscriber_(new ros::Subscriber(node_handle_.subscribe(TOPIC_SCAN, SUBSCRIBER_MESSAGE_QUEUE_SIZE, &ScanMatcher::laserScanSubscriberCallback, this)))
    , gt_pose_subscriber_(new ros::Subscriber(node_handle_.subscribe(GT_POSE_TOPIC, SUBSCRIBER_MESSAGE_QUEUE_SIZE, &ScanMatcher::groundTruthPoseCallback, this)))
    , marker_pub_(new ros::Publisher(node_handle_.advertise<visualization_msgs::Marker>(TOPIC_RVIZ, PUBLISHER_MESSAGE_QUEUE_SIZE)))
    , tf_broadcaster_(new tf2_ros::TransformBroadcaster())
    , tf_listener_(new tf2_ros::TransformListener(tf_buffer_, true))
    , points_viz_(new PointVisualizer(*marker_pub_, "scan_match", FRAME_POINTS))
    , corr_viz_(new CorrespondenceVisualizer(*marker_pub_, "scan_match", FRAME_POINTS))
    , global_tf_(Eigen::Matrix3d::Identity(3,3))
    , signal_pose_reset_(false)
    , scan_msg_frames_dropped_(0)
    , scan_msg_frames_processed_(0)
{
    node_handle_.param<int>("max_number_of_iterations", max_number_of_iterations_, DEFAULT_MAX_NUMBER_OF_ITERATIONS);
    node_handle_.param<float>("max_correspondence_dist", max_correspondence_dist_, DEFAULT_MAX_CORRESPONDENCE_DIST);
    node_handle_.param<float>("inlier_ratio", inlier_ratio_, DEFAULT_INLIER_RATIO);
}

void ScanMatcher::groundTruthPoseCallback(ScanMatcher::PoseStampedMessage pose_msg) {

    // Publish ground truth pose for debugging purposes...

    geometry_msgs::TransformStamped transform_msg;
    transform_msg.header.stamp = pose_msg->header.stamp;
    transform_msg.header.seq = pose_msg->header.seq;
    transform_msg.header.frame_id = "map";          // parent frame id
    transform_msg.child_frame_id = "ground_truth/base_link";  // child frame id

    transform_msg.transform.rotation.x = pose_msg->pose.orientation.x;
    transform_msg.transform.rotation.y = pose_msg->pose.orientation.y;
    transform_msg.transform.rotation.z = pose_msg->pose.orientation.z;
    transform_msg.transform.rotation.w = pose_msg->pose.orientation.w;

    transform_msg.transform.translation.x = pose_msg->pose.position.x;
    transform_msg.transform.translation.y = pose_msg->pose.position.y;
    transform_msg.transform.translation.z = pose_msg->pose.position.z;

    // Not sure if tf2_ros::TransformBroadcaster is thread-safe or not, so let's suppose it's not...
    {
        std::lock_guard<std::mutex> scoped_lock(tf_broadcaster_mutex_);
        tf_broadcaster_->sendTransform(transform_msg);
    }

    bool expected = true;
    if(signal_pose_reset_.compare_exchange_strong(expected, false)) {

        const Eigen::Vector3d translation(
                transform_msg.transform.translation.x,
                transform_msg.transform.translation.y,
                transform_msg.transform.translation.z);
        const Eigen::Quaterniond rotation(
                transform_msg.transform.rotation.w,
                transform_msg.transform.rotation.x,
                transform_msg.transform.rotation.y,
                transform_msg.transform.rotation.z);

        Eigen::Affine3d global_tf = Eigen::Affine3d::Identity();
        typedef Eigen::EulerSystem<Eigen::EULER_Y, Eigen::EULER_X, Eigen::EULER_Z> EulerSystem;
        typedef Eigen::EulerAngles<double, EulerSystem> EulerAngles;
        EulerAngles angles = EulerAngles::FromRotation<true, false, false>(rotation);
        const double& z_angle = angles.angles()(2);
        global_tf.prerotate(Eigen::AngleAxisd(z_angle, Eigen::Vector3d::UnitZ()));
        global_tf.pretranslate(translation);
        {
            std::lock_guard<std::mutex> scoped_lock(global_tf_mutex_);
            global_tf_ = global_tf;
        }
    }
}

void ScanMatcher::joySubscriberCallback(ScanMatcher::JoyMessage joy_msg) {
    if(joy_msg->buttons.size() > POSE_RESET_JOY_BUTTON_INDEX &&
       joy_msg->buttons[POSE_RESET_JOY_BUTTON_INDEX]) {
        signal_pose_reset_.store(true);
    }
}

void ScanMatcher::laserScanSubscriberCallback(ScanMatcher::LaserScanMessage scan_msg) {
    if(!previous_scan_msg_) {
        first_scan_timestamp_ = scan_msg->header.stamp;
        previous_scan_msg_ = scan_msg;
        previous_scan_points_ = ScanMatcher::convertToPoints(scan_msg);
        return;
    }

    if(async_matching_task_.valid() &&
       async_matching_task_.wait_for(std::chrono::nanoseconds(0)) != future_status::ready) {
        scan_msg_frames_dropped_++;
        if(scan_msg_frames_dropped_ % (1000*10) == 0) {
            std::cerr << "INFO: " << scan_msg_frames_dropped_ << " scan messages skipped ("
                << (int)(100.f * (float) (scan_msg_frames_dropped_) / (float) (scan_msg_frames_dropped_ + scan_msg_frames_processed_))
                << "%)" << std::endl;
        }
        return;
    }
    scan_msg_frames_processed_++;

    Points scan_points = ScanMatcher::convertToPoints(scan_msg);

    async_matching_task_ = std::async(std::launch::async,
                                      &ScanMatcher::matchPointSets,
                                      this,
                                      scan_msg->header.stamp,
                                      previous_scan_points_,
                                      scan_points);

    previous_scan_msg_ = scan_msg;
    previous_scan_points_ = scan_points;
}

ScanMatcher::Points ScanMatcher::convertToPoints(ScanMatcher::LaserScanMessage scan_msg) {
    Points points;
    points.reserve(scan_msg->ranges.size());
    const float angle_min = scan_msg->angle_min;
    const float angle_increment = scan_msg->angle_increment;
    for(auto const& element : scan_msg->ranges | boost::adaptors::indexed()) {
        const float range = element.value();
        const float angle = angle_min + (float) element.index() * angle_increment;
        if(std::isfinite(range) && range <= RANGE_LIMIT) {
            points.push_back(Points::value_type(range, angle));
        }
    }
    return(points);
}

void ScanMatcher::matchPointSets(
        const ros::Time& t,
        const ScanMatcher::Points& pts_t0,
        const ScanMatcher::Points& pts_t1) {

    //ICP algorithm

    Transform estimated_transform_t1_to_t0(0.f, 0.f, 0.f);
    const JumpTable jump_table = computeJumpTable(pts_t0);
    unsigned int number_of_iterations = 0;
    const float max_correspondence_dist = max_correspondence_dist_ > 0.0f ?
            max_correspondence_dist_ : std::numeric_limits<float>::max();

    for (int i = 0; i < max_number_of_iterations_; i++) {
        number_of_iterations++;
        Points trans_pts_t1 = transformPoints(pts_t1, estimated_transform_t1_to_t0);

        SimpleCorrespondences correspondences = findCorrespondences(
                pts_t0,
                trans_pts_t1,
                jump_table,
                max_correspondence_dist,
                inlier_ratio_);

        const Transform new_estimated_transform_t1_to_t0 =
                estimated_transform_t1_to_t0 +
                estimateTransformation(correspondences).inverse();

        if (estimated_transform_t1_to_t0 == new_estimated_transform_t1_to_t0) {
            break;
        }
        estimated_transform_t1_to_t0 = new_estimated_transform_t1_to_t0;
    }

    const float inverse_tx = estimated_transform_t1_to_t0.x_disp;
    const float inverse_ty = estimated_transform_t1_to_t0.y_disp;
    const float inverse_rot_theta = estimated_transform_t1_to_t0.theta_rot;

    // Publish map-->base_link coordinate frame transformation.
    geometry_msgs::TransformStamped transform_msg;
    {
        std::lock_guard<std::mutex> scoped_lock(global_tf_mutex_);
        global_tf_.rotate(Eigen::AngleAxisf(inverse_rot_theta, Eigen::Vector3f::UnitZ()).cast<double>());
        global_tf_.translate(Eigen::Vector3f(inverse_tx, inverse_ty, 0.f).cast<double>());
        transform_msg = tf2::eigenToTransform(global_tf_);
    }
    transform_msg.header.stamp = t;
    transform_msg.header.frame_id = "map";   // parent frame id
    transform_msg.child_frame_id = "base_link";   // child frame id

    // Not sure if tf2_ros::TransformBroadcaster is thread-safe or not, so let's suppose it's not...
    std::lock_guard<std::mutex> scoped_lock(tf_broadcaster_mutex_);
    tf_broadcaster_->sendTransform(transform_msg);

    points_viz_->addPoints(pts_t1, toColor(1.0, 0.0, 0.0));
    points_viz_->publishPoints();

}

int main(int argc, char **argv) {
#if !defined(NDEBUG)
    std::cerr << "WARNING: Debug build." << std::endl;
#endif
  ros::init(argc, argv, "scan_matcher");
  ScanMatcher scan_matcher;
  ros::spin();
  return 0;
}
