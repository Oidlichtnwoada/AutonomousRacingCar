#pragma once

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <scan_matcher/correspond.h>

using namespace std;

static int num_visuals = 0;

class PointVisualizer {
protected:
	ros::Publisher& pub;
	visualization_msgs::Marker dots;
	string ns;
	string frame_id;

public:
	PointVisualizer(ros::Publisher& pub, string ns, string frame_id);
	void addPoints(const vector<Point>& points, std_msgs::ColorRGBA color);
	void publishPoints();
    ~PointVisualizer() {};
};

class CorrespondenceVisualizer {
protected:
	ros::Publisher& pub;
	visualization_msgs::Marker line_list;
	string ns;
	string frame_id;

public:
	CorrespondenceVisualizer(ros::Publisher& pub, string ns, string frame_id);
	void addCorrespondences(vector<SimpleCorrespondence> corresponds);
	void publishCorrespondences();
    ~CorrespondenceVisualizer() {};
};
