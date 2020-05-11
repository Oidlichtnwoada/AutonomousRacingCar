#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

using namespace ros;
using namespace std;

class ParticleFilter {
private:
    NodeHandle n;
    Subscriber sub;
    Publisher pub;
public:
    ParticleFilter() {
      n = NodeHandle();
      sub = n.subscribe("scan", 1, &ParticleFilter::callback, this);
      pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("nav", 1);
    }

    void callback(const sensor_msgs::LaserScan::ConstPtr &msg) {
      ackermann_msgs::AckermannDriveStamped m;
      pub.publish(m);
    }
};

int main(int argc, char **argv) {
  init(argc, argv, "particle_filter");
  ParticleFilter particleFilter;
  spin();
  return 0;
}
