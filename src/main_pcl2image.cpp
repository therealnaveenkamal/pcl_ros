#include "magic_subscriber_pcl2image.h"
#include <sensor_msgs/PointCloud2.h>

using sensor_msgs::PointCloud2;

int main(int argc, char **argv) {

  ros::init(argc, argv, "main_pcl2image_node");

  ros::NodeHandle _n("main_pcl2image_ns");
  string topic_name = "/camera/depth_registered/points";

  MagicSubscriber magic_subscriber_object;

  magic_subscriber_object.init<PointCloud2>(_n, topic_name);

  ros::spin();

  return 0;
}