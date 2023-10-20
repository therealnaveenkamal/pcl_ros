#include "magic_subscriber_image_edgedetector.h"
#include <sensor_msgs/Image.h>

using sensor_msgs::Image;

int main(int argc, char **argv) {

  ros::init(argc, argv, "magic_pcl_susbcriber_main_node");

  ros::NodeHandle _n("magic_pcl_susbcriber_main_ns");
  string topic_name = "/camera/rgb/image_raw";

  MagicSubscriber magic_subscriber_object;

  magic_subscriber_object.init<Image>(_n, topic_name);

  ros::spin();

  return 0;
}