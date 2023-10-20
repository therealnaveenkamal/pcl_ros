#ifndef MAGIC_SUBSCRIBER_H
#define MAGIC_SUBSCRIBER_H

// ROS core
#include <ros/ros.h>
// Image message
#include <sensor_msgs/Image.h>
// pcl::toROSMsg
#include <pcl_conversions/pcl_conversions.h>
// stl stuff
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <string>

using namespace std;
using namespace cv;

class MagicSubscriber {
public:
  MagicSubscriber() {
    cout << "MagicSubscriber Constructor is called" << endl;
  };
  ~MagicSubscriber() {
    cout << "MagicSubscriber Destructor is called" << endl;
  };

  template <typename ROSMessageType>
  void init(ros::NodeHandle &ros_node, const string subscriber_topic) {
    // We used an initialiser list
    this->m_subscriber_topic = subscriber_topic;
    this->m_ros_node_object = &ros_node;

    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                       ros::console::levels::Info)) {
      ros::console::notifyLoggerLevelsChanged();
    }

    image_sub_ = this->m_ros_node_object->subscribe(
        this->m_subscriber_topic, 1,
        &MagicSubscriber::CallbackToTopic<ROSMessageType>, this);

    image_pub_ = this->m_ros_node_object->advertise<sensor_msgs::Image>(
        "/image_converter/output_video", 1000);
  };

  template <typename CallBackROSMessageType>
  void CallbackToTopic(const typename CallBackROSMessageType::ConstPtr &msg) {
    // the uint8 is an alias of unsigned char, therefore needs casting to int
    ROS_INFO_STREAM("Call Back Topic Image To Depth Data[0]="
                    << static_cast<int>(msg->data[0]));

    sensor_msgs::PointCloud point_cloud;

    sensor_msgs::convertPointCloud2ToPointCloud(*msg, point_cloud);

    // These are value sextracted form the rgb camera caminfo
    float K[9] = {343.49636753580074,
                  0.0,
                  320.5,
                  0.0,
                  343.49636753580074,
                  240.5,
                  0.0,
                  0.0,
                  1.0};
    float centre_x = K[2];
    float centre_y = K[5];
    float focal_x = K[0];
    float focal_y = K[4];
    int cam_info_height = 480;
    int cam_info_width = 640;

    Mat cv_image = Mat(cam_info_height, cam_info_width, CV_32FC1,
                       Scalar(std::numeric_limits<float>::max()));

    for (int i = 0; i < point_cloud.points.size(); i++) {
      if (point_cloud.points[i].z == point_cloud.points[i].z) {
        float z = point_cloud.points[i].z * 1000.0;
        float u = (point_cloud.points[i].x * 1000.0 * focal_x) / z;
        float v = (point_cloud.points[i].y * 1000.0 * focal_y) / z;
        int pixel_pos_x = (int)(u + centre_x);
        int pixel_pos_y = (int)(v + centre_y);

        if (pixel_pos_x > (cam_info_width - 1)) {
          pixel_pos_x = cam_info_width - 1;
        }
        if (pixel_pos_y > (cam_info_height - 1)) {
          pixel_pos_y = cam_info_height - 1;
        }
        cv_image.at<float>(pixel_pos_y, pixel_pos_x) = z;
      }
    }

    putText(cv_image, "Depth CAM", Point(50, 50), FONT_HERSHEY_DUPLEX, 1,
            Scalar(0, 255, 0), 2, false);

    cv_image.convertTo(cv_image, CV_16UC1);

    sensor_msgs::ImagePtr output_image =
        cv_bridge::CvImage(std_msgs::Header(), "16UC1", cv_image).toImageMsg();
    output_image->header.stamp = ros::Time::now();
    image_pub_.publish(output_image);
  };

private:
  ros::NodeHandle *m_ros_node_object;
  string m_subscriber_topic;
  ros::Subscriber image_sub_;
  ros::Publisher image_pub_;
  sensor_msgs::Image image_;
};

#endif