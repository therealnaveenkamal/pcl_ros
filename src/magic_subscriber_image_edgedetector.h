#ifndef MAGIC_SUBSCRIBER_H
#define MAGIC_SUBSCRIBER_H

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

static const std::string OPENCV_WINDOW = "Image window";

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

    namedWindow(OPENCV_WINDOW);
  };

  template <typename CallBackROSMessageType>
  void CallbackToTopic(const typename CallBackROSMessageType::ConstPtr &msg) {
    // the uint8 is an alias of unsigned char, therefore needs casting to int
    ROS_INFO_STREAM(
        "Call Back Topic Image Data[0]=" << static_cast<int>(msg->data[0]));

    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // The Edge detection
    // Convert to graycsale

    cvtColor(cv_ptr->image, cv_ptr->image, COLOR_BGR2GRAY);
    // Blur the image for better edge detection

    GaussianBlur(cv_ptr->image, cv_ptr->image, Size(3, 3), 0);

    // Canny edge detection

    Canny(cv_ptr->image, cv_ptr->image, 100, 200, 3, false);

    putText(cv_ptr->image, "EDGE DETECTION CAM", cvPoint(30, 30),
            FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200, 200, 250), 1,
            CV_MSA);

    // Display canny edge detected image
    imshow(OPENCV_WINDOW, cv_ptr->image);
    waitKey(3);

    // Update GUI Window
    // imshow(OPENCV_WINDOW, cv_ptr->image);
    // waitKey(3);

    // Output modified video stream
    cvtColor(cv_ptr->image, cv_ptr->image, COLOR_GRAY2BGR);
    image_pub_.publish(cv_ptr->toImageMsg());
  };

private:
  ros::NodeHandle *m_ros_node_object;
  string m_subscriber_topic;
  ros::Subscriber image_sub_;
  ros::Publisher image_pub_;
};

#endif