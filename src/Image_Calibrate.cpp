#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"

static const std::string OPENCV_WINDOWR = "Image window right";
static const std::string OPENCV_WINDOWL = "Image window left";

class Calibrate
{
private:
   ros::NodeHandle node_obj;
   image_transport::Subscriber image_left_subscriber;
   image_transport::Subscriber image_right_subscriber;
   image_transport::Publisher image_left_publisher;
   image_transport::Publisher image_right_publisher;
   image_transport::ImageTransport it;

public:
   Calibrate()
   : it(node_obj)
   {
      image_left_subscriber = it.subscribe("/stereo/left/image_raw_relay", 1, &Calibrate::image_callback_left, this);
      image_right_subscriber = it.subscribe("/stereo/right/image_raw_relay", 1, &Calibrate::image_callback_right, this);
      image_left_publisher = it.advertise("/calibrate/left/image_raw", 1);
      image_right_publisher = it.advertise("/calibrate/right/image_raw", 1);

      cv::namedWindow(OPENCV_WINDOWL);
      cv::namedWindow(OPENCV_WINDOWR);
   }

   ~Calibrate()
   {
      cv::destroyWindow(OPENCV_WINDOWR);
      cv::destroyWindow(OPENCV_WINDOWL);
   }

   void image_callback_left(const sensor_msgs::Image::ConstPtr& msg)
   {
      //ROS_INFO("Received left [%d]", msg->data[2]);
      cv_bridge::CvImagePtr cv_ptr;
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

      if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
         cv::circle(cv_ptr->image, cv::Point(50,50), 10, CV_RGB(255,0,0));

      cv::imshow(OPENCV_WINDOWL, cv_ptr->image);
      cv::waitKey(3);

      image_left_publisher.publish(cv_ptr->toImageMsg());
   }

   void image_callback_right(const sensor_msgs::Image::ConstPtr& msg)
   {
      //ROS_INFO("Received left [%d]", msg->data[2]);
      cv_bridge::CvImagePtr cv_ptr;
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

      if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
         cv::circle(cv_ptr->image, cv::Point(50,50), 10, CV_RGB(255,0,0));

      cv::imshow(OPENCV_WINDOWR, cv_ptr->image);
      cv::waitKey(3);

      image_left_publisher.publish(cv_ptr->toImageMsg());
   }
};

/*void image_callback_right(const sensor_msgs::Image::ConstPtr& msg)
{
   ROS_INFO("Received right [%d]", msg->data[2]);
}*/

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Image_Calibrate");
    Calibrate pr;
    ros::spin();
    return 0;
}