#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "image_transport/subscriber_filter.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/subscriber.h"

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
static const std::string OPENCV_WINDOWD = "Disparity";

class DisparityBM
{
private:
   ros::NodeHandle node_obj_;
   image_transport::SubscriberFilter image_left_subscriber_;
   image_transport::SubscriberFilter image_right_subscriber_;
   image_transport::Publisher disparity_bm_publisher_;
   image_transport::ImageTransport it_;
   message_filters::Synchronizer< MySyncPolicy > sync_;

public:
   DisparityBM()
   : it_(node_obj_),
     image_left_subscriber_(it_, "/stereo/left/image_rect", 1),
     image_right_subscriber_(it_, "/stereo/right/image_rect", 1),
     sync_(MySyncPolicy(10), image_left_subscriber_, image_right_subscriber_)
   {
      sync_.registerCallback(boost::bind(&DisparityBM::disparity_callback, this, _1, _2));
      disparity_bm_publisher_ = it_.advertise("/disparity/bm", 1);    
      cv::namedWindow(OPENCV_WINDOWD);
   }

   ~DisparityBM()
   {
      cv::destroyWindow(OPENCV_WINDOWD);
   }

   void disparity_callback(const sensor_msgs::ImageConstPtr& image_left_msg, const sensor_msgs::ImageConstPtr& image_right_msg)
   {
      ROS_INFO("Received stereo images");

      cv_bridge::CvImageConstPtr cv_ptr_left = cv_bridge::toCvShare(image_left_msg, sensor_msgs::image_encodings::TYPE_8UC1);
      cv_bridge::CvImageConstPtr cv_ptr_right = cv_bridge::toCvShare(image_right_msg, sensor_msgs::image_encodings::TYPE_8UC1);
      const cv::Mat left_image = cv_ptr_left->image;
      const cv::Mat right_image = cv_ptr_right->image;

      cv::Mat imgDisparity8U = cv::Mat(left_image.rows, left_image.cols, CV_8UC1);

      int ndisparity = 0;
      int SADWindowSize = 21;
      cv::Ptr<cv::StereoBM> sbm = cv::StereoBM::create(ndisparity, SADWindowSize);

      sbm->compute(left_image, right_image, imgDisparity8U);

      double minVal;
      double maxVal;

      cv::minMaxLoc( imgDisparity8U, &minVal, &maxVal);

      ROS_INFO("Min disp: %f Max value: %f", minVal, maxVal);

      //imgDisparity16S.convertTo(imgDisparity8U, CV_8UC1, 255/(maxVal - minVal));

      cv::imshow(OPENCV_WINDOWD, imgDisparity8U);
      cv::waitKey(3);

      //disparity_bm_publisher_.publish(cv_ptr->toImageMsg());
   }
};

int main(int argc, char **argv)
{
   ros::init(argc, argv, "DisparityBM");
   DisparityBM dbm;
   while(ros::ok())
      ros::spin();
   return 0;
}