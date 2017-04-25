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

static const std::string OPENCV_WINDOWD = "Disparity";

typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::Image, sensor_msgs::Image
      > MySyncPolicy;

class DisparitySGBM
{
private:
   ros::NodeHandle nh_;
   image_transport::ImageTransport it_;
   image_transport::SubscriberFilter image_left_sub_;
   image_transport::SubscriberFilter image_right_sub_;
   image_transport::Publisher disparity_bm_pub_;
   message_filters::Synchronizer<MySyncPolicy> sync_;

public:
   DisparitySGBM()
   : it_(nh_),
     image_left_sub_(it_, "/stereo/left/image_rect", 1),
     image_right_sub_(it_, "/stereo/right/image_rect", 1),
     sync_(MySyncPolicy(100),image_left_sub_, image_right_sub_)
   {
      sync_.registerCallback(boost::bind(&DisparitySGBM::disparity_callback, this, _1, _2));
      disparity_bm_pub_ = it_.advertise("/disparity/sgbm", 1);    
      cv::namedWindow(OPENCV_WINDOWD);
   }

   ~DisparitySGBM()
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
      const cv::Mat imgDisparity16S = cv::Mat(left_image.rows, left_image.cols, CV_16S);
      const cv::Mat imgDisparity8U = cv::Mat(left_image.rows, left_image.cols, CV_8UC1);

      int minDisparity = 16;
      int numDisparities = 16;
      int blockSize = 1;


      //cv::Ptr<cv::StereoSGBM> ssgbm = cv::StereoSGBM::create(minDisparity, numDisparities, blockSize);
      cv::Ptr<cv::StereoSGBM> ssgbm = cv::StereoSGBM::create(0, 32, 3, 128, 256, 20, 16, 1, 100, 20, true);

      ssgbm->compute(left_image, right_image, imgDisparity16S);

      double minVal;
      double maxVal;

      cv::minMaxLoc( imgDisparity8U, &minVal, &maxVal);

      ROS_INFO("Min disp: %f Max value: %f", minVal, maxVal);

      imgDisparity16S.convertTo(imgDisparity8U, CV_8UC1, 255/(maxVal - minVal));

      cv::imshow(OPENCV_WINDOWD, imgDisparity8U);
      cv::waitKey(3);

      //disparity_bm_publisher_.publish(cv_ptr->toImageMsg());
   }
};

int main(int argc, char **argv)
{
   ros::init(argc, argv, "DisparitySGBM");
   DisparitySGBM dsgbm;
   ros::spin();
   return 0;
}