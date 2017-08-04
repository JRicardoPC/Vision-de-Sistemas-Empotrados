#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "image_transport/subscriber_filter.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include "stereo_msgs/DisparityImage.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/sync_policies/exact_time.h"
#include "message_filters/subscriber.h"
#include <image_geometry/stereo_camera_model.h>
#include <stereo_msgs/DisparityImage.h>
#include "std_msgs/Float64.h"
#include "stdlib.h"

typedef message_filters::sync_policies::ExactTime<
      sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image, sensor_msgs::CameraInfo
      > MySyncPolicy;

class DisparityBM
{
private:
   ros::NodeHandle nh_;
   image_transport::ImageTransport it_;
   image_transport::SubscriberFilter image_left_sub_;
   image_transport::SubscriberFilter image_right_sub_;
   
   message_filters::Subscriber<sensor_msgs::CameraInfo> cameraInfoR_;
   message_filters::Subscriber<sensor_msgs::CameraInfo> cameraInfoL_;
   ros::Publisher camera_Info_Pub_;
   ros::Publisher disparity_bm_pub_;
   ros::Publisher depth_image_pub_;
   message_filters::Synchronizer<MySyncPolicy> sync_;

   mutable cv::Mat_<int16_t> imgDisparity16S; // se define fuera de la función para mejorar la gestion de la memoria
   stereo_msgs::DisparityImage dispImage; 
   const int ndisparity = 16*8;
   const int SADWindowSize = 15;
   const int min_Disparity = 0;

   #if CV_MAJOR_VERSION == 3
 	   cv::Ptr<cv::StereoBM> sbm = cv::StereoBM::create(ndisparity, SADWindowSize);
   #else
	   cv::StereoBM sbm = cv::StereoBM(CV_STEREO_BM_BASIC,ndisparity, SADWindowSize);
   #endif

public:
   DisparityBM()  : it_(nh_),
      image_left_sub_(it_, "/stereo/left/image_rect", 1),
      cameraInfoL_(nh_, "/stereo/left/camera_info", 1),     
      image_right_sub_(it_, "/stereo/right/image_rect", 1),
      cameraInfoR_(nh_, "/stereo/right/camera_info", 1),       
      sync_(MySyncPolicy(10),image_left_sub_, cameraInfoL_, image_right_sub_, cameraInfoR_)
   {

      sync_.registerCallback(boost::bind(&DisparityBM::disparity_callback, this, _1, _2, _3, _4));
      camera_Info_Pub_ = nh_.advertise<sensor_msgs::CameraInfo>("/tfg/camera_info", 1);
      depth_image_pub_ = nh_.advertise<sensor_msgs::Image>("/tfg/depthImage", 1);
      disparity_bm_pub_ = nh_.advertise<stereo_msgs::DisparityImage>("/tfg/disparityImage/bm", 1); 

   }

   ~DisparityBM()
   {

   }

  void disparity_callback(const sensor_msgs::ImageConstPtr& image_left_msg,
                               const sensor_msgs::CameraInfoConstPtr& camera_info_L,
                               const sensor_msgs::ImageConstPtr& image_right_msg,
                               const sensor_msgs::CameraInfoConstPtr& camera_info_R)
   {
      static const int DPP = 16; // disparities per pixel
      static const double inv_dpp = 1.0 / DPP;

      cv_bridge::CvImageConstPtr cv_ptr_left = cv_bridge::toCvShare(image_left_msg, sensor_msgs::image_encodings::TYPE_8UC1);
      cv_bridge::CvImageConstPtr cv_ptr_right = cv_bridge::toCvShare(image_right_msg, sensor_msgs::image_encodings::TYPE_8UC1);
      const cv::Mat left_image = cv_ptr_left->image;
      const cv::Mat right_image = cv_ptr_right->image;

      if (imgDisparity16S.empty()) 
     	   imgDisparity16S = cv::Mat(left_image.rows, left_image.cols, CV_16S);
     
      #if CV_MAJOR_VERSION == 3
         sbm->compute(left_image, right_image, imgDisparity16S);
      #else
         sbm.operator()(left_image, right_image, imgDisparity16S);
      #endif

 	   image_geometry::StereoCameraModel model_;
      model_.fromCameraInfo(camera_info_L,camera_info_R);

      float focal = model_.right().fx();
      float baseline = model_.baseline();

      //Creando mi propio camera_info
      sensor_msgs::CameraInfo myCameraInfo;
      myCameraInfo = *camera_info_R;

     	sensor_msgs::Image& dimage = dispImage.image;
   	dispImage.header = camera_info_L->header;
     	dimage.height = imgDisparity16S.rows;
     	dimage.width = imgDisparity16S.cols;
     	dimage.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
     	dimage.step = dimage.width * sizeof(float);
     	dimage.data.resize(dimage.step * dimage.height);
     	cv::Mat_<float> dmat(dimage.height, dimage.width, (float*)&dimage.data[0], dimage.step);

     	imgDisparity16S.convertTo(dmat, dmat.type(), inv_dpp, -(model_.left().cx() - model_.right().cx()));
    	ROS_ASSERT(dmat.data == &dimage.data[0]);

     // Stereo parameters
     	dispImage.f = model_.right().fx();
     	dispImage.T = model_.baseline();

     // Disparity search range
     	dispImage.min_disparity = min_Disparity;
     	dispImage.max_disparity = min_Disparity + ndisparity - 1;
     	dispImage.delta_d = inv_dpp;

      //calcular depthImage
      sensor_msgs::Image depthImage;
      depthImage.header = dispImage.header;
      depthImage.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
      depthImage.height = dimage.height;
      depthImage.width = dimage.width;
      depthImage.step = depthImage.width * sizeof(float);
      depthImage.data.resize(depthImage.height * depthImage.step, 0.0f);

      float constant = dispImage.f * dispImage.T;

      const float* disp_row = reinterpret_cast<const float*>(&dimage.data[0]);
      int row_step = dimage.step / sizeof(float);
      float* depth_data = reinterpret_cast<float*>(&depthImage.data[0]);
      for (int v = 0; v < (int)dimage.height; ++v)
      {
         for (int u = 0; u < (int)dimage.width; ++u)
         {
            float disp = disp_row[u];
            if (disp > 0)
            *depth_data = constant / disp;
            ++depth_data;
         }
         disp_row += row_step;
      }

      //publicar
      camera_Info_Pub_.publish(myCameraInfo);
      disparity_bm_pub_.publish(dispImage);
      depth_image_pub_.publish(depthImage);
   }
};

int main(int argc, char **argv)
{
   ros::init(argc, argv, "DisparityBM");
   DisparityBM dbm;
   ros::spin();
   return 0;
}
