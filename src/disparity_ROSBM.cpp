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
#include "message_filters/subscriber.h"

//Lo nuevo
#include "std_msgs/Float64.h"
#include "stdlib.h"
#include "depth_image_proc/depth_traits.h"

static const std::string OPENCV_WINDOWD = "Disparity";
//static const std::string OPENCV_WINDOWDC = "DisparityC";

typedef message_filters::sync_policies::ApproximateTime<
      stereo_msgs::DisparityImage, sensor_msgs::CameraInfo
      > MySyncPolicy;

class DisparityROSBM
{
private:
   ros::NodeHandle nh_;
   
   message_filters::Subscriber<stereo_msgs::DisparityImage> disparityImage_;
   message_filters::Subscriber<sensor_msgs::CameraInfo> cameraInfoR_;
   ros::Publisher camera_Info_Pub_;
   ros::Publisher depth_image_pub_;
   message_filters::Synchronizer<MySyncPolicy> sync_;

public:
   DisparityROSBM()
   : disparityImage_(nh_, "/stereo/disparity", 1),
     cameraInfoR_(nh_, "/stereo/right/camera_info", 1),
     //cameraInfoL_(nh_, "/stereo/left/camera_info", 1),
     sync_(MySyncPolicy(100),disparityImage_, cameraInfoR_)
   {
      sync_.registerCallback(boost::bind(&DisparityROSBM::disparity_callback, this, _1, _2));
      camera_Info_Pub_ = nh_.advertise<sensor_msgs::CameraInfo>("/tfg/camera_info", 1);
      depth_image_pub_ = nh_.advertise<sensor_msgs::Image>("/tfg/depthImage", 1);  
      //v::namedWindow(OPENCV_WINDOWD);

      //cv::namedWindow(OPENCV_WINDOWDC);
   }

   ~DisparityROSBM()
   {
      //cv::destroyWindow(OPENCV_WINDOWD);
      //cv::destroyWindow(OPENCV_WINDOWDC);
   }

   void disparity_callback(const stereo_msgs::DisparityImagePtr& disparity_image_msg, const sensor_msgs::CameraInfoConstPtr& camera_info_R)
   {
      ROS_INFO("Received stereo images");


      //Codigo para comparativas
      /*const cv::Mat imgDisparity16SC = cv::Mat(left_image.rows, left_image.cols, CV_16S);
      const cv::Mat imgDisparity8UC = cv::Mat(left_image.rows, left_image.cols, CV_8UC1);
      int ndisparityC = 16*8;
      int SADWindowSizeC = 21;
      cv::Ptr<cv::StereoBM> sbmC = cv::StereoBM::create(ndisparityC, SADWindowSizeC);
      sbmC->compute(left_image, right_image, imgDisparity16SC);
      double minValC;
      double maxValC;
      cv::minMaxLoc(imgDisparity8UC, &minValC, &maxValC);
      imgDisparity16SC.convertTo(imgDisparity8UC, CV_8UC1, 255/(maxValC - minValC));
      cv::imshow(OPENCV_WINDOWDC, imgDisparity8UC);
      */

      //Probando conversiones
      /*std_msgs::Float64 baseline;

      baseline = abs(infoL.P[3] - infoR.P[3]);
      ROS_INFO("baseline: %f", baseline);*/

      //Creando mi propio camera_info
      sensor_msgs::CameraInfo myCameraInfo;
      myCameraInfo = *camera_info_R;

      //Convirtiendo Mat en ros_msgs
      //const cv::Mat imgDisparity32F = cv::Mat(left_image.rows, left_image.cols, CV_32F);
      //imgDisparity8U.convertTo(imgDisparity32F, CV_32F);
      

      //calcular depthImage

      float focal = myCameraInfo.P[0];
      float baseline = -myCameraInfo.P[3]/focal;
      //ROS_INFO("baseline: %f", baseline);

      sensor_msgs::Image depthImage;
      depthImage.header = disparity_image_msg->header;
      depthImage.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
      depthImage.height = disparity_image_msg->image.height;
      depthImage.width = disparity_image_msg->image.width;
      depthImage.step = depthImage.width * sizeof(float);
      depthImage.data.resize(depthImage.height * depthImage.step, 0.0f);


      //float unit_scaling = depth_image_proc::DepthTraits<float>::toMeters(float(1));
      float constant = focal * baseline * 1000;
      uint8_t* data_in = reinterpret_cast<uint8_t*>(&disparity_image_msg->image.data[0]);
      int row_step = disparity_image_msg->image.step / sizeof(uint8_t);
      float* depth_data = reinterpret_cast<float*>(&depthImage.data[0]);
      //int size = currentDepthImage.width * currentDepthImage.height;
      for(int x=0; x<depthImage.height; x++)
      {
         for(int y=0; y<depthImage.width; y++)
         {
            float disp = data_in[y];
            *depth_data = (constant / disp)*20;
            //*disp_data =(float) depth;
            //ROS_INFO("F: %f O: %f", *disp_data, depth);
            ++depth_data;  
         }
         data_in += row_step;
      }

      cv::waitKey(3);

      //Transformar de depth image a laserScan



      //publicar
      camera_Info_Pub_.publish(myCameraInfo);
      depth_image_pub_.publish(depthImage);
   }
};

int main(int argc, char **argv)
{
   ros::init(argc, argv, "DisparityROSBM");
   DisparityROSBM drbm;
   ros::spin();
   return 0;
}