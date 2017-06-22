cmake_minimum_required(VERSION 2.8.3)
project(tfg)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
  cv_bridge
  image_transport
  message_filters
  stereo_msgs
  std_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   imagen.msg
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   actionlib_msgs
   std_msgs
   sensor_msgs
)

catkin_package(
   CATKIN_DEPENDS roscpp rospy std_msgs actionlib actionlib_msgs message_runtime sensor_msgs
)

include_directories(
   include
   ${catkin_INCLUDE_DIRS}
   ${Boost_INCLUDE_DIRS}
)

#This will create executables of the nodes
add_executable(Image_Calibrate src/Image_Calibrate.cpp)
add_executable(DisparityBM src/disparity_BM.cpp)
add_executable(DisparitySGBM src/disparity_SGBM.cpp)


#This will generate message header file before building the target
add_dependencies(Image_Calibrate tfg_generate_messages_cpp)
add_dependencies(DisparityBM tfg_generate_messages_cpp)
add_dependencies(DisparitySGBM tfg_generate_messages_cpp)


#This will link executables to the appropriate libraries
target_link_libraries(Image_Calibrate ${catkin_LIBRARIES})
target_link_libraries(DisparityBM ${catkin_LIBRARIES})
target_link_libraries(DisparitySGBM ${catkin_LIBRARIES})