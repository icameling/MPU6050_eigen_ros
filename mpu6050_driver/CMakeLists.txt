cmake_minimum_required(VERSION 2.8.3)
project(mpu6050_driver)


# 添加Eigen头文件
include_directories( "/usr/include/eigen3" )


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  sensor_msgs
  geometry_msgs
)


add_message_files(
  FILES
  STM32_ImuData.msg
)


generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs # Or other packages containing msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mpu6050_driver
  CATKIN_DEPENDS roscpp rospy
  std_msgs
  geometry_msgs
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable( mpu6050_eigen_node src/MPU6050.cpp src/mpu6050_node.cpp )
target_link_libraries( mpu6050_eigen_node ${catkin_LIBRARIES} )

add_executable( stm32_crazepony_node src/stm32_node.cpp src/crazepony.cpp )
target_link_libraries( stm32_crazepony_node ${catkin_LIBRARIES} )
