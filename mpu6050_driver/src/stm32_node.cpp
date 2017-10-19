#include <iostream>
#include "ros/ros.h"
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include "mpu6050_driver/STM32_ImuData.h"
#include "Eigen/Eigen"

#include "crazepony.h"

using namespace std;

Crazepony crazepony;

bool usartDataReady = false;
unsigned int last_stamp = 0;
sensor_msgs::Imu imu_msg;


void imuCallback(const sensor_msgs::ImuConstPtr& msg)
{
    double t = msg->header.stamp.toSec();

    imu_msg = *msg;
    usartDataReady = true;

    //ROS_INFO("imu call back %d", imu_msg.header.seq);
}

void stm32poseCallback(const mpu6050_driver::STM32_ImuDataConstPtr& msg)
{
  Eigen::Quaterniond q;
  Eigen::Vector3d RPY;

  q.x() = msg->x;
  q.y() = msg->y;
  q.z() = msg->z;
  q.w() = msg->w;

  //cout<<"stm32     q:"<<q.w()<<' '<<q.x()<<' '<<q.y()<<' '<<q.z()<<' '<<endl;;
  q.normalized();

  RPY(0) = atan2(2 * q.y() * q.z() + 2 * q.w() * q.x(), -2 * q.x() * q.x() - 2 * q.y()* q.y() + 1)* 57.3; // roll
  RPY(1) = asin(-2 * q.x() * q.z() + 2 * q.w()* q.y())* 57.3; // pitch
  RPY(2) = atan2(2 * q.x() * q.y() + 2 * q.w() * q.z(), -2 * q.y()*q.y() - 2 * q.z()* q.z() + 1)* 57.3; // yaw

  //cout << "stm32 RPY:"<< RPY(0) << ' ' << RPY(1) <<' '<< RPY(2) << endl;
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "stm32_node");
    ros::NodeHandle n;

    ros::Subscriber imu_sub = n.subscribe("Imu/data", 10, imuCallback);
    ros::Subscriber stm32pose_sub = n.subscribe("STM32_ImuData", 200, stm32poseCallback);

    crazepony.IMUinit(Eigen::Vector3d(0,0,0), Eigen::Vector3d(0,0,0));

    ros::Rate loop_rate(200);

    ROS_INFO("hello stm32_imu");

    while(ros::ok())
    {
      if(usartDataReady)
      {
        //cout << "usartDataReady" << usartDataReady << endl;
        usartDataReady = false;

        if(last_stamp == 0)
        {
          Eigen::Vector3d linear_acceleration;
          linear_acceleration(0) = imu_msg.linear_acceleration.x;
          linear_acceleration(1) = imu_msg.linear_acceleration.y;
          linear_acceleration(2) = imu_msg.linear_acceleration.z;

          //mpu_6050.estimate_gestureByAccel(linear_acceleration);
          last_stamp = imu_msg.header.seq;

          ROS_INFO("last_stamp");

          continue;
        }
        if(last_stamp+1 == imu_msg.header.seq)
        {
          Eigen::Vector3d accel, gyro;
          accel(0) = imu_msg.linear_acceleration.x;
          accel(1) = imu_msg.linear_acceleration.y;
          accel(2) = imu_msg.linear_acceleration.z;
          gyro(0) = imu_msg.angular_velocity.x;
          gyro(1) = imu_msg.angular_velocity.y;
          gyro(2) = imu_msg.angular_velocity.z;


          crazepony.IMUSO3Thread(accel, gyro);
          crazepony.cout_RPY();

          //mpu_6050.MahonyUpdate(accel, gyro);
          //publish_pose();
          //ROS_INFO("haha %d %d", last_stamp,imu_msg.header.seq);
          last_stamp = imu_msg.header.seq;

        }
      }
        ros::spinOnce();
    }

    return 0;
}
