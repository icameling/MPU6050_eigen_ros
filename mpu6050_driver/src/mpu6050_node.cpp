#include <iostream>
#include "ros/ros.h"
#include "MPU6050.h"
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include "mpu6050_driver/STM32_ImuData.h"

using namespace std;

MPU6050 mpu_6050;

bool usartDataReady = false;
unsigned int last_stamp = 0;
sensor_msgs::Imu imu_msg;
ros::Publisher pos_pub;

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

  q.normalized();

  RPY(0) = atan2(2 * q.y() * q.z() + 2 * q.w() * q.x(), -2 * q.x() * q.x() - 2 * q.y()* q.y() + 1)* 57.3; // roll
  RPY(1) = asin(-2 * q.x() * q.z() + 2 * q.w()* q.y())* 57.3; // pitch
  RPY(2) = atan2(2 * q.x() * q.y() + 2 * q.w() * q.z(), -2 * q.y()*q.y() - 2 * q.z()* q.z() + 1)* 57.3; // yaw

  cout << "stm32 RPY "<< RPY(0) << ' ' << RPY(1) <<' '<< RPY(2) << endl;
}

void publish_pose(void)
{
    geometry_msgs::PoseStamped pose;
    Eigen::Vector3d RPY = mpu_6050.get_RPY();
    Eigen::Quaterniond q = mpu_6050.get_q();

    pose.header.frame_id = "imu-cal";
    pose.header.seq = imu_msg.header.seq;
    pose.pose.position.x = RPY(1);
    pose.pose.position.y = RPY(0);
    pose.pose.position.z = RPY(2);
    pose.pose.orientation.w = q.w();
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pos_pub.publish(pose);

    cout<< "eigen RPY "<<RPY(0)<< ' ' << RPY(1) <<' '<< RPY(2) << endl;

}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "mpu6050_eigen_node");
    ros::NodeHandle n;

    ros::Subscriber imu_sub = n.subscribe("Imu/data", 10, imuCallback);
    ros::Subscriber stm32pose_sub = n.subscribe("STM32_ImuData", 200, stm32poseCallback);

    pos_pub = n.advertise<geometry_msgs::PoseStamped>("stm32_pose_cpp",100);
    ros::Rate loop_rate(500);
    ROS_INFO("hello imu");


    while(ros::ok())
    {

      //cout << "usartDataReady" << usartDataReady << endl;
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

          mpu_6050.estimate_gestureByAccel(linear_acceleration);
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

          mpu_6050.MahonyUpdate(accel, gyro);
          publish_pose();

          last_stamp = imu_msg.header.seq;

        }

      }
      ros::spinOnce();
    }

    return 0;
}
