#include "MPU6050.h"

#include <Eigen/Core>
// Eigen 几何模块
#include <Eigen/Geometry>

float InvSqrt(float number)
{
    volatile long i;
    volatile float x, y;
    volatile const float f = 1.5F;

    x = number * 0.5F;
    y = number;
    i = * (( long * ) &y);
    i = 0x5f375a86 - ( i >> 1 );
    y = * (( float * ) &i);
    y = y * ( f - ( x * y * y ) );
    return y;
}

Eigen::Matrix4d get_Rw( Eigen::Vector3d w)
{
    Eigen::Matrix4d Rw;
    Rw << 0,   -w(0),-w(1),-w(2),
          w(0), 0,   -w(2), w(1),
          w(1), w(2), 0,   -w(0),
          w(2),-w(1), w(0), 0;

    return Rw;
}

MPU6050::MPU6050()
{
    data_ready = false;
    mpu6050_bias_initialized = false;
    error_sum.setZero();
    set_bias(Eigen::Vector3d(0,0,0), Eigen::Vector3d(0,0,0));
    q.w() = 1;
    q.x() = 0;
    q.y() = 0;
    q.z() = 0;
}

MPU6050::~MPU6050()
{

}

void MPU6050::reset(void)
{
    data_ready = false;
    mpu6050_bias_initialized = false;
    error_sum.setZero();
}

void MPU6050::set_bias(Eigen::Vector3d accel, Eigen::Vector3d gyro)
{
    acc_bias = accel;
    gyro_bias = gyro;
    mpu6050_bias_initialized = true;
}


void MPU6050::data_received(Eigen::Vector3d accel, Eigen::Vector3d gyro, unsigned int t)
{
    acceleration = accel;
    gyroscope = gyro;
    stamp = t;
}

Eigen::Vector3d MPU6050::get_RPY(void)
{
  return RPY;
}

Eigen::Quaterniond MPU6050::get_q(void)
{
  return q;
}

void MPU6050::estimate_gestureByAccel(Eigen::Vector3d accel)
{
    if(mpu6050_bias_initialized)
    {
      accel -= acc_bias;
    }
    //用加速度计数据来估计初始状态
}
bool MPU6050::MahonyUpdate(Eigen::Vector3d accel, Eigen::Vector3d gyro)
{
    double dt = 0.005; //s
    double Kp = 0.5, Ki = 0.025;

    if(mpu6050_bias_initialized)
    {
      gyro -= gyro_bias;
      //accel -= acc_bias;
    }


    if( !((accel(0) == 0.0f) && (accel(1) == 0.0f) && (accel(2) == 0.0f)) )
    {
        //这里参考向量是重力，假设物体只受重力加速度，忽略其他加速度
        //先把重力g_navigation（0,0,1）转换到载体坐标系g_b
        //g_b再与在载体坐标系下测得的accel对比

        accel = -accel;
        accel /= accel.norm();


        Eigen::Matrix3d rotation_matrix_b2n(q);
        Eigen::Vector3d g_n(0.0, 0.0, 1.0);
        Eigen::Vector3d g_b = rotation_matrix_b2n.transpose()* g_n;
        Eigen::Vector3d error = g_b.cross(accel);

        error_sum = error*Ki;

        gyro += error*Kp + error_sum;
    }

    float qw = q.w(), qx = q.x(), qy = q.y(), qz = q.z();
    //后面等式的+=中，q的数据已经变了，所以上面定义四个变量存储
    q.w() += 0.5*dt*(-qx * gyro(0) - qy * gyro(1) - qz * gyro(2));
    q.x() += 0.5*dt*(qw * gyro(0) + qy * gyro(2) - qz * gyro(1));
    q.y() += 0.5*dt*(qw * gyro(1) - qx * gyro(2) + qz * gyro(0));
    q.z() += 0.5*dt*(qw * gyro(2) + qx * gyro(1) - qy * gyro(0));

    q.normalized();

    RPY(0) = atan2(2 * q.y() * q.z() + 2 * q.w() * q.x(), -2 * q.x() * q.x() - 2 * q.y()* q.y() + 1)* 57.3; // roll
    RPY(1) = asin(-2 * q.x() * q.z() + 2 * q.w()* q.y())* 57.3; // pitch
    RPY(2) = atan2(2 * q.x() * q.y() + 2 * q.w() * q.z(), -2 * q.y()*q.y() - 2 * q.z()* q.z() + 1)* 57.3; // yaw

    if(RPY(2) >= 0.0f)
      RPY(2) = 180 - RPY(2);
    else
      RPY(2) = -180 - RPY(2);


    return true;
}
