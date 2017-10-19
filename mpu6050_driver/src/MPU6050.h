#include <iostream>
#include <Eigen/Eigen>




class MPU6050
{
public:
    MPU6050();
    ~MPU6050();

    void reset(void);

    //用mahony互补滤波来解算
    bool MahonyUpdate(Eigen::Vector3d accel, Eigen::Vector3d gyro);
    void Madgwickupdate(void);
    void estimate_gestureByAccel(Eigen::Vector3d accel);

    void set_bias(Eigen::Vector3d accel, Eigen::Vector3d gyro);
    void data_received(Eigen::Vector3d accel, Eigen::Vector3d gyro, unsigned int t);

    Eigen::Vector3d get_RPY(void);
    Eigen::Quaterniond get_q(void);

private:
    //注意Eigen库中的四元数前三维是虚部,最后一维是实部
    Eigen::Quaterniond q;

    unsigned int stamp;
    Eigen::Matrix<double, 3, 1> gyroscope;
    Eigen::Vector3d acceleration;
    Eigen::Vector3d magnetic;
    Eigen::Vector3d RPY;

    Eigen::Vector3d error_sum;
    bool data_ready;      //串口有数据，表示要更新

    //矫正偏差项，如果有偏差项，则角速度和加速度数据要矫正
    //否则用原始数据
    bool mpu6050_bias_initialized;
    Eigen::Vector3d gyro_bias, acc_bias;
};
