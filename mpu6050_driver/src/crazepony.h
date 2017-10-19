#include <iostream>
#include <Eigen/Eigen>


typedef struct IMU_tt
{
  unsigned int caliPass;
  unsigned int ready;

  float 	accRaw[3];		//m/s^2
  float 	gyroRaw[3];		//rad/s
  float 	magRaw[3];		//

  float   accOffset[3];		//m/s^2
  float   gyroOffset[3]; 		//rad/s
  float   accb[3];	//filted,in body frame
  float   gyro[3];	//filted,rad/s

  float   DCMgb[3][3];

  float   roll;		//deg
  float   pitch;
  float 	yaw;

  float   rollRad;	//rad
  float   pitchRad;
  float 	yawRad;
}imu_t;


typedef struct {
  float cutoff_freq;
  float a1;
  float a2;
  float b0;
  float b1;
  float b2;
  float delay_element_1;        // buffered sample -1
  float delay_element_2;        // buffered sample -2
} LPF2p_t;



class Crazepony
{
public:
    Crazepony();
    ~Crazepony();

    void ReadIMUSensorHandle(Eigen::Vector3d accel, Eigen::Vector3d gyro);
    void IMUinit(Eigen::Vector3d acceloff, Eigen::Vector3d gyrooff);

    void LPF2pSetCutoffFreq(LPF2p_t *_lpf2p, float _sample_freq, float _cutoff_freq);
    float LPF2pApply(LPF2p_t *_lpf2p, float _sample);

    void NonlinearSO3AHRSinit(float ax, float ay, float az, float mx, float my, float mz);
    void NonlinearSO3AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float twoKp, float twoKi, float dt);
    void IMUSO3Thread(Eigen::Vector3d accel, Eigen::Vector3d gyro);

    void cout_RPY(void);
    void cout_q(void);
private:
    //注意Eigen库中的四元数前三维是虚部,最后一维是实部
    Eigen::Quaterniond q;
    imu_t imu;
    LPF2p_t LPF2p_array[6];
    unsigned int stamp;
    Eigen::Matrix<double, 3, 1> gyroscope;
    Eigen::Vector3d acceleration;

    //! Auxiliary variables to reduce number of repeated operations
    float q0, q1, q2, q3;	/** quaternion of sensor frame relative to auxiliary frame */
    float dq0, dq1, dq2, dq3;	/** quaternion of sensor frame relative to auxiliary frame */
    float gyro_bias[3]; /** bias estimation */
    float q0q0, q0q1, q0q2, q0q3;
    float q1q1, q1q2, q1q3;
    float q2q2, q2q3;
    float q3q3;
    unsigned int bFilterInit;


};


