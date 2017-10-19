#include <iostream>
#include "crazepony.h"

#define IMU_SAMPLE_RATE 			200.0f    //100.0f
#define IMU_FILTER_CUTOFF_FREQ	    30.0f

#define CONSTANTS_ONE_G				9.80665f  //m/s^2
#define GYRO_CALC_TIME   3000000l	//us

#define SENSOR_MAX_G 8.0f		//constant g		// tobe fixed to 8g. but IMU need to  correct at the same time
#define SENSOR_MAX_W 2000.0f	//deg/s
#define ACC_SCALE  (SENSOR_MAX_G/32768.0f)
#define GYRO_SCALE  (SENSOR_MAX_W/32768.0f)

#define so3_comp_params_Kp 1.0f
#define so3_comp_params_Ki  0.05f



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


Crazepony::Crazepony(void)
{
    imu.ready = 0;
    imu.caliPass = 0;

    q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
    dq0 = 0.0f, dq1 = 0.0f, dq2 = 0.0f, dq3 = 0.0f;
    gyro_bias[1] = 0.0f;
    gyro_bias[2] = 0.0f;
    bFilterInit = 0;
}

Crazepony::~Crazepony(void)
{

}

void Crazepony::ReadIMUSensorHandle(Eigen::Vector3d accel, Eigen::Vector3d gyro)
{
  for(int i=0;i<3;i++)
  {
    //imu.accRaw[i]  = (float)imu.accADC[i]  * ACC_SCALE  * CONSTANTS_ONE_G ;
    //imu.gyroRaw[i] = (float)imu.gyroADC[i] * GYRO_SCALE * M_PI_F /180.f;		//deg/s
    //imu.accRaw[i] = accel(i);
    //imu.gyroRaw[i] = gyro(i);
    imu.accb[i] = accel(i);
    imu.gyro[i] = gyro(i);
  }

//  imu.accb[0] = LPF2pApply(&LPF2p_array[0], imu.accRaw[0]-imu.accOffset[0]);
//  imu.accb[1] = LPF2pApply(&LPF2p_array[1], imu.accRaw[1]-imu.accOffset[1]);
//  imu.accb[2] = LPF2pApply(&LPF2p_array[2], imu.accRaw[2]-imu.accOffset[2]);

//  imu.gyro[0] = LPF2pApply(&LPF2p_array[3], imu.gyroRaw[0]);
//  imu.gyro[1] = LPF2pApply(&LPF2p_array[4], imu.gyroRaw[1]);
//  imu.gyro[2] = LPF2pApply(&LPF2p_array[5], imu.gyroRaw[2]);
}



void Crazepony::IMUinit(Eigen::Vector3d acceloff, Eigen::Vector3d gyrooff)
{
    imu.caliPass = 1;

    //低通滤波初始化
    for(int i = 0; i < 6; i++)
      LPF2pSetCutoffFreq( &LPF2p_array[i], IMU_SAMPLE_RATE, IMU_FILTER_CUTOFF_FREQ);//30Hz

    //读3000次取平均作为偏移值
    for(int i = 0; i < 3; i++)
    {
      imu.gyroOffset[i] = acceloff(i);
      imu.accOffset[i] = gyrooff(i);
    }

    imu.accOffset[2] = imu.accOffset[2] - CONSTANTS_ONE_G;
    imu.caliPass = 1;
    imu.ready = 1;

}


//! Using accelerometer, sense the gravity vector.
//! Using magnetometer, sense yaw.
void Crazepony::NonlinearSO3AHRSinit(float ax, float ay, float az, float mx, float my, float mz)
{
    float initialRoll, initialPitch;
    float cosRoll, sinRoll, cosPitch, sinPitch;
    float magX, magY;
    float initialHdg, cosHeading, sinHeading;

    initialRoll = atan2(-ay, -az);
    initialPitch = atan2(ax, -az);

    cosRoll = cosf(initialRoll);
    sinRoll = sinf(initialRoll);
    cosPitch = cosf(initialPitch);
    sinPitch = sinf(initialPitch);

    magX = mx * cosPitch + my * sinRoll * sinPitch + mz * cosRoll * sinPitch;

    magY = my * cosRoll - mz * sinRoll;

    initialHdg = atan2f(-magY, magX);


    cosRoll = cosf(initialRoll * 0.5f);
    sinRoll = sinf(initialRoll * 0.5f);

    cosPitch = cosf(initialPitch * 0.5f);
    sinPitch = sinf(initialPitch * 0.5f);

    cosHeading = cosf(initialHdg * 0.5f);
    sinHeading = sinf(initialHdg * 0.5f);

    q0 = cosRoll * cosPitch * cosHeading + sinRoll * sinPitch * sinHeading;
    q1 = sinRoll * cosPitch * cosHeading - cosRoll * sinPitch * sinHeading;
    q2 = cosRoll * sinPitch * cosHeading + sinRoll * cosPitch * sinHeading;
    q3 = cosRoll * cosPitch * sinHeading - sinRoll * sinPitch * cosHeading;

    // auxillary variables to reduce number of repeated operations, for 1st pass
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;
}


void Crazepony::NonlinearSO3AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float twoKp, float twoKi, float dt)
{
    float recipNorm;
    float halfex = 0.0f, halfey = 0.0f, halfez = 0.0f;

    // Make filter converge to initial solution faster
    // This function assumes you are in static position.
    // WARNING : in case air reboot, this can cause problem. But this is very unlikely happen.
    if(bFilterInit == 0) {
      NonlinearSO3AHRSinit(ax,ay,az,mx,my,mz);
      bFilterInit = 1;
    }


    //0.75*G < normAcc < 1.25*G
    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
      float halfvx, halfvy, halfvz;
      float accNorm=0;

      // Normalise accelerometer measurement
      recipNorm = InvSqrt(ax * ax + ay * ay + az * az);


      //--added!!!
      accNorm=1.0f/recipNorm;


      //std::cout<<"accNorm  "<<accNorm<<std::endl;

      //std::cout<<"a "<<ax<<' '<<ay<<' '<<az<<std::endl;
      if(accNorm > 0.75 * CONSTANTS_ONE_G && accNorm < 1.25 * CONSTANTS_ONE_G )
      {
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;


        // Estimated direction of gravity and magnetic field
        halfvx = q1q3 - q0q2;
        halfvy = q0q1 + q2q3;
        halfvz = q0q0 - 0.5f + q3q3;


        // Error is sum of cross product between estimated direction and measured direction of field vectors
        halfex += ay * halfvz - az * halfvy;
        halfey += az * halfvx - ax * halfvz;
        halfez += ax * halfvy - ay * halfvx;
      }
    }

    // Apply feedback only when valid data has been gathered from the accelerometer or magnetometer
    if(halfex != 0.0f && halfey != 0.0f && halfez != 0.0f) {
      // Compute and apply integral feedback if enabled
      if(twoKi > 0.0f) {


        gyro_bias[0] += twoKi * halfex * dt;	// integral error scaled by Ki
        gyro_bias[1] += twoKi * halfey * dt;
        gyro_bias[2] += twoKi * halfez * dt;

        // apply integral feedback
        gx += gyro_bias[0];
        gy += gyro_bias[1];
        gz += gyro_bias[2];
      }
      else {
        gyro_bias[0] = 0.0f;	// prevent integral windup
        gyro_bias[1] = 0.0f;
        gyro_bias[2] = 0.0f;
      }

      // Apply proportional feedback
      gx += twoKp * halfex;
      gy += twoKp * halfey;
      gz += twoKp * halfez;
    }


    // Time derivative of quaternion. q_dot = 0.5*q\otimes omega.
    //! q_k = q_{k-1} + dt*\dot{q}
    //! \dot{q} = 0.5*q \otimes P(\omega)
    dq0 = 0.5f*(-q1 * gx - q2 * gy - q3 * gz);
    dq1 = 0.5f*(q0 * gx + q2 * gz - q3 * gy);
    dq2 = 0.5f*(q0 * gy - q1 * gz + q3 * gx);
    dq3 = 0.5f*(q0 * gz + q1 * gy - q2 * gx);

    q0 += dt*dq0;
    q1 += dt*dq1;
    q2 += dt*dq2;
    q3 += dt*dq3;

    // Normalise quaternion
    recipNorm = InvSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;
}


//每隔0.005s调用更新函数
void Crazepony::IMUSO3Thread(Eigen::Vector3d accel, Eigen::Vector3d gyro)
{
  //! Time constant
  float dt = 0.005f;		    //s

  /* output euler angles */
  float euler[3] = {0.0f, 0.0f, 0.0f};	//rad

  /* Initialization */
  float Rot_matrix[9] = {1.f,  0.0f,  0.0f, 0.0f,  1.f,  0.0f, 0.0f,  0.0f,  1.f };		/**< init: identity matrix */
  float acc[3] = {0.0f, 0.0f, 0.0f};		//m/s^2
  float gyr[3] = {0.0f, 0.0f, 0.0f};		//rad/s
  float mag[3] = {0.0f, 0.0f, 0.0f};

  float Kp = so3_comp_params_Kp;
  float Ki = so3_comp_params_Ki;

  ReadIMUSensorHandle(accel, gyro);

  gyr[0] = imu.gyro[0];// - imu.gyroOffset[0];
  gyr[1] = imu.gyro[1];// - imu.gyroOffset[1];
  gyr[2] = imu.gyro[2];// - imu.gyroOffset[2];

  acc[0] = -imu.accb[0];
  acc[1] = -imu.accb[1];
  acc[2] = -imu.accb[2];


  NonlinearSO3AHRSupdate(gyr[0], gyr[1], gyr[2],
      -acc[0], -acc[1], -acc[2],
      mag[0], mag[1], mag[2],
      Kp,
      Ki,
      dt);

  Rot_matrix[0] = q0q0 + q1q1 - q2q2 - q3q3;// 11
  Rot_matrix[1] = 2.f * (q1*q2 + q0*q3);	// 12
  Rot_matrix[2] = 2.f * (q1*q3 - q0*q2);	// 13
  Rot_matrix[3] = 2.f * (q1*q2 - q0*q3);	// 21
  Rot_matrix[4] = q0q0 - q1q1 + q2q2 - q3q3;// 22   //............................
  Rot_matrix[5] = 2.f * (q2*q3 + q0*q1);	// 23
  Rot_matrix[6] = 2.f * (q1*q3 + q0*q2);	// 31
  Rot_matrix[7] = 2.f * (q2*q3 - q0*q1);	// 32
  Rot_matrix[8] = q0q0 - q1q1 - q2q2 + q3q3;// 33   //..............................

  //1-2-3 Representation.
  //Equation (290)
  //Representing Attitude: Euler Angles, Unit Quaternions, and Rotation Vectors, James Diebel.
  // Existing PX4 EKF code was generated by MATLAB which uses coloum major order matrix.
    euler[0] = atan2f(Rot_matrix[5], Rot_matrix[8]);	//! Roll(rad)
    euler[1] = -asinf(Rot_matrix[2]);					//! Pitch
    euler[2] = atan2f(Rot_matrix[1], Rot_matrix[0]);

    //DCM . ground to body
    for(int i=0;i<9;i++)
    {
      *(&(imu.DCMgb[0][0]) + i) = Rot_matrix[i];
    }

    imu.rollRad = euler[0];
    imu.pitchRad = euler[1];
    imu.yawRad = euler[2];

    imu.roll = euler[0] * 180.0f / M_PI;
    imu.pitch = euler[1] * 180.0f / M_PI;
    imu.yaw = euler[2] * 180.0f / M_PI;


  if(imu.yaw >= 0.0f)
    imu.yaw = 180-imu.yaw;
  else
    imu.yaw = -180-imu.yaw;



}

void Crazepony::cout_RPY(void)
{
   std::cout << "craze RPY:" << imu.roll << ' '<< imu.pitch << ' '<<imu.yaw<<std::endl;
}

void Crazepony::cout_q(void)
{
  std::cout<<"crazepony q:"<<q0<<' '<<q1<<' '<<q2<<' '<<q3<<std::endl;
}

void Crazepony::LPF2pSetCutoffFreq(LPF2p_t *_lpf2p, float _sample_freq, float _cutoff_freq)
{
    float fr =0;
    float ohm =0;
    float c =0;

    fr = _sample_freq / _cutoff_freq;
    ohm = tanf(M_PI / fr);
    c = 1.0f + 2.0f*cosf(M_PI/4.0f) * ohm + ohm*ohm;

    _lpf2p->cutoff_freq = _cutoff_freq;

    if (_lpf2p->cutoff_freq > 0.0f)
    {
        _lpf2p->b0 = ohm*ohm/c;
        _lpf2p->b1 = 2.0f*_lpf2p->b0;
        _lpf2p->b2 = _lpf2p->b0;
        _lpf2p->a1 = 2.0f*(ohm*ohm-1.0f)/c;
        _lpf2p->a2 = (1.0f-2.0f*cosf(M_PI/4.0f)*ohm+ohm*ohm)/c;
    }
    _lpf2p->delay_element_1 = 0.0f;
    _lpf2p->delay_element_2 = 0.0f;
}

float Crazepony::LPF2pApply(LPF2p_t *_lpf2p, float _sample)
{
    float delay_element_0 = 0, output=0;
    if (_lpf2p->cutoff_freq <= 0.0f) {
        // no filtering
        return _sample;
    }
    else
    {
        delay_element_0 = _sample - _lpf2p->delay_element_1 * _lpf2p->a1 - _lpf2p->delay_element_2 * _lpf2p->a2;
        // do the filtering
        if (isnan(delay_element_0) || isinf(delay_element_0)) {
            // don't allow bad values to propogate via the filter
            delay_element_0 = _sample;
        }
        output = delay_element_0 * _lpf2p->b0 + _lpf2p->delay_element_1 * _lpf2p->b1 + _lpf2p->delay_element_2 * _lpf2p->b2;

        _lpf2p->delay_element_2 = _lpf2p->delay_element_1;
        _lpf2p->delay_element_1 = delay_element_0;

        // return the value.  Should be no need to check limits
        return output;
    }
}


