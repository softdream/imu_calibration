#ifndef __IMU_CALIBRATED_DATA_H
#define __IMU_CALIBRATED_DATA_H

#include <Eigen/Dense>

namespace imu
{

template<typename T>
class IMU_Calibrated
{
public:
	using DataType = T;
	using Vector3 = typename Eigen::Matrix<T, 3, 1>;
	using Vector6 = typename Eigen::Matrix<T, 6, 1>;

	static void setAccelerometerParamters( const Vector6& para )
	{
		accelerometer_parameters_ = para;
	}

	static const Vector3 getCalibratedAcceleration( const Vector3 &a )
	{
		return Vector3( ( a[0] - accelerometer_parameters_[3] ) * accelerometer_parameters_[0],
                        ( a[1] - accelerometer_parameters_[4] ) * accelerometer_parameters_[1],
                        ( a[2] - accelerometer_parameters_[5] ) * accelerometer_parameters_[2] );
	}

private:
	static Vector6 accelerometer_parameters_;
	
};

template<typename T>
Eigen::Matrix<T, 6, 1> IMU_Calibrated<T>::accelerometer_parameters_ = Eigen::Matrix<T, 6, 1>::Zero();

}

#endif
