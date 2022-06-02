#ifndef __IMU_CALIBRATION_H
#define __IMU_CALIBRATION_H

#include <iostream>
#include <vector>
#include <Eigen/Dense>

namespace imu
{

template<typename T>
class Accelerometer
{
public:
	using DataType = T;
	using Vector3 = typename Eigen::Matrix<T, 3, 1>;
	using Matrix3 = typename Eigen::Matrix<T, 3, 3>;
	using Vector6 = typename Eigen::Matrix<T, 6, 1>;
	using Matrix6 = typename Eigen::Matrix<T, 6, 6>;

	Accelerometer()
	{

	}

	~Accelerometer()
	{

	}

	bool estimateAccelerometerParameters( const std::vector<Vector3> &accelerations, const int max_iterations = 100 )
	{
		if( accelerations.size() < 6 ){
			std::cerr<<"Input data is fewer than required !"<<std::endl;
			return false;
		}	

		int iteration = 0;
		for( ; iteration < max_iterations; iteration ++ ){
			estimateOnce( accelerations );
		}

		return true;
	}

	const Vector6& getEstimatedParameters() const
	{
		return parameters;
	}

private:
	void estimateOnce( const std::vector<Vector3> &accelerations )
	{
		getDerivedHessianAndB( accelerations );

		Vector6 delta = -H.inverse() * b;
	
		parameters += delta;
	}

	void getDerivedHessianAndB( const std::vector<Vector3> &accelerations )
	{
		H.setZero();
		b.setZero();
	
		// for all input accelaration data
		for( size_t i = 0; i < accelerations.size(); i ++ ){
			std::cout<<"--------------------------- Data: "<<i<<" --------------------"<<std::endl;
			DataType bx = parameters[3]; // bias x
			DataType by = parameters[4]; // bias y
			DataType bz = parameters[5]; // bias z
		
			DataType sx = parameters[0]; // scale x
			DataType sy = parameters[1]; // scale y
			DataType sz = parameters[2]; // scale z

			DataType d_x = accelerations[i](0) - bx; // bias x
			DataType d_y = accelerations[i](1) - by;
			DataType d_z = accelerations[i](2) - bz;
	
			DataType error = g - sx * sx * d_x * d_x 
					   - sy * sy * d_y * d_y
					   - sz * sz * d_z * d_z;
		
			std::cout<<"error = "<<error<<std::endl;

			Vector6 Jacobian( -2 * sx * d_x * d_x,
                                          -2 * sy * d_y * d_y, 
                                          -2 * sz * d_z * d_z, 
                                           2 * sx * sx * d_x, 
                                           2 * sy * sy * d_y, 
                                           2 * sz * sz * d_z );
			
			std::cout<<"Jacobian = "<<std::endl<<Jacobian<<std::endl;		

			b += Jacobian * error();
			H += Jacobian * Jacobian.transpose();
		}
		
		std::cout<<"H = "<<std::endl<<H<<std::endl;
		std::cout<<"b = "<<std::endl<<b<<std::endl;
	}

private:
	// Error parameters
	Vector6 parameters;

	// Nonlinear least squares' parameters
	Matrix6 H;
	Vector6 b;

	// Acceleration of gravity
	DataType g = 9.81;
};

}


#endif
