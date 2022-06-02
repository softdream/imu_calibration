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

			b += Jacobian * error;
			H += Jacobian * Jacobian.transpose();
		}
		
		std::cout<<"H = "<<std::endl<<H<<std::endl;
		std::cout<<"b = "<<std::endl<<b<<std::endl;
	}

protected:
	// Error parameters
	Vector6 parameters;

	// Nonlinear least squares' parameters
	Matrix6 H;
	Vector6 b;

	// Acceleration of gravity
	DataType g = 9.81;
};

template<typename T>
class CalibrateAccelerometer : public Accelerometer<T>
{
public:
	using DataType = T;
        using Vector3 = typename Eigen::Matrix<T, 3, 1>;
        using Matrix3 = typename Eigen::Matrix<T, 3, 3>;
        using Vector6 = typename Eigen::Matrix<T, 6, 1>;
        using Matrix6 = typename Eigen::Matrix<T, 6, 6>;
	
	CalibrateAccelerometer()
	{
	
	}

	~CalibrateAccelerometer()
	{

	}

	const Vector6& operator()( const std::vector<Vector3> &data1,
				 const std::vector<Vector3> &data2,
				 const std::vector<Vector3> &data3,
				 const std::vector<Vector3> &data4,
				 const std::vector<Vector3> &data5, 
                                 const std::vector<Vector3> &data6 )
	{
		Vector3 data1_average = Vector3::Zero();
		Vector3 data2_average = Vector3::Zero();
		Vector3 data3_average = Vector3::Zero();
		Vector3 data4_average = Vector3::Zero();
		Vector3 data5_average = Vector3::Zero();
		Vector3 data6_average = Vector3::Zero();
		
		std::vector<Vector3> averages_input;

		// 1. caculate the average 
		for( size_t i = 0; i < data1.size(); i ++ ){
			data1_average += data1[i];
		}
		data1_average /= data1.size();

		averages_input.push_back( data1_average );
		
		for( size_t i = 0; i < data2.size(); i ++ ){
                        data2_average += data2[i];
                }
                data2_average /= data2.size();

		averages_input.push_back( data2_average );

		for( size_t i = 0; i < data3.size(); i ++ ){
                        data3_average += data3[i];
                }
                data3_average /= data3.size();

		averages_input.push_back( data3_average );

		for( size_t i = 0; i < data4.size(); i ++ ){
                        data4_average += data4[i];
                }
                data4_average /= data4.size();

		averages_input.push_back( data4_average );	

		for( size_t i = 0; i < data5.size(); i ++ ){
                        data5_average += data5[i];
                }
                data5_average /= data5.size();

		averages_input.push_back( data5_average );

		for( size_t i = 0; i < data6.size(); i ++ ){
                        data6_average += data6[i];
                }
                data6_average /= data6.size();

		averages_input.push_back( data6_average );

		// 2. estimate the parameters
		this->estimateAccelerometerParameters( averages_input );	
		
		// 3. get the result
		return this->getEstimatedParameters();
	}

};

template<typename T>
class Gyrometer
{
public:
	using DataType = T;
        using Vector3 = typename Eigen::Matrix<T, 3, 1>;
        using Matrix3 = typename Eigen::Matrix<T, 3, 3>;
	
	Gyrometer()	
	{

	}

	~Gyrometer()
	{
	
	}

	bool setBiases( const Vector3 &biases_ )
	{
		biases = biases_;
		return true;
	}

	bool setBiases( const DataType bx, const DataType by, const DataType bz )
        {
                biases = Vector3( bx, by, bz );
                return true;
        }

private:
	void estimateOnce()
	{
		getDerivedHessianAndB();
		
		Vector3 delta = -H.inverse() * b;
		
		scales += delta;
	}
	
	void getDerivedHessianAndB()
	{
		b.setZero();
		H.setZero();
		
		// 
		for( size_t i = 0; i < w_pre_data.size(); i ++ ){
			// 1. caculate error
			Vector3 R_vector_tmp = caculateRotationVector( w_pre_data[i], w_now_data[i], delta_t );
			Vector3 R_vector = Vector3( R_vector_tmp(0) * scales(0),
						    R_vector_tmp(1) * scales(1),
						    R_vector_tmp(2) * scales(2) );
			Vector3 R_matrix = Eigen::AngleAxis<DataType>( R_vector.norm(), R_vector / R_vector.norm() ).toRotationMatrix();		

			Vector3 error = a_now_data[i] - R_matrix * a_pre_data[i];
		
			// 2. caculate the Jacobian Matrix
			Matrix3 derived_Ra = getAntisymmetricMatrix( R_matrix * a_pre_data[i] );
			Matrix3 derived_s = Matrix3::Zero();
			derived_s( 0, 0 ) = R_vector_tmp(0);
			derived_s( 1, 1 ) = R_vector_tmp(1);
			derived_s( 2, 2 ) = R_vector_tmp(2);

			Matrix3 Jacobian = derived_Ra * derived_s;

			H += Jacobian.transpose() * Jacobian;
			b += Jacobian.transpose() * error;
		}	
		std::cout<<"H = "<<std::endl<<H<<std::endl;
		std::cout<<"b = "<<std::endl<<b<<std::endl;
	}	

	const Vector3 caculateRotationVector( const Vector3 &w_pre,
					      const Vector3 &w_now,
					      const DataType delta_t )
	{
		return Vector3( ( w_pre(0) + w_now(0) - 2 * biases(0) ) * 0.5 * delta_t,
		       		( w_pre(1) + w_now(1) - 2 * biases(1) ) * 0.5 * delta_t,
				( w_pre(2) + w_now(2) - 2 * biases(2) ) * 0.5 * delta_t );
	}
	
	const Matrix3 getAntisymmetricMatrix( const Vector3 &vec )
	{
		Matrix3 mat;
		mat <<  0,      -vec(2),    vec(1),
		        vec(2),    0,      -vec(0),
		       -vec(1),  vec(0),      0;
		return mat;
	}

private:
	// Error paramters
	Vector3 scales; // sx, sy, sz; the state of the solution
	Vector3 biases; // bx, by, bz

	// input data
	std::vector<Vector3> w_pre_data; // w_k-1, angle velocity at k-1 moment
	std::vector<Vector3> w_now_data; // w_k
	std::vector<Vector3> a_pre_data; // a_k_1, acceleration at k-1 moment
	std::vector<Vector3> a_now_data; // a_k	
	
	DataType delta_t = 0;

	Vector3 b;
	Matrix3 H;
};

}


#endif
