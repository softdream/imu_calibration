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

	bool estimateGyrometerParameters( const int max_iterations = 100 )
	{
		if( rotation_vecs.empty() || a_pre_data.empty() || a_now_data.empty() ){
			std::cerr<<"Input data is empty ..."<<std::endl;
			return false;
		}

		if( rotation_vecs.size() != a_pre_data.empty() || rotation_vecs.size() != a_now_data.empty() || a_pre_data.size() != a_now_data.size() ){
			std::cerr<<"Data size is not matched !"<<std::endl;
			return false;
		}

		int iteration = 0;
                for( ; iteration < max_iterations; iteration ++ ){
                        estimateOnce( );
                }

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
		for( size_t i = 0; i < rotation_vecs.size(); i ++ ){
			// 1. caculate error
			Vector3 R_vector = Vector3( rotation_vecs(0) * scales(0),
						    rotation_vecs(1) * scales(1),
						    rotation_vecs(2) * scales(2) );

			// 2. rotation vector -> rotation matrix
			Vector3 R_matrix = Eigen::AngleAxis<DataType>( R_vector.norm(), R_vector / R_vector.norm() ).toRotationMatrix();		

			// 3. caculate the error
			Vector3 error = a_now_data[i] - R_matrix * a_pre_data[i];
		
			// 4. caculate the Jacobian Matrix
			Matrix3 derived_Ra = getAntisymmetricMatrix( R_matrix * a_pre_data[i] );
			Matrix3 derived_s = Matrix3::Zero();
			derived_s( 0, 0 ) = R_vector(0);
			derived_s( 1, 1 ) = R_vector(1);
			derived_s( 2, 2 ) = R_vector(2);

			Matrix3 Jacobian = derived_Ra * derived_s;

			H += Jacobian.transpose() * Jacobian;
			b += Jacobian.transpose() * error;
		}	
		std::cout<<"H = "<<std::endl<<H<<std::endl;
		std::cout<<"b = "<<std::endl<<b<<std::endl;
	}	

	
	const Matrix3 getAntisymmetricMatrix( const Vector3 &vec ) const
	{
		Matrix3 mat;
		mat <<  0,      -vec(2),    vec(1),
		        vec(2),    0,      -vec(0),
		       -vec(1),  vec(0),      0;

		return mat;
	}

protected:
	// Error paramters
	Vector3 scales = Vector3( 1.0, 1.0, 1.0 ); // sx, sy, sz; the state of the solution
	Vector3 biases = Vector3( 0.0, 0.0, 0.0 ); // bx, by, bz

	// input data
	std::vector<Vector3> rotation_vecs;
	std::vector<Vector3> a_pre_data; // a_k_1, acceleration at k-1 moment
	std::vector<Vector3> a_now_data; // a_k	
	

	Vector3 b;
	Matrix3 H;
};

template<typename T>
class CalibrateGyrometer : public Gyrometer<T>
{
public:
	using DataType = T;
        using Vector3 = typename Eigen::Matrix<T, 3, 1>;
	using Vector4 = typename Eigen::Matrix<T, 4, 1>;
	using Vector6 = typename Eigen::Matrix<T, 6, 1>;
        using Matrix3 = typename Eigen::Matrix<T, 3, 3>;

	CalibrateGyrometer()
	{

	}

	~CalibrateGyrometer()
	{

	}

	const Vector6& operator()( const std::vector<Vector4> &data1,
				   const std::vector<Vector4> &data2,
				   const std::vector<Vector4> &data3,
				   const std::vector<Vector4> &data4,
				   const std::vector<Vector3> pre_acc,
				   const std::vector<Vector3> now_acc )
	{
		for( size_t i = 0; i < data1.size(); i ++ ){
			Vector3 w = Vector3( data1[i](1), data1[i](2), data1[i](3) );
			this->biases += w;
		}
	
		this->biases / data1.size();
	
		Vector3 r_v( 0.0, 0.0, 0.0 );
		DataType pre_time = 0;
		Vector3 pre_w( data2[0](1) - this->biases(0), 
			       data2[0](2) - this->biases(1), 
			       data2[0](3) - this->biases(2) );
		
		// caculate the rotation by data1
		for( size_t i = 1; i < data2.size(); i ++ ){
			Vector3 w = Vector3( data2[i](1) - this->biases(0), 
					     data2[i](2) - this->biases(1), 
					     data2[i](3) - this->biases(2) );
	
			DataType delta_t = data2[i](0) - pre_time;
			Vector3 w_m = 0.5 * ( w + pre_w ) * delta_t; 
				
			r_v += w_m; // intergrate
		
			pre_w = w;
			pre_time = data2[i](0);
		}
		this->rotation_vecs.push_back( r_v );

		r_v.setZero();
		pre_time = 0;
		pre_w = Vector3( data3[0](1) - this->biases(0), 
				 data3[0](2) - this->biases(1), 
				 data3[0](3) - this->biases(2) );
		// caculate the rotation by data3
                for( size_t i = 1; i < data3.size(); i ++ ){
                        Vector3 w = Vector3( data3[i](1) - this->biases(0), 
                                             data3[i](2) - this->biases(1), 
                                             data3[i](3) - this->biases(2) );

                        DataType delta_t = data3[i](0) - pre_time;
                        Vector3 w_m = 0.5 * ( w + pre_w ) * delta_t;

                        r_v += w_m; // intergrate

                        pre_w = w;
			pre_time = data3[i](0);
                }
		this->rotation_vecs.push_back( r_v );
		
		r_v.setZero();
                pre_time = 0;
                pre_w = Vector3( data4[0](1) - this->biases(0),
                                 data4[0](2) - this->biases(1),
                                 data4[0](3) - this->biases(2) );
                // caculate the rotation by data4
                for( size_t i = 1; i < data4.size(); i ++ ){
                        Vector3 w = Vector3( data4[i](1) - this->biases(0),
                                             data4[i](2) - this->biases(1),
                                             data4[i](3) - this->biases(2) );

                        DataType delta_t = data4[i](0) - pre_time;
                        Vector3 w_m = 0.5 * ( w + pre_w ) * delta_t;

                        r_v += w_m; // intergrate

                        pre_w = w;
                        pre_time = data4[i](0);
                }
		this->rotation_vecs.push_back( r_v );
		
		this->a_pre_data = pre_acc;
		this->a_now_data = now_acc;
		this->estimateGyrometerParameters();	

		Vector6 ret;
		ret.block( 0, 0, 3, 1 ) = this->biases;
		ret.block( 3, 0, 3, 1 ) = this->scales;
		
		return ret;
	}

};


}


#endif
