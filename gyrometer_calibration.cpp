#include "imu_calibration.h"
#include "file_read.h"

int main()
{
	std::cout<<"------------------ GyroMeter ------------------"<<std::endl;
	
	imu::ImuRead imu_read1;

	imu_read1.openFile( "gyro.txt" );
	
	std::vector<Eigen::Vector4d> gyros;
	while( !imu_read1.endOfFile() ){
		Eigen::Vector4d gyro_data;
		imu_read1.readGyrometerFrame( gyro_data );
	
		gyros.push_back( gyro_data );
	}
	imu_read1.closeFile();

	// -----------------------------------------------------
	imu::ImuRead imu_read2;

        imu_read2.openFile( "gyro_rotation1.txt" );

        std::vector<Eigen::Vector4d> gyros_rotaion1;
        while( !imu_read2.endOfFile() ){
                Eigen::Vector4d gyro_data;
		if( imu_read2.readGyrometerFrame( gyro_data ) ){
			gyros_rotaion1.push_back( gyro_data );
		}
        }
        imu_read2.closeFile();

	//for( size_t i = 0; i < gyros_rotaion1.size(); i ++ ){
	//	std::cout<<"gyro : "<<i<<std::endl<<gyros_rotaion1[i]<<std::endl;
	//}

	// ----------------------------------------------------- 

	imu::ImuRead imu_read3;

        imu_read3.openFile( "gyro_rotation2.txt" );

        std::vector<Eigen::Vector4d> gyros_rotaion2;
        while( !imu_read3.endOfFile() ){
                Eigen::Vector4d gyro_data;
		if( imu_read3.readGyrometerFrame( gyro_data ) ){
                        gyros_rotaion2.push_back( gyro_data );
                }
        }
        imu_read3.closeFile();
	//for( size_t i = 0; i < gyros_rotaion2.size(); i ++ ){
        //      std::cout<<"gyro : "<<i<<std::endl<<gyros_rotaion2[i]<<std::endl;
        //}

	// -----------------------------------------------------
	imu::ImuRead imu_read4;

        imu_read4.openFile( "gyro_rotation3.txt" );

        std::vector<Eigen::Vector4d> gyros_rotaion3;
        while( !imu_read4.endOfFile() ){
                Eigen::Vector4d gyro_data;
        	if( imu_read4.readGyrometerFrame( gyro_data ) ){
                        gyros_rotaion3.push_back( gyro_data );
                }
	}
        imu_read4.closeFile();
	//for( size_t i = 0; i < gyros_rotaion3.size(); i ++ ){
        //      std::cout<<"gyro : "<<i<<std::endl<<gyros_rotaion3[i]<<std::endl;
        //}
        // -----------------------------------------------------

	std::vector<Eigen::Vector3d> pre_accs;
        std::vector<Eigen::Vector3d> now_accs;

        pre_accs.push_back( Eigen::Vector3d( 0, 0, 1 ) );
        pre_accs.push_back( Eigen::Vector3d( 0, 0, 1 ) );
        pre_accs.push_back( Eigen::Vector3d( 0, 0, 1 ) );

        now_accs.push_back( Eigen::Vector3d( 0, -1, 0 ) );
        now_accs.push_back( Eigen::Vector3d( -1, 0, 0 ) );
        now_accs.push_back( Eigen::Vector3d( 0, 1, 0 ) );
	
	imu::CalibrateGyrometer<double>()( gyros, gyros_rotaion1, gyros_rotaion2, gyros_rotaion3, pre_accs, now_accs );

	return 0;
}
