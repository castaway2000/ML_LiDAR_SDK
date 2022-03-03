#include<iostream>
#include <memory>

#ifdef _WIN32
#include<direct.h>
#include<io.h>
#endif // _WIN32
#ifdef __linux__
#include<sys/stat.h>
#endif

#include"ml/libsoslab_ml.h"

int main()
{
	bool success;
	/* LidarML 객체를 생성합니다. */
	std::shared_ptr<SOSLAB::LidarMl> lidar_ml(new SOSLAB::LidarMl);	
	
	/* 연결 정보를 활용하여 장치에 연결합니다. */
 	SOSLAB::ip_settings_t ip_settings_device;
	SOSLAB::ip_settings_t ip_settings_pc;
	ip_settings_pc.ip_address = "0.0.0.0"; 
	ip_settings_pc.port_number = 2000;
	ip_settings_device.ip_address = "192.168.1.10";
	ip_settings_device.port_number = 2000;
	success = lidar_ml->connect(ip_settings_device, ip_settings_pc);
	if (!success) {
		std::cerr << "LiDAR ML :: connection failed." << std::endl;
		return 0;
	}

	/* 데이터 스트리밍을 시작 합니다. */
	success = lidar_ml->tcp_device_run();
	if (!success) {
		std::cerr << "LiDAR ML :: start failed." << std::endl;
		return 0;
	}
	std::cout << "LiDAR ML :: Streaming started!" << std::endl;

	std::string save_directory = "./log/";

#ifdef _WIN32
	if (_access(save_directory.c_str(), 0)) {
		if (_mkdir(save_directory.c_str())) return false;
	}
#endif // _WIN32
#ifdef __linux__
	mkdir(save_directory.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
#endif
	bool retval = lidar_ml->start_recording(save_directory);

	int frame_number = 0;
	int logging_data_size = 10;
	std::string filename;
	while (frame_number < logging_data_size) {
		SOSLAB::LidarMl::scene_t scene;
		/* Stream FIFO로부터 한 프레임씩 Lidar data를 가져옵니다. */
		if (lidar_ml->get_scene(scene)) {
			std::size_t height = scene.rows;	// Lidar frame의 height 정보입니다.
			std::size_t width = scene.cols;		// Lidar frame의 width 정보입니다.

			frame_number++;
			std::cout << frame_number << std::endl;
		}
	}
	lidar_ml->stop_recording();
	
	/* 스트리밍을 종료 합니다. */
	lidar_ml->tcp_device_stop();

	/* Recording File을 Point Cloud로 변환합니다.
	   [FILE_PATH]      = recording file name			 (ex. 02-15-23-48-08.bin)
	   [SAVE_DIRECTORY] = save direcotry for point cloud (ex. "./")
	*/
	//lidar_ml->connect_file([FILE_PATH]);
	//lidar_ml->binary2file([SAVE_DIRECTORY]);

	std::cout << "Streaming stopped!" << std::endl;

	/* 장치 연결을 해제합니다. */
	lidar_ml->disconnect();

	std::cout << "Done." << std::endl;
	system("pause");
}
