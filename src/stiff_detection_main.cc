
#include <iostream>
#include <glog/logging.h>
#include <signal.h>

#include "stiff_detection.h"


void callback(stiff_detection::stiff_dy_paramConfig &config, uint32_t level)
{
	ROS_INFO("Reconfigure Request: %f  %f",
		config.th_dis,
		config.th_height
);
}
int main(int argc, char** argv){
	ros::init(argc, argv, "stiff_depth");
	ros::NodeHandle nh;


	google::ParseCommandLineFlags(&argc, &argv, true);
	google::InitGoogleLogging(argv[0]);
	google::InstallFailureSignalHandler();

	StiffDetection stiffdetection(nh);
	dynamic_reconfigure::Server<stiff_detection::stiff_dy_paramConfig> server;
	dynamic_reconfigure::Server<stiff_detection::stiff_dy_paramConfig>::CallbackType f;
	f = boost::bind(&StiffDetection::dyCallback, &stiffdetection, _1, _2);
	server.setCallback(f);
//	std::thread processth= stiffdetection.ProcessThread();
//	std::thread processth(&StiffDetection::process, &stiffdetection);
//	std::cout << "===================" << processth.get_id() << std::endl;
//	processth.detach();
	ros::spin();
}
