
#include <iostream>
#include <glog/logging.h>
#include <signal.h>
#include "stiff_detection.h"



int main(int argc, char** argv){
	ros::init(argc, argv, "stiff_depth");
	ros::NodeHandle nh;
	google::ParseCommandLineFlags(&argc, &argv, true);
	google::InitGoogleLogging(argv[0]);
	google::InstallFailureSignalHandler();

	StiffDetection stiffdetection(nh);
//	std::thread processth= stiffdetection.ProcessThread();
//	std::thread processth(&StiffDetection::process, &stiffdetection);
//	std::cout << "===================" << processth.get_id() << std::endl;
//	processth.detach();
	ros::spin();
}
