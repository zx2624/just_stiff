#include "stiff_detection.h"

StiffDetection::StiffDetection(ros::NodeHandle& nh):nh_(nh)
{
	ros::param::get("~visulization",visual_on);
	if(visual_on)
		std::cout << "@stiff_detection=========visualization on ===========" << std::endl;
	sub_Lidar_ = nh_.subscribe<sensor_msgs::PointCloud2>
	("lidar_cloud_calibrated", 1, boost::bind(&StiffDetection::LidarMsgHandler,this,_1));
	sub_Gpsdata_ = nh_.subscribe<sensor_driver_msgs::GpswithHeading>
	("gpsdata", 30, boost::bind(&StiffDetection::GpsdataMsgHandler,this,_1));
	pub_Stiff_ = nh_.advertise<stiff_msgs::stiffwater> ("stiffwaterogm",20);

	process_thread_ = new std::thread(&StiffDetection::process, this);
	map_j = new int[32]{
		16,	18,	0,	20,	2,	22,	30,	28,	26,	24,	23,	21,	19,	17,	31,	29,	27,	25,	7,	5,	3,	1,	15,	13,	11,	9,	4,	6,	8,	10,	12,	14
	};
	elementero = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
	elementdil = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
}
StiffDetection::~StiffDetection(){
	std::cout << "调用析构函数" << std::endl;
	delete[] map_j;
	process_thread_->join();
}

void StiffDetection::process(){
#ifdef CLOUDVIEWER
	boost::shared_ptr<PCLVisualizer> cloud_viewer_ (new PCLVisualizer("stiffdetection cloud"));
	PrepareViewer(cloud_viewer_);
#endif //CLOUDVIEWER
	while(ros::ok()){
		sensor_driver_msgs::GpswithHeadingConstPtr tmpmsg=qgwithhmsgs_.PopWithTimeout(common::FromSeconds(0.1));
		sensor_msgs::PointCloud2ConstPtr lidarCloudMsgs_ = q_lidar_msgs_.PopWithTimeout(common::FromSeconds(0.1));
		if(tmpmsg == nullptr || lidarCloudMsgs_ == nullptr){//
			std::cout << "no gps or lidar, continue" << std::endl;
			continue;
		}
		double lidarstamp = lidarCloudMsgs_->header.stamp.toSec();
		double gpsstamp = tmpmsg->gps.header.stamp.toSec();
		if(gpsstamp > lidarstamp){
			std::cout << "time stamp " <<lidarCloudMsgs_->header.stamp << " " << tmpmsg->gps.header.stamp << std::endl;
			qgwithhmsgs_.Push_Front(std::move(tmpmsg));
			usleep(10000);
			continue;
		}
		while(lidarstamp-gpsstamp>0.02){
			if(qgwithhmsgs_.Size()==0){

				std::cout << "----------------gps q 0 " << std::endl;
				usleep(100000);
				continue;
			}
			tmpmsg=qgwithhmsgs_.Pop();
			gpsstamp=tmpmsg->gps.header.stamp.toSec();
		}
		double yaw = tmpmsg->heading, pitch = tmpmsg->pitch, roll = tmpmsg->roll;
		Eigen::AngleAxisd yaw_ = Eigen::AngleAxisd(yaw * PI / 180, Eigen::Vector3d::UnitZ());
		Eigen::AngleAxisd pitch_ = Eigen::AngleAxisd(pitch * PI / 180, Eigen::Vector3d::UnitX());
		Eigen::AngleAxisd roll_ = Eigen::AngleAxisd(roll * PI / 180, Eigen::Vector3d::UnitY());
		Eigen::Matrix3d R = yaw_.matrix() * pitch_.matrix() * roll_.matrix();
		Eigen::Quaterniond q(R);//q代表矫正为水平面的旋转
		cv::Mat grid_msg_show = cv::Mat::zeros(GRIDWH,GRIDWH,CV_8UC1);
		cv::Mat grid_show = cv::Mat::zeros(GRIDWH,GRIDWH,CV_8UC3);
		double t1 = ros::Time::now().toSec();
		if(lidarCloudMsgs_){
			pcl::PointCloud<pcl::PointXYZI>::Ptr tempcloud(new pcl::PointCloud<pcl::PointXYZI>);//当前帧点云（雷达里程计坐标系）
			mtx_lidar_.lock();
			if(lidarCloudMsgs_ != nullptr)
				pcl::fromROSMsg(*lidarCloudMsgs_, *tempcloud);//获取当前帧点云数据
			mtx_lidar_.unlock();
			std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> outputclouds;//多个激光雷达数据包，向量中每个元素为一个激光雷达一帧数据
			std::vector<pcl::PointXYZI> lidarpropertys;//每一个PointType类型都表示一个单独点
			analysisCloud(tempcloud,outputclouds,lidarpropertys);

			for(auto pt : tempcloud->points){
				if(pt.range < 0) continue;
				float x = pt.x;
				float y = pt.y;
				float z = pt.z;
				Eigen::Vector3d pt_rec(x, y, z);
				float z_rec = (q * pt_rec).z();
				int col=boost::math::round((x+35)/0.2);//干！！！！！！！！！！！
				int row=boost::math::round((y+20)/0.2);
				if(col >= 0 && col < GRIDWH && row >= 0 && row < GRIDWH){
					auto ptr = grid_show.ptr<unsigned char>(GRIDWH - 1 - row);
					auto ptr_msg = grid_msg_show.ptr<unsigned char>(GRIDWH - 1 - row);
					ptr_msg[col] = 100;//代表可通行
					ptr[3*col + 1] = 255;
				}
			}
			Detection16(outputclouds, grid_show, q);
			Detection32(outputclouds, grid_show, q);
			PublishMsg(grid_show, grid_msg_show, lidarCloudMsgs_->header.stamp);
			if(visual_on){
				cv::namedWindow("gridshow",CV_WINDOW_NORMAL);
				cv::imshow("gridshow", grid_show);
				cv::waitKey(3);
			}
#ifdef CLOUDVIEWER
			ShowCloud(cloud_viewer_, tempcloud);
			cloud_viewer_->spinOnce();
#endif //CLOUDVIEWER

		}
		usleep(30000);
		double t2 = ros::Time::now().toSec();
//		std::cout << "time cost: " << t2 - t1 << std::endl;
	}
}
bool StiffDetection::ptUseful(pcl::PointXYZI& pt, float dis_th){
	float  z = pt.z;
	float  x = pt.x;
	float  y = pt.y;
	float dis = sqrt(x*x + y*y + z*z);
	if(pt.range < 0 || dis > dis_th || z > 3 || z < -3 || y < 0 || x > 100)
		return false;
	return true;
}
void StiffDetection::Detection16(vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> outputclouds ,cv::Mat grid_show, Eigen::Quaterniond q){
	pcl::PointCloud<pcl::PointXYZI>::Ptr new16_left(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr new16_right(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::transformPointCloud(*outputclouds[1], *new16_left, Eigen::Vector3d(0,0,0), q);
	pcl::transformPointCloud(*outputclouds[2], *new16_right, Eigen::Vector3d(0,0,0), q);
	vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> new_clouds{new16_left, new16_right};

	float far_bound = FAR_BOUND;
	const int layer = 16;
	//	int round[2];
	int round = outputclouds[1]->points.size() / layer;
	//	round[1] = outputclouds[2]->points.size() / layer;
	for(int j = 0; j < 16; ++j){
		for(int i = 0; i + window_big_ < round; ){
			float  height_diff_most = 10, x0 = 0, y0 = 0, x1 = 0, y1 = 0, z0 = 0, z1 = 0;
			float dis_in_window = 0;
			vector<float> test_ys;
			int i_begin = 0;
			float dis_ratio = 0, dis_tocheck = 0, tangent = 0;
			int count_o = 0;
			//高度太高直接可以定为悬崖--不好用
			bool tooHigh = false;
			//大窗口--左雷达
			for(int k = i; k + window_small_ < i + window_big_; ++k){
				float z_high = 0, y_high = 0, x_high = 0;
				float z_low = 0, y_low = 0, x_low = 0;
				float dis_av_high = 0, dis_av_low = 0;
				int count = 0;
				int cnt = 0;
				//小窗口-前
				for(int window_i = k; window_i < k + window_small_; window_i++){
					int index = window_i * layer + j;
					float  z = outputclouds[1]->points[index].z;
					float  x = outputclouds[1]->points[index].x;
					float  y = outputclouds[1]->points[index].y;
					float dis = sqrt(x*x + y*y + z*z);
					if(ptUseful(outputclouds[1]->points[index], 60)) cnt++;
					if(!ptUseful(outputclouds[1]->points[index], NEAR_BOUND)) continue;
					Eigen::Vector3d pt(x, y, z);
					pt = q * pt;
					count ++;
					dis_av_high += dis;
#ifdef NEW
					z_high += pt.z();
#else
					z_high += z;
#endif //NEW
					y_high += y;
					x_high += x;
				}
				if(count > 0) {
					z_high = z_high / count;
					y_high = y_high / count;
					x_high = x_high / count;
					dis_av_high /= count;
					count = 0;
				}
				//小窗口-后
				vector<float> ys_in_window;
				for(int window_i = k + window_small_; window_i < k + 2 * window_small_; window_i++){
					int index = window_i * layer + j;
					float  z = outputclouds[1]->points[index].z;
					float  x = outputclouds[1]->points[index].x;
					float  y = outputclouds[1]->points[index].y;
					float dis = sqrt(x*x + y*y + z*z);
					if(!ptUseful(outputclouds[1]->points[index], far_bound)) continue;
					ys_in_window.push_back(y);
					Eigen::Vector3d pt(x, y, z);
					pt = q * pt;
					count ++;
					dis_av_low +=dis;
#ifdef NEW
					z_low += pt.z();
#else
					z_low += z;
#endif //NEW
					x_low += x;
					y_low += y;
				}
				if(count > window_small_ / 2) {
					z_low = z_low / count;
					x_low = x_low / count;
					y_low = y_low / count;
					dis_av_low /= count;
				}
				float height_diff = 0;
				if(abs(z_high) > 0.0001 && abs(z_low) > 0.0001){
					height_diff =  z_low - z_high;
				}
				sort(ys_in_window.begin(), ys_in_window.end());
				if(height_diff < - 0.6 && height_diff < height_diff_most && count > window_small_ / 2){
					if(height_diff < -1)
						tooHigh = true;
					dis_ratio = dis_av_high / dis_av_low;
					dis_tocheck = dis_av_high;
					height_diff_most = height_diff;
					x0 = x_high; y0 = y_high;z0 = z_high;
					x1 = x_low; y1 = y_low;z1 = z_low;
					i_begin = k;
					count_o = cnt;
					dis_in_window = ys_in_window[ys_in_window.size() / 2] - ys_in_window[0];
					test_ys = ys_in_window;
				}
			}
			if(i_begin != 0){//
				//尝试用窗口相邻高度突变
				bool ok = false;
				float z_diff_nb = 0;
				int index_high = -1, index_low = -1;
				for(int i = 0; i < window_small_; ++i){
					int i_high = (i_begin + window_small_ - 1 - i) * layer + j;
					int i_low = (i_begin + window_small_ + i) * layer + j;
					auto pt_high = outputclouds[1]->points[i_high];
					auto pt_low = outputclouds[1]->points[i_low];
					if(ptUseful(pt_high, NEAR_BOUND) && index_high == -1)
						index_high = i_high;
					if(ptUseful(pt_low, far_bound) && index_low == -1)
						index_low = i_low;
					if(index_high != -1 && index_low != -1)
						break;
				}
				float dis_high = 0;
				float dis_low = 0;
				if(index_high != -1 && index_low != -1){
					auto pt_high = outputclouds[1]->points[index_high];
					auto pt_low = outputclouds[1]->points[index_low];
#ifdef NEW
					auto pt_high_new = new16_left->points[index_high];
					auto pt_low_new = new16_left->points[index_low];
#endif//NEW

#ifdef NEW
					float z_low = pt_low_new.z;
					float z_high = pt_high_new.z;
#else
					float z_low = pt_low.z;
					float z_high = pt_high.z;
#endif//NEW
					float y_high = pt_high.y, x_high = pt_high.x;
					float y_low = pt_low.y, x_low = pt_low.x;
					dis_high = pt_high.range;
					dis_low = pt_low.range;
					float radius = sqrt(pow((y_high - y_low), 2) + pow((x_high - x_low), 2));
					tangent = (z_high - z_low) / radius;
					dis_ratio = dis_high / dis_low;
					z_diff_nb = z_high - z_low;
					if(z_high - z_low > 0.5)
						ok = true;
				}
				//尝试用窗口相邻高度突变--暂时没用上
				//&& dis_in_window < 1.5
				float offset = 0;
				if(dis_high < 10)
					offset = 0;
				if(((dis_ratio < th_dis + offset&& dis_ratio > 0.2) && z0 < 0.5 && tangent > th)
				){// || (dis_tocheck < 15 && z_diff_nb > 1 && z0 < 0.5 && tangent > 0.15)
					//						std::cout << "tanget is ... " << tangent << std::endl;
					//						std::cout << "dis in window are  ... " << dis_in_window << std::endl;
					for(int k = i_begin; k <  i_begin + window_small_; ++k){
						int index_high = k * layer + j;
						int index_low = (k + window_small_) * layer + j;
						auto pt_high_ori = outputclouds[1]->points[index_high];
						auto pt_low_ori = outputclouds[1]->points[index_low];
						auto pt_high = new16_left->points[index_high];
						auto pt_low = new16_left->points[index_low];
						float  z = pt_high.z;
						float  x = pt_high.x;
						float  y = pt_high.y;
						float dis = pt_high.range;
						if(!ptUseful(pt_high_ori, NEAR_BOUND)){}
						else{
#ifdef NEW
							//							left_high_cloud->points.push_back(pt_high);//todo:历史遗留问题
#else
							left_high_cloud->points.push_back(pt_high_ori);
#endif //NEW
						}
						z = pt_low.z;
						x = pt_low.x;
						y = pt_low.y;
						dis = pt_low.range;
						if(!ptUseful(pt_low_ori, far_bound)) continue;
#ifdef NEW
						//						left_low_cloud->points.push_back(pt_low);
#else
						left_low_cloud->points.push_back(pt_low_ori);
#endif //NEW
					}
					int begin_x = (x0 + 35) / 0.2, begin_y = (y0 + 20) / 0.2; begin_y = GRIDWH - 1 - begin_y;
					int end_x = (x1 + 35) / 0.2, end_y = (y1 + 20) / 0.2; end_y = GRIDWH - 1 - end_y;
					if(begin_y < end_y){
						swap(begin_x, end_x);
						swap(begin_y, end_y);
					}
					float ratio = 10 / sqrt(pow((end_y - begin_y), 2) + pow((end_x - begin_x), 2));
					if(ratio < 1){
						end_x = begin_x + ratio * (end_x - begin_x);
						end_y = begin_y + ratio * (end_y - begin_y);
					}
					if(begin_x >= 0 && begin_x < GRIDWH && end_x >= 0 && end_x < GRIDWH
							&& begin_y >=0 && begin_y < GRIDWH
							&& end_y >=0 && end_y < GRIDWH){
						//							std::cout << " --- " << count_o << std::endl;
						//							std::cout << setprecision(3) << "zzzzzzz------- " << z0 << " " << z1 << " dis is -- "
						//									<< dis_tocheck <<
						//									" neighbor height is " << z_diff_nb << std::endl;
						//							std::cout << "dis ratio is " << dis_ratio << std::endl;
						cv::line(grid_show, cvPoint(begin_x, begin_y), cvPoint(end_x, end_y), cvScalar(0,0,125),3);
					}
				}
				i_begin = 0;
				height_diff_most = 10;
			}



			//复位
			height_diff_most = 10;
			x0 = 0; y0 = 0; x1 = 0; y1 = 0; z0 = 0; z1 = 0;
			dis_in_window = 0;
			i_begin = 0;
			dis_ratio = 0;
			//大窗口--right lidar
			for(int k = i; k + window_small_ < i + window_big_; ++k){
				float z_high = 0, y_high = 0, x_high = 0;
				float z_low = 0, y_low = 0, x_low = 0;
				float dis_av_high = 0, dis_av_low = 0;
				int count = 0;
				//小窗口-high
				for(int window_i = k + window_small_; window_i < k + 2 * window_small_; window_i++){
					int index = window_i * layer + j;
					float  z = outputclouds[2]->points[index].z;
					float  x = outputclouds[2]->points[index].x;
					float  y = outputclouds[2]->points[index].y;
					float dis = sqrt(x*x + y*y + z*z);
					if(!ptUseful(outputclouds[2]->points[index], NEAR_BOUND)) continue;
					count ++;
					dis_av_high +=dis;
					Eigen::Vector3d pt(x, y, z);
					pt = q * pt;
#ifdef NEW
					z_high += pt.z();
#else
					z_high += z;
#endif //NEW
					x_high += x;
					y_high += y;
				}
				if(count > 0) {
					z_high = z_high / count;
					x_high = x_high / count;
					y_high = y_high / count;
					dis_av_high /= count;
					count = 0;
				}
				//小窗口-low
				vector<float> ys_in_window;
				for(int window_i = k; window_i < k + window_small_; window_i++){
					int index = window_i * layer + j;
					float  z = outputclouds[2]->points[index].z;
					float  x = outputclouds[2]->points[index].x;
					float  y = outputclouds[2]->points[index].y;
					float dis = sqrt(x*x + y*y + z*z);
					if(!ptUseful(outputclouds[2]->points[index], far_bound)) continue;
					ys_in_window.push_back(y);
					Eigen::Vector3d pt(x, y, z);
					pt = q * pt;
					count ++;
					dis_av_low += dis;
#ifdef NEW
					z_low += pt.z();
#else
					z_low += z;
#endif//NEW
					y_low += y;
					x_low += x;
				}
				if(count > 0) {
					z_low = z_low / count;
					y_low = y_low / count;
					x_low = x_low / count;
					dis_av_low /= count;
				}

				float height_diff = 0;
				if(abs(z_high) > 0.0001 && abs(z_low) > 0.0001){
					height_diff =  z_low - z_high;
				}
				sort(ys_in_window.begin(), ys_in_window.end());
				if(height_diff < - 0.6 && height_diff < height_diff_most && count > window_small_ / 2){
					if(height_diff < -1)
						tooHigh = true;
					dis_ratio = dis_av_high / dis_av_low;
					dis_tocheck = dis_av_high;
					height_diff_most = height_diff;
					x0 = x_high; y0 = y_high;z0 = z_high;
					x1 = x_low; y1 = y_low;z1 = z_low;
					i_begin = k;
					dis_in_window = ys_in_window[ys_in_window.size() / 2] - ys_in_window[0];
				}
			}
			if(i_begin != 0){//
				//寻找相邻窗口点
				bool ok = false;
				float z_diff_nb = 0;
				int index_high = -1, index_low = -1;
				for(int i = 0; i < window_small_; ++i){
					int i_low = (i_begin + window_small_ - 1 - i) * layer + j;
					int i_high = (i_begin + window_small_ + i) * layer + j;
					auto pt_high = outputclouds[2]->points[i_high];
					auto pt_low = outputclouds[2]->points[i_low];
					if(ptUseful(pt_high, NEAR_BOUND) && index_high == -1)
						index_high = i_high;
					if(ptUseful(pt_low, far_bound) && index_low == -1)
						index_low = i_low;
					if(index_high != -1 && index_low != -1)
						break;
				}
				float dis_high = 0;
				float dis_low = 0;
				if(index_high != -1 && index_low != -1){
					auto pt_high = outputclouds[2]->points[index_high];
					auto pt_low = outputclouds[2]->points[index_low];
#ifdef NEW
					auto pt_high_new = new16_right->points[index_high];
					auto pt_low_new = new16_right->points[index_low];
#endif//NEW

#ifdef NEW
					float z_low = pt_low_new.z;
					float z_high = pt_high_new.z;
#else
					float z_low = pt_low.z;
					float z_high = pt_high.z;
#endif//NEW
					float y_high = pt_high.y, x_high = pt_high.x;
					float y_low = pt_low.y, x_low = pt_low.x;
					dis_high = pt_high.range;
					dis_low = pt_low.range;
					float radius = sqrt(pow((y_high - y_low), 2) + pow((x_high - x_low), 2));
					tangent = (z_high - z_low) / radius;
					dis_ratio = dis_high / dis_low;
					z_diff_nb = z_high - z_low;
					if(z_high - z_low > 0.5)
						ok = true;
				}
				//尝试用窗口相邻高度突变--暂时没用上
				float offset = 0;
				if(dis_high < 10)
					offset = 0;
				if(((dis_ratio < th_dis + offset && dis_ratio > 0.2) && z0 < 0.5 && tangent > th)
				){//(dis_tocheck < 15 && z_diff_nb > 1 && z0 < 0.5 && tangent > 0.15)
					//					std::cout << "tangent ... " << tangent << std::endl;
					//					std::cout << "dis_ratio  ... " << dis_ratio << std::endl;
					for(int k = i_begin; k <  i_begin + window_small_; ++k){
						int index_low = k * layer + j;
						int index_high = (k + window_small_) * layer + j;
						auto pt_high = outputclouds[2]->points[index_high];
						auto pt_low = outputclouds[2]->points[index_low];
						auto pt_high_new = new16_right->points[index_high];
						auto pt_low_new = new16_right->points[index_low];
						float  z = pt_high.z;
						float  x = pt_high.x;
						float  y = pt_high.y;
						float dis = pt_high.range;
						if(!ptUseful(pt_high, NEAR_BOUND)){}
						else{
#ifdef NEW
							//							right_high_cloud->points.push_back(pt_high_new);
#else
							right_high_cloud->points.push_back(pt_high);
#endif//NEW
						}
						z = pt_low.z;
						x = pt_low.x;
						y = pt_low.y;
						dis = pt_low.range;
						if(!ptUseful(pt_low, far_bound)) continue;
#ifdef NEW
						//						right_low_cloud->points.push_back(pt_low_new);
#else
						right_low_cloud->points.push_back(pt_low);
#endif//NEW
					}
					int begin_x = (x0 + 35) / 0.2, begin_y = (y0 + 20) / 0.2; begin_y = GRIDWH - 1 - begin_y;
					int end_x = (x1 + 35) / 0.2, end_y = (y1 + 20) / 0.2; end_y = GRIDWH - 1 - end_y;
					if(begin_y < end_y){
						swap(begin_x, end_x);
						swap(begin_y, end_y);
					}
					float ratio = 10 / sqrt(pow((end_y - begin_y), 2) + pow((end_x - begin_x), 2));
					if(ratio < 1){
						end_x = begin_x + ratio * (end_x - begin_x);
						end_y = begin_y + ratio * (end_y - begin_y);
					}
					if(begin_x >= 0 && begin_x < GRIDWH && end_x >= 0 && end_x < GRIDWH
							&& begin_y >=0 && begin_y < GRIDWH
							&& end_y >=0 && end_y < GRIDWH){
						cv::line(grid_show, cvPoint(begin_x, begin_y), cvPoint(end_x, end_y), cvScalar(0,0,125),3);
					}
				}
			}
			i += window_big_ / 2;
		}
	}
}
void StiffDetection::Detection32(vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> outputclouds,
		cv::Mat grid_show, Eigen::Quaterniond q){
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = outputclouds[0];
	pcl::PointCloud<pcl::PointXYZI>::Ptr newcloud(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::transformPointCloud(*cloud, *newcloud, Eigen::Vector3d(0,0,0), q);
	const int round32 = cloud->size() / 32;
	float far_bound = FAR_BOUND;


	for(int j = 0; j < 25; ++j){
		//				if( map_j[j] >  25) continue;
		//		std::cout << " ----------------------- " << std::endl;
		for(int i = 0; i + window_big_ < round32; i+=window_small_){
			float  height_diff_most = 10, x0 = 0, y0 = 0, x1 = 0, y1 = 0, z0 = 0, z1 = 0;
			float dis_in_window = 0;
			int i_begin = 0;
			float dis_ratio = 0, dis_tocheck = 0, tangent = 0;

			//大窗口
			for(int k = i; k + window_small_ < i + window_big_; ++k){
				float z_high = 0, y_high = 0, x_high = 0;
				float z_low = 0, y_low = 0, x_low = 0;
				float dis_av_high = 0, dis_av_low = 0;
				int count = 0;
				int cnt = 0;
				//小窗口-1
				for(int window_i = k; window_i < k + window_small_; window_i++){
					int index = window_i * 32 + map_j[j];
					float  z = newcloud->points[index].z;
					float  x = cloud->points[index].x;
					float  y = cloud->points[index].y;
					float dis = sqrt(x*x + y*y + z*z);
					int ran_cnt = 0;
					float ran_z_high = 0, ran_x_high = 0, ran_y_high = 0, ran_dis_high = 0;
					//进行一个类似ransac的过程，寻找最好的能表示z的值
					for(int ran_i = k; ran_i < k + window_small_; ran_i++){
						int ran_index = ran_i * 32 + map_j[j];
						float  z_ran = newcloud->points[ran_index].z;
						float  z_ran_ori = cloud->points[ran_index].z;
						float  x_ran = cloud->points[ran_index].x;
						float  y_ran = cloud->points[ran_index].y;
						float dis_ran = sqrt(x_ran*x_ran + y_ran*y_ran + z_ran_ori*z_ran_ori);
						if(cloud->points[ran_index].range < 0 || y_ran < 0 || dis_ran > far_bound
								|| abs(dis_ran - dis) > 1) continue;//abs(z_ran - z) > 0.3
						ran_cnt ++;
						ran_dis_high += dis_ran;
						ran_z_high += z_ran;
						ran_y_high += y_ran;
						ran_x_high += x_ran;
						if(ran_cnt > window_small_ / 2){
							count = ran_cnt;
							dis_av_high = ran_dis_high;
							z_high = ran_z_high;
							x_high = ran_x_high;
							y_high = ran_y_high;
							break;
						}
					}
					if(count > 0) break;
				}

				if(count > 0) {
					z_high = z_high / count;
					y_high = y_high / count;
					x_high = x_high / count;
					dis_av_high /= count;
					count = 0;
				}else {
					continue;
				}
				//小窗口-2
				for(int window_i = k + window_small_; window_i < k + 2 * window_small_; window_i++){
					int index = window_i * 32 + map_j[j];
					float  z = newcloud->points[index].z;
					float  x = cloud->points[index].x;
					float  y = cloud->points[index].y;
					float dis = sqrt(x*x + y*y + z*z);

					int ran_cnt = 0;
					float ran_z_low = 0, ran_x_low = 0, ran_y_low = 0, ran_dis_low = 0;
					for(int ran_i = k + window_small_; ran_i < k + 2 * window_small_; ran_i++){
						int ran_index = ran_i * 32 + map_j[j];
						float  z_ran = newcloud->points[ran_index].z;
						float  z_ran_ori = cloud->points[ran_index].z;
						float  x_ran = cloud->points[ran_index].x;
						float  y_ran = cloud->points[ran_index].y;
						float dis_ran = sqrt(x_ran*x_ran + y_ran*y_ran + z_ran_ori*z_ran_ori);
						if(cloud->points[ran_index].range < 0 || y_ran < 0 || dis_ran > far_bound
								|| abs(dis_ran - dis) > 0.3) continue;//abs(z_ran - z) > 0.3
						ran_cnt ++;
						ran_dis_low += dis_ran;
						ran_z_low += z_ran;
						ran_y_low += y_ran;
						ran_x_low += x_ran;
						if(ran_cnt > window_small_ / 2){
							count = ran_cnt;
							dis_av_low = ran_dis_low;
							z_low = ran_z_low;
							x_low = ran_x_low;
							y_low = ran_y_low;
							break;
						}
					}
					if(count > 0) break;
				}
				if(count > 0) {
					z_low = z_low / count;
					x_low = x_low / count;
					y_low = y_low / count;
					dis_av_low /= count;
				}else {
					continue;
				}
				if(z_high < z_low){
					swap(z_high, z_low);
					swap(x_high, x_low);
					swap(y_high, y_low);
					swap(dis_av_high, dis_av_low);
				}
				float height_diff = 0;
				if(abs(z_high) > 0.0001 && abs(z_low) > 0.0001){
					height_diff =  z_low - z_high;
				}
				if(height_diff < - 0.6 && height_diff < height_diff_most && count > 0){
					dis_ratio = dis_av_high / dis_av_low;
					dis_tocheck = dis_av_high;
					height_diff_most = height_diff;
					x0 = x_high; y0 = y_high;z0 = z_high;
					x1 = x_low; y1 = y_low;z1 = z_low;
					i_begin = k;
					float radius = sqrt(pow((x1 - x0), 2) + pow((y1 - y0), 2));
					tangent = (z0 - z1) / radius;
					//					dis_in_window = ys_in_window[ys_in_window.size() / 2] - ys_in_window[0];
				}
			}
			if(i_begin != 0){//
				//尝试用窗口相邻高度突变
				bool ok = false;
				float z_diff_nb = 0;
				int index_high = -1, index_low = -1;
				for(int i = 0; i < window_small_; ++i){
					int i_high = (i_begin + window_small_ - 1 - i) * 32 + map_j[j];
					int i_low = (i_begin + window_small_ + i) * 32 + map_j[j];
					auto pt_high = cloud->points[i_high];
					auto pt_low = cloud->points[i_low];
					if(pt_high.range > 0 && index_high == -1)
						index_high = i_high;
					if(pt_low.range > 0 && index_low == -1)
						index_low = i_low;
					if(index_high != -1 && index_low != -1){
						if(pt_high.z < pt_low.z){
							swap(index_high, index_low);
						}
						break;
					}
				}
				if(index_high != -1 && index_low != -1){
					auto pt_high = cloud->points[index_high];
					auto pt_low = cloud->points[index_low];
					float z_low = pt_low.z;
					float z_high = pt_high.z;
					float y_high = pt_high.y, x_high = pt_high.x;
					float y_low = pt_low.y, x_low = pt_low.x;
					float dis_high = pt_high.range;
					float dis_low = pt_low.range;
					float radius = y_high - y_low;
					//					tangent = (z_high - z_low) / radius;
					//					dis_ratio = dis_high / dis_low;
					z_diff_nb = z_high - z_low;
					if(z_high - z_low > 0.5)
						ok = true;
				}
				//尝试用窗口相邻
				//todo:this th
				if(((dis_ratio < 0.6 && dis_ratio > 0.2) && z0 < 0.5 && tangent > th) ||
						(dis_tocheck < 15 && z_diff_nb > 1 && z0 < 0.5 && tangent > th)){//
					//					std::cout << "tanget is ... " << tangent << std::endl;
					//					std::cout << "dis in window are  ... " << dis_in_window << std::endl;
					int count = 0;
					for(int window_i = i_begin; window_i < i_begin + window_small_; window_i++){
						int ran_cnt = 1;
						int index = window_i * 32 + map_j[j];
						float z = newcloud->points[index].z;
						for(int ran_i = i_begin; ran_i < i_begin + window_small_; ran_i++){
							int ran_index = ran_i * 32 + map_j[j];
							float  z_ran = newcloud->points[ran_index].z;
							float  z_ran_ori = cloud->points[ran_index].z;
							float  x_ran = cloud->points[ran_index].x;
							float  y_ran = cloud->points[ran_index].y;
							float dis_ran = sqrt(x_ran*x_ran + y_ran*y_ran + z_ran_ori*z_ran_ori);
							if(ran_i == window_i ||cloud->points[ran_index].range < 0 || y_ran < 0 || dis_ran > far_bound
									|| abs(z_ran - z) > 0.3) continue;
							ran_cnt ++;
							if(ran_cnt > window_small_ / 2){
								count = ran_cnt;
								break;
							}
						}
					}
					//					std::cout << ">>>>>>>count>>>>>>>>>> " << count << std::endl;
					//					std::cout << ">>>>>>>z0   >>>>>>>>>> " << z0 << std::endl;
					for(int k = i_begin; k <  i_begin + window_small_; ++k){
						int index_high = k * 32 + map_j[j];
						int index_low = (k + window_small_) * 32 + map_j[j];
						auto pt_high = newcloud->points[index_high];
						auto pt_low = newcloud->points[index_low];
						auto pt_high_ori = cloud->points[index_high];
						auto pt_low_ori = cloud->points[index_low];
						float  z = pt_high.z;
						float  x = pt_high.x;
						float  y = pt_high.y;
						float dis = pt_high.range;
						if(pt_high.range > 0){
							//							std::cout << "--- " << z << " --- ";
							//							h_cloud->points.push_back(pt_high_ori);
						}else{
							//							std::cout << "--- " << "              " ;
						}
						z = pt_low.z;
						x = pt_low.x;
						y = pt_low.y;
						dis = pt_low.range;
						if(pt_low.range < 0){
							//							std::cout << std::endl;
							continue;
						}
						//						std::cout << z << std::endl;
						//						l_cloud->points.push_back(pt_low_ori);
					}
					{//test
						//						for(int k = i_begin - 5; k < i_begin + window_small_; ++k){
						//							int index = k * 32 + j;
						//							auto pt = cloud->points[index];
						//							std::cout << pt.z << std::endl;
						//						}
					}
					int begin_x = (x0 + 35) / 0.2, begin_y = (y0 + 20) / 0.2; begin_y = GRIDWH - 1 - begin_y;
					int end_x = (x1 + 35) / 0.2, end_y = (y1 + 20) / 0.2; end_y = GRIDWH - 1 - end_y;
					if(begin_y < end_y){
						swap(begin_x, end_x);
						swap(begin_y, end_y);
					}
					float ratio = 10 / sqrt(pow((end_y - begin_y), 2) + pow((end_x - begin_x), 2));
					if(ratio < 1){
						end_x = begin_x + ratio * (end_x - begin_x);
						end_y = begin_y + ratio * (end_y - begin_y);
					}
					if(begin_x >= 0 && begin_x < GRIDWH && end_x >= 0 && end_x < GRIDWH
							&& begin_y >=0 && begin_y < GRIDWH
							&& end_y >=0 && end_y < GRIDWH){
						cv::line(grid_show, cvPoint(begin_x, begin_y), cvPoint(end_x, end_y), cvScalar(0,0,125),3);
					}
				}
				i_begin = 0;
				height_diff_most = 10;
			}
		}
	}
}
void StiffDetection::PublishMsg(cv::Mat grid_show, cv::Mat grid_msg_show, ros::Time stamp){
	stiff_msgs::stiffwater msg_send;
	msg_send.header.stamp = stamp;
	msg_send.header.frame_id = "stiffwater";
	msg_send.ogmheight = 351;
	msg_send.ogmwidth = 201;
	msg_send.resolution = 0.2;
	msg_send.vehicle_x = 100;
	msg_send.vehicle_y = 100;
	for(int row=0;row<GRIDWH;row++){
		unsigned char* ptr_msg_show=grid_msg_show.ptr<unsigned char>(GRIDWH-1-row);
		unsigned char* ptr_show=grid_show.ptr<unsigned char>(GRIDWH-1-row);
		for(int col=0;col<GRIDWH;col++){
			if(ptr_show[3*col+2]==125){
				ptr_msg_show[col]=255;//障碍物
			}
			//			else if(ptr[3*col + 1] == 255){//水区域 ptr[3*col+2]==125&&ptr[3*col+1]==125&&ptr[3*col]==125
			//				ptrall[col]=200;
			//			}
			else if(ptr_show[3*col+1]==255){
				ptr_msg_show[col]=100;//通行可
			}
		}
	}


	cv::dilate(grid_msg_show, grid_msg_show, elementdil);
	cv::erode(grid_msg_show, grid_msg_show, elementero);
	//	cv::dilate(gridall,gridall,elementero2);
	if(visual_on){
		cv::namedWindow("msg_send", CV_WINDOW_NORMAL);
		cv::imshow("msg_send", grid_msg_show);
	}
	for(int row=0;row<351;row++){
		unsigned char* ptr=grid_msg_show.ptr<unsigned char>(GRIDWH-1-row);
		for(int col=0;col<201;col++){
			int index=row*201+col;
			if(ptr[col+75]==255){
				msg_send.data.push_back(5);
			}
			else if(ptr[col+75]==200){
				if(send_water){
					msg_send.data.push_back(6);}
				else{
					msg_send.data.push_back(0);
				}
			}
			else if(ptr[col+75]==100){
				msg_send.data.push_back(1);
			}
			else{
				msg_send.data.push_back(0);
			}
		}
	}
	pub_Stiff_.publish(msg_send);
}
void StiffDetection::ShowCloud(boost::shared_ptr<PCLVisualizer>& cloud_viewer_, pcl::PointCloud<pcl::PointXYZI>::Ptr incloud){
	cloud_viewer_->removeAllPointClouds();
	if (incloud->size() > 0)
	{
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> cloudHandler( incloud, 0, 255, 0 );
		if (!cloud_viewer_->updatePointCloud(incloud,cloudHandler, "right"))
		{
			cloud_viewer_->addPointCloud(incloud, cloudHandler, "right");
		}
	}
}
void StiffDetection::analysisCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr inputcloud,
		std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& outputclouds,std::vector<pcl::PointXYZI>& lidarpropertys)
{

	//////////////////////////总的点云中可能包含了几组独立的点云数据，对发过来的点云进行处理，将每一组点云都提取出来////////////////////////////////////////
	int cloudnum = inputcloud->size() % 16;//包含的点云包数目
	vector<int> startnum;

	for(int i =0;i<cloudnum;i++)
	{
		pcl::PointXYZI originpoint;
		int flag = (*inputcloud)[inputcloud->size()-cloudnum+i].range;//每一包点云的第一个点的位置
		(*inputcloud)[inputcloud->size()-cloudnum+i].range = -0.5;
		originpoint.x = (*inputcloud)[inputcloud->size()-cloudnum+i].x;//每一包点云中对应的雷达在车体坐标系的x
		originpoint.y = (*inputcloud)[inputcloud->size()-cloudnum+i].y;////每一包点云中对应的雷达在车体坐标系的y
		originpoint.z = (*inputcloud)[inputcloud->size()-cloudnum+i].z;////每一包点云中对应的雷达在车体坐标系的z
		originpoint.intensity = (*inputcloud)[inputcloud->size()-cloudnum+i].azimuth;//每一包点云中对应的雷达线束
		startnum.push_back(flag);
		lidarpropertys.push_back(originpoint);
	}
	for(int i = 0;i < startnum.size();i++)
	{
		int length;
		pcl::PointCloud<pcl::PointXYZI>::Ptr lasercloudptr(new pcl::PointCloud<pcl::PointXYZI>);//每一包点云

		if(i == startnum.size()-1)
		{
			length = inputcloud->size() - cloudnum - startnum.at(i);
		}
		else
		{
			length = startnum.at(i+1) - startnum.at(i);
		}

		lasercloudptr->insert(lasercloudptr->begin(),inputcloud->begin()+startnum.at(i),inputcloud->begin()+startnum.at(i)+length);
		outputclouds.push_back(lasercloudptr);

	}

}
void StiffDetection::LidarMsgHandler(const sensor_msgs::PointCloud2ConstPtr& msg){
	mtx_lidar_.lock();
	q_lidar_msgs_.Push(msg);
	if(q_lidar_msgs_.Size() > 3){
		q_lidar_msgs_.Pop();
	}
	mtx_lidar_.unlock();
}

void StiffDetection::GpsdataMsgHandler(const sensor_driver_msgs::GpswithHeadingConstPtr& msg){
	qgwithhmsgs_.Push(msg);
	if(qgwithhmsgs_.Size() >30){
		qgwithhmsgs_.Pop();
	}
}
void StiffDetection::coordinate_from_vehicle_to_velodyne(float x, float y , float z, float& newx, float& newy, float& newz)
{
	newz = z;
	newx = x;
	newy = y;
}
void StiffDetection::PrepareViewer(boost::shared_ptr<PCLVisualizer>& cloud_viewer_)
{
#ifdef CLOUDVIEWER
	cloud_viewer_->addCoordinateSystem (3.0);
	cloud_viewer_->setBackgroundColor (0, 0, 0);
	cloud_viewer_->initCameraParameters ();
	cloud_viewer_->setCameraPosition (0.0, 0.0, 30.0, 0.0, 1.0, 0.0, 0);
	cloud_viewer_->setCameraClipDistances (0.0, 100.0);
	//	cloud_viewer_->registerKeyboardCallback (&LidarProcess::keyboard_callback, *this);
	{
		//画车身
		float x1 = -1 , x2 = 1 , y1 = -1 , y2 = 3, z = 0;
		float newx1, newx2, newy1, newy2, newz;
		coordinate_from_vehicle_to_velodyne(x1,y1,z,newx1,newy1,newz);
		coordinate_from_vehicle_to_velodyne(x2,y2,z,newx2,newy2,newz);
		pcl::PointXYZI pt1, pt2, pt3, pt4;
		pt1.x = newx1 ;
		pt1.y = newy1 ;
		pt1.z = newz;
		pt2.x = newx1 ;
		pt2.y = newy2 ;
		pt2.z = newz;
		cloud_viewer_->addLine(pt1, pt2, "body1");

		pt1.x = newx2 ;
		pt1.y = newy2 ;
		pt1.z = newz;
		cloud_viewer_->addLine(pt1, pt2, "body2");

		pt2.x = newx2 ;
		pt2.y = newy1 ;
		pt2.z = newz;
		cloud_viewer_->addLine(pt1, pt2, "body3");

		pt1.x = newx1 ;
		pt1.y = newy1 ;
		pt1.z = newz;
		cloud_viewer_->addLine(pt1, pt2, "body4");



		//画上范围
		if(0)
		{
			float x1 = -20 , x2 = 20 , y1 = -1 , y2 = 40, z = Z_MAX;
			float newx1, newx2, newy1, newy2, newz;
			coordinate_from_vehicle_to_velodyne(x1,y1,z,newx1,newy1,newz);
			coordinate_from_vehicle_to_velodyne(x2,y2,z,newx2,newy2,newz);
			pcl::PointXYZI pt1, pt2, pt3, pt4;
			pt1.x = newx1 ;
			pt1.y = newy1 ;
			pt1.z = newz;
			pt2.x = newx1 ;
			pt2.y = newy2 ;
			pt2.z = newz;
			cloud_viewer_->addLine(pt1, pt2, "upper1");

			pt1.x = newx2 ;
			pt1.y = newy2 ;
			pt1.z = newz;
			cloud_viewer_->addLine(pt1, pt2, "upper2");

			pt2.x = newx2 ;
			pt2.y = newy1 ;
			pt2.z = newz;
			cloud_viewer_->addLine(pt1, pt2, "upper3");

			pt1.x = newx1 ;
			pt1.y = newy1 ;
			pt1.z = newz;
			cloud_viewer_->addLine(pt1, pt2, "upper4");
		}

		//画网格线
		char linename[20];
		for(int i = 0 ; i < 20 ; i++)
		{
			x1 = -20 ;
			x2 = 20 ;
			y1 = (i - 2) * 5 ;
			y2 = (i - 2) * 5;
			z = 0;
			coordinate_from_vehicle_to_velodyne(x1,y1,z,newx1,newy1,newz);
			coordinate_from_vehicle_to_velodyne(x2,y2,z,newx2,newy2,newz);
			pt1.x = std::min(newx1 , newx2) ;
			pt1.y = std::min(newy1 , newy2) ;
			pt1.z = newz;
			pt2.x = std::max(newx1 , newx2) ;
			pt2.y = std::max(newy1 , newy2) ;
			pt2.z = newz;
			memset(linename, 0 , 20);
			sprintf(linename , "lat%02d" , i);
			cloud_viewer_->addLine(pt1, pt2, linename);
		}

		for(int i = 0 ; i < 5 ; i++)
		{
			x1 = i * 10 - 20;
			x2 = i * 10 - 20;
			y1 = -20 ;
			y2 = 70 ;
			z = 0;
			coordinate_from_vehicle_to_velodyne(x1,y1,z,newx1,newy1,newz);
			coordinate_from_vehicle_to_velodyne(x2,y2,z,newx2,newy2,newz);
			pt1.x = std::min(newx1 , newx2) ;
			pt1.y = std::min(newy1 , newy2) ;
			pt1.z = newz;
			pt2.x = std::max(newx1 , newx2) ;
			pt2.y = std::max(newy1 , newy2) ;
			pt2.z = newz;
			memset(linename, 0 , 20);
			sprintf(linename , "lng%02d" , i);
			cloud_viewer_->addLine(pt1, pt2, linename);
		}
	}
#endif //VIEWER
}
