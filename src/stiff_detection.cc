#include "stiff_detection.h"

StiffDetection::StiffDetection(ros::NodeHandle& nh):nh_(nh)
,vertical_roi_cloud_(new pcl::PointCloud<pcl::PointXYZI>)
,high_cloud_ ( new pcl::PointCloud<pcl::PointXYZI>),
low_cloud_ (new pcl::PointCloud<pcl::PointXYZI>)
{
	ros::param::get("~visulization",visual_on);
	if(visual_on)
		std::cout << "@stiff_detection=========visualization on ===========" << std::endl;
	//订阅点云和惯导信息
	sub_Lidar_ = nh_.subscribe<sensor_msgs::PointCloud2>
	("lidar_cloud_calibrated", 10, boost::bind(&StiffDetection::LidarMsgHandler,this,_1));
	sub_Gpsdata_ = nh_.subscribe<sensor_driver_msgs::GpswithHeading>
	("gpsdata", 30, boost::bind(&StiffDetection::GpsdataMsgHandler,this,_1));
	//发布栅格地图
	pub_Stiff_ = nh_.advertise<stiff_msgs::stiffwater> ("stiffwaterogm",20);
	//开process线程
	process_thread_ = new std::thread(&StiffDetection::process, this);
	//开垂直墙检测线程
	//	verwall_thread_ = new std::thread(&StiffDetection::verticalWallDetect, this);
	//雷达线束序号的索引的映射
	map_j = new int[32]{
		16,	18,	0,	20,	2,	22,	30,	28,	26,	24,	23,	21,	19,	17,	31,	29,	27,	25,	7,	5,	3,	1,	15,	13,	11,	9,	4,	6,	8,	10,	12,	14
	};
	//定义膨胀腐蚀元素的大小、形状
	elementero = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
	elementdil = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));

}
StiffDetection::~StiffDetection(){
	std::cout << "调用析构函数" << std::endl;
	delete[] map_j;
	//	process_thread_->join();
}

void StiffDetection::process(){
	//cloudviewer的初始化必须和显示在同一个线程
#ifdef CLOUDVIEWER
	boost::shared_ptr<PCLVisualizer> cloud_viewer_ (new PCLVisualizer("stiffdetection cloud"));
	PrepareViewer(cloud_viewer_);
#endif //CLOUDVIEWER
	while(ros::ok()){
		//		if(ros::Time::now().toSec() - last_time_gps_ > 1 && last_time_gps_ > 0){
		//			pub1();
		//		}
		//		if(ros::Time::now().toSec() - last_time_lidar_ > 1 && last_time_lidar_ > 0){
		//			pub2();
		//		}
		//从队列中取出消息
		sensor_driver_msgs::GpswithHeadingConstPtr blocks_j_beginimsg=qgwithhmsgs_.PopWithTimeout(common::FromSeconds(0.1));
		if(blocks_j_beginimsg == nullptr){
			std::cout << "no gps data, continue" << std::endl;
			continue;
		}
		sensor_msgs::PointCloud2ConstPtr lidarCloudMsgs_ = q_lidar_msgs_.PopWithTimeout(common::FromSeconds(0.1));
		if(lidarCloudMsgs_ == nullptr){
			std::cout << "no lidar data, continue" << std::endl;
			continue;
		}

		//时间戳同步
		double lidarstamp = lidarCloudMsgs_->header.stamp.toSec();
		last_time_lidar_ = lidarstamp;
		double gpsstamp = blocks_j_beginimsg->gps.header.stamp.toSec();
		last_time_gps_ = gpsstamp;
		if(gpsstamp > lidarstamp){
			std::cout << "wait for lidar ----" << std::endl;
			qgwithhmsgs_.Push_Front(std::move(blocks_j_beginimsg));
			usleep(10000);
			continue;
		}
		while(lidarstamp-gpsstamp>0.02){
			if(qgwithhmsgs_.Size() == 0){//
				std::cout << "----------------gps q 0 " << std::endl;
				usleep(100000);
				continue;
			}
			blocks_j_beginimsg=qgwithhmsgs_.Pop();
			gpsstamp=blocks_j_beginimsg->gps.header.stamp.toSec();
		}
		//获取姿态角，转换成四元数。
		double yaw = blocks_j_beginimsg->heading, pitch = blocks_j_beginimsg->pitch, roll = blocks_j_beginimsg->roll;
		Eigen::AngleAxisd yaw_ = Eigen::AngleAxisd(yaw * PI / 180, Eigen::Vector3d::UnitZ());
		Eigen::AngleAxisd pitch_ = Eigen::AngleAxisd(pitch * PI / 180, Eigen::Vector3d::UnitX());
		Eigen::AngleAxisd roll_ = Eigen::AngleAxisd(roll * PI / 180, Eigen::Vector3d::UnitY());
		Eigen::Matrix3d R = yaw_.matrix() * pitch_.matrix() * roll_.matrix();
		Eigen::Quaterniond q(R);//q代表矫正为水平面的旋转
		cv::Mat grid_msg_show = cv::Mat::zeros(GRIDWH,GRIDWH,CV_8UC1);
		cv::Mat grid_show = cv::Mat::zeros(GRIDWH,GRIDWH,CV_8UC3);
		if(lidarCloudMsgs_){
			pcl::PointCloud<pcl::PointXYZI>::Ptr tempcloud(new pcl::PointCloud<pcl::PointXYZI>);//当前帧点云（雷达里程计坐标系）
			if(lidarCloudMsgs_ != nullptr)
				pcl::fromROSMsg(*lidarCloudMsgs_, *tempcloud);//获取当前帧点云数据
			std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> outputclouds;//多个激光雷达数据包，向量中每个元素为一个激光雷达一帧数据
			std::vector<pcl::PointXYZI> lidarpropertys;//每一个PointType类型都表示一个单独点
			analysisCloud(tempcloud,outputclouds,lidarpropertys);
			//将点云投影到栅格地图显示，并将有点云的栅格标记为非悬崖区域。耗时3ms左右
			vertical_roi_cloud_->clear();
			mtx_verwall_.lock();
			Eigen::Quaterniond q_noyaw(R*(yaw_.matrix().inverse()));
			pcl::transformPointCloud(*tempcloud, *tempcloud, Eigen::Vector3d(0,0,0), q_noyaw);
			pcl::PointCloud<pcl::PointXYZI>::Ptr blocks_j_beginicld(new pcl::PointCloud<pcl::PointXYZI>);
			for(auto pt : tempcloud->points){
				if(ptUseful(pt, 30) && pt.azimuth > 45 && pt.azimuth < 135 && pt.z > 0.4){
					vertical_roi_cloud_->points.push_back(pt);
				}
				if(pt.range < 80 && pt.range > 0){
					blocks_j_beginicld->points.push_back(pt);
				}
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
					auto ptr_msg_show = grid_msg_show.ptr<unsigned char>(GRIDWH - 1 - row);
					if(y > 0)
						ptr_msg_show[col] = 100;//代表非悬崖
					ptr[3*col + 1] = 255;
				}
			}
			mtx_verwall_.unlock();
#ifdef CLOUDVIEWER
			high_cloud_->clear();
			low_cloud_->clear();
#endif //CLOUDVIEWER
			//利用16线进行检测
			Detection16(outputclouds, grid_show, q_noyaw);
			//利用32线进行检测
			Detection32(outputclouds, grid_show, q_noyaw);
			double t_end = ros::Time::now().toSec();
			//			std::cout << "@stiff_detection: time cost --- " << (t_end - t_begin) * 1000 << "ms" << std::endl;

			vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clouds;
			double t1 = ros::Time::now().toSec();
			//下采样。耗时2ms左右
			//			pcl::UniformSampling<pcl::PointXYZI> filter;
			//			filter.setInputCloud(vertical_roi_cloud_);
			//			filter.setRadiusSearch(0.2f);
			//			pcl::PointCloud<int> keypointIndices;
			//			filter.compute(keypointIndices);
			//			pcl::copyPointCloud(*vertical_roi_cloud_, keypointIndices.points, *vertical_roi_cloud_);
			//			double t2 = ros::Time::now().toSec();
			//			std::cout << "time cost " << t2 - t1 << std::endl;


			//发送消息
			PublishMsg(grid_show, grid_msg_show, lidarCloudMsgs_->header.stamp);
			if(visual_on){
				cv::namedWindow("gridshow",CV_WINDOW_NORMAL);
				cv::imshow("gridshow", grid_show);
				cv::waitKey(3);
			}
#ifdef CLOUDVIEWER
			//			std::cout << "@stiff_detection: the size is  " << vertical_roi_cloud_->size() << std::endl;
			//			if(clouds.size() > 0){//
			//				std::cout << "clouds size is " << clouds.size() << std::endl;
			//				showClouds(cloud_viewer_, clouds);
			//				//				ShowCloud(cloud_viewer_, clouds[0]);
			//			}



			ShowCloud(cloud_viewer_, tempcloud);
			cloud_viewer_->spinOnce();
#endif //CLOUDVIEWER

		}
		//		usleep(30000);

	}
}
bool StiffDetection::ptUseful(pcl::PointXYZI& pt, float dis_th){
	float  z = pt.z;
	float  x = pt.x;
	float  y = pt.y;
	float dis = sqrt(x*x + y*y + z*z);
	if(pt.range < 0 || dis > dis_th || z > 3 || z < -30 || y < 0 || x > 100)
		return false;
	return true;
}
void StiffDetection::pub1(){
	stiff_msgs::stiffwater msg_send;
	msg_send.header.stamp = ros::Time::now();
	msg_send.header.frame_id = "stiffwater";
	msg_send.ogmheight = 351;
	msg_send.ogmwidth = 201;
	msg_send.resolution = 0.2;
	msg_send.vehicle_x = 100;
	msg_send.vehicle_y = 100;
	//	msg_send.monitor_state = 1;//1代表gps超过1s未收到数据
	msg_send.data = vector<short>(msg_send.ogmheight * msg_send.ogmwidth, 0);
	pub_Stiff_.publish(msg_send);
}
void StiffDetection::pub2(){
	stiff_msgs::stiffwater msg_send;
	msg_send.header.stamp = ros::Time::now();
	msg_send.header.frame_id = "stiffwater";
	msg_send.ogmheight = 351;
	msg_send.ogmwidth = 201;
	msg_send.resolution = 0.2;
	msg_send.vehicle_x = 100;
	msg_send.vehicle_y = 100;
	//	msg_send.monitor_state = 2;//2代表lidar超过1s未收到数据
	msg_send.data = vector<short>(msg_send.ogmheight * msg_send.ogmwidth, 0);
	pub_Stiff_.publish(msg_send);
}
void StiffDetection::Detection16(vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> outputclouds ,cv::Mat grid_show, Eigen::Quaterniond q){
	//有时会丢失点云outputclouds里会不足三个点云，此时不能继续进行
	if(outputclouds.size() < 3) return;
	//将点云矫正到水平面
	pcl::PointCloud<pcl::PointXYZI>::Ptr new16_left(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr new16_right(new pcl::PointCloud<pcl::PointXYZI>);
#ifdef NEW
	if(outputclouds[1]->size() > 0)
		pcl::transformPointCloud(*outputclouds[1], *new16_left, Eigen::Vector3d(0,0,0), q);
	if(outputclouds[2]->size() > 0)
		pcl::transformPointCloud(*outputclouds[2], *new16_right, Eigen::Vector3d(0,0,0), q);
	vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> new_clouds{new16_left, new16_right};
#else
	new16_left = outputclouds[1];
	new16_right = outputclouds[2];
#endif //NEW
	//设置有效点云的最远有效阈值
	float far_bound = FAR_BOUND;
	const int layer = 16;
	int round = min(outputclouds[1]->points.size() / layer, outputclouds[2]->points.size() / layer);
	//记录连续丢失点的j和窗口起始i
	std::map<int, int> j_begini_1;
	std::map<int ,int> j_begini_2;
	for(int j = 0; j < 16; ++j){
		for(int i = 0; i + window_big_ < round - 20; ){

			//
			//将垂直墙检测感兴趣区域放入vertical_roi_cloud_,后续进行检测
			//			int index_ver = i * layer + j;
			//			auto pt_ver1 = outputclouds[1]->points[index_ver];
			//			if(ptUseful(pt_ver1, 30) && pt_ver1.azimuth > 45 && pt_ver1.azimuth < 135){
			//				vertical_roi_cloud_->points.push_back(pt_ver1);
			//			}
			//			auto pt_ver2 = outputclouds[2]->points[index_ver];
			//			if(ptUseful(pt_ver2, 30) && pt_ver2.azimuth > 45 && pt_ver2.azimuth < 135){
			//				vertical_roi_cloud_->points.push_back(pt_ver2);
			//			}
			//height_diff_most 大窗口内两个小窗口间最大平均高度差
			//x0，y0，x1，y1，z0，z1 高低小窗口内点云的平均坐标
			float  height_diff_most = 10, x0 = 0, y0 = 0, x1 = 0, y1 = 0, z0 = 0, z1 = 0;

			//i_begin 记录大窗口内最大高度差的小窗口起始索引
			int i_begin = 0;
			//大悬崖无返回点
			int i_deep = 0;
			bool stop = false;
			//dis_ratio 小窗口间的距离比例
			//tangent 小窗口间的正切值
			float dis_ratio = 0, dis_tocheck = 0, tangent = 0;

			//大窗口循环--左雷达--在大窗口内找高度突变最大的小窗口
			for(int k = i; k + window_small_ < i + window_big_; ++k){
				float z_high = 0, y_high = 0, x_high = 0;
				float z_low = 0, y_low = 0, x_low = 0;
				float dis_av_high = 0, dis_av_low = 0;
				int count = 0;
				int cnt = 0;

				int cnt_ill = 0;
				//分别求两个小窗口的平均高度、距离
				//小窗口-前（靠近车这一端，不同的安装方式可能不同）
				for(int window_i = k; window_i < k + window_small_; window_i++){
					int index = window_i * layer + j;
					float  z = outputclouds[1]->points[index].z;
					float  x = outputclouds[1]->points[index].x;
					float  y = outputclouds[1]->points[index].y;
					float dis = sqrt(x*x + y*y + z*z);
					if(ptUseful(outputclouds[1]->points[index], NEAR_BOUND)) cnt++;
					if(!ptUseful(outputclouds[1]->points[index], NEAR_BOUND)) continue;
					//利用姿态角矫正
					Eigen::Vector3d pt(x, y, z);
					pt = q * pt;
					//对各个有效点计数并对坐标加和
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
				//求平均值
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
					if(outputclouds[1]->points[index].range < 0
							&& dis > 80) cnt_ill++;//距离特别近也会造成无效点
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
				if(count > 0) {
					z_low = z_low / count;
					x_low = x_low / count;
					y_low = y_low / count;
					dis_av_low /= count;
				}
				float height_diff = 0;
				if(abs(z_high) > 0.0001 && abs(z_low) > 0.0001){
					height_diff =  z_low - z_high;
				}
				//处理深不见底
				float dis_deep = sqrt(x_high*x_high + y_high*y_high);
				if(dis_deep < 20 && dis_deep != 0 && cnt_ill >= window_small_ - 1 &&
						abs(z_high) > 0.0001 && z_high < 0.5 && !stop){
					i_deep = k;

					stop = true;
				}
				//寻找高度差最大的两个小窗口
				if(height_diff < th_height && height_diff < height_diff_most && count > 0){
					//距离突变
					dis_ratio = dis_av_high / dis_av_low;
					//					dis_tocheck = dis_av_high;
					//寻找高度最大的突变
					height_diff_most = height_diff;
					//并将平均坐标记录下来
					x0 = x_high; y0 = y_high;z0 = z_high;
					x1 = x_low; y1 = y_low;z1 = z_low;
					float radius = sqrt(pow((x1 - x0), 2) + pow((y1 - y0), 2));
					//求两个突变窗口的正切值
					tangent = (z0 - z1) / radius;
					//记录窗口的起始位置
					i_begin = k;
				}
			}
			//大悬崖无返回点
#ifdef BIG
			if(i_deep != 0 ){//
				j_begini_1[j] = i_deep;
			}
#endif //BIG
			//如果存在两个小窗口满足高度差要求的话
			if(i_begin != 0){
				float z_diff_nb = 0;
				int index_high = -1, index_low = -1;
				//找到两个小窗口间相邻的两个有效点，想利用这两个点的信息做些工作
				//目前没用
				//				for(int i = 0; i < window_small_; ++i){
				//					int i_high = (i_begin + window_small_ - 1 - i) * layer + j;
				//					int i_low = (i_begin + window_small_ + i) * layer + j;
				//					auto pt_high = outputclouds[1]->points[i_high];
				//					auto pt_low = outputclouds[1]->points[i_low];
				//					if(ptUseful(pt_high, NEAR_BOUND) && index_high == -1)
				//						index_high = i_high;
				//					if(ptUseful(pt_low, far_bound) && index_low == -1)
				//						index_low = i_low;
				//					if(index_high != -1 && index_low != -1)
				//						break;
				//				}
				//				float dis_high = 0;
				//				float dis_low = 0;
				//
				//				//利用上面找到的有效点求出正切值
				//				if(index_high != -1 && index_low != -1){
				//					auto pt_high = outputclouds[1]->points[index_high];
				//					auto pt_low = outputclouds[1]->points[index_low];
				//#ifdef NEW
				//					auto pt_high_new = new16_left->points[index_high];
				//					auto pt_low_new = new16_left->points[index_low];
				//#endif//NEW
				//
				//#ifdef NEW
				//					float z_low = pt_low_new.z;
				//					float z_high = pt_high_new.z;
				//#else
				//					float z_low = pt_low.z;
				//					float z_high = pt_high.z;
				//#endif//NEW
				//					float y_high = pt_high.y, x_high = pt_high.x;
				//					float y_low = pt_low.y, x_low = pt_low.x;
				//					dis_high = pt_high.range;
				//					dis_low = pt_low.range;
				//					float radius = sqrt(pow((y_high - y_low), 2) + pow((x_high - x_low), 2));
				//#ifdef NEIGHBOUR
				//					tangent = (z_high - z_low) / radius;
				////					dis_ratio = dis_high / dis_low;
				//#endif //NEIGHBOUR
				//					z_diff_nb = z_high - z_low;
				//				}
				float offset = 0;
				//				if(dis_high < 10)
				//					offset = 0;
				//利用距离突变和正切值作为阈值进行筛选
				if(((dis_ratio < th_dis + offset&& dis_ratio > 0.2) && z0 < th_z0 && z1 < th_z1 && tangent > th_tan_16)
				){// || (dis_tocheck < 15 && z_diff_nb > 1 && z0 < 0.5 && tangent > 0.15)
					//						std::cout << "tanget is ... " << tangent << std::endl;
					//					std::cout  << z1 <<  "  ... " <<  z0 << std::endl;

					//根据上面找到的窗口的起始位置将这个小窗口的点云标记出来
#ifdef CLOUDVIEWER

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
							high_cloud_->points.push_back(pt_high_ori);//todo:历史遗留问题
#else
							high_cloud_->points.push_back(pt_high_ori);
#endif //NEW
						}
						z = pt_low.z;
						x = pt_low.x;
						y = pt_low.y;
						dis = pt_low.range;
						if(!ptUseful(pt_low_ori, far_bound)) continue;
#ifdef NEW
						low_cloud_->points.push_back(pt_low_ori);
#else
						low_cloud_->points.push_back(pt_low_ori);
#endif //NEW
					}
#endif //CLOUDVIEWER

					//画线，连接两个小窗口的平均坐标作为悬崖区域，有一定的缩短
					int begin_x = (x0 + 35) / 0.2, begin_y = (y0 + 20) / 0.2; begin_y = GRIDWH - 1 - begin_y;
					int end_x = (x1 + 35) / 0.2, end_y = (y1 + 20) / 0.2; end_y = GRIDWH - 1 - end_y;
					if(begin_y < end_y){
						swap(begin_x, end_x);
						swap(begin_y, end_y);
					}
					float ratio = 10 / sqrt(pow((end_y - begin_y), 2) + pow((end_x - begin_x), 2));
					//如果画线太长的话按比例缩短
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



			//复位，开始处理右侧雷达，或许这两个可以用一个循环解决
			height_diff_most = 10;
			x0 = 0; y0 = 0; x1 = 0; y1 = 0; z0 = 0; z1 = 0;
			i_begin = 0;
			dis_ratio = 0;
			//大悬崖无返回点
			i_deep = 0;
			//大窗口循环 -- 右侧雷达
			for(int k = i; k + window_small_ < i + window_big_; ++k){
				//height_diff_most 大窗口内两个小窗口间最大平均高度差
				//x0，y0，x1，y1，z0，z1 高低小窗口内点云的平均坐标
				float z_high = 0, y_high = 0, x_high = 0;
				float z_low = 0, y_low = 0, x_low = 0;
				float dis_av_high = 0, dis_av_low = 0;
				int count = 0;
				int cnt_ill = 0;
				//小窗口-近处----注意这里和左侧雷达不同索引的顺序
				for(int window_i = k + window_small_; window_i < k + 2 * window_small_; window_i++){
					int index = window_i * layer + j;
					float  z = outputclouds[2]->points[index].z;
					float  x = outputclouds[2]->points[index].x;
					float  y = outputclouds[2]->points[index].y;
					float dis = sqrt(x*x + y*y + z*z);
					if(!ptUseful(outputclouds[2]->points[index], NEAR_BOUND)) continue;
					count ++;
					dis_av_high +=dis;
					//矫正点
					Eigen::Vector3d pt(x, y, z);
					pt = q * pt;
					//求和--求平均
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
				//小窗口-远处
				for(int window_i = k; window_i < k + window_small_; window_i++){
					int index = window_i * layer + j;
					float  z = outputclouds[2]->points[index].z;
					float  x = outputclouds[2]->points[index].x;
					float  y = outputclouds[2]->points[index].y;
					float dis = sqrt(x*x + y*y + z*z);
					if(outputclouds[2]->points[index].range < 0
							&& dis > 80) cnt_ill++;//距离特别近也会是无效点
					if(!ptUseful(outputclouds[2]->points[index], far_bound)) continue;
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
				//处理深不见底
				float dis_deep = sqrt(x_high*x_high + y_high*y_high);
				if(dis_deep < 20 && dis_deep != 0 && cnt_ill >= window_small_ - 1 &&
						abs(z_high) > 0.0001 && z_high < 0.5 ){
					i_deep = k;
					stop = true;
				}
				//
				float height_diff = 0;
				if(abs(z_high) > 0.0001 && abs(z_low) > 0.0001){
					height_diff =  z_low - z_high;
				}
				if(height_diff < th_height && height_diff < height_diff_most && count > 0){
					dis_ratio = dis_av_high / dis_av_low;
					dis_tocheck = dis_av_high;
					height_diff_most = height_diff;
					x0 = x_high; y0 = y_high;z0 = z_high;
					x1 = x_low; y1 = y_low;z1 = z_low;
					float radius = sqrt(pow((x1 - x0), 2) + pow((y1 - y0), 2));
					tangent = (z0 - z1) / radius;
					i_begin = k;
				}
			}
#ifdef BIG
			//大悬崖无返回点
			if(i_deep != 0 ){//
				j_begini_2[j] = i_deep;
			}
#endif //BIG
			if(i_begin != 0){//
				//寻找相邻窗口点
				bool ok = false;
				float z_diff_nb = 0;
				int index_high = -1, index_low = -1;
				//找到两个小窗口间相邻的两个有效点，想利用这两个点的信息做些工作
				//目前没用
				//				for(int i = 0; i < window_small_; ++i){
				//					int i_low = (i_begin + window_small_ - 1 - i) * layer + j;
				//					int i_high = (i_begin + window_small_ + i) * layer + j;
				//					auto pt_high = outputclouds[2]->points[i_high];
				//					auto pt_low = outputclouds[2]->points[i_low];
				//					if(ptUseful(pt_high, NEAR_BOUND) && index_high == -1)
				//						index_high = i_high;
				//					if(ptUseful(pt_low, far_bound) && index_low == -1)
				//						index_low = i_low;
				//					if(index_high != -1 && index_low != -1)
				//						break;
				//				}
				//				float dis_high = 0;
				//				float dis_low = 0;
				//				if(index_high != -1 && index_low != -1){
				//					auto pt_high = outputclouds[2]->points[index_high];
				//					auto pt_low = outputclouds[2]->points[index_low];
				//#ifdef NEW
				//					auto pt_high_new = new16_right->points[index_high];
				//					auto pt_low_new = new16_right->points[index_low];
				//#endif//NEW
				//
				//#ifdef NEW
				//					float z_low = pt_low_new.z;
				//					float z_high = pt_high_new.z;
				//#else
				//					float z_low = pt_low.z;
				//					float z_high = pt_high.z;
				//#endif//NEW
				//					float y_high = pt_high.y, x_high = pt_high.x;
				//					float y_low = pt_low.y, x_low = pt_low.x;
				//					dis_high = pt_high.range;
				//					dis_low = pt_low.range;
				//					float radius = sqrt(pow((y_high - y_low), 2) + pow((x_high - x_low), 2));
				//#ifdef NEIGHBOUR
				//					tangent = (z_high - z_low) / radius;
				////					dis_ratio = dis_high / dis_low;
				//#endif //NEIGHBOUR
				//					z_diff_nb = z_high - z_low;
				//					if(z_high - z_low > 0.5)
				//						ok = true;
				//				}
				float offset = 0;

				//				if(dis_high < 10)
				//					offset = 0;
				if(((dis_ratio < th_dis + offset && dis_ratio > 0.2) && z0 < th_z0 && z1 < th_z1 && tangent > th_tan_16)
				){//(dis_tocheck < 15 && z_diff_nb > 1 && z0 < 0.5 && tangent > 0.15)
					//					std::cout << "tangent ... " << tangent << std::endl;
					//					std::cout  << z1 <<  "  ... " <<  z0 << std::endl;

					//根据上面找到的窗口的起始位置将这个小窗口的点云标记出来
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
#ifdef CLOUDVIEWER
							high_cloud_->points.push_back(pt_high);
#endif //CLOUDVIEWER
#else
							high_cloud_->points.push_back(pt_high);
#endif//NEW
						}
						z = pt_low.z;
						x = pt_low.x;
						y = pt_low.y;
						dis = pt_low.range;
						if(!ptUseful(pt_low, far_bound)) continue;
#ifdef NEW
#ifdef CLOUDVIEWER
						low_cloud_->points.push_back(pt_low);
#endif //CLOUDVIEWER
#else
						low_cloud_->points.push_back(pt_low);
#endif//NEW
					}
					//如果画线太长的话按比例缩短
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
					//画线，连接两个小窗口的平均坐标作为悬崖区域，有一定的缩短
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
#ifdef BIG
	//将j_begini进行分组，间隔近的分为一组，每组的线束足够多才认为可能是悬崖
	//-----左侧雷达
	{
		vector<vector<pair<int,int> > > blocks_j_begini;
		if(j_begini_1.size() > 0){
			auto it = j_begini_1.begin(),it_next = std::next(it);
			blocks_j_begini.push_back({std::make_pair(it->first, it->second)});
			for(;it_next != j_begini_1.end();it++,it_next++){
				vector<pair<int,int> >& vec_now = *(std::prev(blocks_j_begini.end()));
				int i1 = it_next->first, i0 = it->first;
				if(i1 - i0 < 2){
					vec_now.push_back(std::make_pair(i1, j_begini_1[i1]));
				}else{
					blocks_j_begini.push_back({std::make_pair(i1, j_begini_1[i1])});
				}
			}
		}
		for(auto out : blocks_j_begini){//out --- vector<pair<int,int> >
			if(out.size() > 5){//每组有超过5条线束
				for(auto in : out){//in --- <pair<int,int>
					int j = in.first, i_deep = in.second;
					float x_begin,y_begin,x_end,y_end;
					bool begin_found = false, end_found  = false;
					for(int k = i_deep; k <  i_deep + window_small_; ++k){
						int index_high = k * layer + j;
						int index_ = (k + window_small_) * layer + j;
						auto pt_high = new16_left->points[index_high];
						auto pt_ = new16_left->points[index_];
						high_cloud_->points.push_back(pt_high);
						//					std::cout << pt_high.x << " " << pt_high.y <<  " "
						//							 << pt_high.z << " " << pt_high.range << std::endl;
						if(pt_high.range > 0){
							if(!begin_found){
								x_begin = pt_high.x;
								y_begin = pt_high.y;
								begin_found = true;
							}
							if(begin_found && !end_found){
								x_end = pt_high.x;
								y_end = pt_high.y;
								end_found = true;
							}
						}
					}
					if(begin_found && end_found){
						if(x_begin > -2 && x_begin < 2 && y_begin < 4){}
						else{
							int begin_x = (x_begin + 35) / 0.2, begin_y = (y_begin + 20) / 0.2; begin_y = GRIDWH - 1 - begin_y;
							int end_x = (x_end + 35) / 0.2, end_y = (y_end + 20) / 0.2; end_y = GRIDWH - 1 - end_y;
							if(begin_x >= 0 && begin_x < GRIDWH && end_x >= 0 && end_x < GRIDWH
									&& begin_y >=0 && begin_y < GRIDWH
									&& end_y >=0 && end_y < GRIDWH){
								cv::line(grid_show, cvPoint(begin_x, begin_y), cvPoint(end_x, end_y), cvScalar(0,125,125),5);
							}
						}
					}
				}
			}
		}
	}
	//------右侧雷达
	{
		vector<vector<pair<int,int> > > blocks_j_begini;
		if(j_begini_2.size() > 0){
			auto it = j_begini_2.begin(),it_next = std::next(it);
			blocks_j_begini.push_back({std::make_pair(it->first, it->second)});
			for(;it_next != j_begini_2.end();it++,it_next++){
				vector<pair<int,int> >& vec_now = *(std::prev(blocks_j_begini.end()));
				int i1 = it_next->first, i0 = it->first;
				if(i1 - i0 < 2){
					vec_now.push_back(std::make_pair(i1, j_begini_2[i1]));
				}else{
					blocks_j_begini.push_back({std::make_pair(i1, j_begini_2[i1])});
				}
			}
		}
		for(auto out : blocks_j_begini){//out --- vector<pair<int,int> >
			if(out.size() > 5){//每组有超过5条线束
				for(auto in : out){//in --- <pair<int,int>
					int j = in.first, i_deep = in.second;
					float x_begin,y_begin,x_end,y_end;
					bool begin_found = false, end_found  = false;
					for(int k = i_deep + window_small_; k <  i_deep + 2 * window_small_; ++k){
						int index_high = k * layer + j;
						int index_ = (k - window_small_) * layer + j;
						auto pt_high = new16_right->points[index_high];
						auto pt_ = new16_right->points[index_];
//						std::cout << pt_.x << " " << pt_.y << " " << pt_.z << " " << pt_.range << std::endl;
						high_cloud_->points.push_back(pt_high);
						if(pt_high.range > 0){
							if(!begin_found){
								x_begin = pt_high.x;
								y_begin = pt_high.y;
								begin_found = true;
							}
							if(begin_found && !end_found){
								x_end = pt_high.x;
								y_end = pt_high.y;
								end_found = true;
							}
						}
					}
					if(begin_found && end_found){
						if(x_begin > -2 && x_begin < 2 && y_begin < 4){}
						else{
							int begin_x = (x_begin + 35) / 0.2, begin_y = (y_begin + 20) / 0.2; begin_y = GRIDWH - 1 - begin_y;
							int end_x = (x_end + 35) / 0.2, end_y = (y_end + 20) / 0.2; end_y = GRIDWH - 1 - end_y;
							if(begin_x >= 0 && begin_x < GRIDWH && end_x >= 0 && end_x < GRIDWH
									&& begin_y >=0 && begin_y < GRIDWH
									&& end_y >=0 && end_y < GRIDWH){
								cv::line(grid_show, cvPoint(begin_x, begin_y), cvPoint(end_x, end_y), cvScalar(125,0,125),5);
							}
						}
					}
				}
			}
		}
	}
#endif //BIG
}
//32线检测的总体思路和16线类似
void StiffDetection::Detection32(vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> outputclouds,
		cv::Mat grid_show, Eigen::Quaterniond q){
	//有时会丢失点云outputclouds里会不足三个点云，此时不能继续进行
	if(outputclouds.size() < 3) return;
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = outputclouds[0];
	pcl::PointCloud<pcl::PointXYZI>::Ptr newcloud(new pcl::PointCloud<pcl::PointXYZI>);
	//将点云矫正到水平面
#ifdef NEW
	pcl::transformPointCloud(*cloud, *newcloud, Eigen::Vector3d(0,0,0), q);
#else
	newcloud = cloud;
#endif //NEW

	const int round32 = cloud->size() / 32;
	//设置有效点云的最远有效阈值
	float far_bound = FAR_BOUND;
	for(int j = 0; j < 25; ++j){
		for(int i = 0; i + window_big_ < round32 - 100; i+=window_small_){

			//			int index_ver0 = i * 32 + j;
			//			auto pt_ver0 = outputclouds[0]->points[index_ver0];
			//			if(ptUseful(pt_ver0, 30) && pt_ver0.azimuth > 45 && pt_ver0.azimuth < 135){
			//				vertical_roi_cloud_->points.push_back(pt_ver0);
			//			}
			//			std::cout << "=== " << i << std::endl;


			//dis_ratio 小窗口间的距离比例
			//tangent 小窗口间的正切值
			//i_begin 记录大窗口内最大高度差的小窗口起始索引
			float  height_diff_most = 10, x0 = 0, y0 = 0, x1 = 0, y1 = 0, z0 = 0, z1 = 0;
			float dis_in_window = 0;
			int i_begin = 0;
			float dis_ratio = 0, dis_tocheck = 0, tangent = 0;

			//大窗口循环--左雷达--在大窗口内找高度突变最大的小窗口
			for(int k = i; k + window_small_ < i + window_big_; ++k){
				//height_diff_most 大窗口内两个小窗口间最大平均高度差
				//x0，y0，x1，y1，z0，z1 高低小窗口内点云的平均坐标
				float z_high = 0, y_high = 0, x_high = 0;
				float z_low = 0, y_low = 0, x_low = 0;
				float dis_av_high = 0, dis_av_low = 0;
				int count = 0;
				int cnt = 0;
				//分别求两个小窗口的平均高度、距离
				//小窗口 --- 靠近车这一端
				for(int window_i = k; window_i < k + window_small_; window_i++){
					int index = window_i * 32 + map_j[j];
					float  z = newcloud->points[index].z;
					float  x = cloud->points[index].x;
					float  y = cloud->points[index].y;
					float dis = sqrt(x*x + y*y + z*z);
					int ran_cnt = 0;
					float ran_z_high = 0, ran_x_high = 0, ran_y_high = 0, ran_dis_high = 0;
					//进行一个类似ransac的过程，寻找最好的能表示z的值---小窗口内占大多数的距离值
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
				//求平均
				if(count > 0) {
					z_high = z_high / count;
					y_high = y_high / count;
					x_high = x_high / count;
					dis_av_high /= count;
					count = 0;
				}else {
					continue;
				}
				//小窗口---远离车
				for(int window_i = k + window_small_; window_i < k + 2 * window_small_; window_i++){
					int index = window_i * 32 + map_j[j];
					float  z = newcloud->points[index].z;
					float  x = cloud->points[index].x;
					float  y = cloud->points[index].y;
					float dis = sqrt(x*x + y*y + z*z);
					//进行一个类似ransac的过程，寻找最好的能表示z的值---小窗口内占大多数的距离值
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
				//求平均
				if(count > 0) {
					z_low = z_low / count;
					x_low = x_low / count;
					y_low = y_low / count;
					dis_av_low /= count;
				}else {
					continue;
				}
				//如果高度反了，交换一下
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
				//寻找高度差最大的两个小窗口
				if(height_diff < th_height && height_diff < height_diff_most && count > 0){
					//距离突变
					dis_ratio = dis_av_high / dis_av_low;
					//					dis_tocheck = dis_av_high;
					//寻找高度最大的突变
					height_diff_most = height_diff;
					//并将平均坐标记录下来
					x0 = x_high; y0 = y_high;z0 = z_high;
					x1 = x_low; y1 = y_low;z1 = z_low;
					//记录窗口的起始位置
					i_begin = k;
					float radius = sqrt(pow((x1 - x0), 2) + pow((y1 - y0), 2));
					//求两个突变窗口的正切值
					tangent = (z0 - z1) / radius;
					//					dis_in_window = ys_in_window[ys_in_window.size() / 2] - ys_in_window[0];
				}
			}
			//如果存在两个小窗口满足高度差要求的话
			if(i_begin != 0){//
				//尝试用窗口相邻高度突变
				bool ok = false;
				float z_diff_nb = 0;
				//找到两个小窗口间相邻的两个有效点，想利用这两个点的信息做些工作
				//目前没用
				//				int index_high = -1, index_low = -1;
				//				for(int i = 0; i < window_small_; ++i){
				//					int i_high = (i_begin + window_small_ - 1 - i) * 32 + map_j[j];
				//					int i_low = (i_begin + window_small_ + i) * 32 + map_j[j];
				//					auto pt_high = cloud->points[i_high];
				//					auto pt_low = cloud->points[i_low];
				//					if(pt_high.range > 0 && index_high == -1)
				//						index_high = i_high;
				//					if(pt_low.range > 0 && index_low == -1)
				//						index_low = i_low;
				//					if(index_high != -1 && index_low != -1){
				//						if(pt_high.z < pt_low.z){
				//							swap(index_high, index_low);
				//						}
				//						break;
				//					}
				//				}
				//				if(index_high != -1 && index_low != -1){
				//					auto pt_high = cloud->points[index_high];
				//					auto pt_low = cloud->points[index_low];
				//					float z_low = pt_low.z;
				//					float z_high = pt_high.z;
				//					float y_high = pt_high.y, x_high = pt_high.x;
				//					float y_low = pt_low.y, x_low = pt_low.x;
				//					float dis_high = pt_high.range;
				//					float dis_low = pt_low.range;
				//					float radius = y_high - y_low;
				//#ifdef NEIGHBOUR
				//					tangent = (z_high - z_low) / radius;
				////					dis_ratio = dis_high / dis_low;
				//#endif //NEIGHBOUR
				//					z_diff_nb = z_high - z_low;
				//					if(z_high - z_low > 0.5)
				//						ok = true;
				//				}
				//利用距离突变和正切值作为阈值进行筛选
				if(((dis_ratio < th_dis && dis_ratio > 0.2) && z0 < th_z0 && z1 < th_z1 && tangent > th_tan_32)){// ||
					//					(dis_tocheck < 15 && z_diff_nb > 1 && z0 < 0.5 && tangent > th)
					//					std::cout << "tanget is ... " << tangent << std::endl;
					//					std::cout  << z1 <<  "  ... " <<  z0 << std::endl;

					//					std::cout << ">>>>>>>count>>>>>>>>>> " << count << std::endl;
					//					std::cout << ">>>>>>>z0   >>>>>>>>>> " << z0 << std::endl;

					//根据上面找到的窗口的起始位置将这个小窗口的点云标记出来
#ifdef CLOUDVIEWER

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
							high_cloud_->points.push_back(pt_high_ori);
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
						low_cloud_->points.push_back(pt_low_ori);
					}

#endif //CLOUDVIEWER

					{//test
						//						for(int k = i_begin - 5; k < i_begin + window_small_; ++k){
						//							int index = k * 32 + j;
						//							auto pt = cloud->points[index];
						//							std::cout << pt.z << std::endl;
						//						}
					}
					//画线，连接两个小窗口的平均坐标作为悬崖区域，有一定的缩短
					int begin_x = (x0 + 35) / 0.2, begin_y = (y0 + 20) / 0.2; begin_y = GRIDWH - 1 - begin_y;
					int end_x = (x1 + 35) / 0.2, end_y = (y1 + 20) / 0.2; end_y = GRIDWH - 1 - end_y;
					if(begin_y < end_y){
						swap(begin_x, end_x);
						swap(begin_y, end_y);
					}
					//					if(begin_x > GRIDWH / 2)
					//						std::cout << "the z0 z1 -- " << z0 << " " << z1 << " " <<
					//						i_begin << std::endl;
					//如果画线太长的话按比例缩短
					float ratio = 10 / sqrt(pow((end_y - begin_y), 2) + pow((end_x - begin_x), 2));
					if(ratio < 1){
						end_x = begin_x + ratio * (end_x - begin_x);
						end_y = begin_y + ratio * (end_y - begin_y);
					}
					if(begin_x >= 0 && begin_x < GRIDWH && end_x >= 0 && end_x < GRIDWH
							&& begin_y >=0 && begin_y < GRIDWH
							&& end_y >=0 && end_y < GRIDWH){
						cv::line(grid_show, cvPoint(begin_x, begin_y), cvPoint(end_x, end_y), cvScalar(0,0,125),2);
					}
				}
			}
		}
	}
	//	std::cout << "222222222" << std::endl;
}
void StiffDetection::verticalWallDetect(){
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZI>);
	//cloudviewer的初始化必须和显示在同一个线程
#ifdef CLOUDVIEWER_VER
	boost::shared_ptr<PCLVisualizer> cloud_viewer_ (new PCLVisualizer("stiffdetection cloud"));
	PrepareViewer(cloud_viewer_);
#endif //CLOUDVIEWER
	ros::Rate r(5);
	while(ros::ok()){
		pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2), cloud_filtered_blob (new pcl::PCLPointCloud2);
		pcl::PointCloud<pcl::PointXYZI>::Ptr  cloud_in(new pcl::PointCloud<pcl::PointXYZI>)
		,cloud_p (new pcl::PointCloud<pcl::PointXYZI>), cloud_f (new pcl::PointCloud<pcl::PointXYZI>);
		//	// Fill in the cloud data
		//	pcl::PCDReader reader;
		//	reader.read ("table_scene_lms400.pcd", *cloud_blob);
		//
		//
		//	std::cerr << "PointCloud before filtering: " << cloud_blob->width * cloud_blob->height << " data points." << std::endl;
		//
		//	// Create the filtering object: downsample the dataset using a leaf size of 1cm
		if(vertical_roi_cloud_->size() == 0){
			usleep(100000);
			continue;
		}
		double t1 = ros::Time::now().toSec();
		//实验证明，这里加锁还是很有必要的
		mtx_verwall_.lock();
		pcl::copyPointCloud(*vertical_roi_cloud_, *cloud_in);
		mtx_verwall_.unlock();
		double t2 = ros::Time::now().toSec();
		//		cloud_in = vertical_roi_cloud_;
		//		pcl::UniformSampling<pcl::PointXYZI> filter;
		//		filter.setInputCloud(vertical_roi_cloud_);
		//		filter.setRadiusSearch(0.3f);
		//		pcl::PointCloud<int> keypointIndices;
		//		filter.compute(keypointIndices);
		//		pcl::copyPointCloud(*vertical_roi_cloud_, keypointIndices.points, *cloud_in);
		//		sor.filter (*cloud_filtered_blob);
		//
		//	// Convert to the templated PointCloud
		//	pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *cloud_filtered);
		//
		//	std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;
		//
		//	// Write the downsampled version to disk
		//	pcl::PCDWriter writer;
		//	writer.write<pcl::PointXYZ> ("table_scene_lms400_downsampled.pcd", *cloud_filtered, false);

		pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
		pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
		// Create the segmentation object
		pcl::SACSegmentation<pcl::PointXYZI> seg;
		// Optional
		seg.setOptimizeCoefficients (true);
		// Mandatory
		seg.setModelType (pcl::SACMODEL_PLANE);
		seg.setMethodType (pcl::SAC_RANSAC);
		seg.setMaxIterations (1000);
		seg.setDistanceThreshold (0.03);

		// Create the filtering object
		pcl::ExtractIndices<pcl::PointXYZI> extract;

		int i = 0;// nr_points = (int) vertical_roi_cloud_->points.size ();
		//	-0.00775922  -0.99996  -0.00453188
		//	-0.00860226  -0.999912  0.0100643

		// While 30% of the original cloud is still there
		cloud_out->clear();
		while (i < 2) //cloud_in->points.size () > 0.3 * nr_points
			//TODO:这里可以加一个for循环
			//	if(1)
		{

			inliers->indices.clear();

			if(cloud_in->size() < 10) break;
			std::chrono::steady_clock::time_point  now = std::chrono::steady_clock::now();
			// Segment the largest planar component from the remaining cloud
			seg.setInputCloud (cloud_in);
			seg.segment (*inliers, *coefficients);
			if (inliers->indices.size () == 0)
			{
				std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
				break;
			}
			//			std::cout << "the " << i <<"'s ieration has"<<
			//					cloud_in->size() << "points" << std::endl;
			// Extract the inliers
			extract.setInputCloud (cloud_in);
			extract.setIndices (inliers);
			extract.setNegative (false);
			extract.filter (*cloud_p);
			//		std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;
			Eigen::Vector3d abc(coefficients->values[0],coefficients->values[1],coefficients->values[2]);
			abc.normalize();


			std::stringstream ss;
			//		writer.write<pcl::PointXYZ> (ss.str (), *cloud_p, false);

			// Create the filtering object
			extract.setNegative (true);
			extract.filter (*cloud_f);
			cloud_in.swap (cloud_f);
			if(abs(abc.z()) < 0.1){//
				*cloud_out += *cloud_p;
				break;
			}else if(abs(abc.z()) > 0.9){//
				std::cout << abc.x() << "  " << abc.y() << "  " << abc.z() << "  " <<
						coefficients->values[3]	<< std::endl;
				//				*cloud_out += *cloud_p;
			}
			i++;
			auto t2 = std::chrono::steady_clock::now();
			std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - now);
			//			printf("%f\n",time_span);
		}
#ifdef CLOUDVIEWER_VER
		if(cloud_out->size() > 0){
			ShowCloud(cloud_viewer_, cloud_out);
		}
		cloud_viewer_->spinOnce();
#endif//CLOUDVIEWER_VER
		r.sleep();
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
	//	msg_send.monitor_state = 0;
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
			else if(ptr_show[3*col+1]==255 && row > 100){
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
#ifdef CLOUDVIEWER
	if (high_cloud_->size() > 0)
	{
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> cloudHandler( high_cloud_, 255, 0, 0 );
		if (!cloud_viewer_->updatePointCloud(high_cloud_,cloudHandler, "high"))
		{
			cloud_viewer_->addPointCloud(high_cloud_, cloudHandler, "high");
		}
	}
	if (low_cloud_->size() > 0)
	{
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> cloudHandler( low_cloud_, 0, 0, 255 );
		if (!cloud_viewer_->updatePointCloud(low_cloud_,cloudHandler, "low"))
		{
			cloud_viewer_->addPointCloud(low_cloud_, cloudHandler, "low");
		}
	}
#endif //CLOUDVIEWER
}
void StiffDetection::showClouds(boost::shared_ptr<PCLVisualizer>& cloud_viewer_,
		vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> incloud){
	cloud_viewer_->removeAllPointClouds();
	if (incloud[0]->size() > 0)
	{
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> cloudHandler( incloud[0],255,0,0 );
		if (!cloud_viewer_->updatePointCloud(incloud[0],cloudHandler, "1"))
		{
			cloud_viewer_->addPointCloud(incloud[0], cloudHandler, "1");
		}
	}
	if (incloud[1]->size() > 0)
	{
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> cloudHandler( incloud[1],255,255,0 );
		if (!cloud_viewer_->updatePointCloud(incloud[1],cloudHandler, "2"))
		{
			cloud_viewer_->addPointCloud(incloud[1], cloudHandler, "2");
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
	q_lidar_msgs_.Push(msg);
	if(q_lidar_msgs_.Size() > 3){
		q_lidar_msgs_.Pop();
	}
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
#ifdef CLOUDVIEWER_VER
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
#endif //VIEWER_ver
}
