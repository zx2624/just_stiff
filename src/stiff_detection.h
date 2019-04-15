#ifndef STIFF_DEPTH_
#define STIFF_DEPTH_

#include <iostream>
#include <mutex>
#include <thread>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/keypoints/uniform_sampling.h>

#include <opencv2/core/core.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/shared_ptr.hpp>
#include "velodyne/HDL32Structure.h"
#include "common/blocking_queue.h"
#include "stiff_msgs/stiffwater.h"
#include "sensor_driver_msgs/GpswithHeading.h"
#include "std_msgs/Int8.h"
#include "stiff_detection/stiff_dy_paramConfig.h"
#include <dynamic_reconfigure/server.h>

//For plane segmentation
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>


#define PI 3.141592653
#define CLOUDVIEWER //悬崖检测可视化点云可视化
//#define CLOUDVIEWER_VER
#define GRIDWH 351
#define FAR_BOUND 45 //可用的远处点云范围
#define NEAR_BOUND 45 //可用的近处点云范围
//#define NEIGHBOUR
#define NEW

/*!
 * 此类作用：
 * 分别利用经过惯导信息矫正过投影到水平面上的32线和16线雷达的同一“层”
 * 激光雷达线之间的高度和距离突变特征，进行悬崖检测，并将检测结果进行
 * 适当的膨胀腐蚀处理投影到栅格地图进行消息发送。
 */
class StiffDetection{
public:
	/*!
	 * \brief 构造函数，传入节点句柄，订阅话题，开始处理线程
	 * \param nh 传入的节点句柄
	 */
	StiffDetection(ros::NodeHandle& nh);
	~StiffDetection();
	/*!
	 * \brief 主要的处理函数，分别调用Detection16和Detection32
	 * 进行利用两个16线和一个32线雷达进行悬崖检测。
	 */
	void process();
	/*!
	 * \brief 利用两个16线雷达进行悬崖检测。主要利用大窗口内小窗口
	 * 高度和距离突变是否满足阈值条件。
	 * \param outputclouds 三个雷达点云的指针
	 * \param grid_show 主要用于显示的栅格地图
	 * \param grid_all 主要用于存储检测结果的栅格地图
	 */
	void Detection16(vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> outputclouds, cv::Mat grid_show, Eigen::Quaterniond q);
	/*!
	 * \brief 利用一个32线雷达进行悬崖检测。主要利用大窗口内小窗口
	 * 高度和距离突变是否满足阈值条件。
	 */
	void Detection32(vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> outputclouds, cv::Mat grid_show, Eigen::Quaterniond q);
	/*!
	 * \brief 发送消息函数。
	 * \param grid_show 存储初步检测结果并且用来显示的Mat。
	 * \param grid_msg_show 存储经过膨胀腐蚀后用来发送检测结果的Mat。
	 */
	void PublishMsg(cv::Mat grid_show, cv::Mat grid_msg_show, ros::Time stamp);
	/*!
	 * \brief 激光雷达点云回调函数 --topic "lidar_cloud_calibrated"
	 */
	void LidarMsgHandler(const sensor_msgs::PointCloud2ConstPtr& msg);
	/*!
	 * \brief 惯导信息回调函数 --topic "gpsdata"
	 */
	void GpsdataMsgHandler(const sensor_driver_msgs::GpswithHeadingConstPtr& msg);
	/*!
	 * \brief 准备雷达点云viewer，画网格，车体等。
	 */
	void PrepareViewer(boost::shared_ptr<PCLVisualizer>& cloud_viewer_);
	void coordinate_from_vehicle_to_velodyne(float x, float y , float z, float& newx, float& newy, float& newz);
	/*!
	 * \brief jkj,将合在一起的三个雷达点云拆分
	 * \param inputcloud 混合点云
	 * \param outputclouds 拆分之后的三个雷达点云
	 */
	void analysisCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr inputcloud,
			std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& outputclouds,std::vector<pcl::PointXYZI>& lidarpropertys);
	/*!
	 * \brief 点云显示程序
	 */
	void ShowCloud(boost::shared_ptr<PCLVisualizer>& cloud_viewer, pcl::PointCloud<pcl::PointXYZI>::Ptr inputcloud);
	void showClouds(boost::shared_ptr<PCLVisualizer>& cloud_viewer,
			vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> inputcloud);
	/*!
	 * \brief 判断检测悬崖时用的激光雷达点是否有效。
	 */
	bool ptUseful(pcl::PointXYZI& pt, float dis_th);
	/*!
	 * \brief 垂直墙检测
	 */
	void verticalWallDetect();
	void pub1();
	void pub2();
	void dyCallback(stiff_detection::stiff_dy_paramConfig &config, uint32_t level)
	{
		ROS_INFO("动态参数--- th_dis: %f,  th_tan_16 %f, th_tan_32 %f, th_height %f \n th_z0: %f, th_z1: %f"
			,

			config.th_dis,
			config.th_tan_16,
			config.th_tan_32,
			config.th_height,
			config.th_z0,
			config.th_z1
			);
		th_dis  = config.th_dis;
		th_tan_16 = config.th_tan_16;
		th_tan_32 = config.th_tan_32;
		th_z0 = config.th_z0;
		th_z1 = config.th_z1;
		th_height = config.th_height;

	}
private:
	ros::NodeHandle nh_;			 /**< nodehandle */
	ros::Subscriber sub_Lidar_;		  /**< 订阅点云Subscriber */
	ros::Subscriber sub_Gpsdata_;     /**< 订阅惯导（gpsdata）Subscriber */
	ros::Publisher pub_Stiff_;	/**< 发送检测结果消息 Publisher */
	common::BlockingQueue<sensor_driver_msgs::GpswithHeadingConstPtr> qgwithhmsgs_; /**< 存放惯导惯导消息队列 */
	common::BlockingQueue<sensor_msgs::PointCloud2ConstPtr> q_lidar_msgs_;	/**< 存放点云消息队列 */

	cv::Mat elementero;
	cv::Mat elementdil;
	int window_big_ = 20;	/**< 大窗口的点云数量 */
	int window_small_ = 5;	/**< 小窗口的点云数量 */
	float th_dis = 0.65;	/**< 距离突变阈值 */
	float th_tan_16 = 0.166;	/**< 16线正切阈值 */
	float th_tan_32 = 0.113;	/**< 32线正切阈值 */
	float th_height = -0.6;	/**< 高度阈值 */
	float th_z0 = 0.5;	/**< 障碍物过滤阈值 */
	float th_z1 = -0.5;	/**< 悬崖最低高度阈值 */
	int* map_j;	/**< 雷达线按照从低到高的索引 */
	bool send_water = false;
	bool visual_on = true;	/**< 是否开启可视化 */
	double last_time_gps_ = -1;	/**< 记录上次gps时间戳 */
	double last_time_lidar_ = -1;	/**< 记录上次雷达点云时间戳 */
#ifdef CLOUDVIEWER
	pcl::PointCloud<pcl::PointXYZI>::Ptr high_cloud_;
	pcl::PointCloud<pcl::PointXYZI>::Ptr low_cloud_;
#endif //CLOUDVIEWER
	pcl::PointCloud<pcl::PointXYZI>::Ptr vertical_roi_cloud_;	/**< 用于垂直墙检测的点云 */


	std::thread* process_thread_;	/**< process函数线程 */
	std::mutex mtx_lidar_;	/**< 为lidarCloudMsgs_加锁 */
	std::thread* verwall_thread_;
	std::mutex mtx_verwall_;
};


#endif  //STIFF_DEPTH_
