// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is an implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

//correctionfile 以及 z_offset都没输入请记.zxzx

#include <cmath>
#include "transform/rigid_transform.h"
#include <common/common.h>
#include <nav_msgs/Odometry.h>
#include <opencv/cv.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <fstream>
#include <list>
#include <glog/logging.h>
#include <string>

#include "velodyne/HDL32Structure.h"
#include "util/boostudp/boostudp.h"
#include "common/blocking_queue.h"
#include "common/make_unique.h"
#include "velodyne/data_types.hpp"
#include "sensor_driver_msgs/OdometrywithGps.h"


#define LOCAL_IP "192.168.0.112"
//#define LOCAL_IP "127.0.0.1"
//#define FUSE
#define NOOLD
#define SMALLGRID
//#define TEST
#ifdef SMALLGRID
#define POINT_COUNT_THRESH 1
#define FROMLADAR_LOCAL_PORT 9906
#define PATHTHRESH 0.5//判断激光雷达到悬崖候选区域是否有障碍物zx
#define GRID_THRESH 6 //小栅格区域的检测阈值
#define GRID_THRESH_BIG 6//大栅格区域的检测阈值
#define GRID_THRESH2 24//6
#define GRID_THRESH3 24//10
#define FLAT_THRESH 10
#define RADIUS 100
#define FACTOR 5
#define DIFFTHRESH1 2
#define DIFFTHRESH2 25
#define DIFFTHRESH3 64
#define PI 3.141592653


#else
#define POINT_COUNT_THRESH 1
#define FROMLADAR_LOCAL_PORT 9906
#define PATHTHRESH 1//判断激光雷达到悬崖候选区域是否有障碍物zx
#define GRID_THRESH 2
#define GRID_THRESH2 6//6
#define GRID_THRESH3 6//10
#define FLAT_THRESH 0.4
#endif

typedef pcl::PointCloud<pcl::PointXYZI> Cloud;
typedef Cloud::ConstPtr CloudConstPtr;


class PostProcess
{
public:
	typedef std::pair<double,transform::Rigid3d> TimePosePair;
#ifdef SMALLGRID
	PostProcess(ros::NodeHandle& nodehandle,const std::string& correctionfiles):nodehandle_(nodehandle)
	,processthread_(NULL)
	,processthreadfinished_ (false), minz_ogm_(70,40,0.2),maxz_ogm_big_(70,40,0.4),point_count_ogm_(70,40,0.2),point_count_ogm_big_(70,40,0.4),cloud_viewer_(new PCLVisualizer ("HDL Cloud")),maxz_ogm_(70,40,0.2),ogm_msg_(70,40,0.2),totalclouds_(new pcl::PointCloud<pcl::PointXYZI>)
	{//cloud_viewer_(new PCLVisualizer ("HDL Cloud")),
		float temp1[22]={3.203784768,
				3.3816101569,
				3.5733802842,
				3.7826623217,
				4.0140399584,
				4.2674698704,
				4.5487847706,
				4.8657186892,
				5.220207097,
				5.6230012884,
				6.0889667952,
				6.6260874433,
				7.2578495764,
				8.0188092879,
				8.938797208,
				10.0844052811,
				11.5646366577,
				13.5192024725,
				16.2473413697,
				20.3654400202,
				27.1712658878,
				40.7427597648
};
		float tempdiff2[22]={0.0316218689,
				0.0367757817,
				0.0437989712,
				0.0535356108,
				0.0642267203,
				0.0791380731,
				0.1004471088,
				0.1256620313,
				0.1622431606,
				0.2171238535,
				0.2884985906,
				0.3991233928,
				0.5790596825,
				0.8463777731,
				1.3124178572,
				2.1910849283,
				3.8203275244,
				7.4427418424,
				16.9587364953,
				46.3192657401,
				184.1854460534,
				10000

};


		float tempr2[22]={3.203784768*3.203784768,
				3.3816101569*3.3816101569,
				3.5733802842*3.5733802842,
				3.7826623217*3.7826623217,
				4.0140399584*4.0140399584,
				4.2674698704*4.2674698704,
				4.5487847706*4.5487847706,
				4.8657186892*4.8657186892,
				5.220207097*5.220207097,
				5.6230012884*5.6230012884,
				6.0889667952*6.0889667952,
				6.6260874433*6.6260874433,
				7.2578495764*7.2578495764,
				8.0188092879*8.0188092879,
				8.938797208*8.938797208,
				10.0844052811*10.0844052811,
				11.5646366577*11.5646366577,
				13.5192024725*13.5192024725,
				16.2473413697*16.2473413697,
				20.3654400202*20.3654400202,
				27.1712658878*27.1712658878,
				40.7427597648*40.7427597648
};
		for(int i=0;i<22;i++){
			theoryr2table_[i]=tempr2[i];
			theorydiff2table_[i]=tempdiff2[i];
		}
		init(correctionfiles);
//		std::cout<<point_count_ogm_.ogmwidth_cell<<"...............................,,,,,,,,,<<<<<<<<<<<<"<<std::endl;
	}
#else
	PostProcess(ros::NodeHandle& nodehandle,const std::string& correctionfiles):nodehandle_(nodehandle)
	,processthread_(NULL)
	,processthreadfinished_ (false), point_count_ogm_(70,40,0.8),maxz_ogm_(70,40,0.2),ogm_msg_(70,40,0.2),cloud_viewer_(new PCLVisualizer ("HDL Cloud")),totalclouds_(new pcl::PointCloud<pcl::PointXYZI>)
	{

		init(correctionfiles);
//		std::cout<<point_count_ogm_.ogmwidth_cell<<"...............................,,,,,,,,,<<<<<<<<<<<<"<<std::endl;
	}
#endif
	~PostProcess()
	{
	  lidarOdoms_.stopQueue();
	  processthreadfinished_ = true;
	  processthread_->join();

	}

	struct LaserData{
        int number;
        double angle;
    };


	void init(const std::string& correctionsfile);


	void SendData(OGMData<unsigned char>& ogmdata);  //udp通信 发送端例程

	void laserOdometryHandler(const sensor_driver_msgs::OdometrywithGps::ConstPtr& laserOdometry); //雷达里程计

	void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudFullRes2);//点云数据


	void analysisCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr inputcloud,
			std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& outputclouds,std::vector<pcl::PointXYZI>& lidarpropertys);

	void process();
	void circleradiusDetection(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
	void radiusDetection(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
	void radiusPointpair(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);//圆周方向点对
	void displayPointCloud(Cloud::Ptr cloud_show);
	void coordinate_from_vehicle_to_velodyne(float x, float y , float z, float& newx, float& newy, float& newz);
	void keyboard_callback (const KeyboardEvent& event, void* cookie);
	void mouse_callback (const MouseEvent& mouse_event, void* cookie);
	void SetViewerPCL(boost::shared_ptr<PCLVisualizer> cloud_viewer_);
	void SetViewerEgoVehicleModel(boost::shared_ptr<PCLVisualizer> cloud_viewer_);
	void ShowViewerCloudPoints( boost::shared_ptr<PCLVisualizer> cloud_viewer_, vector<Cloud::ConstPtr> cloud_show_,
			char cloudname[], double color_red_, double color_green_, double color_blue_ );
	void ShowViewerCloudPoints( boost::shared_ptr<PCLVisualizer> cloud_viewer_, Cloud::ConstPtr cloud_show_,
				char cloudname[], double color_red_, double color_green_, double color_blue_ );
	void countogmpoints(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);//
	void countogmpoints(vector<pcl::PointCloud<pcl::PointXYZI>::ConstPtr> cloud);//
	void showOGM(const char* windowname ,const OGMData<int>& ogmdata,vector<int> vecside,vector<int> vecupdown);
	void showOGM(const char* windowname,const OGMData<int>& ogmdata,const vector<int>& vecindex);
	bool pathClear(int height,int width);
	void radialDetection(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
	int  radiusCount(const float k,int& height0,int& width0,vector<int>& vecindex,int& anglei);
	float caldis2(int beginh,int beginw,int endh,int endw);
	OGMData<unsigned char>* getOGM(){return &ogm_msg_;}
	ros::Time gettime(){return timestamp_;}


protected:
   static const int HDL_MAX_NUM_LASERS = 64;

	ros::Subscriber subLaserOdometry_ ;


	ros::Subscriber subLaserCloudFullRes_ ;//经过筛选且转换之后的点云

	common::BlockingQueue<std::unique_ptr<TimePosePair>> lidarOdoms_;
	common::BlockingQueue<sensor_msgs::PointCloud2ConstPtr> lidarCloudMsgs_;
//	std::fstream file_;
	boost::thread* processthread_;

	ros::NodeHandle& nodehandle_;
	unsigned char freeze_;
	bool cloudupdate;
	bool processthreadfinished_;
	LaserData indexmaptable[HDL_MAX_NUM_LASERS];
	 std::map<double,int> map_tanangle_index;//sort by angle

	struct HDLLaserCorrection
    {
        double azimuthCorrection;
        double verticalCorrection;
        double distanceCorrection;
        double verticalOffsetCorrection;
        double horizontalOffsetCorrection;
        double sinVertCorrection;
        double cosVertCorrection;
        double sinVertOffsetCorrection;
        double cosVertOffsetCorrection;


    };
private:
	OGMData<int> point_count_ogm_;
	OGMData<int> point_count_ogm_big_;
	OGMData<float> maxz_ogm_;
	OGMData<float> minz_ogm_;
	OGMData<float> maxz_ogm_big_;
	OGMData<unsigned char> ogm_msg_;
	float theoryr2table_[22];
	float theorydiff2table_[22];
	vector<int> vec4side_;
	vector<int> vecleft_;
	vector<int> vec2side_;
	vector<int> vecright_big_;
	vector<int> vecup_big_;
	vector<int> vecup_;
	vector<int> vecindex_;
	Cloud::Ptr stiffcloud_;
	Cloud::Ptr normalcloud_;
	int LASER_LAYER=64;
	 double z_offset_h ;
	void loadCorrectionsFile (const std::string& correctionsFile);
	HDLLaserCorrection laser_corrections_[HDL_MAX_NUM_LASERS];
	double  theorydis[MAX_LAYER];
	 boost::thread* thread_displayPointCloud;
	 ros::Time timestamp_;
	 boost::shared_ptr<PCLVisualizer> cloud_viewer_;
//	 for multi clouds fusion
	 std::list<pcl::PointCloud<pcl::PointXYZI>::Ptr> lworldclouds_;
	 vector<pcl::PointCloud<pcl::PointXYZI>::ConstPtr> vtotalcloud_;
	 pcl::PointCloud<pcl::PointXYZI>::Ptr totalclouds_;
};

