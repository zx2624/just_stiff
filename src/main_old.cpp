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
#include <string.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <thread>

#include "velodyne/HDL32Structure.h"
#include "util/boostudp/boostudp.h"
#include "common/blocking_queue.h"
#include "common/make_unique.h"
#include "postprocess.h"
//#include "sensor_driver_msgs/stiffwater.h"
#include "stiff_msgs/stiffwater.h"
#include "sensor_driver_msgs/GpswithHeading.h"
#include "depth_image_utils/HeightMap.h"
#include "lanelet_map_msgs/Way.h"


bool visual_on=true;
bool send_water=false;
//#define lineiter
//#define SIXT
//#define BEIQI
#define TOYOTA_RS
//#define SIXT_RS
//#define TOYOTA
//#define SIXT64
#define showgreen
//#define USENEIGHBER
#define CONDITION(x) if(0)
#define PI 3.141592653
#define WINDOW 10
#define COUNTTHRESH 6
#define ZEROCOUNTTH 4
#define TESTSHOW
//#define COUNTTHRESH 6
#define HEIGHTTHRESH 0.2
#define GRIDWH 351
double pitchrad[32] = { -30.67, -29.33, -28, -26.67, -25.33, -24, -22.67, -21.33, -20, -18.67, -17.33, -16, -14.67, -13.33, -12, -10.67, -9.3299999, -8, -6.6700001, -5.3299999, -4, -2.6700001, -1.33, 0, 1.33, 2.6700001, 4, 5.3299999, 6.6700001, 8, 9.3299999, 10.67 };
double pitchrad_beiqi[32] = {-25,-14.6045,-10.2463,-7.8749,-6.371,-5.407,-4.7026,-4.2618,-4,-3.5957,-3.333,-3,-2.667,
		-2.333,-1.9463,-1.667,-1.333,-0.9821,-0.6312,-0.333,0,0.3688,0.7207,1,1.3151,1.667,2.333,3.2973,4.6314,7,10.333,15
};
double pitchrad_toyota[32]={
		-24.9411,-14.638,-10.333,-7.9276,-6.424,-5.4425,-4.667,-4.333,-4.0713,-3.6492,-3.3152,-2.9821,-2.6491,
		-2.3151,-2.0179,-1.667,-1.2972,-0.9821,-0.6491,-0.2972,0,0.333,0.6849,1,1.3688,1.6849,2.3509,
		3.3152,4.667,	7,10.3503,15.0334
};
double pitchrad_6t[32]={
		-10.281,
		-7.91,
		-6.424,
		-5.407,
		-4.6492,
		-4.3508,
		-4,
		-3.667,
		-3.333,
		-3.0179,
		-2.667,
		-2.3509,
		-2,
		-1.667,
		-1.3151,
		-0.9821,
		-0.6491,
		-0.333,
		-0.0358,
		0.3509,
		0.6849,
		1,
		1.333,
		1.667,
		2.333,
		3.2973,
		4.667,
		7,
		10.3677,
		15

};
double pitchrad64[64] = {1.9367,
		1.58567,
		1.29305,
		1.00036,
		0.602232,
		0.286024,
		-0.006777,
		-0.393271,
		-0.674338,
		-1.01391,
		-1.41195,
		-1.78644,
		-2.06722,
		-2.41804,
		-2.73363,
		-3.07241,
		-3.44598,
		-3.78429,
		-4.12232,
		-4.47171,
		-4.75099,
		-5.123,
		-5.40172,
		-5.87728,
		-6.16686,
		-6.51394,
		-6.81435,
		-7.18358,
		-7.48313,
		-7.89721,
		-8.16134,
		-8.56259,
		-8.76308,
		-9.3342,
		-9.71012,
		-10.21,
		-10.6179,
		-11.284,
		-11.745,
		-12.3611,
		-12.7628,
		-13.1854,
		-13.7614,
		-14.3785,
		-14.861,
		-15.4393,
		-15.8845,
		-16.2954,
		-16.7368,
		-17.4004,
		-17.9638,
		-18.5343,
		-18.8915,
		-19.2054,
		-19.7779,
		-20.4697,
		-21.0636,
		-21.5819,
		-21.8952,
		-22.4376,
		-22.8367,
		-23.5492,
		-24.1273,
		-24.6422
};

double sintable[870];
double costable[870];
double disdiff=0;
double disdifflast=0;
//用另一种方法检测时的参数
const int grid_width = 101;
const int grid_height = 175;
const double res = 0.4;
cv::Mat elementero = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
cv::Mat elementdil = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
cv::Mat elementero2 = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(9, 9));
//Eigen::Vector3d yprlast(0,0,0);
Eigen::Isometry3d T=Eigen::Isometry3d::Identity();
Eigen::Isometry3d T1=Eigen::Isometry3d::Identity();
common::BlockingQueue<sensor_driver_msgs::GpswithHeadingConstPtr> qgwithhmsgs_;
common::BlockingQueue<depth_image_utils::HeightMapConstPtr> qheightmap_;
sensor_msgs::PointCloud2ConstPtr lidarCloudMsgs_;
ros::Publisher pubStiffwaterOgm;
#define VIEWER
#ifdef VIEWER
boost::shared_ptr<PCLVisualizer> 	cloud_viewer_(new PCLVisualizer ("zx Cloud"));
#endif
//#define VIEWER_ORI
#ifdef VIEWER_ORI
boost::shared_ptr<PCLVisualizer> 	cloud_viewer_origin(new PCLVisualizer ("zx_origin Cloud"));
#endif
//deque<sensor_driver_msgs::GpswithHeadingConstPtr> qypr;
//Eigen::Vector3d yprmin(100,100,100);
void analysisCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr inputcloud,
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

#ifdef SIXT
#define THRESH 10
#define left
#define right
void callback(const depth_image_utils::HeightMapConstPtr& height_msg){
	//		cout<<"height map timestamp is "<<height_msg->header.stamp<<endl;
	//	sensor_driver_msgs::GpswithHeadingConstPtr tmpmsg=qgwithhmsgs_.PopWithTimeout(common::FromSeconds(0.1));
	//	if(tmpmsg==nullptr) return;
	//	double gpsstamp=tmpmsg->gps.header.stamp.toSec();
	//	double lidarstamp=height_msg->header.stamp.toSec();
	//	if(gpsstamp>lidarstamp){
	//		cout<<"waite for height map"<<endl;
	//		qgwithhmsgs_.Push_Front(std::move(tmpmsg));
	//		return;
	//	}
	//	while(lidarstamp-gpsstamp>0.00001){
	//		if(qgwithhmsgs_.Size()==0){
	//			return;//这里可以增加队列这样不用浪费一帧信息
	//		}
	//		tmpmsg=qgwithhmsgs_.Pop();
	//		gpsstamp=tmpmsg->gps.header.stamp.toSec();
	//	}
	//	double yprdiff=(tmpmsg->pitch-yprmin[1])*(tmpmsg->pitch-yprmin[1])+(tmpmsg->roll-yprmin[2])*(tmpmsg->roll-yprmin[2]);
	//	double yprdiff=(tmpmsg->pitch-yprlast[1])*(tmpmsg->pitch-yprlast[1])+(tmpmsg->roll-yprlast[2])*(tmpmsg->roll-yprlast[2]);
	//	if(0){
	//		//		cout<<"height"<<height_msg->header.stamp<<endl;
	//		//		cout<<"gpsdata"<<tmpmsg->gps.header.stamp<<endl;
	//		cout<<"the ypr  is"<<endl;
	//		cout<<yprdiff<<endl;
	//		//		cout<<yprmin<<endl;
	//		//		cout<<tmpmsg->heading<<endl;
	//		cout<<tmpmsg->pitch<<endl;
	//		cout<<tmpmsg->roll<<endl;
	//	}
	//	yprmin<<100,100,100;
	//	cv::namedWindow("range",CV_WINDOW_NORMAL);
	//	cv::namedWindow("height",CV_WINDOW_NORMAL);
	if(vi)
#ifdef TESTSHOW
		cv::namedWindow("grid",CV_WINDOW_NORMAL);
	cv::namedWindow("grid1",CV_WINDOW_NORMAL);
#endif
	cv::namedWindow("gridall",CV_WINDOW_NORMAL);
	cv::namedWindow("show",CV_WINDOW_NORMAL);
	cv::Mat heightmat_0 = cv::Mat::zeros(32,870,CV_32F);
	cv::Mat heightmat_1 = cv::Mat::zeros(32,870,CV_32F);
	cv::Mat dismat_0 = cv::Mat::zeros(32,870,CV_32F);
	cv::Mat dismat_1 = cv::Mat::zeros(32,870,CV_32F);
	for(int i=0;i<32;++i){
		for(int j =0;j<870;++j){
			if(abs(height_msg->data[i*870+j]+100)>0.0001){//原始值为-100赋值为0
				dismat_0.at<float>(i,j)=height_msg->data[i*870+j+32*870];
			}
			if(abs(height_msg->data[i*870+j+2*32*870]+100)>0.0001){
				dismat_1.at<float>(i,j)=height_msg->data[i*870+j+3*32*870];
			}
		}
	}

	cv::Mat gridshow_0 = cv::Mat::zeros(GRIDWH,GRIDWH,CV_8UC3);
	cv::Mat gridshow_1 = cv::Mat::zeros(GRIDWH,GRIDWH,CV_8UC3);
	cv::Mat justshow = cv::Mat::zeros(GRIDWH,GRIDWH,CV_8UC3);
	//最大高度、最小高度、高度差
	cv::Mat heightgrid_0 = cv::Mat::zeros(GRIDWH,GRIDWH,CV_32FC1);
	cv::Mat heightgrid_1 = cv::Mat::zeros(GRIDWH,GRIDWH,CV_32FC1);
	cv::Mat heightgridmin_0 = cv::Mat::ones(GRIDWH,GRIDWH,CV_32FC1)*100;
	cv::Mat heightgridmax_0 = cv::Mat::ones(GRIDWH,GRIDWH,CV_32FC1)*(-100);
	cv::Mat heightgridmin_1 = cv::Mat::zeros(GRIDWH,GRIDWH,CV_32FC1)*100;
	cv::Mat heightgridmax_1 = cv::Mat::zeros(GRIDWH,GRIDWH,CV_32FC1)*(-100);

	for(int j=0;j<870;j++){

		for(int i=26;i>6;i--){
			int index=i*870+j;
			double dis=(double)dismat_0.at<float>(i,j);
			double dis1=(double)dismat_1.at<float>(i,j);

			double radius=dis*std::cos(pitchrad[31-i]*PI/180);
			double radius1=dis1*std::cos(pitchrad[31-i]*PI/180);

			double originx=radius*costable[j];
			double originy=radius*sintable[j];
			double originz=dis*std::sin(pitchrad[31-i]*PI/180);
			double originx1=radius1*costable[j];
			double originy1=radius1*sintable[j];
			double originz1=dis1*std::sin(pitchrad[31-i]*PI/180);

			Eigen::Matrix<double, 3, 1> originp;
			Eigen::Matrix<double, 3, 1> originp1;

			originp<<originx,originy,originz;
			originp1<<originx1,originy1,originz1;

			double x=(T*originp)[0];
			double y=(T*originp)[1];
			double z=(T*originp)[2];

			double x1=(T1*originp1)[0];
			double y1=(T1*originp1)[1];
			double z1=(T1*originp1)[2];

			if(x>-1.5&&x<1.5&&y<4&&y>-1){;}//
			else{
				heightmat_0.at<float>(i,j)=z;
				int col=boost::math::round((x+35)/0.2);//干！！！！！！！！！！！
				int row=boost::math::round((y+20)/0.2);
				int gridindex=0;
				unsigned char* ptr=gridshow_0.ptr<unsigned char>(GRIDWH-1-row);
				unsigned char* ptrshow=justshow.ptr<unsigned char>(GRIDWH-1-row);
				float* ptrheight=heightgrid_0.ptr<float>(row);
				float* ptrmin=heightgridmin_0.ptr<float>(row);
				float* ptrmax=heightgridmax_0.ptr<float>(row);
				if(col<GRIDWH&&row<GRIDWH&&col>=0&&row>=0){
					ptr[3*col]=0;
					ptr[3*col+1]=255;
					ptr[3*col+2]=0;
					ptrshow[3*col+1]=255;
					if(ptrmin[col]>z){
						ptrmin[col]=z;
					}
					if(ptrmax[col]<z){
						ptrmax[col]=z;
					}
					ptrheight[col]=ptrmax[col]-ptrmin[col];

				}
			}

			if(x1>-1.5&&x1<1.5&&y1<4&&y1>-1){;}
			else{
				heightmat_1.at<float>(i,j)=z1;
				int col=boost::math::round((x1+35)/0.2);//干！！！！！！！！！！！
				int row=boost::math::round((y1+20)/0.2);
				int gridindex=0;
				unsigned char* ptr=gridshow_1.ptr<unsigned char>(GRIDWH-1-row);
				unsigned char* ptrshow=justshow.ptr<unsigned char>(GRIDWH-1-row);
				float* ptrheight1=heightgrid_1.ptr<float>(row);
				float* ptrheightmin1=heightgridmin_1.ptr<float>(row);
				float* ptrheightmax1=heightgridmax_1.ptr<float>(row);
				if(col<GRIDWH&&row<GRIDWH&&col>=0&&row>=0){
					ptr[3*col]=0;
					ptr[3*col+1]=255;
					ptr[3*col+2]=0;
					ptrshow[3*col+1]=255;
					if(ptrheightmin1[col]>z1){
						ptrheightmin1[col]=z1;
					}
					if(ptrheightmax1[col]<z1){
						ptrheightmax1[col]=z1;
					}
					ptrheight1[col]=ptrheightmax1[col]-ptrheightmin1[col];
				}
			}
		}
	}

	//		cv::LineIterator it(gridshow_0,cvPoint(0,0),cvPoint(1000,10000));
	//		for(int i=0;i<it.count;i++,++it){
	//			(*it)[0]=255;
	//			(*it)[1]=0;
	//			(*it)[2]=0;
	//		}

	int rightend=(int)38/0.2;
	int leftend=(int)32/0.2;
	//
	//	//以下对两个雷达进行检测//////////////////////////////////////////////////////////////////////////////
	//
	//
	//1、0雷达，为左侧 ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef left
	cout<<"==================================="<<endl;
	for(int j=0;j<870;j++){
		int myj;
		int countlaser=0;
		for(int i=26;i>6;i--){
			//			if(j==800){
			//				cout<<"height is "<<heightmat_0.at<float>(i,j)<<endl;
			//			}
			int index=i*870+j;
			int indexnext=(i-1)*870+j;
			double disbefore=(double)dismat_0.at<float>(i+1,j);
			double dis=(double)dismat_0.at<float>(i,j);
			if(abs(dis-disbefore)<1)continue;
			double radius=dis*std::cos(pitchrad[31-i]*PI/180);
			double originx=radius*costable[j];
			double originy=radius*sintable[j];
			double originz=dis*std::sin(pitchrad[31-i]*PI/180);

			//计算起始点及前一点高度
			double height=heightmat_0.at<float>(i,j);
			double heightbefore=heightmat_0.at<float>(i+1,j);

			//当前线雷达与上一线雷达距离差
			double diff2=(double)dismat_0.at<float>(i,j)-(double)dismat_0.at<float>(i+1,j);
			double disnext=(double)dismat_0.at<float>(i-1,j);
			int count=0;
			while(disnext==0&&i>6){
				count++;
				i--;
				disnext=(double)dismat_0.at<float>(i-1,j);
			}
			//计算下一点及下下一点高度
			double heightnext=heightmat_0.at<float>(i-1,j);
			double heightnene=heightmat_0.at<float>(i-2,j);

			//下一线与当前线雷达距离差
			double diff1=disnext-dis;
			if(count>=2)break;
			//到达0°后break
			if(i==6) break;

			double radiusnext=disnext*std::cos(pitchrad[31-i+1]*PI/180);
			double originxnext=radiusnext*costable[j];
			double originynext=radiusnext*sintable[j];
			double originznext=dis*std::sin(pitchrad[31-i+1]*PI/180);


			Eigen::Matrix<double, 3, 1> originp;
			Eigen::Matrix<double, 3, 1> originpnext;
			Eigen::Matrix<double, 3, 1> originpvirtual;

			originp<<originx,originy,originz;
			originpnext<<originxnext,originynext,originznext;

			double x=(T*originp)[0];
			double y=(T*originp)[1];
			double z=(T*originp)[2];

			double xnext=(T*originpnext)[0];
			double ynext=(T*originpnext)[1];
			double znext=(T*originpnext)[2];
			//
			if(x>-1.5&&x<1.5&&y<4&&y>-1||xnext>-1.5&&xnext<1.5&&ynext<4&&ynext>-1) {
				continue;
			}
			//用于判断雷达点是否走出车身，可以正式开始算法
			countlaser++;
			//			heightmat_0.at<float>(i,j)=z;
			//			if(xnext>-1.2&&xnext<1.2&&ynext<4&&ynext>-1) continue;
			//			heightmat_0.at<float>(i,j)=z;



			double radiusvirtual=4.0;//虚拟半径，用于盲区检测
			double originxvirtual=radiusvirtual*costable[j];
			double originyvirtual=radiusvirtual*sintable[j];
			double originzvirtual=0;
			originpvirtual<<originxvirtual,originyvirtual,originzvirtual;

			double xvirtual=(T*originpvirtual)[0];
			double yvirtual=(T*originpvirtual)[1];



			int col=boost::math::round((x+35)/0.2);
			int row=boost::math::round((y+20)/0.2);
			int colvirtual;
			int rowvirtual;
			if(countlaser==1){
				colvirtual=boost::math::round((xvirtual+35)/0.2);
				rowvirtual=boost::math::round((yvirtual+20)/0.2);
				if(radius>7&&height<-0.6&&(colvirtual<32/0.2||colvirtual>38/0.2)){
					//					unsigned char* ptr=gridshow_0.ptr<unsigned char>(GRIDWH-1-rowvirtual);
					//					ptr[3*colvirtual]=0;
					//					ptr[3*colvirtual+1]=0;
					//					ptr[3*colvirtual+1]=255;
					cv::line(gridshow_0,cvPoint(col,GRIDWH-1-row),cvPoint(colvirtual,GRIDWH-1-rowvirtual),cvScalar(0,0,125));
					cv::line(justshow,cvPoint(col,GRIDWH-1-row),cvPoint(colvirtual,GRIDWH-1-rowvirtual),cvScalar(0,0,125));
				}
				//				第一条线自己给定初始diff2
				diff2=0.3;
			}
			int colnext=boost::math::round((xnext+35)/0.2);
			int rownext=boost::math::round((ynext+20)/0.2);

			int gridindex=0;
			unsigned char* ptr=gridshow_0.ptr<unsigned char>(GRIDWH-1-row);
			if(col<GRIDWH&&row<GRIDWH&&col>=0&&row>=0){
				//				增加不同半径下的不同高度阈值
				if(radius<8){if(height>0.3)break;}
				else if(radius<13){if(height>0.5)break;}
				else{if(height>0.8)break;}
				//				if(col>170&&col<180){
				//					cout<<"j is : "<<j<<endl;
				//				}
				if(j==800){
					ptr[3*col]=255;
					ptr[3*col+1]=255;
					ptr[3*col+2]=255;
					//					cv::line(gridshow_0,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext),cvScalar(255,255,255));
					//					cout<<radius<<" height diff "<<height-heightbefore<<endl;
				}
				//				针对远距离误检不同阈值
				if(1){
					//短距离用高度差和距离突变来定义
					if(radius<10){
						int thresh=THRESH;
						if(col>155&&col<195) thresh=2*THRESH;
						if(heightnext-height<-0.5&&(disnext-dis)/(dis-disbefore)>thresh){//diff1/diff2
							//					if((height-heightnext)/(radiusnext-radius)>0.1&&diff1/diff2>3){
							ptr[3*col]=0;
							ptr[3*col+1]=0;
							ptr[3*col+2]=255;
							//							cout<<radius<<"==================>"<<diff1/diff2<<endl;
							//排除径向距离比较小的
							if((colnext-col)*(colnext-col)*0.04+(rownext-row)*(rownext-row)*0.04<1){
#ifdef showgreen
								cv::line(gridshow_0,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext),cvScalar(0,150,0));
#endif
								continue;
							}
							//连线
							//							if(colnext<GRIDWH&&rownext<GRIDWH&&colnext>=0&&rownext>=0){
#ifdef lineiter
							cv::LineIterator it(gridshow_0,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext));
							it++;
							bool state=true;
							int pointcount=0;
							for(int i=0;i<it.count;i++,++it){
								//									if(i<it.count*0.2||i>it.count*0.8) continue;
								if((*it)[1]==255){
									pointcount++;
									int rows=GRIDWH-1-it.pos().y;
									int cols=it.pos().x;
									if((cols-col)*(cols-col)*0.04+(rows-row)*(rows-row)*0.04<1){
										continue;
									}
									(*it)[0]=255;
									(*it)[2]=255;
									//										cout<<row<<" "<<col<<" "<<rows<<" "<<cols<<endl;
									//										cout<<heightgrid_0.at<float>(row,col)<<"  "<<heightgrid_0.at<float>(rows,cols)<<endl;
									//用其实点处栅格最大高度减去终点处最小高度
									if(heightgridmax_0.at<float>(row,col)-heightgridmin_0.at<float>(rows,cols)<0.5){
										state=false;
										break;
									}
								}
								//									(*it)[2]=125;
							}
							if(!state) continue;
#endif
#ifdef USENEIGHBER
							int pointcount=0;
							int zerocount=0;
							for(int l=-2;l<=1;l++){
								for(int k=-WINDOW;k<WINDOW+1;k++){
									//									CONDITION(j){
									//										cout<<dismat_0.at<float>(i+1,j+k)/500<<" "<<
									//												dismat_0.at<float>(i,j+k)/500<<"  "<<dismat_0.at<float>(i-1,j)/500
									//												<<"  "<<dismat_0.at<float>(i-2,j+k)/500<<" "
									//												<<dismat_0.at<float>(i-3,j)/500<<endl;
									//									}
									if(l==0||l==1){
										if(dismat_0.at<float>(i+l,j+k)==0){
											zerocount++;
										}
									}
									if(l==-2){
										if((dismat_0.at<float>(i-1,j)-dismat_0.at<float>(i+l,j+k))/dismat_0.at<float>(i-1,j)>HEIGHTTHRESH){
											pointcount++;
										}
									}
								}
							}
							CONDITION(j){
								cout<<j<<"  "<<pointcount<<" "<<zerocount<<endl;
								//								cout<<j<<"  "<<radius<<"  "<<radiusnext<<endl;
							}
							if(pointcount>COUNTTHRESH||zerocount>ZEROCOUNTTH){
#ifdef showgreen
								cv::line(gridshow_0,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext),cvScalar(0,150,0));
#endif
								continue;
							}
#endif
							//							vector<double> temp;
							//							temp.clear();
							//							for(int i=-5;i<6;++i){
							//								for(int j=-5;j<6;++j){
							//									temp.push_back(heightgridmax_0.at<float>(rownext+i,colnext+j));
							//								}
							//							}
							//							sort(temp.begin(),temp.end());
							//							cout<<"--------------------------"<<endl;
							//							for(auto it:temp){
							//								cout<<it<<endl;
							//							}
							cv::LineIterator it(heightgridmax_0,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext));
							//								cout<<"the heights are......"<<endl;
							double hm=-100;
							for(int k=0;k<it.count;++k,it++){
								int x=it.pos().x;
								int y=it.pos().y;
								y=GRIDWH-1-y;
								for(int i=-2;i<3;++i){
									for(int j=-2;j<3;++j){
										double height=0;
										if(x+i>=0&&x+i<GRIDWH&&y+j>=0&&y+j<GRIDWH){
											float* ptr=heightgridmax_0.ptr<float>(y+j);
											height=(double)ptr[x+i];
										}
										if(height>hm) hm=height;
									}
								}
							}
							//								cout<<hm<<endl;
							if(hm<0.4){//temp[temp.size()-1]<1&&temp[temp.size()-1]!=-100
								//								cout<<"the height is "<<temp[temp.size()-1]<<endl;
								//								cout<<"dis is "<<disbefore<<" "<<dis<<" "<<disnext<<"  ratio is "<<diff1/diff2<<
								//										" height is "<<height<<" "<<heightnext<<endl;
								cv::line(justshow,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext),cvScalar(0,0,125));
								cv::line(gridshow_0,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext),cvScalar(0,0,125));
							}
							//							}

						}
						else{
#ifdef showgreen
							cv::line(gridshow_0,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext),cvScalar(0,150,0));
#endif
						}
					}
					//大于10m小于17m需要加上颠簸程度、俯仰角、（起始点高度？--这个还不太确定）、正切值、最小高度、距离突变比
					else if(radius<20){
						int thresh=THRESH-1;
						if(col>155&&col<195) thresh=2*THRESH;
						if(1){//&&height>-0.4 &&yprdiff<7 tmpmsg->pitch<5
							//					if((heightmat_0.at<float>(i-1,j)-heightmat_0.at<float>(i,j))<-1&&diff1/diff2>3){
							if((height-heightnext)/(radiusnext-radius)>0.07
									&&heightnext-height<-1&&(disnext-dis)/(dis-disbefore)>thresh){//
								ptr[3*col]=0;
								ptr[3*col+1]=0;
								ptr[3*col+2]=255;

								//排除径向距离比较小的
								if((colnext-col)*(colnext-col)*0.04+(rownext-row)*(rownext-row)*0.04<1){
#ifdef showgreen
									cv::line(gridshow_0,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext),cvScalar(0,150,0));
#endif
									continue;
								}
								//连线
								//								if(colnext<GRIDWH&&rownext<GRIDWH&&colnext>=0&&rownext>=0){
#ifdef lineiter
								cv::LineIterator it(gridshow_0,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext));
								it++;
								bool state=true;
								int pointcount=0;
								for(int i=0;i<it.count;i++,++it){
									//										if(i<it.count*0.2||i>it.count*0.8) continue;
									if((*it)[1]==255){
										//											pointcount++;
										int rows=GRIDWH-1-it.pos().y;
										int cols=it.pos().x;
										if((cols-col)*(cols-col)*0.04+(rows-row)*(rows-row)*0.04<1){
											continue;
										}
										(*it)[0]=255;
										(*it)[2]=255;
										//											cout<<row<<" "<<col<<" "<<rows<<" "<<cols<<endl;
										//											cout<<heightgrid_0.at<float>(row,col)<<"  "<<heightgrid_0.at<float>(rows,cols)<<endl;
										if(heightgridmax_0.at<float>(row,col)-heightgridmin_0.at<float>(rows,cols)<0.5){
											state=false;
											break;
										}
									}
									//									(*it)[2]=125;
								}
								if(!state) continue;
#endif
#ifdef USENEIGHBER
								int pointcount=0;
								int zerocount=0;
								for(int l=-2;l<=1;l++){
									for(int k=-WINDOW;k<WINDOW+1;k++){
										//										CONDITION(j){
										//											cout<<dismat_0.at<float>(i+1,j+k)/500<<" "<<
										//													dismat_0.at<float>(i,j+k)/500<<"  "<<dismat_0.at<float>(i-1,j)/500
										//													<<"  "<<dismat_0.at<float>(i-2,j+k)/500<<" "
										//													<<dismat_0.at<float>(i-3,j)/500<<endl;
										//										}
										if(l==0||l==1){
											if(dismat_0.at<float>(i+l,j+k)==0){
												zerocount++;
											}
										}
										if(l==-2){
											if((dismat_0.at<float>(i-1,j)-dismat_0.at<float>(i+l,j+k))/dismat_0.at<float>(i-1,j)>HEIGHTTHRESH){
												pointcount++;
											}
										}
									}
								}
								CONDITION(j){
									cout<<j<<"  "<<pointcount<<" "<<zerocount<<endl;
									//								cout<<j<<"  "<<radius<<"  "<<radiusnext<<endl;
								}
								if(pointcount>COUNTTHRESH||zerocount>ZEROCOUNTTH){

#ifdef showgreen
									cv::line(gridshow_0,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext),cvScalar(0,150,0));
#endif
									continue;
								}
#endif
								//								vector<double> temp;
								//								temp.clear();
								//								for(int i=-5;i<6;++i){
								//									for(int j=-5;j<6;++j){
								//										temp.push_back(heightgridmax_0.at<float>(rownext+i,colnext+j));
								//									}
								//								}
								//								sort(temp.begin(),temp.end());
								//								cout<<"--------------------------"<<endl;
								//								for(auto it:temp){
								//									cout<<it<<endl;
								//								}
								cv::LineIterator it(heightgridmax_0,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext));
								//								cout<<"the heights are......"<<endl;
								double hm=-100;
								for(int k=0;k<it.count;++k,it++){
									int x=it.pos().x;
									int y=it.pos().y;
									y=GRIDWH-1-y;
									for(int i=-2;i<3;++i){
										for(int j=-2;j<3;++j){
											double height=0;
											if(x+i>=0&&x+i<GRIDWH&&y+j>=0&&y+j<GRIDWH){
												float* ptr=heightgridmax_0.ptr<float>(y+j);
												height=(double)ptr[x+i];
											}
											if(height>hm) hm=height;
										}
									}
								}
								//								cout<<hm<<endl;
								if(hm<0.4){//temp[temp.size()-1]<1&&temp[temp.size()-1]!=-100
									//									cout<<"dis is "<<disbefore<<" "<<dis<<" "<<disnext<<"  ratio is "<<diff1/diff2<<
									//											" height is "<<height<<" "<<heightnext<<endl;
									cv::line(justshow,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext),cvScalar(0,0,125));
									cv::line(gridshow_0,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext),cvScalar(0,0,125));
								}
								//								}

							}
							else{
#ifdef showgreen
								cv::line(gridshow_0,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext),cvScalar(0,150,0));
#endif
							}
						}
					}
				}
			}
		}
	}
#endif
	//
	//
#ifdef right
	//2、 1雷达，应该为右侧//////////////////////////////////////////////////////////////////////////////
	for(int j=0;j<870;j++){
		int myj;
		//		if(j==0){
		//			cout<<"the height is "<<<<endl;
		//			myj=j;
		//		}
		int countlaser=0;
		for(int i=26;i>6;i--){
			int index=i*870+j;
			int indexnext=(i-1)*870+j;
			double dis=(double)dismat_1.at<float>(i,j);
			double radius=dis*std::cos(pitchrad[31-i]*PI/180);
			double originx=radius*costable[j];
			double originy=radius*sintable[j];
			double originz=dis*std::sin(pitchrad[31-i]*PI/180);
			//当前线雷达与上一线雷达距离差
			double diff2=(double)dismat_1.at<float>(i,j)-(double)dismat_1.at<float>(i+1,j);
			double disnext=(double)dismat_1.at<float>(i-1,j);
			int count=0;
			double height=heightmat_1.at<float>(i,j);
			double heightbefore=heightmat_1.at<float>(i+1,j);

			while(disnext==0&i>6){
				count++;
				i--;
				disnext=(double)dismat_1.at<float>(i-1,j)/500;
			}

			double heightnext=heightmat_1.at<float>(i-1,j);
			double heightnene=heightmat_1.at<float>(i-2,j);

			//下一线与当前线雷达距离差
			double diff1=disnext-dis;

			//			double discheck=dismat_1.at<float>(i-2,j)/500;
			//			double discheck2=dismat_1.at<float>(i-3,j)/500;
			//			//			if(discheck-disnext<-8) break;
			//			//			if(discheck2-disnext<-8) break;
			if(count>=2)break;
			//到达0°后break
			if(i==6) break;

			double radiusnext=disnext*std::cos(pitchrad[31-i+1]*PI/180);
			double originxnext=radiusnext*costable[j];
			double originynext=radiusnext*sintable[j];
			double originznext=dis*std::sin(pitchrad[31-i+1]*PI/180);

			Eigen::Matrix<double, 3, 1> originp;
			Eigen::Matrix<double, 3, 1> originpnext;
			Eigen::Matrix<double, 3, 1> originpvirtual;

			originp<<originx,originy,originz;
			originpnext<<originxnext,originynext,originznext;

			double x=(T1*originp)[0];
			double y=(T1*originp)[1];
			double z=(T1*originp)[2];

			double xnext=(T1*originpnext)[0];
			double ynext=(T1*originpnext)[1];
			double znext=(T1*originpnext)[2];
			//
			if(x>-1.5&&x<1.5&&y<4&&y>-1||xnext>-1.5&&xnext<1.5&&ynext<4&&ynext>-1) {
				continue;
			}
			//			heightmat_0.at<float>(i,j)=z;
			//			if(xnext>-1.2&&xnext<1.2&&ynext<4&&ynext>-1) continue;
			//			heightmat_0.at<float>(i,j)=z;

			countlaser++;

			double radiusvirtual=4.0;//虚拟半径，用于盲区检测
			double originxvirtual=radiusvirtual*costable[j];
			double originyvirtual=radiusvirtual*sintable[j];
			double originzvirtual=0;
			originpvirtual<<originxvirtual,originyvirtual,originzvirtual;

			double xvirtual=(T1*originpvirtual)[0];
			double yvirtual=(T1*originpvirtual)[1];

			int col=boost::math::round((x+35)/0.2);
			int row=boost::math::round((y+20)/0.2);

			int colvirtual;
			int rowvirtual;
			if(countlaser==1){
				colvirtual=boost::math::round((xvirtual+35)/0.2);
				rowvirtual=boost::math::round((yvirtual+20)/0.2);
				if(radius>7&&height<-0.6&&(colvirtual<32/0.2||colvirtual>38/0.2)){
					cv::line(gridshow_1,cvPoint(col,GRIDWH-1-row),cvPoint(colvirtual,GRIDWH-1-rowvirtual),cvScalar(0,0,125));
					cv::line(justshow,cvPoint(col,GRIDWH-1-row),cvPoint(colvirtual,GRIDWH-1-rowvirtual),cvScalar(0,0,125));
				}
				diff2=0.3;
			}

			int colnext=boost::math::round((xnext+35)/0.2);
			int rownext=boost::math::round((ynext+20)/0.2);

			int gridindex=0;
			unsigned char* ptr=gridshow_1.ptr<unsigned char>(GRIDWH-1-row);
			if(col<GRIDWH&&row<GRIDWH&&col>=0&&row>=0){

				//增加不同半径下的不同高度阈值
				if(radius<8){if(heightmat_1.at<float>(i,j)>0.3)break;}
				else if(radius<13){if(heightmat_1.at<float>(i,j)>0.5)break;}
				else{if(heightmat_1.at<float>(i,j)>0.8)break;}
				if(j==myj){//
					ptr[3*col]=255;
					ptr[3*col+1]=255;
					ptr[3*col+2]=255;
					cv::line(gridshow_1,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext),cvScalar(255,255,255));
				}
				//针对远距离误检不同阈值
				//短距离用高度差和距离突变来定义
				if(radius<10){
					int thresh=THRESH;
					if(col>155&&col<195) thresh=2*THRESH;
					if(heightnext-height<-0.5&&diff1/diff2>thresh&&diff1/diff2<50){//&&heightnene-height<-0.5
						//					if((height-heightnext)/(radiusnext-radius)>0.1&&diff1/diff2>3){
						ptr[3*col]=0;
						ptr[3*col+1]=0;
						ptr[3*col+2]=255;
						//排除径向距离比较小的
						if((colnext-col)*(colnext-col)*0.04+(rownext-row)*(rownext-row)*0.04<1){
#ifdef showgreen
							cv::line(gridshow_1,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext),cvScalar(0,150,0));
#endif
							continue;
						}
						//连线
						//							if(colnext<GRIDWH&&rownext<GRIDWH&&colnext>=0&&rownext>=0){
#ifdef lineiter
						cv::LineIterator it(gridshow_1,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext));
						it++;
						bool state=true;
						int pointcout=0;
						for(int i=1;i<it.count;i++,++it){
							//									if(i<it.count*0.2||i>it.count*0.8) continue;
							if((*it)[1]==255){
								//										pointcout++;
								int rows=GRIDWH-1-it.pos().y;
								int cols=it.pos().x;
								if((cols-col)*(cols-col)*0.04+(rows-row)*(rows-row)*0.04<1){
									continue;
								}
								(*it)[0]=255;
								(*it)[2]=255;
								if(heightgridmax_1.at<float>(row,col)-heightgridmin_1.at<float>(rows,cols)<0.5){
									state=false;
									break;
								}
							}
							//									(*it)[2]=125;
						}
						if(!state) continue;
#endif
#ifdef USENEIGHBER
						int pointcount=0;
						int zerocount=0;
						for(int l=-2;l<=1;l++){
							for(int k=-WINDOW;k<WINDOW+1;k++){
								//								CONDITION(j){
								//									cout<<dismat_1.at<float>(i+1,j+k)/500<<" "<<
								//											dismat_1.at<float>(i,j+k)/500<<"  "<<dismat_1.at<float>(i-1,j)/500
								//											<<"  "<<dismat_1.at<float>(i-2,j+k)/500<<" "
								//											<<dismat_1.at<float>(i-3,j)/500<<endl;
								//								}
								if(l==0||l==1){
									if(dismat_1.at<float>(i+l,j+k)==0){
										zerocount++;
									}
								}
								if(l==-2){
									if((dismat_1.at<float>(i-1,j)-dismat_1.at<float>(i+l,j+k))/dismat_1.at<float>(i-1,j)>HEIGHTTHRESH){
										pointcount++;
									}
								}
							}
						}
						CONDITION(j){
							cout<<j<<"  "<<pointcount<<" "<<zerocount<<endl;
							//								cout<<j<<"  "<<radius<<"  "<<radiusnext<<endl;
						}
						if(pointcount>COUNTTHRESH||zerocount>ZEROCOUNTTH){
#ifdef showgreen
							cv::line(gridshow_1,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext),cvScalar(0,150,0));
#endif
							continue;
						}
#endif
						//
						//						vector<double> temp;
						//						temp.clear();
						//						for(int i=-1;i<2;++i){
						//							for(int j=-1;j<2;++j){
						//								temp.push_back(heightgridmax_1.at<float>(rownext+i,colnext+j));
						//							}
						//						}
						//						sort(temp.begin(),temp.end());
						cv::LineIterator it(heightgridmax_1,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext));
						//								cout<<"the heights are......"<<endl;
						double hm=-100;
						for(int k=0;k<it.count;++k,it++){
							int x=it.pos().x;
							int y=it.pos().y;
							y=GRIDWH-1-y;
							for(int i=-2;i<3;++i){
								for(int j=-2;j<3;++j){
									double height=0;
									if(x+i>=0&&x+i<GRIDWH&&y+j>=0&&y+j<GRIDWH){
										float* ptr=heightgridmax_1.ptr<float>(y+j);
										height=(double)ptr[x+i];
									}
									if(height>hm) hm=height;
								}
							}
						}
						//								cout<<hm<<endl;
						if(hm<0.4){//temp[temp.size()-1]<1&&temp[temp.size()-1]!=-100
							//							cout<<"the height is "<<temp[temp.size()-1]<<endl;
							//							cout<<"dis is "<<dis<<"  ratio is "<<diff1/diff2<<endl;
							cv::line(justshow,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext),cvScalar(0,0,125));
							cv::line(gridshow_1,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext),cvScalar(0,0,125));
						}
						//							}
					}
					else{
#ifdef showgreen
						cv::line(gridshow_1,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext),cvScalar(0,150,0));
#endif
					}
				}
				//大于10m小于17m需要加上颠簸程度、俯仰角、（起始点高度？--这个还不太确定）、正切值、最小高度、距离突变比
				else if(radius<20){
					int thresh=THRESH-1;
					if(col>155&&col<195) thresh=2*THRESH;
					if(1){//&&height>-0.4 &&yprdiff<7 tmpmsg->pitch<5
						//					if((heightmat_0.at<float>(i-1,j)-heightmat_0.at<float>(i,j))<-1&&diff1/diff2>3){
						if((height-heightnext)/(radiusnext-radius)>0.07&&heightnext-height<-1
								&&diff1/diff2>thresh&&diff1/diff2<50){//&&heightnene-height<-1

							ptr[3*col]=0;
							ptr[3*col+1]=0;
							ptr[3*col+2]=255;
							//排除径向距离比较小的
							if((colnext-col)*(colnext-col)*0.04+(rownext-row)*(rownext-row)*0.04<1){
#ifdef showgreen
								cv::line(gridshow_1,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext),cvScalar(0,150,0));
#endif
								continue;
							}
							//连线
							//								if(colnext<GRIDWH&&rownext<GRIDWH&&colnext>=0&&rownext>=0){
#ifdef lineiter
							cv::LineIterator it(gridshow_1,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext));
							it++;
							bool state=true;
							int pointcount=0;
							for(int i=0;i<it.count;i++,++it){
								//										if(i<it.count*0.2||i>it.count*0.8) continue;
								if((*it)[1]==255){
									//											pointcount++;;
									int rows=GRIDWH-1-it.pos().y;
									int cols=it.pos().x;
									if((cols-col)*(cols-col)*0.04+(rows-row)*(rows-row)*0.04<1){
										continue;
									}
									(*it)[0]=255;
									(*it)[2]=255;
									cout<<heightgridmax_1.at<float>(row,col)<<" "<<heightgridmax_1.at<float>(rows,cols)<<endl;
									if(heightgridmax_1.at<float>(row,col)-heightgridmin_1.at<float>(rows,cols)<0.5){
										state=false;
										break;
									}
								}
								//									(*it)[2]=125;
							}
							if(!state) continue;
#endif
#ifdef USENEIGHBER
							int pointcount=0;
							int zerocount=0;
							for(int l=-2;l<=1;l++){
								for(int k=-WINDOW;k<WINDOW+1;k++){
									//									CONDITION(j){
									//										cout<<dismat_1.at<float>(i+1,j+k)/500<<" "<<
									//												dismat_1.at<float>(i,j+k)/500<<"  "<<dismat_1.at<float>(i-1,j)/500
									//												<<"  "<<dismat_1.at<float>(i-2,j+k)/500<<" "
									//												<<dismat_1.at<float>(i-3,j)/500<<endl;
									//									}
									if(l==0||l==1){
										if(dismat_1.at<float>(i+l,j+k)==0){
											zerocount++;
										}
									}
									if(l==-2){
										if((dismat_1.at<float>(i-1,j)-dismat_1.at<float>(i+l,j+k))/dismat_1.at<float>(i-1,j)>HEIGHTTHRESH){
											pointcount++;
										}
									}
								}
							}
							CONDITION(j){
								cout<<j<<"  "<<pointcount<<" "<<zerocount<<endl;
								//								cout<<j<<"  "<<radius<<"  "<<radiusnext<<endl;
							}
							if(pointcount>COUNTTHRESH||zerocount>ZEROCOUNTTH){
#ifdef showgreen
								cv::line(gridshow_1,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext),cvScalar(0,150,0));
#endif
								continue;
							}
#endif
							//							vector<double> temp;
							//							temp.clear();
							//							for(int i=-1;i<2;++i){
							//								for(int j=-1;j<2;++j){
							//									temp.push_back(heightgridmax_1.at<float>(rownext+i,colnext+j));
							//								}
							//							}
							//							sort(temp.begin(),temp.end());
							cv::LineIterator it(heightgridmax_1,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext));
							//								cout<<"the heights are......"<<endl;
							double hm=-100;
							for(int k=0;k<it.count;++k,it++){
								int x=it.pos().x;
								int y=it.pos().y;
								y=GRIDWH-1-y;
								for(int i=-2;i<3;++i){
									for(int j=-2;j<3;++j){
										double height=0;
										if(x+i>=0&&x+i<GRIDWH&&y+j>=0&&y+j<GRIDWH){
											float* ptr=heightgridmax_1.ptr<float>(y+j);
											height=(double)ptr[x+i];
										}
										if(height>hm) hm=height;
									}
								}
							}
							//								cout<<hm<<endl;
							if(hm<0.4){//temp[temp.size()-1]<1&&temp[temp.size()-1]!=-100
								//								cout<<"the height is "<<temp[temp.size()-1]<<endl;
								//								cout<<"dis is "<<dis<<"  ratio is "<<diff1/diff2<<endl;
								cv::line(justshow,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext),cvScalar(0,0,125));
								cv::line(gridshow_1,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext),cvScalar(0,0,125));
							}
							//								}
						}
						else{
#ifdef showgreen
							cv::line(gridshow_1,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext),cvScalar(0,150,0));
#endif
						}
					}
				}
			}
			if(count>0) i-=count-1;
		}
	}
#endif
	//将两个雷达合一起
	stiff_msgs::stiffwater msg_send;
	msg_send.header.stamp=height_msg->header.stamp;
	msg_send.header.frame_id="stiffwater";
	msg_send.ogmheight=351;
	msg_send.ogmwidth=201;
	msg_send.resolution=0.2;
	msg_send.vehicle_x=100;
	msg_send.vehicle_y=100;
	cv::Mat gridall = cv::Mat::zeros(GRIDWH,GRIDWH,CV_8UC1);
	for(int row=0;row<GRIDWH;row++){
		unsigned char* ptrall=gridall.ptr<unsigned char>(GRIDWH-1-row);
		unsigned char* ptr=gridshow_0.ptr<unsigned char>(GRIDWH-1-row);
		unsigned char* ptr1=gridshow_1.ptr<unsigned char>(GRIDWH-1-row);
		for(int col=0;col<GRIDWH;col++){
			if(ptr[3*col+2]==125||ptr1[3*col+2]==125){
				ptrall[col]=255;
			}
			else if(ptr[3*col+1]==150||ptr1[3*col+1]==150){
				ptrall[col]=100;
			}
		}
	}


	cv::dilate(gridall,gridall,elementdil);
	cv::erode(gridall,gridall,elementero);
	//	cv::dilate(gridall,gridall,elementero2);

	for(int row=0;row<351;row++){
		unsigned char* ptr=gridall.ptr<unsigned char>(GRIDWH-1-row);
		for(int col=0;col<201;col++){
			int index=row*201+col;
			if(ptr[col+75]==255){
				msg_send.data.push_back(5);
			}
			else if(ptr[col+75]==100){
				msg_send.data.push_back(1);
			}
			else{
				msg_send.data.push_back(0);
			}
		}
	}
	pubStiffwaterOgm.publish(msg_send);


	for(int i=4;i<21;i+=2){
		if(i==8||i==14){
			cv::circle(gridshow_0,cvPoint(175,250),(int)(i/0.2),cvScalar(80,0,80));
			cv::circle(gridshow_1,cvPoint(175,250),(int)(i/0.2),cvScalar(80,0,80));
			cv::circle(justshow,cvPoint(175,250),(int)(i/0.2),cvScalar(80,0,80));
			cv::circle(gridall,cvPoint(175,250),(int)(i/0.2),cvScalar(80,0,80));
		}
		else{
			cv::circle(gridshow_0,cvPoint(175,250),(int)(i/0.2),cvScalar(80,80,0));
			cv::circle(gridshow_1,cvPoint(175,250),(int)(i/0.2),cvScalar(80,80,0));
			cv::circle(justshow,cvPoint(175,250),(int)(i/0.2),cvScalar(80,80,0));
			cv::circle(gridall,cvPoint(175,250),(int)(i/0.2),cvScalar(80,0,80));
		}
	}
	//给justshow画角度线
	double unitangle=2.0/870*PI;
	for(int i=0;i<8;i++){
		double angle=-(unitangle*i*100+PI/2)+PI;
		int x=175+1000*std::cos(angle);
		int y=-100+GRIDWH-1-1000*std::sin(angle);
		cv::line(justshow,cvPoint(175,250),cvPoint(x,y),cvScalar(0,165,135));
	}
	cv::namedWindow("show",CV_WINDOW_NORMAL);
	//	int row=350-100-16;
	//		cv::line(gridshow_1,cvPoint(0,row),cvPoint(350,row),cvScalar(0,150,0));
	//		cv::line(gridshow_0,cvPoint(0,row),cvPoint(350,row),cvScalar(0,150,0));
	cv::line(gridshow_0,cvPoint(leftend,0),cvPoint(leftend,GRIDWH-1),cvScalar(0,126,0));
	cv::line(gridshow_0,cvPoint(rightend,0),cvPoint(rightend,GRIDWH-1),cvScalar(0,126,0));
	cv::line(gridshow_1,cvPoint(leftend,0),cvPoint(leftend,GRIDWH-1),cvScalar(0,126,0));
	cv::line(gridshow_1,cvPoint(rightend,0),cvPoint(rightend,GRIDWH-1),cvScalar(0,126,0));
	cv::line(justshow,cvPoint(leftend,0),cvPoint(leftend,GRIDWH-1),cvScalar(0,126,0));
	cv::line(justshow,cvPoint(rightend,0),cvPoint(rightend,GRIDWH-1),cvScalar(0,126,0));
#ifdef TESTSHOW
	cv::imshow("grid",gridshow_0);
	cv::imshow("grid1",gridshow_1);
#endif
	cv::imshow("show",justshow);
	cv::imshow("gridall",gridall);
	//	cv::imshow("range", dismat_0);
	//	cv::imshow("height", heightmat_0);
	cv::waitKey(10);
}

void gpsdatacllbak(const sensor_driver_msgs::GpswithHeadingConstPtr& msg){
	//			cout<<"gpsdata stamp is "<<msg->gps.header.stamp<<endl;
	//		cout<<yprmin<<endl;
	qgwithhmsgs_.Push(msg);
	//	qypr.push_back(msg);
	if(qgwithhmsgs_.Size()>10){
		qgwithhmsgs_.Pop();
	}
	//		if(qypr.size()>100){
	//			qypr.pop_front();
	//		}
	//		mtx.lock();
	//		for(auto it=qypr.begin();it!=qypr.end();it++){
	//			double pitch=(*it)->pitch;
	//			double roll=(*it)->roll;
	//			if(pitch*pitch+roll*roll<yprmin[1]*yprmin[1]+yprmin[2]*yprmin[2]){
	//				yprmin<<0,pitch,roll;
	//			}
	//		}
	//		mtx.unlock();
}
#endif

#ifdef TOYOTA_RS
#define THRESH 3
#define left
std::mutex mtx_cloud;
std::mutex mtx_gpsdata;
float roll = 0;//这里只储存绝对值
float pitch = 0;//不是绝对值
double gpstime = -1;
const float NEAR_BOUND = 25;
const float FAR_BOUND = 45;
const int window_big = 20;
const int window_small_32 = 5;
const int window_small = 5;
const float th = 0.17;
const float th_dis = 0.65;
const vector<int> map_j{
	16,	18,	0,	20,	2,	22,	30,	28,	26,	24,	23,	21,	19,	17,	31,	29,	27,	25,	7,	5,	3,	1,	15,	13,	11,	9,	4,	6,	8,	10,	12,	14
};
bool ptUseful(pcl::PointXYZI& pt, float dis_th){
	float  z = pt.z;
	float  x = pt.x;
	float  y = pt.y;
	float dis = sqrt(x*x + y*y + z*z);
	if(pt.range < 0 || dis > dis_th || z > 3 || z < -3 || y < 0 || x > 100)
		return false;
	return true;
}
void useOne32(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
		pcl::PointCloud<pcl::PointXYZI>::Ptr newcloud,
		pcl::PointCloud<pcl::PointXYZI>::Ptr h_cloud,
		pcl::PointCloud<pcl::PointXYZI>::Ptr l_cloud, int layer, cv::Mat grid32,
		Eigen::Quaterniond q,
		pcl::PointCloud<pcl::PointXYZI>::Ptr single){
	const int round32 = cloud->size() / 32;
	float far_bound = FAR_BOUND;

	for(int j = 0; j < 0; ++j){
		for(int i = 0; i < round32; ++i){
			int index_single = i * 32 + map_j[j];
				auto pt = newcloud->points[index_single];
				float z = pt.z;
				auto pt_ori = cloud->points[index_single];
				if(z < 0.3)
					single->points.push_back(pt_ori);
		}
	}
	for(int j = 0; j < 25; ++j){
//				if( map_j[j] >  25) continue;
		//		std::cout << " ----------------------- " << std::endl;
		for(int i = 0; i + window_big < round32; i+=window_small_32){
			float  height_diff_most = 10, x0 = 0, y0 = 0, x1 = 0, y1 = 0, z0 = 0, z1 = 0;
			float dis_in_window = 0;
			int i_begin = 0;
			float dis_ratio = 0, dis_tocheck = 0, tangent = 0;

			//大窗口
			for(int k = i; k + window_small_32 < i + window_big; ++k){
				float z_high = 0, y_high = 0, x_high = 0;
				float z_low = 0, y_low = 0, x_low = 0;
				float dis_av_high = 0, dis_av_low = 0;
				int count = 0;
				int cnt = 0;
				//小窗口-1
				for(int window_i = k; window_i < k + window_small_32; window_i++){
					int index = window_i * 32 + map_j[j];
					float  z = newcloud->points[index].z;
					float  x = cloud->points[index].x;
					float  y = cloud->points[index].y;
					float dis = sqrt(x*x + y*y + z*z);
					int ran_cnt = 0;
					float ran_z_high = 0, ran_x_high = 0, ran_y_high = 0, ran_dis_high = 0;
					//进行一个类似ransac的过程，寻找最好的能表示z的值
					for(int ran_i = k; ran_i < k + window_small_32; ran_i++){
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
						if(ran_cnt > window_small_32 / 2){
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
				for(int window_i = k + window_small_32; window_i < k + 2 * window_small_32; window_i++){
					int index = window_i * 32 + map_j[j];
					float  z = newcloud->points[index].z;
					float  x = cloud->points[index].x;
					float  y = cloud->points[index].y;
					float dis = sqrt(x*x + y*y + z*z);

					int ran_cnt = 0;
					float ran_z_low = 0, ran_x_low = 0, ran_y_low = 0, ran_dis_low = 0;
					for(int ran_i = k + window_small_32; ran_i < k + 2 * window_small_32; ran_i++){
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
						if(ran_cnt > window_small_32 / 2){
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
				for(int i = 0; i < window_small_32; ++i){
					int i_high = (i_begin + window_small_32 - 1 - i) * 32 + map_j[j];
					int i_low = (i_begin + window_small_32 + i) * 32 + map_j[j];
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
					for(int window_i = i_begin; window_i < i_begin + window_small_32; window_i++){
						int ran_cnt = 1;
						int index = window_i * 32 + map_j[j];
						float z = newcloud->points[index].z;
						for(int ran_i = i_begin; ran_i < i_begin + window_small_32; ran_i++){
							int ran_index = ran_i * 32 + map_j[j];
							float  z_ran = newcloud->points[ran_index].z;
							float  z_ran_ori = cloud->points[ran_index].z;
							float  x_ran = cloud->points[ran_index].x;
							float  y_ran = cloud->points[ran_index].y;
							float dis_ran = sqrt(x_ran*x_ran + y_ran*y_ran + z_ran_ori*z_ran_ori);
							if(ran_i == window_i ||cloud->points[ran_index].range < 0 || y_ran < 0 || dis_ran > far_bound
									|| abs(z_ran - z) > 0.3) continue;
							ran_cnt ++;
							if(ran_cnt > window_small_32 / 2){
								count = ran_cnt;
								break;
							}
						}
					}
					//					std::cout << ">>>>>>>count>>>>>>>>>> " << count << std::endl;
					//					std::cout << ">>>>>>>z0   >>>>>>>>>> " << z0 << std::endl;
					for(int k = i_begin; k <  i_begin + window_small_32; ++k){
						int index_high = k * 32 + map_j[j];
						int index_low = (k + window_small_32) * 32 + map_j[j];
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
							h_cloud->points.push_back(pt_high_ori);
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
						l_cloud->points.push_back(pt_low_ori);
					}
					{//test
						//						for(int k = i_begin - 5; k < i_begin + window_small_32; ++k){
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
						cv::line(grid32, cvPoint(begin_x, begin_y), cvPoint(end_x, end_y), cvScalar(0,0,125),3);
					}
				}
				i_begin = 0;
				height_diff_most = 10;
			}
		}
	}
}
#define NEW
void useTwo16(cv::Mat grid, cv::Mat grid_msg_show, cv::Mat grid_height, cv::Mat grid_h_show, Eigen::Quaterniond q){
	pcl::PointCloud<pcl::PointXYZI>::Ptr tempcloud(new pcl::PointCloud<pcl::PointXYZI>);//当前帧点云（雷达里程计坐标系）
	mtx_cloud.lock();
	if(lidarCloudMsgs_ != nullptr)
		pcl::fromROSMsg(*lidarCloudMsgs_, *tempcloud);//获取当前帧点云数据
	mtx_cloud.unlock();
	if(lidarCloudMsgs_ != nullptr){
		std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> outputclouds;//多个激光雷达数据包，向量中每个元素为一个激光雷达一帧数据
		std::vector<pcl::PointXYZI> lidarpropertys;//每一个PointType类型都表示一个单独点
		analysisCloud(tempcloud,outputclouds,lidarpropertys);

		//两侧雷达的高低点云
		pcl::PointCloud<pcl::PointXYZI>::Ptr left_high_cloud(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZI>::Ptr left_low_cloud(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZI>::Ptr right_high_cloud(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZI>::Ptr right_low_cloud(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZI>::Ptr check(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZI>::Ptr single(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZI>::Ptr single_ori(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZI>::Ptr new16_left(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZI>::Ptr new16_right(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZI>::Ptr new32(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZI>::Ptr high_cloud_32(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZI>::Ptr low_cloud_32(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::transformPointCloud(*outputclouds[0], *new32, Eigen::Vector3d(0,0,0), q);
		pcl::transformPointCloud(*outputclouds[1], *new16_left, Eigen::Vector3d(0,0,0), q);
		pcl::transformPointCloud(*outputclouds[2], *new16_right, Eigen::Vector3d(0,0,0), q);
		const int layer = 16;
		const int round = outputclouds[1]->points.size() / layer;
		//点云投影到栅格
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
				auto ptr = grid.ptr<unsigned char>(GRIDWH - 1 - row);
				auto ptrall = grid_msg_show.ptr<unsigned char>(GRIDWH - 1 - row);
				auto ptr_show = grid_h_show.ptr<unsigned char>(GRIDWH - 1 - row);
				ptrall[col] = 1;
				ptr[3*col + 1] = 255;
				ptr_show[3*col + 1] = 255;
			}
			if(col >= 32 / 0.2 && col <= 38 / 0.2 && row >= 20 / 0.2 && row <= 35 / 0.2){
				if(col >= 34 /0.2 && col <= 36 / 0.2)
					continue;
				auto ptr_height = grid_height.ptr<float>(row);

				float z_val = z_rec;
				if(z_val > ptr_height[col] && z_val < 1){
					ptr_height[col] = z_val;
				}
			}
		}
		for(int row = 20 / 0.2 ; row <= 35 / 0.2; row++){
			auto ptr = grid_height.ptr<float>(row);
			auto ptr_show = grid_h_show.ptr<unsigned char>(GRIDWH - 1 - row);
			float height_diff = -1;
			int col_target = -1;
			for(int col = 35 / 0.2; col <= 38 / 0.2 - 1; col++){
				if(ptr[col] - ptr[col + 1] > height_diff){
					height_diff = ptr[col] - ptr[col + 1];
					col_target = col;
				}
			}
			if(height_diff > 0.3){
				ptr_show[3 * col_target + 1] = 0;
				ptr_show[3 * col_target + 2] = 255;
			}
		}
#ifdef NEW
		pcl::transformPointCloud(*tempcloud, *tempcloud, Eigen::Vector3d(0,0,0), q);
#endif//NEW
		float far_bound = FAR_BOUND;
		for(int j = 0; j < 16; ++j){
			for(int i = 0; i + window_big < round; ){
				float  height_diff_most = 10, x0 = 0, y0 = 0, x1 = 0, y1 = 0, z0 = 0, z1 = 0;
				float dis_in_window = 0;
				vector<float> test_ys;
				int i_begin = 0;
				float dis_ratio = 0, dis_tocheck = 0, tangent = 0;
				int count_o = 0;
				//高度太高直接可以定为悬崖--不好用
				bool tooHigh = false;
				//大窗口--左雷达
				for(int k = i; k + window_small < i + window_big; ++k){
					float z_high = 0, y_high = 0, x_high = 0;
					float z_low = 0, y_low = 0, x_low = 0;
					float dis_av_high = 0, dis_av_low = 0;
					int count = 0;
					int cnt = 0;
					//小窗口-前
					for(int window_i = k; window_i < k + window_small; window_i++){
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
					for(int window_i = k + window_small; window_i < k + 2 * window_small; window_i++){
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
					if(count > window_small / 2) {
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
					if(height_diff < - 0.6 && height_diff < height_diff_most && count > window_small / 2){
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
					for(int i = 0; i < window_small; ++i){
						int i_high = (i_begin + window_small - 1 - i) * layer + j;
						int i_low = (i_begin + window_small + i) * layer + j;
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
						check->points.push_back(pt_high);
						check->points.push_back(pt_low);
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
						offset = 0.15;
					if(((dis_ratio < th_dis + offset&& dis_ratio > 0.2) && z0 < 0.5 && tangent > th)
							){// || (dis_tocheck < 15 && z_diff_nb > 1 && z0 < 0.5 && tangent > 0.15)
//						std::cout << "tanget is ... " << tangent << std::endl;
//						std::cout << "dis in window are  ... " << dis_in_window << std::endl;
						for(int k = i_begin; k <  i_begin + window_small; ++k){
							int index_high = k * layer + j;
							int index_low = (k + window_small) * layer + j;
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
								left_high_cloud->points.push_back(pt_high);//todo:历史遗留问题
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
							left_low_cloud->points.push_back(pt_low);
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
							cv::line(grid, cvPoint(begin_x, begin_y), cvPoint(end_x, end_y), cvScalar(0,0,125),3);
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
				for(int k = i; k + window_small < i + window_big; ++k){
					float z_high = 0, y_high = 0, x_high = 0;
					float z_low = 0, y_low = 0, x_low = 0;
					float dis_av_high = 0, dis_av_low = 0;
					int count = 0;
					//小窗口-high
					for(int window_i = k + window_small; window_i < k + 2 * window_small; window_i++){
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
					for(int window_i = k; window_i < k + window_small; window_i++){
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
					if(height_diff < - 0.6 && height_diff < height_diff_most && count > window_small / 2){
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
					for(int i = 0; i < window_small; ++i){
						int i_low = (i_begin + window_small - 1 - i) * layer + j;
						int i_high = (i_begin + window_small + i) * layer + j;
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
						check->points.push_back(pt_high);
						check->points.push_back(pt_low);
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
						offset = 0.15;
					if(((dis_ratio < th_dis + offset && dis_ratio > 0.2) && z0 < 0.5 && tangent > th)
							){//(dis_tocheck < 15 && z_diff_nb > 1 && z0 < 0.5 && tangent > 0.15)
						std::cout << "tangent ... " << tangent << std::endl;
						std::cout << "dis_ratio  ... " << dis_ratio << std::endl;
						for(int k = i_begin; k <  i_begin + window_small; ++k){
							int index_low = k * layer + j;
							int index_high = (k + window_small) * layer + j;
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
								right_high_cloud->points.push_back(pt_high_new);
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
							right_low_cloud->points.push_back(pt_low_new);
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
							cv::line(grid, cvPoint(begin_x, begin_y), cvPoint(end_x, end_y), cvScalar(0,0,125),3);
						}
					}
				}
				i += window_big / 2;
			}
		}



		//			test

		int layer32 = 15;
		//		const int laye32 = outputclouds[0]->size() / 32;
		//		for(int i = 0; i < laye32; ++i){
		//			int index = i * 32 + 28;
		//			auto pt = outputclouds[0]->points[index];
		//			single->points.push_back(pt);
		//		}
		useOne32(outputclouds[0], new32, high_cloud_32, low_cloud_32, 34, grid, q, single);

		#ifdef VIEWER
		cloud_viewer_->removeAllPointClouds();
		{
			//test
#ifdef NEW
			if (  0 )//newcloud->size() >
			{
				pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> cloudHandler( new32, 0, 255, 0 );
				if (!cloud_viewer_->updatePointCloud(new32,cloudHandler, "new"))
				{

					cloud_viewer_->addPointCloud(new32, cloudHandler, "new");
				}
			}
#else
			if ( outputclouds[1]->size() > 0 ) //
			{

				pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> cloudHandler( outputclouds[1], 0, 255, 0 );
				if (!cloud_viewer_->updatePointCloud(outputclouds[1],cloudHandler, "out1"))
				{
					//				std::cout<<"cloudgeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeet"<<std::endl;
					cloud_viewer_->addPointCloud(outputclouds[1], cloudHandler, "out1");
				}
			}
#endif //NEW
		}
		if (tempcloud->size() > 0)//
		{
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> cloudHandler( tempcloud, 0, 255, 0 );
			if (!cloud_viewer_->updatePointCloud(tempcloud,cloudHandler, "0"))
			{
				cloud_viewer_->addPointCloud(tempcloud, cloudHandler, "0");
			}
		}
		if (0)//new16_right->size() >
		{
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> cloudHandler( new16_right, 0, 255, 0 );
			if (!cloud_viewer_->updatePointCloud(new16_right,cloudHandler, "new16r"))
			{
				cloud_viewer_->addPointCloud(new16_right, cloudHandler, "new16l");
			}
		}
		if (  0 ) //outputclouds[0]->size() >
		{

			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> cloudHandler( outputclouds[0], 0, 255, 0 );
			if (!cloud_viewer_->updatePointCloud(outputclouds[0],cloudHandler, "0"))
			{
				cloud_viewer_->addPointCloud(outputclouds[0], cloudHandler, "0");
			}
		}
		if ( high_cloud_32->size() > 0 )//
		{

			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> cloudHandler( high_cloud_32, 0, 0, 255 );
			if (!cloud_viewer_->updatePointCloud(high_cloud_32,cloudHandler, "32high"))
			{
				cloud_viewer_->addPointCloud(high_cloud_32, cloudHandler, "32high");
			}
		}
		if ( low_cloud_32->size() > 0 )//
		{

			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> cloudHandler( low_cloud_32, 255, 0, 0 );
			if (!cloud_viewer_->updatePointCloud(low_cloud_32,cloudHandler, "32low"))
			{
				cloud_viewer_->addPointCloud(low_cloud_32, cloudHandler, "32low");
			}
		}

		if (  0 )//outputclouds[1]->size() >
		{

			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> cloudHandler( outputclouds[1], 0, 255, 0 );
			if (!cloud_viewer_->updatePointCloud(outputclouds[1],cloudHandler, "left"))
			{
				cloud_viewer_->addPointCloud(outputclouds[1], cloudHandler, "left");
			}
		}
		if ( 0 )//outputclouds[2]->size() >
		{

			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> cloudHandler( outputclouds[2], 0, 255, 0 );
			if (!cloud_viewer_->updatePointCloud(outputclouds[2],cloudHandler, "right"))
			{
				cloud_viewer_->addPointCloud(outputclouds[2], cloudHandler, "right");
			}
		}
		if ( right_high_cloud->size() > 0 )
		{
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> cloudHandler( right_high_cloud, 255, 0, 0 );
			if (!cloud_viewer_->updatePointCloud(right_high_cloud,cloudHandler, "color"))
			{
				cloud_viewer_->addPointCloud(right_high_cloud, cloudHandler, "color");
			}
		}
		if ( right_low_cloud->size() > 0 ){
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> cloudHandler( right_low_cloud, 0, 0, 255 );
			if (!cloud_viewer_->updatePointCloud(right_low_cloud,cloudHandler, "colorb"))
			{
				cloud_viewer_->addPointCloud(right_low_cloud, cloudHandler, "colorb");
			}
		}
		if ( left_high_cloud->size() > 0 ) //
		{

			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> cloudHandler( left_high_cloud, 255, 0, 0 );
			if (!cloud_viewer_->updatePointCloud(left_high_cloud,cloudHandler, "stiff"))
			{
				cloud_viewer_->addPointCloud(left_high_cloud, cloudHandler, "stiff");
			}
		}
		if (  left_low_cloud->size() > 0 ) //
		{
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> cloudHandler( left_low_cloud, 0, 0, 255 );
			if (!cloud_viewer_->updatePointCloud(left_low_cloud,cloudHandler, "stiff_back"))
			{
				cloud_viewer_->addPointCloud(left_low_cloud, cloudHandler, "stiff_back");
			}
		}
		if (  check->size() > 0 ) //
		{

			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> cloudHandler( check, 255, 255, 255 );
			if (!cloud_viewer_->updatePointCloud(check,cloudHandler, "check"))
			{
				cloud_viewer_->addPointCloud(check, cloudHandler, "check");
			}
		}
		if (  single->size() > 0 ) //
		{

			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> cloudHandler( single, 255, 0, 0 );
			if (!cloud_viewer_->updatePointCloud(single,cloudHandler, "single"))
			{
				cloud_viewer_->addPointCloud(single, cloudHandler, "single");
			}
		}
		cloud_viewer_->spinOnce();
#endif //VIEWER
#ifdef VIEWER_ORI
		cloud_viewer_origin->removeAllPointClouds();
		if (  outputclouds[1]->size() > 0 ) //
		{

			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> cloudHandler( outputclouds[1], 0, 255, 0 );
			if (!cloud_viewer_origin->updatePointCloud(outputclouds[1],cloudHandler, "out1"))
			{
				//				std::cout<<"cloudgeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeet"<<std::endl;
				cloud_viewer_origin->addPointCloud(outputclouds[1], cloudHandler, "out1");
			}
		}
		if (  single_ori->size() > 0 ) //
		{

			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> cloudHandler(single_ori, 255, 0, 0 );
			if (!cloud_viewer_origin->updatePointCloud(single_ori,cloudHandler, "11"))
			{
				//				std::cout<<"cloudgeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeet"<<std::endl;
				cloud_viewer_origin->addPointCloud(single_ori, cloudHandler, "11");
			}
		}
		cloud_viewer_origin->spinOnce();
#endif //VIEWER_ORI
	}//lidarCloudMsgs_ != nullptr
}
void callback(const depth_image_utils::HeightMapConstPtr& height_msg){
//	cout<<"height map timestamp is "<<height_msg->header.stamp<<endl;
	//	sensor_driver_msgs::GpswithHeadingConstPtr tmpmsg=qgwithhmsgs_.PopWithTimeout(common::FromSeconds(0.1));
	//	if(tmpmsg==nullptr) return;
	//	double gpsstamp=tmpmsg->gps.header.stamp.toSec();
	//	double lidarstamp=height_msg->header.stamp.toSec();
	//	if(gpsstamp>lidarstamp){
	//		cout<<"waite for height map"<<endl;
	//		qgwithhmsgs_.Push_Front(std::move(tmpmsg));
	//		return;
	//	}
	//	while(lidarstamp-gpsstamp>0.00001){
	//		if(qgwithhmsgs_.Size()==0){
	//			return;//这里可以增加队列这样不用浪费一帧信息
	//		}
	//		tmpmsg=qgwithhmsgs_.Pop();
	//		gpsstamp=tmpmsg->gps.header.stamp.toSec();
	//	}
	//	double yprdiff=(tmpmsg->pitch-yprmin[1])*(tmpmsg->pitch-yprmin[1])+(tmpmsg->roll-yprmin[2])*(tmpmsg->roll-yprmin[2]);
	//	double yprdiff=(tmpmsg->pitch-yprlast[1])*(tmpmsg->pitch-yprlast[1])+(tmpmsg->roll-yprlast[2])*(tmpmsg->roll-yprlast[2]);
	//	if(0){
	//		//		cout<<"height"<<height_msg->header.stamp<<endl;
	//		//		cout<<"gpsdata"<<tmpmsg->gps.header.stamp<<endl;
	//		cout<<"the ypr  is"<<endl;
	//		cout<<yprdiff<<endl;
	//		//		cout<<yprmin<<endl;
	//		//		cout<<tmpmsg->heading<<endl;
	//		cout<<tmpmsg->pitch<<endl;
	//		cout<<tmpmsg->roll<<endl;
	//	}
	//	yprmin<<100,100,100;
	//	cv::namedWindow("range",CV_WINDOW_NORMAL);
	sensor_driver_msgs::GpswithHeadingConstPtr tmpmsg=qgwithhmsgs_.PopWithTimeout(common::FromSeconds(0.1));
	if(tmpmsg==nullptr || lidarCloudMsgs_ == nullptr){//
		std::cout << "I m returnning ....... " << std::endl;
		return;
	}
	double lidarstamp = lidarCloudMsgs_->header.stamp.toSec();
	double gpsstamp=tmpmsg->gps.header.stamp.toSec();
	if(gpsstamp>lidarstamp){
		cout<<"waite for height map"<<endl;
		qgwithhmsgs_.Push_Front(std::move(tmpmsg));
		return;
	}
	while(fabs(lidarstamp-gpsstamp)>0.02){
		if(qgwithhmsgs_.Size()==0){
			return;//这里可以增加队列这样不用浪费一帧信息
		}
		tmpmsg=qgwithhmsgs_.Pop();
		gpsstamp=tmpmsg->gps.header.stamp.toSec();
	}
	//	{
	//test
	//		std::cout << lidarCloudMsgs_->header.stamp.toSec() - gpsstamp << std::endl;
	//		std::cout << "------------------------" << std::endl;
	//		std::cout << "roll: " << tmpmsg->roll << std::endl;
	//		std::cout << "pitch: " << tmpmsg->pitch << std::endl;
	pcl::PointCloud<pcl::PointXYZI>::Ptr tempcloud(new pcl::PointCloud<pcl::PointXYZI>);//当前帧点云（雷达里程计坐标系）

	mtx_cloud.lock();
	if(lidarCloudMsgs_ != nullptr)
		pcl::fromROSMsg(*lidarCloudMsgs_, *tempcloud);//获取当前帧点云数据
	mtx_cloud.unlock();
	pcl::PointCloud<pcl::PointXYZI>::Ptr newcloud(new pcl::PointCloud<pcl::PointXYZI>);//当前帧点云（雷达里程计坐标系）

	if(tempcloud != nullptr){//
		double yaw = tmpmsg->heading, pitch = tmpmsg->pitch, roll = tmpmsg->roll;
		Eigen::AngleAxisd yaw_ = Eigen::AngleAxisd(yaw * PI / 180, Eigen::Vector3d::UnitZ());
		Eigen::AngleAxisd pitch_ = Eigen::AngleAxisd(pitch * PI / 180, Eigen::Vector3d::UnitX());
		Eigen::AngleAxisd roll_ = Eigen::AngleAxisd(roll * PI / 180, Eigen::Vector3d::UnitY());

		Eigen::Matrix3d R2 = yaw_.matrix() * roll_.matrix() * pitch_.matrix();
		Eigen::Matrix3d R = yaw_.matrix() * pitch_.matrix() * roll_.matrix();
//				Eigen::Quaterniond test=transform::RollPitchYaw( roll * PI / 180,pitch * PI / 180 ,yaw * PI / 180);//0,0,0,1
		Eigen::Quaterniond test(R);
		//		Eigen::Quaterniond test(0,0,0,1);
		Eigen::Quaterniond test2(R2);
		//		std::cout << test.w() << " "
		//				<< test.x() << " "
		//				<< test.y() << " "
		//				<< test.z() << " ---  "
		//				<< test2.w() << " "
		//				<< test2.x() << " "
		//				<< test2.y() << " "
		//				<< test2.z() << " " << std::endl;
		std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> outputclouds;//多个激光雷达数据包，向量中每个元素为一个激光雷达一帧数据
		std::vector<pcl::PointXYZI> lidarpropertys;//每一个PointType类型都表示一个单独点
		analysisCloud(tempcloud,outputclouds,lidarpropertys);
		pcl::transformPointCloud(*outputclouds[0], *newcloud, Eigen::Vector3d(0,0,0), test);

		//	}
		cv::Mat gridall = cv::Mat::zeros(GRIDWH,GRIDWH,CV_8UC1);
		cv::Mat grid16 = cv::Mat::zeros(GRIDWH,GRIDWH,CV_8UC3);
		cv::Mat grid32 = cv::Mat::zeros(GRIDWH,GRIDWH,CV_8UC3);
		cv::Mat grid_h = cv::Mat::zeros(GRIDWH,GRIDWH,CV_32FC1);
		cv::Mat grid_h_show = cv::Mat::zeros(GRIDWH,GRIDWH,CV_8UC3);
		{//ｔｅｓｔ
//			int grid_width = GRIDWH / 4;
//			cv::Mat grid_max = cv::Mat::ones(grid_width,grid_width,CV_32FC1) * (-10);
//			cv::Mat grid_min = cv::Mat::ones(grid_width,grid_width,CV_32FC1) * 10;
//			cv::Mat grid_count = cv::Mat::zeros(grid_width,grid_width,CV_8UC1);
//			cv::Mat grid_show = cv::Mat::zeros(grid_width,grid_width,CV_8UC3);
//			for(auto pt : tempcloud->points){
//				if(pt.range < 0) continue;
//				float x = pt.x;
//				float y = pt.y;
//				float z = pt.z;
//				Eigen::Vector3d pt_rec(x, y, z);
//				if(z > 1) continue;
//				float z_rec = (test * pt_rec).z();
//				int col=boost::math::round((x+35)/0.8);//干！！！！！！！！！！！
//				int row=boost::math::round((y+20)/0.8);
//				if(col >= 0 && col < grid_width && row >= 0 && row < grid_width){
//					auto ptr_max = grid_max.ptr<float>(row);
//					auto ptr_min = grid_min.ptr<float>(row);
//					auto ptr_count = grid_count.ptr<unsigned char>(row);
//					auto ptr_show = grid_show.ptr<unsigned char>(grid_width - 1 - row);
//					ptr_show[3*col + 1] = 255;
//					if(z > ptr_max[col]){
//						ptr_max[col] = z;
//					}
//					if(z < ptr_min[col]){
//						ptr_min[col] = z;
//					}
//					if(ptr_max[col] - ptr_min[col] > 0.3){
//						ptr_count[col]++;
////						if(ptr_count[col] > 5){
////							ptr_show[3*col + 1] = 0;
////							ptr_show[3*col + 2] = 255;
////						}
//					}
//				}
//			}
//			for(int row = 20 / 0.8; row < 35 / 0.8; row++){
//				float max = -11, min = 11;
//				int x_max = 0, x_min = 0;
//				for(int col = 37 / 0.8; col < 37 / 0.8 + 5; col++){
//					int index = row * grid_width + col;
//					if(grid_max.at<float>(row, col) > max){
//						max = grid_max.at<float>(row, col);
//						x_max = col;
//					}
//					if(grid_max.at<float>(row, col) < min){
//						min = grid_max.at<float>(row, col);
//						x_min = col;
//					}
//				}
//
//				if(max - min > 0.5 && x_max < x_min && max < 0.3){
//					auto ptr_show = grid_show.ptr<unsigned char>(grid_width - 1 - row);
//					for(int col = 37 / 0.8; col < 37 / 0.8 + 5; col++){
//						ptr_show[3*col + 1] = 0;
//						ptr_show[3*col + 2] = 255;
//					}
//				}
//			}
//			cv::namedWindow("hdiff",CV_WINDOW_NORMAL);
//			cv::imshow("hdiff", grid_show);
		}//test
		useTwo16(grid16, gridall, grid_h, grid_h_show, test);
		cv::namedWindow("16",CV_WINDOW_NORMAL);
		cv::namedWindow("3",CV_WINDOW_NORMAL);
		cv::imshow("16", grid16);
		cv::imshow("3", grid_h_show);
		cv::waitKey(5);
		if(visual_on){

		}
		//	cv::imshow("32", grid32);
		//	cv::waitKey(5);
		if(visual_on){
			cv::namedWindow("another",CV_WINDOW_NORMAL);
			cv::namedWindow("show",CV_WINDOW_NORMAL);
		}

		////////////////////////////////////////////////////from here ///////////////////////////////////////////////////////////

		stiff_msgs::stiffwater msg_send;
		msg_send.header.stamp=height_msg->header.stamp;
		msg_send.header.frame_id="stiffwater";
		msg_send.ogmheight=351;
		msg_send.ogmwidth=201;
		msg_send.resolution=0.2;
		msg_send.vehicle_x=100;
		msg_send.vehicle_y=100;
		for(int row=0;row<GRIDWH;row++){
			unsigned char* ptrall=gridall.ptr<unsigned char>(GRIDWH-1-row);
			unsigned char* ptr=grid16.ptr<unsigned char>(GRIDWH-1-row);
			for(int col=0;col<GRIDWH;col++){
				if(ptr[3*col+2]==125){
					ptrall[col]=255;//障碍物
				}
				//			else if(ptr[3*col + 1] == 255){//水区域 ptr[3*col+2]==125&&ptr[3*col+1]==125&&ptr[3*col]==125
				//				ptrall[col]=200;
				//			}
				else if(ptr[3*col+1]==255){
					ptrall[col]=100;//通行可
				}
			}
		}


			cv::dilate(gridall,gridall,elementdil);
			cv::erode(gridall,gridall,elementero);
		//	cv::dilate(gridall,gridall,elementero2);

		for(int row=0;row<351;row++){
			unsigned char* ptr=gridall.ptr<unsigned char>(GRIDWH-1-row);
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
		pubStiffwaterOgm.publish(msg_send);


		for(int i=4;i<21;i+=2){
			if(i==8||i==14){
				//			cv::circle(gridshow_0,cvPoint(175,250),(int)(i/0.2),cvScalar(80,0,80));
				//			cv::circle(justshow,cvPoint(175,250),(int)(i/0.2),cvScalar(80,0,80));
				cv::circle(gridall,cvPoint(175,250),(int)(i/0.2),cvScalar(80,0,80));
			}
			else{
				//			cv::circle(gridshow_0,cvPoint(175,250),(int)(i/0.2),cvScalar(80,80,0));
				//			cv::circle(justshow,cvPoint(175,250),(int)(i/0.2),cvScalar(80,80,0));
				cv::circle(gridall,cvPoint(175,250),(int)(i/0.2),cvScalar(80,0,80));
			}
		}
		//给justshow画角度线
		double unitangle=2.0/870*PI;
		for(int i=0;i<8;i++){
			double angle=-(unitangle*i*100+PI/2)+PI;
			int x=175+1000*std::cos(angle);
			int y=-100+GRIDWH-1-1000*std::sin(angle);
			//		cv::line(justshow,cvPoint(175,250),cvPoint(x,y),cvScalar(0,165,135));
		}
		//	int row=350-100-16;
		//		cv::line(gridshow_1,cvPoint(0,row),cvPoint(350,row),cvScalar(0,150,0));
		//		cv::line(gridshow_0,cvPoint(0,row),cvPoint(350,row),cvScalar(0,150,0));
		//	cv::line(gridshow_0,cvPoint(leftend,0),cvPoint(leftend,GRIDWH-1),cvScalar(0,126,0));
		//	cv::line(gridshow_0,cvPoint(rightend,0),cvPoint(rightend,GRIDWH-1),cvScalar(0,126,0));
		//	cv::line(justshow,cvPoint(leftend,0),cvPoint(leftend,GRIDWH-1),cvScalar(0,126,0));
		//	cv::line(justshow,cvPoint(rightend,0),cvPoint(rightend,GRIDWH-1),cvScalar(0,126,0));
		if(visual_on){
#ifdef TESTSHOW
			//		cv::imshow("grid",gridshow_0);
#endif
			//		cv::imshow("show",justshow);
			cv::imshow("gridall",gridall);
			//	cv::imshow("range", dismat_0);
			//	cv::imshow("height", heightmat_0);
			cv::waitKey(10);
		}
	}
}

void gpsdatacllbak(const sensor_driver_msgs::GpswithHeadingConstPtr& msg){
//	cout<<"gpsdata stamp is "<<msg->gps.header.stamp<<endl;
	//		cout<<yprmin<<endl;
	qgwithhmsgs_.Push(msg);
	//	qypr.push_back(msg);
	if(qgwithhmsgs_.Size()>20){
		qgwithhmsgs_.Pop();
	}
	mtx_gpsdata.lock();
	gpstime = msg->gps.header.stamp.toSec();
	roll = fabs(msg->roll);
	pitch = msg->pitch > 0 ? msg->pitch : 0;
	mtx_gpsdata.unlock();

}
#endif
#ifdef SIXT_RS
#define THRESH 3
#define left
void callback(const depth_image_utils::HeightMapConstPtr& height_msg){
	//		cout<<"height map timestamp is "<<height_msg->header.stamp<<endl;
	//	sensor_driver_msgs::GpswithHeadingConstPtr tmpmsg=qgwithhmsgs_.PopWithTimeout(common::FromSeconds(0.1));
	//	if(tmpmsg==nullptr) return;
	//	double gpsstamp=tmpmsg->gps.header.stamp.toSec();
	//	double lidarstamp=height_msg->header.stamp.toSec();
	//	if(gpsstamp>lidarstamp){
	//		cout<<"waite for height map"<<endl;
	//		qgwithhmsgs_.Push_Front(std::move(tmpmsg));
	//		return;
	//	}
	//	while(lidarstamp-gpsstamp>0.00001){
	//		if(qgwithhmsgs_.Size()==0){
	//			return;//这里可以增加队列这样不用浪费一帧信息
	//		}
	//		tmpmsg=qgwithhmsgs_.Pop();
	//		gpsstamp=tmpmsg->gps.header.stamp.toSec();
	//	}
	//	double yprdiff=(tmpmsg->pitch-yprmin[1])*(tmpmsg->pitch-yprmin[1])+(tmpmsg->roll-yprmin[2])*(tmpmsg->roll-yprmin[2]);
	//	double yprdiff=(tmpmsg->pitch-yprlast[1])*(tmpmsg->pitch-yprlast[1])+(tmpmsg->roll-yprlast[2])*(tmpmsg->roll-yprlast[2]);
	//	if(0){
	//		//		cout<<"height"<<height_msg->header.stamp<<endl;
	//		//		cout<<"gpsdata"<<tmpmsg->gps.header.stamp<<endl;
	//		cout<<"the ypr  is"<<endl;
	//		cout<<yprdiff<<endl;
	//		//		cout<<yprmin<<endl;
	//		//		cout<<tmpmsg->heading<<endl;
	//		cout<<tmpmsg->pitch<<endl;
	//		cout<<tmpmsg->roll<<endl;
	//	}
	//	yprmin<<100,100,100;
	//	cv::namedWindow("range",CV_WINDOW_NORMAL);
	cv::Mat gridall = cv::Mat::zeros(GRIDWH,GRIDWH,CV_8UC1);
	if(visual_on)
		cv::namedWindow("another",CV_WINDOW_NORMAL);
	if(lidarCloudMsgs_ != nullptr){
		//		cout<<"got lidar"<<endl;
		mtx_cloud.lock();
		double timeLaserCloudFullRes = lidarCloudMsgs_->header.stamp.toSec();
		pcl::PointCloud<pcl::PointXYZI>::Ptr tempcloud(new pcl::PointCloud<pcl::PointXYZI>);//当前帧点云（雷达里程计坐标系）
		pcl::fromROSMsg(*lidarCloudMsgs_, *tempcloud);//获取当前帧点云数据
		mtx_cloud.unlock();
		std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> outputclouds;
		std::vector<pcl::PointXYZI> lidarpropertys;
		analysisCloud(tempcloud, outputclouds, lidarpropertys);
		//		cloud_viewer_->removeAllPointClouds();
		//		char cloud_name[50];
		//		memset( cloud_name, 0 , 50);
		//		sprintf( cloud_name, "passablecloud");
		//		if (tempcloud->size() > 0 )
		//		{
		//
		//			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> cloudHandler( tempcloud, 0, 255, 0 );
		//			if (!cloud_viewer_->updatePointCloud(tempcloud,cloudHandler, cloud_name))
		//			{
		//				//				std::cout<<"cloudgeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeet"<<std::endl;
		//				cloud_viewer_->addPointCloud(tempcloud, cloudHandler, cloud_name);
		//			}

		cv::Mat point_count_grid = cv::Mat::zeros(grid_height,grid_width, CV_8UC1);
		cv::Mat point_count_show = cv::Mat::zeros(grid_height,grid_width, CV_8UC3);
		cv::Mat maxz = cv::Mat::ones(grid_height,grid_width,  CV_32FC1) * (-11.11);
		float ogm_y_offset = 20.0f;
		for (int i = 0; i < tempcloud->points.size(); i++)
		{
			float x = tempcloud->points[i].x,
					y = tempcloud->points[i].y,
					z = tempcloud->points[i].z;
			float newy = y + ogm_y_offset;
			//			if(cloud->points[i].intensity < 15.0f)//排除噪声干扰，只考虑反射率较高的点
			//			                continue;
			if(x>-1.5&&x<1.5&&y>-1&&y<4){		//排除车身周围------------------------------------zx
				continue;
			}
			int col = boost::math::round(x / res) + ( grid_width - 1 ) / 2;
			int row = boost::math::round(newy / res) ;
			if((col >=0 && col < grid_width) &&
					(row >=0 && row < grid_height) &&
					( z <=  Z_MAX))
			{
				int index = row * grid_width + col;
				point_count_grid.at<unsigned char>(row,col) += 1;
				if(maxz.at<float>(row,col)<z){
					maxz.at<float>(row,col)=z;
				}
			}
		}
		for(int i = 0; i< grid_height; ++i){
			unsigned char* ptr = point_count_show.ptr(grid_height -i -1);
			for(int j = 0; j < grid_width; ++j){
				ptr[3*j + 1] = point_count_grid.at<unsigned char>(i, j)*15;
			}
		}
		for(int j=grid_width/2-4/res;j<grid_width/2+4/res;j++)//车体上下范围
		{

			for(int i=20/res;i<40/res;i++)
			{
				int count=0;
				//				if(j>grid_width/2-3/res&&j<grid_width/2+3/res
				//						&&i<grid_height/2+3/res&&i>grid_height/2-4/res)
				//					continue;
				if(maxz.at<float>(i, j) > 0.5) break;//前方有较高障碍物时break
				int index=i*grid_width+j;
				float bound=(j-20/res)*0.4*(j-20/res)*0.4+(i-20/res)*0.4
						*(i-20/res)*0.4;

				int i_begin = i ,j_begin = j;//存储最先发现无点区域的坐标
				while(point_count_grid.at<unsigned char>(i_begin,j_begin)<1&&count<50/res-i)//
				{
					//						cout<<"the point count is  "<<i<<"  "<<j<<endl;
					//						cout<<"the index is "<<index<<endl;
					//					bound=(j-20/res)*0.4*(j-20/res)*0.4+(i-20/res+count)*0.4
					//							*(i-20/res+count)*0.4;

					count++;
					i_begin += 1;
				}
				if(count>6)								//可调参数
				{
					//									if(pathClear(4*i,4*j))	//判断一下由雷达到无点区域中点之间有无障碍  pathClear(j,i+count/2)
					double maxz1 = -11.11, maxz2 = -11.11;
					for(int k = i; k>0; k--){
						int zindex = k * grid_width + j;
						if( abs(maxz.at<float>(k, j) + 11.11)>0.01){
							maxz1 = maxz.at<float>(k, j);
							break;
						}
					}
					for(int k = i + count; k < i + count +20; k++){
						int zindex = k * grid_width + j;
						if( abs(maxz.at<float>(k, j) + 11.11)>0.01){
							maxz2 = maxz.at<float>(k, j);
							break;
						}
					}

					bool state = false;
					if(abs(maxz1 + 11.11) > 0.001 && abs(maxz2 + 11.11) > 0.001&& maxz1 - maxz2 > 1) state  = true ;
					//					cout<<"=============="<<maxz1<<"  "<<maxz2<<endl;
					if(state)//&& pathClear(2*i,2*j)&&pathClear((2*i+2),2*j)&&pathClear((2*i+count),2*j)
					{
						if(i < 24 / res) continue; //排除车体中间那块盲区
						for(int k = 0; k < count; k++){
							if(2*(i+k)+1 >=GRIDWH) continue;
							gridall.at<unsigned char>(GRIDWH - 1 - 2*(i+k), 2*j + 75) = 255;
							gridall.at<unsigned char>(GRIDWH - 1 - 2*(i+k), 2*j+1 + 75) = 255;
							gridall.at<unsigned char>(GRIDWH - 1 - 2*(i+k) - 1, 2*j + 75) = 255;
							gridall.at<unsigned char>(GRIDWH - 1 - 2*(i+k) - 1, 2*j + 1 + 75) = 255;
							unsigned char* ptr = point_count_show.ptr(grid_height - (i + k) - 1);
							ptr[3 * j] = 255;
							ptr[3 * j + 1] = 255;
							ptr[3 * j + 2] = 255;
						}

					}
				}
				if(count > 2){
					double maxz1 = -11.11, maxz2 = -11.11;
					for(int k = i; k>0; k--){
						int zindex = k * grid_width + j;
						if( abs(maxz.at<float>(k, j) + 11.11)>0.01){
							maxz1 = maxz.at<float>(k, j);
							break;
						}
					}
					for(int k = i + count; k < i + count +20; k++){
						int zindex = k * grid_width + j;
						if( abs(maxz.at<float>(k, j) + 11.11)>0.01){
							maxz2 = maxz.at<float>(k, j);
							break;
						}
					}

					bool state = false;
					if(abs(maxz1 + 11.11) > 0.001 && abs(maxz2 + 11.11) > 0.001&&abs( maxz1 - maxz2) < 0.5) state  = true ;
					//					cout<<"=============="<<maxz1<<"  "<<maxz2<<endl;
					if(0)//&& pathClear(2*i,2*j)&&pathClear((2*i+2),2*j)&&pathClear((2*i+count),2*j)
					{
						if(i < 24 / res) continue; //排除车体中间那块盲区
						for(int k = 0; k < count; k++){
							unsigned char* ptr = point_count_show.ptr(grid_height - (i + k) - 1);
							ptr[3 * j] = 125;
							ptr[3 * j + 1] = 125;
							ptr[3 * j + 2] = 125;
						}

					}
				}
				if(count>0) i+=count-1;
			}
		}
		cv::line(point_count_show,cvPoint(0,175 - 25 / 0.4),cvPoint(38/0.4, 175 - 25 / 0.4), cvScalar(255,255,255));
		if(visual_on){
			cv::imshow("another", point_count_show);
		}

	}
	if(visual_on){
#ifdef TESTSHOW
		cv::namedWindow("grid",CV_WINDOW_NORMAL);
		//	cv::namedWindow("grid1",CV_WINDOW_NORMAL);
#endif
		cv::namedWindow("gridall",CV_WINDOW_NORMAL);
		cv::namedWindow("show",CV_WINDOW_NORMAL);
	}
	cv::Mat heightmat_0 = cv::Mat::zeros(32,870,CV_32F);
	cv::Mat dismat_0 = cv::Mat::zeros(32,870,CV_32F);
	for(int i=0;i<32;++i){
		for(int j =0;j<870;++j){
			if(abs(height_msg->data[i*870+j+32*870]+100)>0.0001){//原始值为-100赋值为0
				dismat_0.at<float>(i,j)=height_msg->data[i*870+j+32*870];
			}
		}
	}

	cv::Mat gridshow_0 = cv::Mat::zeros(GRIDWH,GRIDWH,CV_8UC3);
	cv::Mat justshow = cv::Mat::zeros(GRIDWH,GRIDWH,CV_8UC3);
	//最大高度、最小高度、高度差
	cv::Mat heightgrid_0 = cv::Mat::zeros(GRIDWH,GRIDWH,CV_32FC1);
	cv::Mat heightgridmin_0 = cv::Mat::ones(GRIDWH,GRIDWH,CV_32FC1)*100;
	cv::Mat heightgridmax_0 = cv::Mat::ones(GRIDWH,GRIDWH,CV_32FC1)*(-100);

	for(int j=0;j<870;j++){

		for(int i=31;i>6;i--){
			int index=i*870+j;
			double dis=(double)dismat_0.at<float>(i,j);
			double radius=dis*std::cos(pitchrad_6t[31-i]*PI/180);

			double originx=radius*costable[j];
			double originy=radius*sintable[j];
			double originz=dis*std::sin(pitchrad_6t[31-i]*PI/180);

			Eigen::Matrix<double, 3, 1> originp;

			originp<<originx,originy,originz;

			double x=(T*originp)[0];
			double y=(T*originp)[1];
			double z=(T*originp)[2];
			if(x>-1.5&&x<1.5&&y<4&&y>-1){;}//
			else{
				heightmat_0.at<float>(i,j)=z;
				int col=boost::math::round((x+35)/0.2);//干！！！！！！！！！！！
				int row=boost::math::round((y+20)/0.2);
				int gridindex=0;
				unsigned char* ptr=gridshow_0.ptr<unsigned char>(GRIDWH-1-row);
				unsigned char* ptrshow=justshow.ptr<unsigned char>(GRIDWH-1-row);
				float* ptrheight=heightgrid_0.ptr<float>(row);
				float* ptrmin=heightgridmin_0.ptr<float>(row);
				float* ptrmax=heightgridmax_0.ptr<float>(row);
				if(col<GRIDWH&&row<GRIDWH&&col>=0&&row>=0){
					ptr[3*col]=0;
					ptr[3*col+1]=255;
					ptr[3*col+2]=0;
					ptrshow[3*col+1]=255;
					if(i == 28){
						ptrshow[3*col]=255;
						ptrshow[3*col+2]=255;
					}
					if(ptrmin[col]>z){
						ptrmin[col]=z;
					}
					if(ptrmax[col]<z){
						ptrmax[col]=z;
					}
					ptrheight[col]=ptrmax[col]-ptrmin[col];

				}
			}
		}
	}


	int rightend=(int)38/0.2;
	int leftend=(int)32/0.2;
	//
	//	//以下对两个雷达进行检测//////////////////////////////////////////////////////////////////////////////
	//
	//
	//1、0雷达，为左侧 ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef left
	//	cout<<"==================================="<<endl;
	for(int j=0;j<870;j++){
		int myj;
		int countlaser=0;
		for(int i=31;i>6;i--){
			//			if(j==800){
			//				cout<<"height is "<<heightmat_0.at<float>(i,j)<<endl;
			//			}
			int index=i*870+j;
			int indexnext=(i-1)*870+j;
			double disbefore=(double)dismat_0.at<float>(i+1,j);
			double dis=(double)dismat_0.at<float>(i,j);
			if(abs(dis-disbefore)<1)continue;
			double radius=dis*std::cos(pitchrad_6t[31-i]*PI/180);
			double originx=radius*costable[j];
			double originy=radius*sintable[j];
			double originz=dis*std::sin(pitchrad_6t[31-i]*PI/180);

			//计算起始点及前一点高度
			double height=heightmat_0.at<float>(i,j);
			double heightbefore=heightmat_0.at<float>(i+1,j);

			//当前线雷达与上一线雷达距离差
			double diff2=(double)dismat_0.at<float>(i,j)-(double)dismat_0.at<float>(i+1,j);
			double disnext=(double)dismat_0.at<float>(i-1,j);
			int count=0;
			while(disnext==0&&i>15){
				count++;
				i--;
				disnext=(double)dismat_0.at<float>(i-1,j);
			}
			//计算下一点及下下一点高度
			double heightnext=heightmat_0.at<float>(i-1,j);
			double heightnene=heightmat_0.at<float>(i-2,j);

			//下一线与当前线雷达距离差
			double diff1=disnext-dis;
			//到达0°后break
			if(i==6) break;

			double radiusnext=disnext*std::cos(pitchrad_6t[31-i+1]*PI/180);
			double originxnext=radiusnext*costable[j];
			double originynext=radiusnext*sintable[j];
			double originznext=dis*std::sin(pitchrad_6t[31-i+1]*PI/180);


			Eigen::Matrix<double, 3, 1> originp;
			Eigen::Matrix<double, 3, 1> originpnext;
			Eigen::Matrix<double, 3, 1> originpvirtual;

			originp<<originx,originy,originz;
			originpnext<<originxnext,originynext,originznext;

			double x=(T*originp)[0];
			double y=(T*originp)[1];
			double z=(T*originp)[2];

			double xnext=(T*originpnext)[0];
			double ynext=(T*originpnext)[1];
			double znext=(T*originpnext)[2];
			//
			if(x>-1.5&&x<1.5&&y<4&&y>-1||xnext>-1.5&&xnext<1.5&&ynext<4&&ynext>-1) {
				continue;
			}
			//用于判断雷达点是否走出车身，可以正式开始算法
			countlaser++;


			double radiusvirtual=4.0;//虚拟半径，用于盲区检测
			double originxvirtual=radiusvirtual*costable[j];
			double originyvirtual=radiusvirtual*sintable[j];
			double originzvirtual=0;
			originpvirtual<<originxvirtual,originyvirtual,originzvirtual;

			double xvirtual=(T*originpvirtual)[0];
			double yvirtual=(T*originpvirtual)[1];



			int col=boost::math::round((x+35)/0.2);
			int row=boost::math::round((y+20)/0.2);
			int colvirtual;
			int rowvirtual;
			if((i == 31 || i == 30) && (x < -4 || x > 4)){
				colvirtual=boost::math::round((xvirtual+35)/0.2);
				rowvirtual=boost::math::round((yvirtual+20)/0.2);
				if(height < -1){// &&(colvirtual<32/0.2||colvirtual>38/0.2)
					cout<<"======================== "<<i<<endl;
					cv::line(gridshow_0, cvPoint(colvirtual, GRIDWH - 1 -rowvirtual), cvPoint(col, GRIDWH - 1 - row),
							cvScalar(0, 0, 125));
					cv::line(justshow, cvPoint(colvirtual, GRIDWH - 1 -rowvirtual), cvPoint(col, GRIDWH - 1 - row),
							cvScalar(0, 0, 125));
				}
				//				第一条线自己给定初始diff2
				//				diff2=0.3;
			}

			int colnext=boost::math::round((xnext+35)/0.2);
			int rownext=boost::math::round((ynext+20)/0.2);

			int gridindex=0;
			unsigned char* ptr=gridshow_0.ptr<unsigned char>(GRIDWH-1-row);
			if(col<GRIDWH&&row<GRIDWH&&col>=0&&row>=0){


				//				增加不同半径下的不同高度阈值
				if(radius<8){if(height>0.3)break;}
				else if(radius<13){if(height>0.5)break;}
				else{if(height>0.8)break;}

				if(j==800){
					ptr[3*col]=255;
					ptr[3*col+1]=255;
					ptr[3*col+2]=255;
				}
				//				针对远距离误检不同阈值
				if(1){
					//短距离用高度差和距离突变来定义
					if(0){//radius<10
						int thresh=THRESH;
						if(col>155&&col<195) thresh=2*THRESH;
						if(heightnext-height<-0.5&&(disnext-dis)/(dis-disbefore)>thresh){//diff1/diff2
							//					if((height-heightnext)/(radiusnext-radius)>0.1&&diff1/diff2>3){
							ptr[3*col]=0;
							ptr[3*col+1]=0;
							ptr[3*col+2]=255;
							//							cout<<radius<<"==================>"<<diff1/diff2<<endl;
							//排除径向距离比较小的
							if((colnext-col)*(colnext-col)*0.04+(rownext-row)*(rownext-row)*0.04<1){
#ifdef showgreen
								cv::line(gridshow_0,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext),cvScalar(0,150,0));
#endif
								continue;
							}
							//连线
							//							if(colnext<GRIDWH&&rownext<GRIDWH&&colnext>=0&&rownext>=0){


							cv::LineIterator it(heightgridmax_0,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext));
							//								cout<<"the heights are......"<<endl;
							double hm=-100;
							for(int k=0;k<it.count;++k,it++){
								int x=it.pos().x;
								int y=it.pos().y;
								y=GRIDWH-1-y;
								for(int i=-2;i<3;++i){
									for(int j=-2;j<3;++j){
										double height=0;
										if(x+i>=0&&x+i<GRIDWH&&y+j>=0&&y+j<GRIDWH){
											float* ptr=heightgridmax_0.ptr<float>(y+j);
											height=(double)ptr[x+i];
										}
										if(height>hm) hm=height;
									}
								}
							}
							//								cout<<hm<<endl;
							if(hm<0.4&&!(y < 0 && x > -4 && x < 4)){//temp[temp.size()-1]<1&&temp[temp.size()-1]!=-100
								//								cout<<"the height is "<<temp[temp.size()-1]<<endl;
								//								cout<<"dis is "<<disbefore<<" "<<dis<<" "<<disnext<<"  ratio is "<<diff1/diff2<<
								//										" height is "<<height<<" "<<heightnext<<endl;
								cv::line(justshow,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext),cvScalar(0,0,125));
								cv::line(gridshow_0,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext),cvScalar(0,0,125));
							}
							//							}

						}
						else{
#ifdef showgreen
							cv::line(gridshow_0,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext),cvScalar(0,150,0));
#endif
						}
					}
					//大于10m小于17m需要加上颠簸程度、俯仰角、（起始点高度？--这个还不太确定）、正切值、最小高度、距离突变比
					//else
					if(radiusnext<100 && radiusnext > 20 && radius < 22){
						int thresh=8;
						if(col>155&&col<195) thresh=10;
						if(1){//&&height>-0.4 &&yprdiff<7 tmpmsg->pitch<5
							//					if((heightmat_0.at<float>(i-1,j)-heightmat_0.at<float>(i,j))<-1&&diff1/diff2>3){
							if((height-heightnext)/(radiusnext-radius)>0.07
									&&heightnext-height<-1&&(disnext-dis)/(dis-disbefore)>thresh){//
								ptr[3*col]=0;
								ptr[3*col+1]=0;
								ptr[3*col+2]=255;
								//排除径向距离比较小的
								if((colnext-col)*(colnext-col)*0.04+(rownext-row)*(rownext-row)*0.04<1){
#ifdef showgreen
									cv::line(gridshow_0,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext),cvScalar(0,150,0));
#endif
									continue;
								}
								//连线
								//								if(colnext<GRIDWH&&rownext<GRIDWH&&colnext>=0&&rownext>=0){


								cv::LineIterator it(heightgridmax_0,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext));
								//								cout<<"the heights are......"<<endl;
								double hm=-100;
								for(int k=0;k<it.count;++k,it++){
									int x=it.pos().x;
									int y=it.pos().y;
									y=GRIDWH-1-y;
									for(int i=-2;i<3;++i){
										for(int j=-2;j<3;++j){
											double height=0;
											if(x+i>=0&&x+i<GRIDWH&&y+j>=0&&y+j<GRIDWH){
												float* ptr=heightgridmax_0.ptr<float>(y+j);
												height=(double)ptr[x+i];
											}
											if(height>hm) hm=height;
										}
									}
								}
								//								cout<<hm<<endl;
								if(hm<0.4&&!(y < 0 && x > -4 && x < 4)){//temp[temp.size()-1]<1&&temp[temp.size()-1]!=-100
									//									cout<<"dis is "<<disbefore<<" "<<dis<<" "<<disnext<<"  ratio is "<<diff1/diff2<<
									//											" height is "<<height<<" "<<heightnext<<endl;
									cv::line(justshow,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext),cvScalar(0,0,125));
									cv::line(gridshow_0,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext),cvScalar(0,0,125));
									break;
								}
								//								}

							}
							else{
#ifdef showgreen
								cv::line(gridshow_0,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext),cvScalar(0,150,0));
#endif
							}
						}
					}
				}
			}
			//把水的可疑区域画出来
			cv::LineIterator it(heightgridmax_0,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext));
			//								cout<<"the heights are......"<<endl;
			if(count>2){
				double hm=-100;
				for(int k=0;k<it.count;++k,it++){
					int x=it.pos().x;
					int y=it.pos().y;
					y=GRIDWH-1-y;
					for(int i=-2;i<3;++i){
						for(int j=-2;j<3;++j){
							double height=0;
							if(x+i>=0&&x+i<GRIDWH&&y+j>=0&&y+j<GRIDWH){
								float* ptr=heightgridmax_0.ptr<float>(y+j);
								height=(double)ptr[x+i];
							}
							if(height>hm) hm=height;
						}
					}
				}
				if(hm<0.4&&colnext>0&&colnext<GRIDWH
						&&rownext>0&&rownext<GRIDWH&&disnext<30){//&&i>20
					//					cv::line(gridshow_0,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext),cvScalar(125,125,125));
					//					cv::line(justshow,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext),cvScalar(125,125,125));

				}
			}
			////把水的可疑区域画出来
		}
	}
#endif


	stiff_msgs::stiffwater msg_send;
	msg_send.header.stamp=height_msg->header.stamp;
	msg_send.header.frame_id="stiffwater";
	msg_send.ogmheight=351;
	msg_send.ogmwidth=201;
	msg_send.resolution=0.2;
	msg_send.vehicle_x=100;
	msg_send.vehicle_y=100;
	for(int row=0;row<GRIDWH;row++){
		unsigned char* ptrall=gridall.ptr<unsigned char>(GRIDWH-1-row);
		unsigned char* ptr=gridshow_0.ptr<unsigned char>(GRIDWH-1-row);
		for(int col=0;col<GRIDWH;col++){
			if(ptr[3*col+2]==125){
				ptrall[col]=255;
			}
			else if(ptr[3*col+2]==125&&ptr[3*col+1]==125&&ptr[3*col]==125){//水区域
				ptrall[col]=200;
			}
			else if(ptr[3*col+1]==150){
				ptrall[col]=100;
			}
		}
	}


	cv::dilate(gridall,gridall,elementdil);
	cv::erode(gridall,gridall,elementero);
	//	cv::dilate(gridall,gridall,elementero2);

	for(int row=0;row<351;row++){
		unsigned char* ptr=gridall.ptr<unsigned char>(GRIDWH-1-row);
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
	pubStiffwaterOgm.publish(msg_send);


	for(int i=4;i<21;i+=2){
		if(i==8||i==14){
			cv::circle(gridshow_0,cvPoint(175,250),(int)(i/0.2),cvScalar(80,0,80));
			cv::circle(justshow,cvPoint(175,250),(int)(i/0.2),cvScalar(80,0,80));
			cv::circle(gridall,cvPoint(175,250),(int)(i/0.2),cvScalar(80,0,80));
		}
		else{
			cv::circle(gridshow_0,cvPoint(175,250),(int)(i/0.2),cvScalar(80,80,0));
			cv::circle(justshow,cvPoint(175,250),(int)(i/0.2),cvScalar(80,80,0));
			cv::circle(gridall,cvPoint(175,250),(int)(i/0.2),cvScalar(80,0,80));
		}
	}
	//给justshow画角度线
	double unitangle=2.0/870*PI;
	for(int i=0;i<8;i++){
		double angle=-(unitangle*i*100+PI/2)+PI;
		int x=175+1000*std::cos(angle);
		int y=-100+GRIDWH-1-1000*std::sin(angle);
		cv::line(justshow,cvPoint(175,250),cvPoint(x,y),cvScalar(0,165,135));
	}
	cv::namedWindow("show",CV_WINDOW_NORMAL);
	//	int row=350-100-16;
	//		cv::line(gridshow_1,cvPoint(0,row),cvPoint(350,row),cvScalar(0,150,0));
	//		cv::line(gridshow_0,cvPoint(0,row),cvPoint(350,row),cvScalar(0,150,0));
	cv::line(gridshow_0,cvPoint(leftend,0),cvPoint(leftend,GRIDWH-1),cvScalar(0,126,0));
	cv::line(gridshow_0,cvPoint(rightend,0),cvPoint(rightend,GRIDWH-1),cvScalar(0,126,0));
	cv::line(justshow,cvPoint(leftend,0),cvPoint(leftend,GRIDWH-1),cvScalar(0,126,0));
	cv::line(justshow,cvPoint(rightend,0),cvPoint(rightend,GRIDWH-1),cvScalar(0,126,0));
	if(visual_on){
#ifdef TESTSHOW
		cv::imshow("grid",gridshow_0);
#endif
		cv::imshow("show",justshow);
		cv::imshow("gridall",gridall);
		//	cv::imshow("range", dismat_0);
		//	cv::imshow("height", heightmat_0);
		cv::waitKey(10);
	}
}

void gpsdatacllbak(const sensor_driver_msgs::GpswithHeadingConstPtr& msg){
	//			cout<<"gpsdata stamp is "<<msg->gps.header.stamp<<endl;
	//		cout<<yprmin<<endl;
	qgwithhmsgs_.Push(msg);
	//	qypr.push_back(msg);
	if(qgwithhmsgs_.Size()>10){
		qgwithhmsgs_.Pop();
	}

}
#endif
#ifdef BEIQI
#define THRESH 3
#define left
void callback(const depth_image_utils::HeightMapConstPtr& height_msg){
	//		cout<<"height map timestamp is "<<height_msg->header.stamp<<endl;
	//	sensor_driver_msgs::GpswithHeadingConstPtr tmpmsg=qgwithhmsgs_.PopWithTimeout(common::FromSeconds(0.1));
	//	if(tmpmsg==nullptr) return;
	//	double gpsstamp=tmpmsg->gps.header.stamp.toSec();
	//	double lidarstamp=height_msg->header.stamp.toSec();
	//	if(gpsstamp>lidarstamp){
	//		cout<<"waite for height map"<<endl;
	//		qgwithhmsgs_.Push_Front(std::move(tmpmsg));
	//		return;
	//	}
	//	while(lidarstamp-gpsstamp>0.00001){
	//		if(qgwithhmsgs_.Size()==0){
	//			return;//这里可以增加队列这样不用浪费一帧信息
	//		}
	//		tmpmsg=qgwithhmsgs_.Pop();
	//		gpsstamp=tmpmsg->gps.header.stamp.toSec();
	//	}
	//	double yprdiff=(tmpmsg->pitch-yprmin[1])*(tmpmsg->pitch-yprmin[1])+(tmpmsg->roll-yprmin[2])*(tmpmsg->roll-yprmin[2]);
	//	double yprdiff=(tmpmsg->pitch-yprlast[1])*(tmpmsg->pitch-yprlast[1])+(tmpmsg->roll-yprlast[2])*(tmpmsg->roll-yprlast[2]);
	//	if(0){
	//		//		cout<<"height"<<height_msg->header.stamp<<endl;
	//		//		cout<<"gpsdata"<<tmpmsg->gps.header.stamp<<endl;
	//		cout<<"the ypr  is"<<endl;
	//		cout<<yprdiff<<endl;
	//		//		cout<<yprmin<<endl;
	//		//		cout<<tmpmsg->heading<<endl;
	//		cout<<tmpmsg->pitch<<endl;
	//		cout<<tmpmsg->roll<<endl;
	//	}
	//	yprmin<<100,100,100;
	//	cv::namedWindow("range",CV_WINDOW_NORMAL);
	cv::Mat gridall = cv::Mat::zeros(GRIDWH,GRIDWH,CV_8UC1);
	if(visual_on)
		cv::namedWindow("another",CV_WINDOW_NORMAL);
	if(lidarCloudMsgs_ != nullptr){
		//		cout<<"got lidar"<<endl;
		mtx_cloud.lock();
		double timeLaserCloudFullRes = lidarCloudMsgs_->header.stamp.toSec();
		pcl::PointCloud<pcl::PointXYZI>::Ptr tempcloud(new pcl::PointCloud<pcl::PointXYZI>);//当前帧点云（雷达里程计坐标系）
		pcl::fromROSMsg(*lidarCloudMsgs_, *tempcloud);//获取当前帧点云数据
		mtx_cloud.unlock();
		std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> outputclouds;
		std::vector<pcl::PointXYZI> lidarpropertys;
		analysisCloud(tempcloud, outputclouds, lidarpropertys);
		//		cloud_viewer_->removeAllPointClouds();
		//		char cloud_name[50];
		//		memset( cloud_name, 0 , 50);
		//		sprintf( cloud_name, "passablecloud");
		//		if (tempcloud->size() > 0 )
		//		{
		//
		//			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> cloudHandler( tempcloud, 0, 255, 0 );
		//			if (!cloud_viewer_->updatePointCloud(tempcloud,cloudHandler, cloud_name))
		//			{
		//				//				std::cout<<"cloudgeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeet"<<std::endl;
		//				cloud_viewer_->addPointCloud(tempcloud, cloudHandler, cloud_name);
		//			}
		//		}
		//		cloud_viewer_->spinOnce();
		//		cv::Mat grid16 = cv::Mat::zeros(351,201, CV_8UC1);
		//		cv::Mat grid16_show = cv::Mat::zeros(351,201, CV_8UC3);
		//		cv::Mat maxz_16 = cv::Mat::ones(351,201, CV_32FC1) * -11.11;
		//		for(int i = 0; i < tempcloud->size();++i){
		//			float x = tempcloud->points[i].x,
		//					y = tempcloud->points[i].y,
		//					z = tempcloud->points[i].z;
		//
		//			//			Eigen::Vector3d originp(x,y,z);
		//			//			originp = T16_1 * originp;
		//			//			x = originp.x();
		//			//			y = originp.y();
		//			//			z = originp.z();
		//			float newy = y + 20;
		//			//			if(cloud->points[i].intensity < 15.0f)//排除噪声干扰，只考虑反射率较高的点
		//			//			                continue;
		//			if(x>-1.5&&x<1.5&&y>-1&&y<4){		//排除车身周围------------------------------------zx
		//				continue;
		//			}
		//			int col = boost::math::round(x / 0.2) + 100;
		//			int row = boost::math::round(newy / 0.2) ;
		//			if((col >=0 && col < 201) &&
		//					(row >=0 && row < 351) &&
		//					( z <=  Z_MAX))
		//			{
		//				grid16.at<unsigned char>(row, col)++;
		//				unsigned char* ptr = grid16_show.ptr(350 - row);
		//				ptr[3*col + 1] =  255;
		//				if(maxz_16.at<float>(row,col)<z){
		//					maxz_16.at<float>(row,col)=z;
		//				}
		//			}
		//		}
		//		for(int j=80; j<120;j++)//车体上下范围
		//		{
		//
		//			for(int i=115;i<175;i++)
		//			{
		//				int count=0;
		//				if(maxz_16.at<float>(i, j) > 0.5) break;//前方有较高障碍物时break
		//				int index=i*grid_width+j;
		//				float bound=(j-20/res)*0.4*(j-20/res)*0.4+(i-20/res)*0.4
		//						*(i-20/res)*0.4;
		//
		//				int i_begin = i ,j_begin = j;//存储最先发现无点区域的坐标
		//				while(grid16.at<unsigned char>(i_begin,j_begin)<1&&count<200-i)//
		//				{
		//					count++;
		//					i_begin += 1;
		//				}
		//				if(count>6)								//可调参数
		//				{
		//					//									if(pathClear(4*i,4*j))	//判断一下由雷达到无点区域中点之间有无障碍  pathClear(j,i+count/2)
		//					double maxz1 = -11.11, maxz2 = -11.11;
		//					for(int k = i; k>0; k--){
		//						int zindex = k * grid_width + j;
		//						if( abs(maxz_16.at<float>(k, j) + 11.11)>0.01){
		//							maxz1 = maxz_16.at<float>(k, j);
		//							break;
		//						}
		//					}
		//					for(int k = i + count; k < i + count +20; k++){
		//						int zindex = k * grid_width + j;
		//						if( abs(maxz_16.at<float>(k, j) + 11.11)>0.01){
		//							maxz2 = maxz_16.at<float>(k, j);
		//							break;
		//						}
		//					}
		//
		//					bool state = false;
		//					if(abs(maxz1 + 11.11) > 0.001 && abs(maxz2 + 11.11) > 0.001&& maxz1 - maxz2 < -1) state  = true ;
		//					//					cout<<"=============="<<maxz1<<"  "<<maxz2<<endl;
		//					if(state)//&& pathClear(2*i,2*j)&&pathClear((2*i+2),2*j)&&pathClear((2*i+count),2*j)
		//					{
		//						for(int k = 0; k < count; k++){
		//							//							if(2*(i+k)+1 >=GRIDWH) continue;
		//							//							gridall.at<unsigned char>(GRIDWH - 1 - 2*(i+k), 2*j + 75) = 255;
		//							//							gridall.at<unsigned char>(GRIDWH - 1 - 2*(i+k), 2*j+1 + 75) = 255;
		//							//							gridall.at<unsigned char>(GRIDWH - 1 - 2*(i+k) - 1, 2*j + 75) = 255;
		//							//							gridall.at<unsigned char>(GRIDWH - 1 - 2*(i+k) - 1, 2*j + 1 + 75) = 255;
		//							unsigned char* ptr = grid16_show.ptr(351 - (i + k) - 1);
		//							ptr[3 * j] = 125;
		//							ptr[3 * j + 1] = 125;
		//							ptr[3 * j + 2] = 125;
		//						}
		//
		//					}
		//				}
		//				if(count > 2){
		//					double maxz1 = -11.11, maxz2 = -11.11;
		//					for(int k = i; k>0; k--){
		//						int zindex = k * grid_width + j;
		//						if( abs(maxz_16.at<float>(k, j) + 11.11)>0.01){
		//							maxz1 = maxz_16.at<float>(k, j);
		//							break;
		//						}
		//					}
		//					for(int k = i + count; k < i + count +20; k++){
		//						int zindex = k * grid_width + j;
		//						if( abs(maxz_16.at<float>(k, j) + 11.11)>0.01){
		//							maxz2 = maxz_16.at<float>(k, j);
		//							break;
		//						}
		//					}
		//
		//					bool state = false;
		//					if(abs(maxz1 + 11.11) > 0.001 && abs(maxz2 + 11.11) > 0.001&&abs( maxz1 - maxz2 ) < 0.5) state  = true ;
		//					//					cout<<"=============="<<maxz1<<"  "<<maxz2<<endl;
		//					if(state)//&& pathClear(2*i,2*j)&&pathClear((2*i+2),2*j)&&pathClear((2*i+count),2*j)
		//					{
		//						for(int k = 0; k < count; k++){
		//							//							if(2*(i+k)+1 >=GRIDWH) continue;
		//							//							gridall.at<unsigned char>(GRIDWH - 1 - 2*(i+k), 2*j + 75) = 255;
		//							//							gridall.at<unsigned char>(GRIDWH - 1 - 2*(i+k), 2*j+1 + 75) = 255;
		//							//							gridall.at<unsigned char>(GRIDWH - 1 - 2*(i+k) - 1, 2*j + 75) = 255;
		//							//							gridall.at<unsigned char>(GRIDWH - 1 - 2*(i+k) - 1, 2*j + 1 + 75) = 255;
		//							unsigned char* ptr = grid16_show.ptr(351 - (i + k) - 1);
		//							ptr[3 * j] = 125;
		//							ptr[3 * j + 1] = 125;
		//							ptr[3 * j + 2] = 125;
		//						}
		//
		//					}
		//				}
		//				if(count>0) i+=count-1;
		//			}
		//		}

		//		int col_cnts = outputclouds[1]->size() / 16 ;
		//		for(int i = 0; i < 16; ++i){
		//			for(int j = 0; j < col_cnts; ++j){
		//				int index = j * 16 + i ;
		//				float x = outputclouds[1]->points[index].x,
		//						y = outputclouds[1]->points[index].y,
		//						z = outputclouds[1]->points[index].z;
		//				float newy = y + 20 ;
		//				//			if(cloud->points[i].intensity < 15.0f)//排除噪声干扰，只考虑反射率较高的点
		//				//			                continue;
		//				if(x>-1.5&&x<1.5&&y>-1&&y<4){		//排除车身周围------------------------------------zx
		//					continue;
		//				}
		//				int col = boost::math::round(x / 0.2) + 100;
		//				int row = boost::math::round(newy / 0.2) ;
		//				if((col >=0 && col < 201) &&
		//						(row >=0 && row < 351) &&
		//						( z <=  Z_MAX))
		//				{
		//					unsigned char* ptr = grid16.ptr(350 - row);
		//					ptr[3*col + 1] =  255;
		//					//				if(maxz.at<float>(row,col)<z){
		//					//					maxz.at<float>(row,col)=z;
		//					//				}
		//				}
		//
		//				int count = 0;
		//				while(outputclouds[1]->points[index].range < 0 && count < col_cnts - j){
		//					count++;
		//					index += 16;
		//					if(index % 16 != i) break;
		//				}
		//				if(count > 0){
		//					if(count < 50 && count > 5){
		//						int index_begin = j * 16 + i -16;
		//						float x0 = outputclouds[1]->points[index_begin].x;
		//						float y0 = outputclouds[1]->points[index_begin].y;
		//						float z0 = outputclouds[1]->points[index_begin].z;
		//						float x1 = outputclouds[1]->points[index].x;
		//						float y1 = outputclouds[1]->points[index].y;
		//						float z1 = outputclouds[1]->points[index].z;
		//						while(z1 < -1){
		//							index += 16;
		//							x1 = outputclouds[1]->points[index].x;
		//							y1 = outputclouds[1]->points[index].y;
		//							z1 = outputclouds[1]->points[index].z;
		//						}
		//						if(index % 16 != i){
		//							j += count-1;
		//							continue;
		//						}
		//						int col0 = boost::math::round(x0 / 0.2) + 100;
		//						int row0 = boost::math::round((y0+20) / 0.2) ;
		//						int col1 = boost::math::round(x1 / 0.2) + 100;
		//						int row1 = boost::math::round((y1+20) / 0.2) ;
		//						if((col0 >=0 && col0 < 201) &&
		//								(row0 >=0 && row0 < 351) &&
		//								(col1 >=0 && col1 < 201) &&
		//								(row1 >=0 && row1 < 351))
		//						{
		//							cout<<"the zs are "<<z0<<"  "<<z1<<endl;
		//							cv::line(grid16,cvPoint(col0, 350 - row0), cvPoint(col1, 350 - row1),cvScalar(255,255,255));
		//						}
		//					}
		//					j += count - 1 ;
		//				}
		//			}
		//		}
		//		cv::imshow("grid16",grid16_show);
		//		cv::waitKey(3);

		cv::Mat point_count_grid = cv::Mat::zeros(grid_height,grid_width, CV_8UC1);
		cv::Mat point_count_show = cv::Mat::zeros(grid_height,grid_width, CV_8UC3);
		cv::Mat maxz = cv::Mat::ones(grid_height,grid_width,  CV_32FC1) * (-11.11);
		float ogm_y_offset = 20.0f;
		for (int i = 0; i < tempcloud->points.size(); i++)
		{
			float x = tempcloud->points[i].x,
					y = tempcloud->points[i].y,
					z = tempcloud->points[i].z;
			float newy = y + ogm_y_offset;
			//			if(cloud->points[i].intensity < 15.0f)//排除噪声干扰，只考虑反射率较高的点
			//			                continue;
			if(x>-1.5&&x<1.5&&y>-1&&y<4){		//排除车身周围------------------------------------zx
				continue;
			}
			int col = boost::math::round(x / res) + ( grid_width - 1 ) / 2;
			int row = boost::math::round(newy / res) ;
			if((col >=0 && col < grid_width) &&
					(row >=0 && row < grid_height) &&
					( z <=  Z_MAX))
			{
				int index = row * grid_width + col;
				point_count_grid.at<unsigned char>(row,col) += 1;
				if(maxz.at<float>(row,col)<z){
					maxz.at<float>(row,col)=z;
				}
			}
		}
		for(int i = 0; i< grid_height; ++i){
			unsigned char* ptr = point_count_show.ptr(grid_height -i -1);
			for(int j = 0; j < grid_width; ++j){
				ptr[3*j + 1] = point_count_grid.at<unsigned char>(i, j)*15;
			}
		}
		for(int j=grid_width/2-4/res;j<grid_width/2+4/res;j++)//车体上下范围
		{

			for(int i=20/res;i<40/res;i++)
			{
				int count=0;
				//				if(j>grid_width/2-3/res&&j<grid_width/2+3/res
				//						&&i<grid_height/2+3/res&&i>grid_height/2-4/res)
				//					continue;
				if(maxz.at<float>(i, j) > 0.5) break;//前方有较高障碍物时break
				int index=i*grid_width+j;
				float bound=(j-20/res)*0.4*(j-20/res)*0.4+(i-20/res)*0.4
						*(i-20/res)*0.4;

				int i_begin = i ,j_begin = j;//存储最先发现无点区域的坐标
				while(point_count_grid.at<unsigned char>(i_begin,j_begin)<1&&count<50/res-i)//
				{
					//						cout<<"the point count is  "<<i<<"  "<<j<<endl;
					//						cout<<"the index is "<<index<<endl;
					//					bound=(j-20/res)*0.4*(j-20/res)*0.4+(i-20/res+count)*0.4
					//							*(i-20/res+count)*0.4;

					count++;
					i_begin += 1;
				}
				if(count>6)								//可调参数
				{
					//									if(pathClear(4*i,4*j))	//判断一下由雷达到无点区域中点之间有无障碍  pathClear(j,i+count/2)
					double maxz1 = -11.11, maxz2 = -11.11;
					for(int k = i; k>0; k--){
						int zindex = k * grid_width + j;
						if( abs(maxz.at<float>(k, j) + 11.11)>0.01){
							maxz1 = maxz.at<float>(k, j);
							break;
						}
					}
					for(int k = i + count; k < i + count +20; k++){
						int zindex = k * grid_width + j;
						if( abs(maxz.at<float>(k, j) + 11.11)>0.01){
							maxz2 = maxz.at<float>(k, j);
							break;
						}
					}

					bool state = false;
					if(abs(maxz1 + 11.11) > 0.001 && abs(maxz2 + 11.11) > 0.001&& maxz1 - maxz2 > 1) state  = true ;
					//					cout<<"=============="<<maxz1<<"  "<<maxz2<<endl;
					if(state)//&& pathClear(2*i,2*j)&&pathClear((2*i+2),2*j)&&pathClear((2*i+count),2*j)
					{
						if(i < 24 / res) continue; //排除车体中间那块盲区
						for(int k = 0; k < count; k++){
							if(2*(i+k)+1 >=GRIDWH) continue;
							gridall.at<unsigned char>(GRIDWH - 1 - 2*(i+k), 2*j + 75) = 255;
							gridall.at<unsigned char>(GRIDWH - 1 - 2*(i+k), 2*j+1 + 75) = 255;
							gridall.at<unsigned char>(GRIDWH - 1 - 2*(i+k) - 1, 2*j + 75) = 255;
							gridall.at<unsigned char>(GRIDWH - 1 - 2*(i+k) - 1, 2*j + 1 + 75) = 255;
							unsigned char* ptr = point_count_show.ptr(grid_height - (i + k) - 1);
							ptr[3 * j] = 255;
							ptr[3 * j + 1] = 255;
							ptr[3 * j + 2] = 255;
						}

					}
				}
				if(count > 3 && i < 35 / res && i > 25 / res
						&& j > 17 / res && j < 23 / res){
					double maxz1 = -11.11, maxz2 = -11.11;
					for(int k = i; k>0; k--){
						int zindex = k * grid_width + j;
						if( abs(maxz.at<float>(k, j) + 11.11)>0.01){
							maxz1 = maxz.at<float>(k, j);
							break;
						}
					}
					for(int k = i + count; k < i + count +20; k++){
						int zindex = k * grid_width + j;
						if( abs(maxz.at<float>(k, j) + 11.11)>0.01){
							maxz2 = maxz.at<float>(k, j);
							break;
						}
					}

					bool state = false;
					if(abs(maxz1 + 11.11) > 0.001 && abs(maxz2 + 11.11) > 0.001&&abs( maxz1 - maxz2) < 0.5) state  = true ;
					//					cout<<"=============="<<maxz1<<"  "<<maxz2<<endl;
					if(state)//&& pathClear(2*i,2*j)&&pathClear((2*i+2),2*j)&&pathClear((2*i+count),2*j)
					{
						if(i < 24 / res) continue; //排除车体中间那块盲区
						for(int k = 0; k < count; k++){
							if(2*(i+k)+1 >=GRIDWH) continue;
							//							gridall.at<unsigned char>(GRIDWH - 1 - 2*(i+k), 2*j + 75) = 255;
							//							gridall.at<unsigned char>(GRIDWH - 1 - 2*(i+k), 2*j+1 + 75) = 255;
							//							gridall.at<unsigned char>(GRIDWH - 1 - 2*(i+k) - 1, 2*j + 75) = 255;
							//							gridall.at<unsigned char>(GRIDWH - 1 - 2*(i+k) - 1, 2*j + 1 + 75) = 255;
							unsigned char* ptr = point_count_show.ptr(grid_height - (i + k) - 1);
							ptr[3 * j] = 125;
							ptr[3 * j + 1] = 125;
							ptr[3 * j + 2] = 125;
						}

					}
				}
				if(count>0) i+=count-1;
			}
		}

		if(visual_on){
			cv::imshow("another", point_count_show);
		}

	}
	if(visual_on){
#ifdef TESTSHOW
		cv::namedWindow("grid",CV_WINDOW_NORMAL);
		//	cv::namedWindow("grid1",CV_WINDOW_NORMAL);
#endif
		cv::namedWindow("gridall",CV_WINDOW_NORMAL);
		cv::namedWindow("show",CV_WINDOW_NORMAL);
	}
	cv::Mat heightmat_0 = cv::Mat::zeros(32,870,CV_32F);
	cv::Mat dismat_0 = cv::Mat::zeros(32,870,CV_32F);
	for(int i=0;i<32;++i){
		for(int j =0;j<870;++j){
			if(abs(height_msg->data[i*870+j+32*870]+100)>0.0001){//原始值为-100赋值为0
				dismat_0.at<float>(i,j)=height_msg->data[i*870+j+32*870];
			}
		}
	}

	cv::Mat gridshow_0 = cv::Mat::zeros(GRIDWH,GRIDWH,CV_8UC3);
	cv::Mat justshow = cv::Mat::zeros(GRIDWH,GRIDWH,CV_8UC3);
	//最大高度、最小高度、高度差
	cv::Mat heightgrid_0 = cv::Mat::zeros(GRIDWH,GRIDWH,CV_32FC1);
	cv::Mat heightgridmin_0 = cv::Mat::ones(GRIDWH,GRIDWH,CV_32FC1)*100;
	cv::Mat heightgridmax_0 = cv::Mat::ones(GRIDWH,GRIDWH,CV_32FC1)*(-100);

	for(int j=0;j<870;j++){

		for(int i=31;i>6;i--){
			int index=i*870+j;
			double dis=(double)dismat_0.at<float>(i,j);
			double radius=dis*std::cos(pitchrad_beiqi[31-i]*PI/180);

			double originx=radius*costable[j];
			double originy=radius*sintable[j];
			double originz=dis*std::sin(pitchrad_beiqi[31-i]*PI/180);

			Eigen::Matrix<double, 3, 1> originp;

			originp<<originx,originy,originz;

			double x=(T*originp)[0];
			double y=(T*originp)[1];
			double z=(T*originp)[2];
			if(x>-1.5&&x<1.5&&y<4&&y>-1){;}//
			else{
				heightmat_0.at<float>(i,j)=z;
				int col=boost::math::round((x+35)/0.2);//干！！！！！！！！！！！
				int row=boost::math::round((y+20)/0.2);
				int gridindex=0;
				unsigned char* ptr=gridshow_0.ptr<unsigned char>(GRIDWH-1-row);
				unsigned char* ptrshow=justshow.ptr<unsigned char>(GRIDWH-1-row);
				float* ptrheight=heightgrid_0.ptr<float>(row);
				float* ptrmin=heightgridmin_0.ptr<float>(row);
				float* ptrmax=heightgridmax_0.ptr<float>(row);
				if(col<GRIDWH&&row<GRIDWH&&col>=0&&row>=0){
					ptr[3*col]=0;
					ptr[3*col+1]=255;
					ptr[3*col+2]=0;
					ptrshow[3*col+1]=255;
					if(i == 28){
						ptrshow[3*col]=255;
						ptrshow[3*col+2]=255;
					}
					if(ptrmin[col]>z){
						ptrmin[col]=z;
					}
					if(ptrmax[col]<z){
						ptrmax[col]=z;
					}
					ptrheight[col]=ptrmax[col]-ptrmin[col];

				}
			}
		}
	}

	//		cv::LineIterator it(gridshow_0,cvPoint(0,0),cvPoint(1000,10000));
	//		for(int i=0;i<it.count;i++,++it){
	//			(*it)[0]=255;
	//			(*it)[1]=0;
	//			(*it)[2]=0;
	//		}

	int rightend=(int)38/0.2;
	int leftend=(int)32/0.2;
	//
	//	//以下对两个雷达进行检测//////////////////////////////////////////////////////////////////////////////
	//
	//
	//1、0雷达，为左侧 ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef left
	//	cout<<"==================================="<<endl;
	for(int j=0;j<870;j++){
		int myj;
		int countlaser=0;
		for(int i=31;i>6;i--){
			//			if(j==800){
			//				cout<<"height is "<<heightmat_0.at<float>(i,j)<<endl;
			//			}
			int index=i*870+j;
			int indexnext=(i-1)*870+j;
			double disbefore=(double)dismat_0.at<float>(i+1,j);
			double dis=(double)dismat_0.at<float>(i,j);
			if(abs(dis-disbefore)<1)continue;
			double radius=dis*std::cos(pitchrad_beiqi[31-i]*PI/180);
			double originx=radius*costable[j];
			double originy=radius*sintable[j];
			double originz=dis*std::sin(pitchrad_beiqi[31-i]*PI/180);

			//计算起始点及前一点高度
			double height=heightmat_0.at<float>(i,j);
			double heightbefore=heightmat_0.at<float>(i+1,j);

			//当前线雷达与上一线雷达距离差
			double diff2=(double)dismat_0.at<float>(i,j)-(double)dismat_0.at<float>(i+1,j);
			double disnext=(double)dismat_0.at<float>(i-1,j);
			int count=0;
			while(disnext==0&&i>15){
				count++;
				i--;
				disnext=(double)dismat_0.at<float>(i-1,j);
			}
			//计算下一点及下下一点高度
			double heightnext=heightmat_0.at<float>(i-1,j);
			double heightnene=heightmat_0.at<float>(i-2,j);

			//下一线与当前线雷达距离差
			double diff1=disnext-dis;
			//到达0°后break
			if(i==6) break;

			double radiusnext=disnext*std::cos(pitchrad_beiqi[31-i+1]*PI/180);
			double originxnext=radiusnext*costable[j];
			double originynext=radiusnext*sintable[j];
			double originznext=dis*std::sin(pitchrad_beiqi[31-i+1]*PI/180);


			Eigen::Matrix<double, 3, 1> originp;
			Eigen::Matrix<double, 3, 1> originpnext;
			Eigen::Matrix<double, 3, 1> originpvirtual;

			originp<<originx,originy,originz;
			originpnext<<originxnext,originynext,originznext;

			double x=(T*originp)[0];
			double y=(T*originp)[1];
			double z=(T*originp)[2];

			double xnext=(T*originpnext)[0];
			double ynext=(T*originpnext)[1];
			double znext=(T*originpnext)[2];
			//
			if(x>-1.5&&x<1.5&&y<4&&y>-1||xnext>-1.5&&xnext<1.5&&ynext<4&&ynext>-1) {
				continue;
			}
			//用于判断雷达点是否走出车身，可以正式开始算法
			countlaser++;

			//			heightmat_0.at<float>(i,j)=z;
			//			if(xnext>-1.2&&xnext<1.2&&ynext<4&&ynext>-1) continue;
			//			heightmat_0.at<float>(i,j)=z;



			double radiusvirtual=4.0;//虚拟半径，用于盲区检测
			double originxvirtual=radiusvirtual*costable[j];
			double originyvirtual=radiusvirtual*sintable[j];
			double originzvirtual=0;
			originpvirtual<<originxvirtual,originyvirtual,originzvirtual;

			double xvirtual=(T*originpvirtual)[0];
			double yvirtual=(T*originpvirtual)[1];



			int col=boost::math::round((x+35)/0.2);
			int row=boost::math::round((y+20)/0.2);
			int colvirtual;
			int rowvirtual;
			if((i == 31 || i == 30) && (x < -4 || x > 4)){
				colvirtual=boost::math::round((xvirtual+35)/0.2);
				rowvirtual=boost::math::round((yvirtual+20)/0.2);
				if(height < -1){// &&(colvirtual<32/0.2||colvirtual>38/0.2)
					cout<<"======================== "<<i<<endl;
					cv::line(gridshow_0, cvPoint(colvirtual, GRIDWH - 1 -rowvirtual), cvPoint(col, GRIDWH - 1 - row),
							cvScalar(0, 0, 125));
					cv::line(justshow, cvPoint(colvirtual, GRIDWH - 1 -rowvirtual), cvPoint(col, GRIDWH - 1 - row),
							cvScalar(0, 0, 125));
				}
				//				第一条线自己给定初始diff2
				//				diff2=0.3;
			}

			int colnext=boost::math::round((xnext+35)/0.2);
			int rownext=boost::math::round((ynext+20)/0.2);

			int gridindex=0;
			unsigned char* ptr=gridshow_0.ptr<unsigned char>(GRIDWH-1-row);
			if(col<GRIDWH&&row<GRIDWH&&col>=0&&row>=0){


				//				增加不同半径下的不同高度阈值
				if(radius<8){if(height>0.3)break;}
				else if(radius<13){if(height>0.5)break;}
				else{if(height>0.8)break;}


				//				if(col>170&&col<180){
				//					cout<<"j is : "<<j<<endl;
				//				}
				if(j==800){
					ptr[3*col]=255;
					ptr[3*col+1]=255;
					ptr[3*col+2]=255;
					//					cv::line(gridshow_0,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext),cvScalar(255,255,255));
					//					cout<<radius<<" height diff "<<height-heightbefore<<endl;
				}
				//				针对远距离误检不同阈值
				if(1){
					//短距离用高度差和距离突变来定义
					if(0){//radius<10
						int thresh=THRESH;
						if(col>155&&col<195) thresh=2*THRESH;
						if(heightnext-height<-0.5&&(disnext-dis)/(dis-disbefore)>thresh){//diff1/diff2
							//					if((height-heightnext)/(radiusnext-radius)>0.1&&diff1/diff2>3){
							ptr[3*col]=0;
							ptr[3*col+1]=0;
							ptr[3*col+2]=255;
							//							cout<<radius<<"==================>"<<diff1/diff2<<endl;
							//排除径向距离比较小的
							if((colnext-col)*(colnext-col)*0.04+(rownext-row)*(rownext-row)*0.04<1){
#ifdef showgreen
								cv::line(gridshow_0,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext),cvScalar(0,150,0));
#endif
								continue;
							}
							//连线
							//							if(colnext<GRIDWH&&rownext<GRIDWH&&colnext>=0&&rownext>=0){
#ifdef lineiter
							cv::LineIterator it(gridshow_0,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext));
							it++;
							bool state=true;
							int pointcount=0;
							for(int i=0;i<it.count;i++,++it){
								//									if(i<it.count*0.2||i>it.count*0.8) continue;
								if((*it)[1]==255){
									pointcount++;
									int rows=GRIDWH-1-it.pos().y;
									int cols=it.pos().x;
									if((cols-col)*(cols-col)*0.04+(rows-row)*(rows-row)*0.04<1){
										continue;
									}
									(*it)[0]=255;
									(*it)[2]=255;
									//										cout<<row<<" "<<col<<" "<<rows<<" "<<cols<<endl;
									//										cout<<heightgrid_0.at<float>(row,col)<<"  "<<heightgrid_0.at<float>(rows,cols)<<endl;
									//用其实点处栅格最大高度减去终点处最小高度
									if(heightgridmax_0.at<float>(row,col)-heightgridmin_0.at<float>(rows,cols)<0.5){
										state=false;
										break;
									}
								}
								//									(*it)[2]=125;
							}
							if(!state) continue;
#endif
#ifdef USENEIGHBER
							int pointcount=0;
							int zerocount=0;
							for(int l=-2;l<=1;l++){
								for(int k=-WINDOW;k<WINDOW+1;k++){
									//									CONDITION(j){
									//										cout<<dismat_0.at<float>(i+1,j+k)/500<<" "<<
									//												dismat_0.at<float>(i,j+k)/500<<"  "<<dismat_0.at<float>(i-1,j)/500
									//												<<"  "<<dismat_0.at<float>(i-2,j+k)/500<<" "
									//												<<dismat_0.at<float>(i-3,j)/500<<endl;
									//									}
									if(l==0||l==1){
										if(dismat_0.at<float>(i+l,j+k)==0){
											zerocount++;
										}
									}
									if(l==-2){
										if((dismat_0.at<float>(i-1,j)-dismat_0.at<float>(i+l,j+k))/dismat_0.at<float>(i-1,j)>HEIGHTTHRESH){
											pointcount++;
										}
									}
								}
							}
							CONDITION(j){
								cout<<j<<"  "<<pointcount<<" "<<zerocount<<endl;
								//								cout<<j<<"  "<<radius<<"  "<<radiusnext<<endl;
							}
							if(pointcount>COUNTTHRESH||zerocount>ZEROCOUNTTH){
#ifdef showgreen
								cv::line(gridshow_0,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext),cvScalar(0,150,0));
#endif
								continue;
							}
#endif
							//							vector<double> temp;
							//							temp.clear();
							//							for(int i=-5;i<6;++i){
							//								for(int j=-5;j<6;++j){
							//									temp.push_back(heightgridmax_0.at<float>(rownext+i,colnext+j));
							//								}
							//							}
							//							sort(temp.begin(),temp.end());
							//							cout<<"--------------------------"<<endl;
							//							for(auto it:temp){
							//								cout<<it<<endl;
							//							}
							cv::LineIterator it(heightgridmax_0,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext));
							//								cout<<"the heights are......"<<endl;
							double hm=-100;
							for(int k=0;k<it.count;++k,it++){
								int x=it.pos().x;
								int y=it.pos().y;
								y=GRIDWH-1-y;
								for(int i=-2;i<3;++i){
									for(int j=-2;j<3;++j){
										double height=0;
										if(x+i>=0&&x+i<GRIDWH&&y+j>=0&&y+j<GRIDWH){
											float* ptr=heightgridmax_0.ptr<float>(y+j);
											height=(double)ptr[x+i];
										}
										if(height>hm) hm=height;
									}
								}
							}
							//								cout<<hm<<endl;
							if(hm<0.4&&!(y < 0 && x > -4 && x < 4)){//temp[temp.size()-1]<1&&temp[temp.size()-1]!=-100
								//								cout<<"the height is "<<temp[temp.size()-1]<<endl;
								//								cout<<"dis is "<<disbefore<<" "<<dis<<" "<<disnext<<"  ratio is "<<diff1/diff2<<
								//										" height is "<<height<<" "<<heightnext<<endl;
								cv::line(justshow,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext),cvScalar(0,0,125));
								cv::line(gridshow_0,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext),cvScalar(0,0,125));
							}
							//							}

						}
						else{
#ifdef showgreen
							cv::line(gridshow_0,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext),cvScalar(0,150,0));
#endif
						}
					}
					//大于10m小于17m需要加上颠簸程度、俯仰角、（起始点高度？--这个还不太确定）、正切值、最小高度、距离突变比
					//else
					if(radiusnext<100 && radiusnext > 20 && radius < 22){
						int thresh=8;
						if(col>155&&col<195) thresh=10;
						if(1){//&&height>-0.4 &&yprdiff<7 tmpmsg->pitch<5
							//					if((heightmat_0.at<float>(i-1,j)-heightmat_0.at<float>(i,j))<-1&&diff1/diff2>3){
							if((height-heightnext)/(radiusnext-radius)>0.07
									&&heightnext-height<-1&&(disnext-dis)/(dis-disbefore)>thresh){//
								ptr[3*col]=0;
								ptr[3*col+1]=0;
								ptr[3*col+2]=255;
								//排除径向距离比较小的
								if((colnext-col)*(colnext-col)*0.04+(rownext-row)*(rownext-row)*0.04<1){
#ifdef showgreen
									cv::line(gridshow_0,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext),cvScalar(0,150,0));
#endif
									continue;
								}
								//连线
								//								if(colnext<GRIDWH&&rownext<GRIDWH&&colnext>=0&&rownext>=0){
#ifdef lineiter
								cv::LineIterator it(gridshow_0,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext));
								it++;
								bool state=true;
								int pointcount=0;
								for(int i=0;i<it.count;i++,++it){
									//										if(i<it.count*0.2||i>it.count*0.8) continue;
									if((*it)[1]==255){
										//											pointcount++;
										int rows=GRIDWH-1-it.pos().y;
										int cols=it.pos().x;
										if((cols-col)*(cols-col)*0.04+(rows-row)*(rows-row)*0.04<1){
											continue;
										}
										(*it)[0]=255;
										(*it)[2]=255;
										//											cout<<row<<" "<<col<<" "<<rows<<" "<<cols<<endl;
										//											cout<<heightgrid_0.at<float>(row,col)<<"  "<<heightgrid_0.at<float>(rows,cols)<<endl;
										if(heightgridmax_0.at<float>(row,col)-heightgridmin_0.at<float>(rows,cols)<0.5){
											state=false;
											break;
										}
									}
									//									(*it)[2]=125;
								}
								if(!state) continue;
#endif
#ifdef USENEIGHBER
								int pointcount=0;
								int zerocount=0;
								for(int l=-2;l<=1;l++){
									for(int k=-WINDOW;k<WINDOW+1;k++){
										//										CONDITION(j){
										//											cout<<dismat_0.at<float>(i+1,j+k)/500<<" "<<
										//													dismat_0.at<float>(i,j+k)/500<<"  "<<dismat_0.at<float>(i-1,j)/500
										//													<<"  "<<dismat_0.at<float>(i-2,j+k)/500<<" "
										//													<<dismat_0.at<float>(i-3,j)/500<<endl;
										//										}
										if(l==0||l==1){
											if(dismat_0.at<float>(i+l,j+k)==0){
												zerocount++;
											}
										}
										if(l==-2){
											if((dismat_0.at<float>(i-1,j)-dismat_0.at<float>(i+l,j+k))/dismat_0.at<float>(i-1,j)>HEIGHTTHRESH){
												pointcount++;
											}
										}
									}
								}
								CONDITION(j){
									cout<<j<<"  "<<pointcount<<" "<<zerocount<<endl;
									//								cout<<j<<"  "<<radius<<"  "<<radiusnext<<endl;
								}
								if(pointcount>COUNTTHRESH||zerocount>ZEROCOUNTTH){

#ifdef showgreen
									cv::line(gridshow_0,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext),cvScalar(0,150,0));
#endif
									continue;
								}
#endif
								//								vector<double> temp;
								//								temp.clear();
								//								for(int i=-5;i<6;++i){
								//									for(int j=-5;j<6;++j){
								//										temp.push_back(heightgridmax_0.at<float>(rownext+i,colnext+j));
								//									}
								//								}
								//								sort(temp.begin(),temp.end());
								//								cout<<"--------------------------"<<endl;
								//								for(auto it:temp){
								//									cout<<it<<endl;
								//								}
								cv::LineIterator it(heightgridmax_0,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext));
								//								cout<<"the heights are......"<<endl;
								double hm=-100;
								for(int k=0;k<it.count;++k,it++){
									int x=it.pos().x;
									int y=it.pos().y;
									y=GRIDWH-1-y;
									for(int i=-2;i<3;++i){
										for(int j=-2;j<3;++j){
											double height=0;
											if(x+i>=0&&x+i<GRIDWH&&y+j>=0&&y+j<GRIDWH){
												float* ptr=heightgridmax_0.ptr<float>(y+j);
												height=(double)ptr[x+i];
											}
											if(height>hm) hm=height;
										}
									}
								}
								//								cout<<hm<<endl;
								if(hm<0.4&&!(y < 0 && x > -4 && x < 4)){//temp[temp.size()-1]<1&&temp[temp.size()-1]!=-100
									//									cout<<"dis is "<<disbefore<<" "<<dis<<" "<<disnext<<"  ratio is "<<diff1/diff2<<
									//											" height is "<<height<<" "<<heightnext<<endl;
									cv::line(justshow,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext),cvScalar(0,0,125));
									cv::line(gridshow_0,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext),cvScalar(0,0,125));
									break;
								}
								//								}

							}
							else{
#ifdef showgreen
								cv::line(gridshow_0,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext),cvScalar(0,150,0));
#endif
							}
						}
					}
				}
			}
			//把水的可疑区域画出来
			cv::LineIterator it(heightgridmax_0,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext));
			//								cout<<"the heights are......"<<endl;
			if(count>2){
				double hm=-100;
				for(int k=0;k<it.count;++k,it++){
					int x=it.pos().x;
					int y=it.pos().y;
					y=GRIDWH-1-y;
					for(int i=-2;i<3;++i){
						for(int j=-2;j<3;++j){
							double height=0;
							if(x+i>=0&&x+i<GRIDWH&&y+j>=0&&y+j<GRIDWH){
								float* ptr=heightgridmax_0.ptr<float>(y+j);
								height=(double)ptr[x+i];
							}
							if(height>hm) hm=height;
						}
					}
				}
				if(hm<0.4&&colnext>0&&colnext<GRIDWH
						&&rownext>0&&rownext<GRIDWH&&disnext<30){//&&i>20
					//					cv::line(gridshow_0,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext),cvScalar(125,125,125));
					//					cv::line(justshow,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext),cvScalar(125,125,125));

				}
			}
			////把水的可疑区域画出来
		}
	}
#endif


	stiff_msgs::stiffwater msg_send;
	msg_send.header.stamp=height_msg->header.stamp;
	msg_send.header.frame_id="stiffwater";
	msg_send.ogmheight=351;
	msg_send.ogmwidth=201;
	msg_send.resolution=0.2;
	msg_send.vehicle_x=100;
	msg_send.vehicle_y=100;
	for(int row=0;row<GRIDWH;row++){
		unsigned char* ptrall=gridall.ptr<unsigned char>(GRIDWH-1-row);
		unsigned char* ptr=gridshow_0.ptr<unsigned char>(GRIDWH-1-row);
		for(int col=0;col<GRIDWH;col++){
			if(ptr[3*col+2]==125){
				ptrall[col]=255;
			}
			else if(ptr[3*col+2]==125&&ptr[3*col+1]==125&&ptr[3*col]==125){//水区域
				ptrall[col]=200;
			}
			else if(ptr[3*col+1]==150){
				ptrall[col]=100;
			}
		}
	}


	cv::dilate(gridall,gridall,elementdil);
	cv::erode(gridall,gridall,elementero);
	//	cv::dilate(gridall,gridall,elementero2);

	for(int row=0;row<351;row++){
		unsigned char* ptr=gridall.ptr<unsigned char>(GRIDWH-1-row);
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
	pubStiffwaterOgm.publish(msg_send);


	for(int i=4;i<21;i+=2){
		if(i==8||i==14){
			cv::circle(gridshow_0,cvPoint(175,250),(int)(i/0.2),cvScalar(80,0,80));
			cv::circle(justshow,cvPoint(175,250),(int)(i/0.2),cvScalar(80,0,80));
			cv::circle(gridall,cvPoint(175,250),(int)(i/0.2),cvScalar(80,0,80));
		}
		else{
			cv::circle(gridshow_0,cvPoint(175,250),(int)(i/0.2),cvScalar(80,80,0));
			cv::circle(justshow,cvPoint(175,250),(int)(i/0.2),cvScalar(80,80,0));
			cv::circle(gridall,cvPoint(175,250),(int)(i/0.2),cvScalar(80,0,80));
		}
	}
	//给justshow画角度线
	double unitangle=2.0/870*PI;
	for(int i=0;i<8;i++){
		double angle=-(unitangle*i*100+PI/2)+PI;
		int x=175+1000*std::cos(angle);
		int y=-100+GRIDWH-1-1000*std::sin(angle);
		cv::line(justshow,cvPoint(175,250),cvPoint(x,y),cvScalar(0,165,135));
	}
	cv::namedWindow("show",CV_WINDOW_NORMAL);
	//	int row=350-100-16;
	//		cv::line(gridshow_1,cvPoint(0,row),cvPoint(350,row),cvScalar(0,150,0));
	//		cv::line(gridshow_0,cvPoint(0,row),cvPoint(350,row),cvScalar(0,150,0));
	cv::line(gridshow_0,cvPoint(leftend,0),cvPoint(leftend,GRIDWH-1),cvScalar(0,126,0));
	cv::line(gridshow_0,cvPoint(rightend,0),cvPoint(rightend,GRIDWH-1),cvScalar(0,126,0));
	cv::line(justshow,cvPoint(leftend,0),cvPoint(leftend,GRIDWH-1),cvScalar(0,126,0));
	cv::line(justshow,cvPoint(rightend,0),cvPoint(rightend,GRIDWH-1),cvScalar(0,126,0));
	if(visual_on){
#ifdef TESTSHOW
		cv::imshow("grid",gridshow_0);
#endif
		cv::imshow("show",justshow);
		cv::imshow("gridall",gridall);
		//	cv::imshow("range", dismat_0);
		//	cv::imshow("height", heightmat_0);
		cv::waitKey(10);
	}
}

void gpsdatacllbak(const sensor_driver_msgs::GpswithHeadingConstPtr& msg){
	//			cout<<"gpsdata stamp is "<<msg->gps.header.stamp<<endl;
	//		cout<<yprmin<<endl;
	qgwithhmsgs_.Push(msg);
	//	qypr.push_back(msg);
	if(qgwithhmsgs_.Size()>10){
		qgwithhmsgs_.Pop();
	}

}
#endif
#ifdef TOYOTA
void callback(const depth_image_utils::HeightMapConstPtr& height_msg){
	qheightmap_.Push(height_msg);
	if(qheightmap_.Size()>1){
		qheightmap_.Pop();
	}
}
void gpsdatacllbak(const sensor_driver_msgs::GpswithHeadingConstPtr& msg){
	qgwithhmsgs_.Push(msg);
	if(qgwithhmsgs_.Size()>10){
		qgwithhmsgs_.Pop();
	}
}
void process(){
	while(true){
		//		cout<<"size is "<<qheightmap_.Size()<<endl;
		const depth_image_utils::HeightMapConstPtr heightmapmsg=qheightmap_.PopWithTimeout(common::FromSeconds(0.1));
		if(heightmapmsg==nullptr) continue;
		double timeheightmap=heightmapmsg->header.stamp.toSec();
		//		auto imumsg=qgwithhmsgs_.PopWithTimeout(common::FromSeconds(0.1));
		//		if(imumsg==nullptr) continue;
		//		double timeimu=imumsg->gps.header.stamp.toSec();
		//		if(timeimu>timeheightmap){
		//			cout<<"waite for height map"<<endl;
		//			qgwithhmsgs_.Push_Front(std::move(imumsg));
		//			continue;
		//		}
		//		while(timeheightmap-timeimu>0.00001){
		//
		//			if(qgwithhmsgs_.Size()==0){
		//				qheightmap_.Push_Front(std::move(heightmapmsg));//将此帧高度图放回去，否则丢失信息
		//				continue;
		//			}
		//			imumsg=qgwithhmsgs_.Pop();
		//			timeimu=imumsg->gps.header.stamp.toSec();
		//		}
		if(visual_on){
#ifdef TESTSHOW
			cv::namedWindow("grid",CV_WINDOW_NORMAL);
			cv::namedWindow("justshow",CV_WINDOW_NORMAL);
#endif
			//		cv::namedWindow("gridall",CV_WINDOW_NORMAL);
			//		cv::namedWindow("show",CV_WINDOW_NORMAL);
		}
		cv::Mat heightmat_0 = cv::Mat::zeros(64,870,CV_32F);
		cv::Mat dismat_0 = cv::Mat::zeros(64,870,CV_32F);
		for(int i=0;i<64;++i){
			for(int j =0;j<870;++j){
				if(abs(heightmapmsg->data[i*870+j]+100)>0.0001){//原始值为-100赋值为0
					dismat_0.at<float>(i,j)=heightmapmsg->data[i*870+j+64*870];
				}
			}
		}
		cv::Mat gridshow_0 = cv::Mat::zeros(GRIDWH,GRIDWH,CV_8UC3);
		cv::Mat justshow = cv::Mat::zeros(GRIDWH,GRIDWH,CV_8UC3);
		cv::Mat heightshow = cv::Mat::zeros(GRIDWH,GRIDWH,CV_8UC1);
		//最大高度、最小高度、高度差
		cv::Mat heightgrid_0 = cv::Mat::zeros(GRIDWH,GRIDWH,CV_32FC1);
		cv::Mat heightgridmin_0 = cv::Mat::ones(GRIDWH,GRIDWH,CV_32FC1)*100;
		cv::Mat heightgridmax_0 = cv::Mat::ones(GRIDWH,GRIDWH,CV_32FC1)*(-100);
		//		cout<<"-------------------------------"<<endl;
		for(int j=0;j<870;j++){
			for(int i=58;i>6;i--){
				int index=i*870+j;
				double dis=(double)dismat_0.at<float>(i,j);
				double radius=dis*std::cos(pitchrad64[i]*PI/180);
				double originx=radius*costable[j];
				double originy=radius*sintable[j];
				double originz=dis*std::sin(pitchrad64[i]*PI/180);
				Eigen::Matrix<double, 3, 1> originp;
				originp<<originx,originy,originz;

				double x=(T*originp)[0];
				double y=(T*originp)[1];
				double z=(T*originp)[2];

				if(x>-1.5&&x<1.5&&y<4&&y>-1){;}//
				else{
					heightmat_0.at<float>(i,j)=z;
					int col=boost::math::round((x+35)/0.2);//干！！！！！！！！！！！
					int row=boost::math::round((y+20)/0.2);
					int gridindex=0;
					unsigned char* ptr=gridshow_0.ptr<unsigned char>(GRIDWH-1-row);
					unsigned char* ptrshow=justshow.ptr<unsigned char>(GRIDWH-1-row);
					float* ptrheight=heightgrid_0.ptr<float>(row);
					float* ptrmin=heightgridmin_0.ptr<float>(row);
					float* ptrmax=heightgridmax_0.ptr<float>(row);
					unsigned char* testshow=heightshow.ptr<unsigned char>(GRIDWH-1-row);


					if(col<GRIDWH&&row<GRIDWH&&col>=0&&row>=0){
						ptr[3*col]=0;
						ptr[3*col+1]=255;
						ptr[3*col+2]=0;
						ptrshow[3*col+1]=255;
						if(ptrmin[col]>z){
							ptrmin[col]=z;
						}
						if(ptrmax[col]<z){
							ptrmax[col]=z;
							testshow[col]=255;

						}
						ptrheight[col]=ptrmax[col]-ptrmin[col];

					}
				}
			}
		}
		//输出一下高度
		//		for(int i=0;i<GRIDWH;++i){
		//			float* ptr=heightgridmax_0.ptr<float>(i);
		//			for(int j=0;j<GRIDWH;++j){
		//				if(heightgridmax_0.at<float>(i,j)!=-100||ptr[j]!=-100){
		//					cout<<heightgridmax_0.at<float>(i,j)-ptr[j]<<endl;
		//				}
		//			}
		//		}

		//开始检测
		//		cout<<"========================"<<endl;
		int rightend=(int)38/0.2;
		int leftend=(int)32/0.2;
		for(int j=0;j<870;j++){
			int myj;
			int countlaser=0;
			for(int i=58;i>15;i--){
				//			if(j==800){
				//				cout<<"height is "<<heightmat_0.at<float>(i,j)<<endl;
				//			}
				int index=i*870+j;
				int indexnext=(i-1)*870+j;
				double dis=(double)dismat_0.at<float>(i,j);
				double radius=dis*std::cos(pitchrad64[i]*PI/180);
				double originx=radius*costable[j];
				double originy=radius*sintable[j];
				double originz=dis*std::sin(pitchrad64[i]*PI/180);

				//计算起始点及前一点高度
				double height=heightmat_0.at<float>(i,j);
				double heightbefore=heightmat_0.at<float>(i+1,j);
				//当前线雷达与上一线雷达距离差
				double diff2=(double)dismat_0.at<float>(i,j)-(double)dismat_0.at<float>(i+1,j);
				double disnext=(double)dismat_0.at<float>(i-1,j);
				int count=0;
				while(disnext==0&&i>15){
					count++;
					i--;
					disnext=(double)dismat_0.at<float>(i-1,j);
				}
				//计算下一点及下下一点高度
				double heightnext=heightmat_0.at<float>(i-1,j);
				double heightnene=heightmat_0.at<float>(i-2,j);

				//下一线与当前线雷达距离差
				double diff1=disnext-dis;
				//				if(count>=2){
				//					break;
				//				}
				//到达3°后break
				if(i==15) break;

				double radiusnext=disnext*std::cos(pitchrad64[63-i+1]*PI/180);
				double originxnext=radiusnext*costable[j];
				double originynext=radiusnext*sintable[j];
				double originznext=disnext*std::sin(pitchrad64[63-i+1]*PI/180);



				Eigen::Matrix<double, 3, 1> originp;
				Eigen::Matrix<double, 3, 1> originpnext;
				Eigen::Matrix<double, 3, 1> originpvirtual;

				originp<<originx,originy,originz;
				originpnext<<originxnext,originynext,originznext;

				double x=(T*originp)[0];
				double y=(T*originp)[1];
				double z=(T*originp)[2];

				double xnext=(T*originpnext)[0];
				double ynext=(T*originpnext)[1];
				double znext=(T*originpnext)[2];
				//
				if(x>-1.5&&x<1.5&&y<4&&y>-1||xnext>-1.5&&xnext<1.5&&ynext<4&&ynext>-1) {
					continue;
				}
				//用于判断雷达点是否走出车身，可以正式开始算法
				countlaser++;
				//			heightmat_0.at<float>(i,j)=z;
				//			if(xnext>-1.2&&xnext<1.2&&ynext<4&&ynext>-1) continue;
				//			heightmat_0.at<float>(i,j)=z;



				double radiusvirtual=4.0;//虚拟半径，用于盲区检测
				double originxvirtual=radiusvirtual*costable[j];
				double originyvirtual=radiusvirtual*sintable[j];
				double originzvirtual=0;
				originpvirtual<<originxvirtual,originyvirtual,originzvirtual;

				double xvirtual=(T*originpvirtual)[0];
				double yvirtual=(T*originpvirtual)[1];



				int col=boost::math::round((x+35)/0.2);
				int row=boost::math::round((y+20)/0.2);
				int colvirtual;
				int rowvirtual;
				if(countlaser==1){
					colvirtual=boost::math::round((xvirtual+35)/0.2);
					rowvirtual=boost::math::round((yvirtual+20)/0.2);
					if(radius>7&&height<-1&&(colvirtual<32/0.2||colvirtual>38/0.2)){
						//					unsigned char* ptr=gridshow_0.ptr<unsigned char>(GRIDWH-1-rowvirtual);
						//					ptr[3*colvirtual]=0;
						//					ptr[3*colvirtual+1]=0;
						//					ptr[3*colvirtual+1]=255;
						cv::line(gridshow_0,cvPoint(col,GRIDWH-1-row),cvPoint(colvirtual,GRIDWH-1-rowvirtual),cvScalar(0,0,125));
						cv::line(justshow,cvPoint(col,GRIDWH-1-row),cvPoint(colvirtual,GRIDWH-1-rowvirtual),cvScalar(0,0,125));
					}
					//				第一条线自己给定初始diff2
					diff2=0.3;
				}
				int colnext=boost::math::round((xnext+35)/0.2);
				int rownext=boost::math::round((ynext+20)/0.2);


				int gridindex=0;
				unsigned char* ptr=gridshow_0.ptr<unsigned char>(GRIDWH-1-row);


				//下面是为了查看从当前点到下一个点之间是否有障碍物，所以延长了一段，水的倒影有可能在后面一点
				double radiuswater=(disnext)*std::cos(pitchrad64[63-i+1]*PI/180);
				double xwater=radiuswater*costable[j];
				double ywater=radiuswater*sintable[j];
				double zwater=(disnext)*std::sin(pitchrad64[63-i+1]*PI/180);
				Eigen::Matrix<double, 3, 1> pointwater;
				pointwater<<xwater,ywater,zwater;
				double xwater1=(T*pointwater)[0];
				double ywater1=(T*pointwater)[1];
				double zwater1=(T*pointwater)[2];

				int colwater=boost::math::round((xwater1+35)/0.2);
				int rowwater=boost::math::round((ywater1+20)/0.2);

				if(col<GRIDWH&&row<GRIDWH&&col>=0&&row>=0){
					//				增加不同半径下的不同高度阈值
					if(radius<3)break;
					if(radius<8){if(height>0.3)break;}
					else if(radius<13){if(height>0.5)break;}
					else{if(height>0.8)break;}
					unsigned char* ptrshow=justshow.ptr<unsigned char>(GRIDWH-1-row);
					if(col==35/0.2){
						ptrshow[3*col]=255;
						ptrshow[3*col+1]=255;
						ptrshow[3*col+2]=255;
					}

					//				针对远距离误检不同阈值
					if(1){
						//短距离用高度差和距离突变来定义
						if(radius<10){
							int thresh=6;
							if(col>155&&col<195) thresh=15;
							if(heightnext-height<-0.5&&diff1/diff2>thresh){//

								//					if((height-heightnext)/(radiusnext-radius)>0.1&&diff1/diff2>3){
								ptr[3*col]=0;
								ptr[3*col+1]=0;
								ptr[3*col+2]=255;
								//							cout<<radius<<"==================>"<<diff1/diff2<<endl;
								//排除径向距离比较小的
								if((colnext-col)*(colnext-col)*0.04+(rownext-row)*(rownext-row)*0.04<1){
#ifdef showgreen
									cv::line(gridshow_0,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext),cvScalar(0,150,0));
#endif
									continue;
								}
								//连线
								//							if(colnext<GRIDWH&&rownext<GRIDWH&&colnext>=0&&rownext>=0){
#ifdef lineiter
								cv::LineIterator it(gridshow_0,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext));
								it++;
								bool state=true;
								int pointcount=0;
								for(int i=0;i<it.count;i++,++it){
									//									if(i<it.count*0.2||i>it.count*0.8) continue;
									if((*it)[1]==255){
										pointcount++;
										int rows=GRIDWH-1-it.pos().y;
										int cols=it.pos().x;
										if((cols-col)*(cols-col)*0.04+(rows-row)*(rows-row)*0.04<1){
											continue;
										}
										(*it)[0]=255;
										(*it)[2]=255;
										//										cout<<row<<" "<<col<<" "<<rows<<" "<<cols<<endl;
										//										cout<<heightgrid_0.at<float>(row,col)<<"  "<<heightgrid_0.at<float>(rows,cols)<<endl;
										//用其实点处栅格最大高度减去终点处最小高度
										if(heightgridmax_0.at<float>(row,col)-heightgridmin_0.at<float>(rows,cols)<0.5){
											state=false;
											break;
										}
									}
									//									(*it)[2]=125;
								}
								if(!state) continue;
#endif
#ifdef USENEIGHBER
								int pointcount=0;
								int zerocount=0;
								for(int l=-2;l<=1;l++){
									for(int k=-WINDOW;k<WINDOW+1;k++){
										//									CONDITION(j){
										//										cout<<dismat_0.at<float>(i+1,j+k)/500<<" "<<
										//												dismat_0.at<float>(i,j+k)/500<<"  "<<dismat_0.at<float>(i-1,j)/500
										//												<<"  "<<dismat_0.at<float>(i-2,j+k)/500<<" "
										//												<<dismat_0.at<float>(i-3,j)/500<<endl;
										//									}
										if(l==0||l==1){
											if(dismat_0.at<float>(i+l,j+k)==0){
												zerocount++;
											}
										}
										if(l==-2){
											if((dismat_0.at<float>(i-1,j)-dismat_0.at<float>(i+l,j+k))/dismat_0.at<float>(i-1,j)>HEIGHTTHRESH){
												pointcount++;
											}
										}
									}
								}
								CONDITION(j){
									cout<<j<<"  "<<pointcount<<" "<<zerocount<<endl;
									//								cout<<j<<"  "<<radius<<"  "<<radiusnext<<endl;
								}
								if(pointcount>COUNTTHRESH||zerocount>ZEROCOUNTTH){
#ifdef showgreen
									cv::line(gridshow_0,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext),cvScalar(0,150,0));
#endif
									continue;
								}
#endif
								//								vector<double> temp;
								//								temp.clear();
								//								for(int i=-1;i<2;++i){
								//									for(int j=-1;j<2;++j){
								////										cout<<rownext<<"  "<<colnext<<endl;
								//										if(rownext+i<351&&colnext+j<351){
								//											temp.push_back(heightgridmax_0.at<float>(rownext+i,colnext+j));
								//										}
								//									}
								//								}
								//								sort(temp.begin(),temp.end());
								//								if(temp.size()==0)continue;
								cv::LineIterator it(heightgridmax_0,cvPoint(col,GRIDWH-1-row),cvPoint(colwater,GRIDWH-1-rowwater));
								//								cout<<"the heights are......"<<endl;
								double hm=-100;
								for(int k=0;k<it.count;++k,it++){
									int x=it.pos().x;
									int y=it.pos().y;
									y=GRIDWH-1-y;
									for(int i=-5;i<6;++i){
										for(int j=-5;j<6;++j){
											double height=0;
											if(x+i>=0&&x+i<GRIDWH&&y+j>=0&&y+j<GRIDWH){
												float* ptr=heightgridmax_0.ptr<float>(y+j);
												height=(double)ptr[x+i];
											}
											if(height>hm) hm=height;
										}
									}
								}
								//								cout<<hm<<endl;
								if(hm<0.4){//temp[temp.size()-1]<1&&temp[temp.size()-1]!=-100
									//									cout<<"the height is "<<temp[temp.size()-1]<<endl;
									//									cout<<hm<<endl;
									//									cv::line(gridshow_0,cvPoint(col,GRIDWH-1-row),cvPoint(colwater,GRIDWH-1-rowwater),cvScalar(0,125,125));
									cv::line(justshow,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext),cvScalar(0,0,125));
									cv::line(gridshow_0,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext),cvScalar(0,0,125));
								}
								//							}

							}
							else{
#ifdef showgreen
								cv::line(gridshow_0,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext),cvScalar(0,150,0));
#endif
							}
						}
						//大于10m小于17m需要加上颠簸程度、俯仰角、（起始点高度？--这个还不太确定）、正切值、最小高度、距离突变比
						//tmpmsg->pitch<5
						else if(radius<30){
							int thresh=5;
							if(col>155&&col<195) thresh=15;
							if(1){//&&height>-0.4 &&yprdiff<7
								//					if((heightmat_0.at<float>(i-1,j)-heightmat_0.at<float>(i,j))<-1&&diff1/diff2>3){
								if((height-heightnext)/(radiusnext-radius)>0.07
										&&heightnext-height<-1&&diff1/diff2>thresh){//
									ptr[3*col]=0;
									ptr[3*col+1]=0;
									ptr[3*col+2]=255;

									//排除径向距离比较小的
									if((colnext-col)*(colnext-col)*0.04+(rownext-row)*(rownext-row)*0.04<1){
#ifdef showgreen
										cv::line(gridshow_0,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext),cvScalar(0,150,0));
#endif
										continue;
									}
									//连线
									//								if(colnext<GRIDWH&&rownext<GRIDWH&&colnext>=0&&rownext>=0){
#ifdef lineiter
									cv::LineIterator it(gridshow_0,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext));
									it++;
									bool state=true;
									int pointcount=0;
									for(int i=0;i<it.count;i++,++it){
										//										if(i<it.count*0.2||i>it.count*0.8) continue;
										if((*it)[1]==255){
											//											pointcount++;
											int rows=GRIDWH-1-it.pos().y;
											int cols=it.pos().x;
											if((cols-col)*(cols-col)*0.04+(rows-row)*(rows-row)*0.04<1){
												continue;
											}
											(*it)[0]=255;
											(*it)[2]=255;
											//											cout<<row<<" "<<col<<" "<<rows<<" "<<cols<<endl;
											//											cout<<heightgrid_0.at<float>(row,col)<<"  "<<heightgrid_0.at<float>(rows,cols)<<endl;
											if(heightgridmax_0.at<float>(row,col)-heightgridmin_0.at<float>(rows,cols)<0.5){
												state=false;
												break;
											}
										}
										//									(*it)[2]=125;
									}
									if(!state) continue;
#endif
#ifdef USENEIGHBER
									int pointcount=0;
									int zerocount=0;
									for(int l=-2;l<=1;l++){
										for(int k=-WINDOW;k<WINDOW+1;k++){
											//										CONDITION(j){
											//											cout<<dismat_0.at<float>(i+1,j+k)/500<<" "<<
											//													dismat_0.at<float>(i,j+k)/500<<"  "<<dismat_0.at<float>(i-1,j)/500
											//													<<"  "<<dismat_0.at<float>(i-2,j+k)/500<<" "
											//													<<dismat_0.at<float>(i-3,j)/500<<endl;
											//										}
											if(l==0||l==1){
												if(dismat_0.at<float>(i+l,j+k)==0){
													zerocount++;
												}
											}
											if(l==-2){
												if((dismat_0.at<float>(i-1,j)-dismat_0.at<float>(i+l,j+k))/dismat_0.at<float>(i-1,j)>HEIGHTTHRESH){
													pointcount++;
												}
											}
										}
									}
									CONDITION(j){
										cout<<j<<"  "<<pointcount<<" "<<zerocount<<endl;
										//								cout<<j<<"  "<<radius<<"  "<<radiusnext<<endl;
									}
									if(pointcount>COUNTTHRESH||zerocount>ZEROCOUNTTH){

#ifdef showgreen
										cv::line(gridshow_0,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext),cvScalar(0,150,0));
#endif
										continue;
									}
#endif
									//									vector<double> temp;
									//									temp.clear();
									//									for(int i=-1;i<2;++i){
									//										for(int j=-1;j<2;++j){
									////											cout<<rownext<<"  "<<colnext<<endl;
									//											if(rownext+i<351&&colnext+j<351){
									//												temp.push_back(heightgridmax_0.at<float>(rownext+i,colnext+j));
									//											}
									//										}
									//									}
									//									if(temp.size()==0)continue;
									//									sort(temp.begin(),temp.end());
									cv::LineIterator it(heightgridmax_0,cvPoint(col,GRIDWH-1-row),cvPoint(colwater,GRIDWH-1-rowwater));
									//								cout<<"the heights are......"<<endl;
									double hm=-100;
									for(int k=0;k<it.count;++k,it++){
										int x=it.pos().x;
										int y=it.pos().y;
										y=GRIDWH-y-1;
										for(int i=-5;i<6;++i){
											for(int j=-5;j<6;++j){
												double height=0;
												if(x+i>=0&&x+i<GRIDWH&&y+j>=0&&y+j<GRIDWH){
													float* ptr=heightgridmax_0.ptr<float>(y+j);
													height=(double)ptr[x+i];
												}
												if(height>hm) hm=height;
											}
										}
										//									if(height!=(-100)&&height!=0){
										//										cout<<height<<" ";
										//									}
									}
									//									cout<<hm<<endl;
									if(hm<0.4){//temp[temp.size()-1]<1&&temp[temp.size()-1]!=-100
										//										cout<<"the height is "<<temp[temp.size()-1]<<endl;

										//										cout<<hm<<endl;
										//										cv::line(gridshow_0,cvPoint(col,GRIDWH-1-row),cvPoint(colwater,GRIDWH-1-rowwater),cvScalar(0,125,125));
										cv::line(justshow,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext),cvScalar(0,0,125));
										cv::line(gridshow_0,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext),cvScalar(0,0,125));
									}
									//								}

								}
								else{
#ifdef showgreen
									cv::line(gridshow_0,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext),cvScalar(0,150,0));
#endif
								}
							}
						}
					}
				}


				//把水的可疑区域画出来
				cv::LineIterator it(heightgridmax_0,cvPoint(col,GRIDWH-1-row),cvPoint(colwater,GRIDWH-1-rowwater));
				//								cout<<"the heights are......"<<endl;
				if(count>2){
					double hm=-100;
					for(int k=0;k<it.count;++k,it++){
						int x=it.pos().x;
						int y=it.pos().y;
						y=GRIDWH-1-y;
						for(int i=-2;i<3;++i){
							for(int j=-2;j<3;++j){
								double height=0;
								if(x+i>=0&&x+i<GRIDWH&&y+j>=0&&y+j<GRIDWH){
									float* ptr=heightgridmax_0.ptr<float>(y+j);
									height=(double)ptr[x+i];
								}
								if(height>hm) hm=height;
							}
						}
					}
					if(hm<0.4&&colnext>0&&colnext<GRIDWH
							&&rownext>0&&rownext<GRIDWH&&disnext<30){//&&i>20
						cv::line(gridshow_0,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext),cvScalar(125,125,125));
						cv::line(justshow,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext),cvScalar(125,125,125));

					}
				}
				////把水的可疑区域画出来
			}
		}


		//发送消息
		stiff_msgs::stiffwater msg_send;
		msg_send.header.stamp=heightmapmsg->header.stamp;
		msg_send.header.frame_id="stiffwater";
		msg_send.ogmheight=351;
		msg_send.ogmwidth=201;
		msg_send.resolution=0.2;
		msg_send.vehicle_x=100;
		msg_send.vehicle_y=100;
		cv::Mat gridall = cv::Mat::zeros(GRIDWH,GRIDWH,CV_8UC1);
		for(int row=0;row<GRIDWH;row++){
			unsigned char* ptrall=gridall.ptr<unsigned char>(GRIDWH-1-row);
			unsigned char* ptr=gridshow_0.ptr<unsigned char>(GRIDWH-1-row);
			for(int col=0;col<GRIDWH;col++){
				if(ptr[3*col+2]==125&&ptr[3*col]==0){//悬崖区域
					ptrall[col]=255;
				}
				else if(ptr[3*col+2]==125&&ptr[3*col+1]==125&&ptr[3*col]==125){//水区域
					ptrall[col]=200;
				}
				else if(ptr[3*col+1]==150){//通行区域
					ptrall[col]=100;
				}
			}
		}
		cv::dilate(gridall,gridall,elementdil);
		cv::erode(gridall,gridall,elementero);
		//send
		int max_cnt=-10;
		for(int row=0;row<351;row++){
			unsigned char* ptr=gridall.ptr<unsigned char>(GRIDWH-1-row);
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

				if(col>75&&col<125&&row>100&&row<175){//车身两侧五米车前十五米看有没有水，
					int count=0;
					for(int windowi=0;windowi<10&&windowi+row<351;++windowi){
						unsigned char* ptrwater=gridall.ptr<unsigned char>(GRIDWH-1-row-windowi);
						for(int windowj=0;windowj<10;++windowj){
							if(ptrwater[col+75+windowj]==200){
								count++;
							}
						}
					}
					if(count>max_cnt){
						max_cnt=count;
					}
				}
			}
		}
		if(max_cnt>70){
			msg_send.havewater=1;
		}else{
			msg_send.havewater=0;
		}
		//		cout<<"water max_cnt is ==================== "<<max_cnt<<endl;
		pubStiffwaterOgm.publish(msg_send);
		if(visual_on){
			cv::imshow("show",gridall);
			cv::imshow("grid",gridshow_0);
			cv::imshow("justshow",justshow);
		}
		cv::waitKey(10);
	}
}
#endif
#ifdef SIXT64
void callback(const depth_image_utils::HeightMapConstPtr& height_msg){
	qheightmap_.Push(height_msg);
	if(qheightmap_.Size()>1){
		qheightmap_.Pop();
	}
}
void gpsdatacllbak(const sensor_driver_msgs::GpswithHeadingConstPtr& msg){
	qgwithhmsgs_.Push(msg);
	if(qgwithhmsgs_.Size()>10){
		qgwithhmsgs_.Pop();
	}
}
void process(){
	while(true){
		//		cout<<"size is "<<qheightmap_.Size()<<endl;
		const depth_image_utils::HeightMapConstPtr heightmapmsg=qheightmap_.PopWithTimeout(common::FromSeconds(0.1));
		if(heightmapmsg==nullptr) continue;
		double timeheightmap=heightmapmsg->header.stamp.toSec();
		//		auto imumsg=qgwithhmsgs_.PopWithTimeout(common::FromSeconds(0.1));
		//		if(imumsg==nullptr) continue;
		//		double timeimu=imumsg->gps.header.stamp.toSec();
		//		if(timeimu>timeheightmap){
		//			cout<<"waite for height map"<<endl;
		//			qgwithhmsgs_.Push_Front(std::move(imumsg));
		//			continue;
		//		}
		//		while(timeheightmap-timeimu>0.00001){
		//
		//			if(qgwithhmsgs_.Size()==0){
		//				qheightmap_.Push_Front(std::move(heightmapmsg));//将此帧高度图放回去，否则丢失信息
		//				continue;
		//			}
		//			imumsg=qgwithhmsgs_.Pop();
		//			timeimu=imumsg->gps.header.stamp.toSec();
		//		}
		if(visual_on){
#ifdef TESTSHOW
			cv::namedWindow("grid",CV_WINDOW_NORMAL);
			cv::namedWindow("justshow",CV_WINDOW_NORMAL);
#endif
			//		cv::namedWindow("gridall",CV_WINDOW_NORMAL);
			//		cv::namedWindow("show",CV_WINDOW_NORMAL);
		}
		cv::Mat heightmat_0 = cv::Mat::zeros(64,870,CV_32F);
		cv::Mat dismat_0 = cv::Mat::zeros(64,870,CV_32F);
		for(int i=0;i<64;++i){
			for(int j =0;j<870;++j){
				if(abs(heightmapmsg->data[i*870+j]+100)>0.0001){//原始值为-100赋值为0
					dismat_0.at<float>(i,j)=heightmapmsg->data[i*870+j+64*870];
				}
			}
		}
		cv::Mat gridshow_0 = cv::Mat::zeros(GRIDWH,GRIDWH,CV_8UC3);
		cv::Mat justshow = cv::Mat::zeros(GRIDWH,GRIDWH,CV_8UC3);
		cv::Mat heightshow = cv::Mat::zeros(GRIDWH,GRIDWH,CV_8UC1);
		//最大高度、最小高度、高度差
		cv::Mat heightgrid_0 = cv::Mat::zeros(GRIDWH,GRIDWH,CV_32FC1);
		cv::Mat heightgridmin_0 = cv::Mat::ones(GRIDWH,GRIDWH,CV_32FC1)*100;
		cv::Mat heightgridmax_0 = cv::Mat::ones(GRIDWH,GRIDWH,CV_32FC1)*(-100);
		//		cout<<"-------------------------------"<<endl;
		for(int j=0;j<870;j++){
			for(int i=58;i>6;i--){
				int index=i*870+j;
				double dis=(double)dismat_0.at<float>(i,j);
				double radius=dis*std::cos(pitchrad64[i]*PI/180);
				double originx=radius*costable[j];
				double originy=radius*sintable[j];
				double originz=dis*std::sin(pitchrad64[i]*PI/180);
				Eigen::Matrix<double, 3, 1> originp;
				originp<<originx,originy,originz;

				double x=(T*originp)[0];
				double y=(T*originp)[1];
				double z=(T*originp)[2];

				if(x>-1.5&&x<1.5&&y<4&&y>-1){;}//
				else{
					heightmat_0.at<float>(i,j)=z;
					int col=boost::math::round((x+35)/0.2);//干！！！！！！！！！！！
					int row=boost::math::round((y+20)/0.2);
					int gridindex=0;
					unsigned char* ptr=gridshow_0.ptr<unsigned char>(GRIDWH-1-row);
					unsigned char* ptrshow=justshow.ptr<unsigned char>(GRIDWH-1-row);
					float* ptrheight=heightgrid_0.ptr<float>(row);
					float* ptrmin=heightgridmin_0.ptr<float>(row);
					float* ptrmax=heightgridmax_0.ptr<float>(row);
					unsigned char* testshow=heightshow.ptr<unsigned char>(GRIDWH-1-row);


					if(col<GRIDWH&&row<GRIDWH&&col>=0&&row>=0){
						ptr[3*col]=0;
						ptr[3*col+1]=255;
						ptr[3*col+2]=0;
						ptrshow[3*col+1]=255;
						if(ptrmin[col]>z){
							ptrmin[col]=z;
						}
						if(ptrmax[col]<z){
							ptrmax[col]=z;
							testshow[col]=255;

						}
						ptrheight[col]=ptrmax[col]-ptrmin[col];

					}
				}
			}
		}
		//输出一下高度
		//		for(int i=0;i<GRIDWH;++i){
		//			float* ptr=heightgridmax_0.ptr<float>(i);
		//			for(int j=0;j<GRIDWH;++j){
		//				if(heightgridmax_0.at<float>(i,j)!=-100||ptr[j]!=-100){
		//					cout<<heightgridmax_0.at<float>(i,j)-ptr[j]<<endl;
		//				}
		//			}
		//		}

		//开始检测
		//		cout<<"========================"<<endl;
		int rightend=(int)38/0.2;
		int leftend=(int)32/0.2;
		for(int j=0;j<870;j++){
			int myj;
			int countlaser=0;
			for(int i=58;i>15;i--){
				//			if(j==800){
				//				cout<<"height is "<<heightmat_0.at<float>(i,j)<<endl;
				//			}
				int index=i*870+j;
				int indexnext=(i-1)*870+j;
				double dis=(double)dismat_0.at<float>(i,j);
				double radius=dis*std::cos(pitchrad64[i]*PI/180);
				double originx=radius*costable[j];
				double originy=radius*sintable[j];
				double originz=dis*std::sin(pitchrad64[i]*PI/180);

				//计算起始点及前一点高度
				double height=heightmat_0.at<float>(i,j);
				double heightbefore=heightmat_0.at<float>(i+1,j);
				//当前线雷达与上一线雷达距离差
				double diff2=(double)dismat_0.at<float>(i,j)-(double)dismat_0.at<float>(i+1,j);
				double disnext=(double)dismat_0.at<float>(i-1,j);
				int count=0;
				while(disnext==0&&i>15){
					count++;
					i--;
					disnext=(double)dismat_0.at<float>(i-1,j);
				}
				//计算下一点及下下一点高度
				double heightnext=heightmat_0.at<float>(i-1,j);
				double heightnene=heightmat_0.at<float>(i-2,j);

				//下一线与当前线雷达距离差
				double diff1=disnext-dis;
				//				if(count>=2){
				//					break;
				//				}
				//到达3°后break
				if(i==15) break;

				double radiusnext=disnext*std::cos(pitchrad64[63-i+1]*PI/180);
				double originxnext=radiusnext*costable[j];
				double originynext=radiusnext*sintable[j];
				double originznext=disnext*std::sin(pitchrad64[63-i+1]*PI/180);



				Eigen::Matrix<double, 3, 1> originp;
				Eigen::Matrix<double, 3, 1> originpnext;
				Eigen::Matrix<double, 3, 1> originpvirtual;

				originp<<originx,originy,originz;
				originpnext<<originxnext,originynext,originznext;

				double x=(T*originp)[0];
				double y=(T*originp)[1];
				double z=(T*originp)[2];

				double xnext=(T*originpnext)[0];
				double ynext=(T*originpnext)[1];
				double znext=(T*originpnext)[2];
				//
				if(x>-1.5&&x<1.5&&y<4&&y>-1||xnext>-1.5&&xnext<1.5&&ynext<4&&ynext>-1) {
					continue;
				}
				//用于判断雷达点是否走出车身，可以正式开始算法
				countlaser++;
				//			heightmat_0.at<float>(i,j)=z;
				//			if(xnext>-1.2&&xnext<1.2&&ynext<4&&ynext>-1) continue;
				//			heightmat_0.at<float>(i,j)=z;



				double radiusvirtual=4.0;//虚拟半径，用于盲区检测
				double originxvirtual=radiusvirtual*costable[j];
				double originyvirtual=radiusvirtual*sintable[j];
				double originzvirtual=0;
				originpvirtual<<originxvirtual,originyvirtual,originzvirtual;

				double xvirtual=(T*originpvirtual)[0];
				double yvirtual=(T*originpvirtual)[1];



				int col=boost::math::round((x+35)/0.2);
				int row=boost::math::round((y+20)/0.2);
				int colvirtual;
				int rowvirtual;
				if(countlaser==1){
					colvirtual=boost::math::round((xvirtual+35)/0.2);
					rowvirtual=boost::math::round((yvirtual+20)/0.2);
					if(radius>7&&height<-1&&(colvirtual<32/0.2||colvirtual>38/0.2)){
						//					unsigned char* ptr=gridshow_0.ptr<unsigned char>(GRIDWH-1-rowvirtual);
						//					ptr[3*colvirtual]=0;
						//					ptr[3*colvirtual+1]=0;
						//					ptr[3*colvirtual+1]=255;
						cv::line(gridshow_0,cvPoint(col,GRIDWH-1-row),cvPoint(colvirtual,GRIDWH-1-rowvirtual),cvScalar(0,0,125));
						cv::line(justshow,cvPoint(col,GRIDWH-1-row),cvPoint(colvirtual,GRIDWH-1-rowvirtual),cvScalar(0,0,125));
					}
					//				第一条线自己给定初始diff2
					diff2=0.3;
				}
				int colnext=boost::math::round((xnext+35)/0.2);
				int rownext=boost::math::round((ynext+20)/0.2);


				int gridindex=0;
				unsigned char* ptr=gridshow_0.ptr<unsigned char>(GRIDWH-1-row);


				//下面是为了查看从当前点到下一个点之间是否有障碍物，所以延长了一段，水的倒影有可能在后面一点
				double radiuswater=(disnext)*std::cos(pitchrad64[63-i+1]*PI/180);
				double xwater=radiuswater*costable[j];
				double ywater=radiuswater*sintable[j];
				double zwater=(disnext)*std::sin(pitchrad64[63-i+1]*PI/180);
				Eigen::Matrix<double, 3, 1> pointwater;
				pointwater<<xwater,ywater,zwater;
				double xwater1=(T*pointwater)[0];
				double ywater1=(T*pointwater)[1];
				double zwater1=(T*pointwater)[2];

				int colwater=boost::math::round((xwater1+35)/0.2);
				int rowwater=boost::math::round((ywater1+20)/0.2);

				if(col<GRIDWH&&row<GRIDWH&&col>=0&&row>=0){
					//				增加不同半径下的不同高度阈值
					if(radius<3)break;
					if(radius<8){if(height>0.3)break;}
					else if(radius<13){if(height>0.5)break;}
					else{if(height>0.8)break;}
					unsigned char* ptrshow=justshow.ptr<unsigned char>(GRIDWH-1-row);
					if(col==35/0.2){
						ptrshow[3*col]=255;
						ptrshow[3*col+1]=255;
						ptrshow[3*col+2]=255;
					}

					//				针对远距离误检不同阈值
					if(1){
						//短距离用高度差和距离突变来定义
						if(radius<10){
							int thresh=6;
							if(col>155&&col<195) thresh=15;
							if(heightnext-height<-0.5&&diff1/diff2>thresh){//

								//					if((height-heightnext)/(radiusnext-radius)>0.1&&diff1/diff2>3){
								ptr[3*col]=0;
								ptr[3*col+1]=0;
								ptr[3*col+2]=255;
								//							cout<<radius<<"==================>"<<diff1/diff2<<endl;
								//排除径向距离比较小的
								if((colnext-col)*(colnext-col)*0.04+(rownext-row)*(rownext-row)*0.04<1){
#ifdef showgreen
									cv::line(gridshow_0,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext),cvScalar(0,150,0));
#endif
									continue;
								}
								//连线
								//							if(colnext<GRIDWH&&rownext<GRIDWH&&colnext>=0&&rownext>=0){
#ifdef lineiter
								cv::LineIterator it(gridshow_0,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext));
								it++;
								bool state=true;
								int pointcount=0;
								for(int i=0;i<it.count;i++,++it){
									//									if(i<it.count*0.2||i>it.count*0.8) continue;
									if((*it)[1]==255){
										pointcount++;
										int rows=GRIDWH-1-it.pos().y;
										int cols=it.pos().x;
										if((cols-col)*(cols-col)*0.04+(rows-row)*(rows-row)*0.04<1){
											continue;
										}
										(*it)[0]=255;
										(*it)[2]=255;
										//										cout<<row<<" "<<col<<" "<<rows<<" "<<cols<<endl;
										//										cout<<heightgrid_0.at<float>(row,col)<<"  "<<heightgrid_0.at<float>(rows,cols)<<endl;
										//用其实点处栅格最大高度减去终点处最小高度
										if(heightgridmax_0.at<float>(row,col)-heightgridmin_0.at<float>(rows,cols)<0.5){
											state=false;
											break;
										}
									}
									//									(*it)[2]=125;
								}
								if(!state) continue;
#endif
#ifdef USENEIGHBER
								int pointcount=0;
								int zerocount=0;
								for(int l=-2;l<=1;l++){
									for(int k=-WINDOW;k<WINDOW+1;k++){
										//									CONDITION(j){
										//										cout<<dismat_0.at<float>(i+1,j+k)/500<<" "<<
										//												dismat_0.at<float>(i,j+k)/500<<"  "<<dismat_0.at<float>(i-1,j)/500
										//												<<"  "<<dismat_0.at<float>(i-2,j+k)/500<<" "
										//												<<dismat_0.at<float>(i-3,j)/500<<endl;
										//									}
										if(l==0||l==1){
											if(dismat_0.at<float>(i+l,j+k)==0){
												zerocount++;
											}
										}
										if(l==-2){
											if((dismat_0.at<float>(i-1,j)-dismat_0.at<float>(i+l,j+k))/dismat_0.at<float>(i-1,j)>HEIGHTTHRESH){
												pointcount++;
											}
										}
									}
								}
								CONDITION(j){
									cout<<j<<"  "<<pointcount<<" "<<zerocount<<endl;
									//								cout<<j<<"  "<<radius<<"  "<<radiusnext<<endl;
								}
								if(pointcount>COUNTTHRESH||zerocount>ZEROCOUNTTH){
#ifdef showgreen
									cv::line(gridshow_0,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext),cvScalar(0,150,0));
#endif
									continue;
								}
#endif
								//								vector<double> temp;
								//								temp.clear();
								//								for(int i=-1;i<2;++i){
								//									for(int j=-1;j<2;++j){
								////										cout<<rownext<<"  "<<colnext<<endl;
								//										if(rownext+i<351&&colnext+j<351){
								//											temp.push_back(heightgridmax_0.at<float>(rownext+i,colnext+j));
								//										}
								//									}
								//								}
								//								sort(temp.begin(),temp.end());
								//								if(temp.size()==0)continue;
								cv::LineIterator it(heightgridmax_0,cvPoint(col,GRIDWH-1-row),cvPoint(colwater,GRIDWH-1-rowwater));
								//								cout<<"the heights are......"<<endl;
								double hm=-100;
								for(int k=0;k<it.count;++k,it++){
									int x=it.pos().x;
									int y=it.pos().y;
									y=GRIDWH-1-y;
									for(int i=-5;i<6;++i){
										for(int j=-5;j<6;++j){
											double height=0;
											if(x+i>=0&&x+i<GRIDWH&&y+j>=0&&y+j<GRIDWH){
												float* ptr=heightgridmax_0.ptr<float>(y+j);
												height=(double)ptr[x+i];
											}
											if(height>hm) hm=height;
										}
									}
								}
								//								cout<<hm<<endl;
								if(hm<0.4){//temp[temp.size()-1]<1&&temp[temp.size()-1]!=-100
									//									cout<<"the height is "<<temp[temp.size()-1]<<endl;
									//									cout<<hm<<endl;
									//									cv::line(gridshow_0,cvPoint(col,GRIDWH-1-row),cvPoint(colwater,GRIDWH-1-rowwater),cvScalar(0,125,125));
									cv::line(justshow,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext),cvScalar(0,0,125));
									cv::line(gridshow_0,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext),cvScalar(0,0,125));
								}
								//							}

							}
							else{
#ifdef showgreen
								cv::line(gridshow_0,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext),cvScalar(0,150,0));
#endif
							}
						}
						//大于10m小于17m需要加上颠簸程度、俯仰角、（起始点高度？--这个还不太确定）、正切值、最小高度、距离突变比
						//tmpmsg->pitch<5
						else if(radius<30){
							int thresh=5;
							if(col>155&&col<195) thresh=15;
							if(1){//&&height>-0.4 &&yprdiff<7
								//					if((heightmat_0.at<float>(i-1,j)-heightmat_0.at<float>(i,j))<-1&&diff1/diff2>3){
								if((height-heightnext)/(radiusnext-radius)>0.07
										&&heightnext-height<-1&&diff1/diff2>thresh){//
									ptr[3*col]=0;
									ptr[3*col+1]=0;
									ptr[3*col+2]=255;

									//排除径向距离比较小的
									if((colnext-col)*(colnext-col)*0.04+(rownext-row)*(rownext-row)*0.04<1){
#ifdef showgreen
										cv::line(gridshow_0,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext),cvScalar(0,150,0));
#endif
										continue;
									}
									//连线
									//								if(colnext<GRIDWH&&rownext<GRIDWH&&colnext>=0&&rownext>=0){
#ifdef lineiter
									cv::LineIterator it(gridshow_0,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext));
									it++;
									bool state=true;
									int pointcount=0;
									for(int i=0;i<it.count;i++,++it){
										//										if(i<it.count*0.2||i>it.count*0.8) continue;
										if((*it)[1]==255){
											//											pointcount++;
											int rows=GRIDWH-1-it.pos().y;
											int cols=it.pos().x;
											if((cols-col)*(cols-col)*0.04+(rows-row)*(rows-row)*0.04<1){
												continue;
											}
											(*it)[0]=255;
											(*it)[2]=255;
											//											cout<<row<<" "<<col<<" "<<rows<<" "<<cols<<endl;
											//											cout<<heightgrid_0.at<float>(row,col)<<"  "<<heightgrid_0.at<float>(rows,cols)<<endl;
											if(heightgridmax_0.at<float>(row,col)-heightgridmin_0.at<float>(rows,cols)<0.5){
												state=false;
												break;
											}
										}
										//									(*it)[2]=125;
									}
									if(!state) continue;
#endif
#ifdef USENEIGHBER
									int pointcount=0;
									int zerocount=0;
									for(int l=-2;l<=1;l++){
										for(int k=-WINDOW;k<WINDOW+1;k++){
											//										CONDITION(j){
											//											cout<<dismat_0.at<float>(i+1,j+k)/500<<" "<<
											//													dismat_0.at<float>(i,j+k)/500<<"  "<<dismat_0.at<float>(i-1,j)/500
											//													<<"  "<<dismat_0.at<float>(i-2,j+k)/500<<" "
											//													<<dismat_0.at<float>(i-3,j)/500<<endl;
											//										}
											if(l==0||l==1){
												if(dismat_0.at<float>(i+l,j+k)==0){
													zerocount++;
												}
											}
											if(l==-2){
												if((dismat_0.at<float>(i-1,j)-dismat_0.at<float>(i+l,j+k))/dismat_0.at<float>(i-1,j)>HEIGHTTHRESH){
													pointcount++;
												}
											}
										}
									}
									CONDITION(j){
										cout<<j<<"  "<<pointcount<<" "<<zerocount<<endl;
										//								cout<<j<<"  "<<radius<<"  "<<radiusnext<<endl;
									}
									if(pointcount>COUNTTHRESH||zerocount>ZEROCOUNTTH){

#ifdef showgreen
										cv::line(gridshow_0,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext),cvScalar(0,150,0));
#endif
										continue;
									}
#endif
									//									vector<double> temp;
									//									temp.clear();
									//									for(int i=-1;i<2;++i){
									//										for(int j=-1;j<2;++j){
									////											cout<<rownext<<"  "<<colnext<<endl;
									//											if(rownext+i<351&&colnext+j<351){
									//												temp.push_back(heightgridmax_0.at<float>(rownext+i,colnext+j));
									//											}
									//										}
									//									}
									//									if(temp.size()==0)continue;
									//									sort(temp.begin(),temp.end());
									cv::LineIterator it(heightgridmax_0,cvPoint(col,GRIDWH-1-row),cvPoint(colwater,GRIDWH-1-rowwater));
									//								cout<<"the heights are......"<<endl;
									double hm=-100;
									for(int k=0;k<it.count;++k,it++){
										int x=it.pos().x;
										int y=it.pos().y;
										y=GRIDWH-y-1;
										for(int i=-5;i<6;++i){
											for(int j=-5;j<6;++j){
												double height=0;
												if(x+i>=0&&x+i<GRIDWH&&y+j>=0&&y+j<GRIDWH){
													float* ptr=heightgridmax_0.ptr<float>(y+j);
													height=(double)ptr[x+i];
												}
												if(height>hm) hm=height;
											}
										}
										//									if(height!=(-100)&&height!=0){
										//										cout<<height<<" ";
										//									}
									}
									//									cout<<hm<<endl;
									if(hm<0.4){//temp[temp.size()-1]<1&&temp[temp.size()-1]!=-100
										//										cout<<"the height is "<<temp[temp.size()-1]<<endl;

										//										cout<<hm<<endl;
										//										cv::line(gridshow_0,cvPoint(col,GRIDWH-1-row),cvPoint(colwater,GRIDWH-1-rowwater),cvScalar(0,125,125));
										cv::line(justshow,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext),cvScalar(0,0,125));
										cv::line(gridshow_0,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext),cvScalar(0,0,125));
									}
									//								}

								}
								else{
#ifdef showgreen
									cv::line(gridshow_0,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext),cvScalar(0,150,0));
#endif
								}
							}
						}
					}
				}


				//把水的可疑区域画出来
				cv::LineIterator it(heightgridmax_0,cvPoint(col,GRIDWH-1-row),cvPoint(colwater,GRIDWH-1-rowwater));
				//								cout<<"the heights are......"<<endl;
				if(count>2){
					double hm=-100;
					for(int k=0;k<it.count;++k,it++){
						int x=it.pos().x;
						int y=it.pos().y;
						y=GRIDWH-1-y;
						for(int i=-2;i<3;++i){
							for(int j=-2;j<3;++j){
								double height=0;
								if(x+i>=0&&x+i<GRIDWH&&y+j>=0&&y+j<GRIDWH){
									float* ptr=heightgridmax_0.ptr<float>(y+j);
									height=(double)ptr[x+i];
								}
								if(height>hm) hm=height;
							}
						}
					}
					if(hm<0.4&&colnext>0&&colnext<GRIDWH
							&&rownext>0&&rownext<GRIDWH&&disnext<30){//&&i>20
						cv::line(gridshow_0,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext),cvScalar(125,125,125));
						cv::line(justshow,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext),cvScalar(125,125,125));

					}
				}
				////把水的可疑区域画出来
			}
		}


		//发送消息
		stiff_msgs::stiffwater msg_send;
		msg_send.header.stamp=heightmapmsg->header.stamp;
		msg_send.header.frame_id="stiffwater";
		msg_send.ogmheight=351;
		msg_send.ogmwidth=201;
		msg_send.resolution=0.2;
		msg_send.vehicle_x=100;
		msg_send.vehicle_y=100;
		cv::Mat gridall = cv::Mat::zeros(GRIDWH,GRIDWH,CV_8UC1);
		for(int row=0;row<GRIDWH;row++){
			unsigned char* ptrall=gridall.ptr<unsigned char>(GRIDWH-1-row);
			unsigned char* ptr=gridshow_0.ptr<unsigned char>(GRIDWH-1-row);
			for(int col=0;col<GRIDWH;col++){
				if(ptr[3*col+2]==125&&ptr[3*col]==0){//悬崖区域
					ptrall[col]=255;
				}
				else if(ptr[3*col+2]==125&&ptr[3*col+1]==125&&ptr[3*col]==125){//水区域
					ptrall[col]=200;
				}
				else if(ptr[3*col+1]==150){//通行区域
					ptrall[col]=100;
				}
			}
		}
		cv::dilate(gridall,gridall,elementdil);
		cv::erode(gridall,gridall,elementero);
		//send
		int max_cnt=-10;
		for(int row=0;row<351;row++){
			unsigned char* ptr=gridall.ptr<unsigned char>(GRIDWH-1-row);
			for(int col=0;col<201;col++){
				int index=row*201+col;
				if(ptr[col+75]==255){
					msg_send.data.push_back(5);
				}
				else if(ptr[col+75]==200){
					if(send_water){
						msg_send.data.push_back(6);
					}
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

				if(col>75&&col<125&&row>100&&row<175){//车身两侧五米车前十五米看有没有水，
					int count=0;
					for(int windowi=0;windowi<10&&windowi+row<351;++windowi){
						unsigned char* ptrwater=gridall.ptr<unsigned char>(GRIDWH-1-row-windowi);
						for(int windowj=0;windowj<10;++windowj){
							if(ptrwater[col+75+windowj]==200){
								count++;
							}
						}
					}
					if(count>max_cnt){
						max_cnt=count;
					}
				}
			}
		}
		if(max_cnt>70){
			msg_send.havewater=1;
		}else{
			msg_send.havewater=0;
		}
		//		cout<<"water max_cnt is ==================== "<<max_cnt<<endl;
		pubStiffwaterOgm.publish(msg_send);
		if(visual_on){
			cv::imshow("show",gridall);
			cv::imshow("grid",gridshow_0);
			cv::imshow("justshow",justshow);
		}
		cv::waitKey(10);
	}
}
#endif

void lidar_handler(const sensor_msgs::PointCloud2ConstPtr& cloudmsg){
	mtx_cloud.lock();
	lidarCloudMsgs_ = cloudmsg;
	mtx_cloud.unlock();
	//	pcl::PointCloud<pcl::PointXYZI>::Ptr tempcloud(new pcl::PointCloud<pcl::PointXYZI>);//当前帧点云（雷达里程计坐标系）
	//	pcl::fromROSMsg(*cloudmsg, *tempcloud);//获取当前帧点云数据
	//	std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> outputclouds;//多个激光雷达数据包，向量中每个元素为一个激光雷达一帧数据
	//	std::vector<pcl::PointXYZI> lidarpropertys;//每一个PointType类型都表示一个单独点
	//	analysisCloud(tempcloud,outputclouds,lidarpropertys);
	//	pcl::PointCloud<pcl::PointXYZI>::Ptr left_high_cloud(new pcl::PointCloud<pcl::PointXYZI>);
	//	pcl::PointCloud<pcl::PointXYZI>::Ptr left_low_cloud(new pcl::PointCloud<pcl::PointXYZI>);
	//	pcl::PointCloud<pcl::PointXYZI>::Ptr right_high_cloud(new pcl::PointCloud<pcl::PointXYZI>);
	//	pcl::PointCloud<pcl::PointXYZI>::Ptr right_low_cloud(new pcl::PointCloud<pcl::PointXYZI>);
	//	const int layer = 16;
	//	const int round = outputclouds[1]->points.size() / layer;
	//	const int window_big = 50;
	//	const int window_small = 10;
	//	{//test
	//		//		for(int j = 0; j < layer; ++j){
	//		//			for(int i = 0; i < round; ++i){
	//		//				int index = i * layer + j;
	//		//				auto pt = outputclouds[1]->points[index];
	//		//				if(abs(pt.z - 0.1) < 0.0001){
	//		//					std::cout << pt.z << " " << pt.x << " "
	//		//							<< pt.y << " " << pt.range << std::endl;
	//		//					color->points.push_back(pt);
	//		//				}else{
	//		////					colorb->points.push_back(pt);
	//		//				}
	//		//			}
	//		//		}
	//	}
	//	for(int j = 0; j < layer; ++j){
	//		for(int i = 0; i + window_big < round; ){
	//			//			int index =i * layer + j;
	//			//			single->points.push_back(outputclouds[2]->points[index]);
	//
	//			{//考虑距离突变
	//				//				int index = i * layer + j;
	//				//				float  z = outputclouds[2]->points[index].z;
	//				//				float  x = outputclouds[2]->points[index].x;
	//				//				float  y = outputclouds[2]->points[index].y;
	//				//				float dis = sqrt(x*x + y*y + z*z);
	//				//
	//				//
	//				//				int index1 = (i + 1) * layer + j;
	//				//				float  z1 = outputclouds[2]->points[index1].z;
	//				//				float  x1 = outputclouds[2]->points[index1].x;
	//				//				float  y1 = outputclouds[2]->points[index1].y;
	//				//				float dis1 = sqrt(x1*x1 + y1*y1 + z1*z1);
	//				//				if(dis1 / dis > 1.1 && dis > 4 && dis < 40 && dis1 > 4 && dis1 < 40)
	//				//				{
	//				//					//				std::cout << dis1 / dis << " " << dis1 << "  " << dis << std::endl;
	//				//					//				single->points.push_back(outputclouds[2]->points[index]);
	//				//					//				single->points.push_back(outputclouds[2]->points[index1]);
	//				//				}
	//			}
	//			float  height_diff_most = 10;
	//			int i_begin = 0;
	//			for(int k = i; k + window_small < i + window_big; ++k){
	//				float z_high = 0;
	//				float z_low = 0;
	//				int count = 0;
	//				for(int window_i = k; window_i < k + window_small; window_i++){
	//					int index = window_i * layer + j;
	//					float  z = outputclouds[1]->points[index].z;
	//					float  x = outputclouds[1]->points[index].x;
	//					float  y = outputclouds[1]->points[index].y;
	//					float dis = sqrt(x*x + y*y + z*z);
	//					if(abs(z - 0.1) < 0.00001 || dis > 40 || z > 3 || z < -3 || y < 0) continue;
	//					count ++;
	//					z_high += z;
	//				}
	//				if(count > 0) {
	//					z_high = z_high / count;
	//					count = 0;
	//				}
	//				for(int window_i = k + window_small; window_i < k + 2 * window_small; window_i++){
	//					int index = window_i * layer + j;
	//					float  z = outputclouds[1]->points[index].z;
	//					float  x = outputclouds[1]->points[index].x;
	//					float  y = outputclouds[1]->points[index].y;
	//					float dis = sqrt(x*x + y*y + z*z);
	//					if(abs(z - 0.1) < 0.00001 || dis > 40 || z > 3 || z < -3 || y < 0) continue;
	//					count ++;
	//					z_low += z;
	//				}
	//				if(count > 0) {
	//					z_low = z_low / count;
	//				}
	//				float height_diff = 0;
	//				if(abs(z_high) > 0.0001 && abs(z_low) > 0.0001){
	//					height_diff =  z_low - z_high;
	//				}
	//				if(height_diff < - 0.6 && height_diff < height_diff_most){
	//					height_diff_most = height_diff;
	//					i_begin = k;
	//				}
	//				if(abs(height_diff + 0.863364) < 0.000001){
	//					std::cout << std::endl << " there it is -------- " << height_diff << std::endl;
	//					for(int l = k; l <  k + window_small; ++l){
	//						int index_front = l * layer + j;
	//						int index_back = (l + window_small) * layer + j;
	//						auto pt1 = outputclouds[1]->points[index_front];
	//						auto pt2 = outputclouds[1]->points[index_back];
	//						left_high_cloud->points.push_back(pt1);
	//						left_low_cloud->points.push_back(pt2);
	//						std::cout << pt1.z << "   " << pt2.z << std::endl;
	//					}
	//				}
	//			}
	//			if(i_begin != 0){
	//				//				std::cout << "height_diff_most ------- ：" << height_diff_most << " --- "
	//				//						<< i_begin << std::endl;
	//				for(int k = i_begin; k <  i_begin + window_small; ++k){
	//					int index_front = k * layer + j;
	//					int index_back = (k + window_small) * layer + j;
	//					auto pt1 = outputclouds[1]->points[index_front];
	//					auto pt2 = outputclouds[1]->points[index_back];
	//					left_high_cloud->points.push_back(pt1);
	//					left_low_cloud->points.push_back(pt2);
	//					//					std::cout << pt1.z << "   " << pt2.z << std::endl;
	//				}
	//				i_begin = 0;
	//				height_diff_most = 10;
	//			}
	//			height_diff_most = 10;
	//			i_begin = 0;
	//			//right lidar
	//			for(int k = i; k + window_small < i + window_big; ++k){
	//				float z_high = 0;
	//				float z_low = 0;
	//				int count = 0;
	//				for(int window_i = k; window_i < k + window_small; window_i++){
	//					int index = window_i * layer + j;
	//					float  z = outputclouds[2]->points[index].z;
	//					float  x = outputclouds[2]->points[index].x;
	//					float  y = outputclouds[2]->points[index].y;
	//					float dis = sqrt(x*x + y*y + z*z);
	//					//todo: z < -3 is not a good condition
	//					if(abs(z - 0.1) < 0.00001 || dis > 40 || z > 3 || z < -3 || y < 0) continue;
	//					count ++;
	//					z_low += z;
	//				}
	//				if(count > 0) {
	//					z_low = z_low / count;
	//					count = 0;
	//				}
	//				for(int window_i = k + window_small; window_i < k + 2 * window_small; window_i++){
	//					int index = window_i * layer + j;
	//					float  z = outputclouds[2]->points[index].z;
	//					float  x = outputclouds[2]->points[index].x;
	//					float  y = outputclouds[2]->points[index].y;
	//					float dis = sqrt(x*x + y*y + z*z);
	//					if(abs(z - 0.1) < 0.00001 || dis > 40 || z > 3 || z < -3 || y < 0) continue;
	//					count ++;
	//					z_high += z;
	//				}
	//				if(count > 0) {
	//					z_high = z_high / count;
	//				}
	//				float height_diff = 0;
	//				if(abs(z_high) > 0.0001 && abs(z_low) > 0.0001){
	//					height_diff =  z_low - z_high;
	//				}
	//				if(height_diff < - 0.6 && height_diff < height_diff_most){
	//					height_diff_most = height_diff;
	//					i_begin = k;
	//				}
	//			}
	//			if(i_begin != 0){
	//				//				std::cout << "height_diff_most ------- ：" << height_diff_most << " --- "
	//				//						<< i_begin << std::endl;
	//				for(int k = i_begin; k <  i_begin + window_small; ++k){
	//					int index_low = k * layer + j;
	//					int index_high = (k + window_small) * layer + j;
	//					auto pt_low = outputclouds[2]->points[index_low];
	//					auto pt_high = outputclouds[2]->points[index_high];
	//					right_high_cloud->points.push_back(pt_high);
	//					right_low_cloud->points.push_back(pt_low);
	//					//					std::cout << pt1.z << "   " << pt2.z << std::endl;
	//				}
	//				i_begin = 0;
	//				height_diff_most = 10;
	//			}
	//			i += window_big;
	//		}
	//	}
	//	cloud_viewer_->removeAllPointClouds();
	//
	//	if ( outputclouds[1]->size() > 0 )
	//	{
	//
	//		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> cloudHandler( outputclouds[1], 0, 255, 0 );
	//		if (!cloud_viewer_->updatePointCloud(outputclouds[1],cloudHandler, "left"))
	//		{
	//			//				std::cout<<"cloudgeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeet"<<std::endl;
	//			cloud_viewer_->addPointCloud(outputclouds[1], cloudHandler, "left");
	//		}
	//	}
	//	if ( outputclouds[2]->size() > 0 )
	//	{
	//
	//		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> cloudHandler( outputclouds[2], 0, 255, 0 );
	//		if (!cloud_viewer_->updatePointCloud(outputclouds[2],cloudHandler, "right"))
	//		{
	//			//				std::cout<<"cloudgeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeet"<<std::endl;
	//			cloud_viewer_->addPointCloud(outputclouds[2], cloudHandler, "right");
	//		}
	//	}
	//	if ( right_high_cloud->size() > 0 )
	//	{
	//		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> cloudHandler( right_high_cloud, 255, 0, 0 );
	//		if (!cloud_viewer_->updatePointCloud(right_high_cloud,cloudHandler, "color"))
	//		{
	//			//				std::cout<<"cloudgeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeet"<<std::endl;
	//			cloud_viewer_->addPointCloud(right_high_cloud, cloudHandler, "color");
	//		}
	//	}
	//	if ( right_low_cloud->size() > 0 ){
	//		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> cloudHandler( right_low_cloud, 0, 0, 255 );
	//		if (!cloud_viewer_->updatePointCloud(right_low_cloud,cloudHandler, "colorb"))
	//		{
	//			//				std::cout<<"cloudgeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeet"<<std::endl;
	//			cloud_viewer_->addPointCloud(right_low_cloud, cloudHandler, "colorb");
	//		}
	//	}
	//	if ( left_high_cloud->size() > 0 )
	//	{
	//
	//		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> cloudHandler( left_high_cloud, 255, 0, 0 );
	//		if (!cloud_viewer_->updatePointCloud(left_high_cloud,cloudHandler, "stiff"))
	//		{
	//			//				std::cout<<"cloudgeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeet"<<std::endl;
	//			cloud_viewer_->addPointCloud(left_high_cloud, cloudHandler, "stiff");
	//		}
	//	}
	//	if ( left_low_cloud->size() > 0 )
	//	{
	//
	//		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> cloudHandler( left_low_cloud, 0, 0, 255 );
	//		if (!cloud_viewer_->updatePointCloud(left_low_cloud,cloudHandler, "stiff_back"))
	//		{
	//			//				std::cout<<"cloudgeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeet"<<std::endl;
	//			cloud_viewer_->addPointCloud(left_low_cloud, cloudHandler, "stiff_back");
	//		}
	//	}
	//	cloud_viewer_->spinOnce();
}
void sendcallback(const lanelet_map_msgs::Way msg){
	if(msg.open_water_det==1){
		send_water=true;
	}
}
void coordinate_from_vehicle_to_velodyne(float x, float y , float z, float& newx, float& newy, float& newz)
{
	newz = z;
	newx = x;
	newy = y;
}
void prepare_viewer()
{
#ifdef VIEWER
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
			pt1.x = min(newx1 , newx2) ;
			pt1.y = min(newy1 , newy2) ;
			pt1.z = newz;
			pt2.x = max(newx1 , newx2) ;
			pt2.y = max(newy1 , newy2) ;
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
			pt1.x = min(newx1 , newx2) ;
			pt1.y = min(newy1 , newy2) ;
			pt1.z = newz;
			pt2.x = max(newx1 , newx2) ;
			pt2.y = max(newy1 , newy2) ;
			pt2.z = newz;
			memset(linename, 0 , 20);
			sprintf(linename , "lng%02d" , i);
			cloud_viewer_->addLine(pt1, pt2, linename);
		}
	}
#endif //VIEWER
#ifdef VIEWER_ORI
	cloud_viewer_origin->addCoordinateSystem (3.0);
	cloud_viewer_origin->setBackgroundColor (0, 0, 0);
	cloud_viewer_origin->initCameraParameters ();
	cloud_viewer_origin->setCameraPosition (0.0, 0.0, 30.0, 0.0, 1.0, 0.0, 0);
	cloud_viewer_origin->setCameraClipDistances (0.0, 100.0);
	//	cloud_viewer_origin->registerKeyboardCallback (&LidarProcess::keyboard_callback, *this);
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
		cloud_viewer_origin->addLine(pt1, pt2, "body1");

		pt1.x = newx2 ;
		pt1.y = newy2 ;
		pt1.z = newz;
		cloud_viewer_origin->addLine(pt1, pt2, "body2");

		pt2.x = newx2 ;
		pt2.y = newy1 ;
		pt2.z = newz;
		cloud_viewer_origin->addLine(pt1, pt2, "body3");

		pt1.x = newx1 ;
		pt1.y = newy1 ;
		pt1.z = newz;
		cloud_viewer_origin->addLine(pt1, pt2, "body4");



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
			cloud_viewer_origin->addLine(pt1, pt2, "upper1");

			pt1.x = newx2 ;
			pt1.y = newy2 ;
			pt1.z = newz;
			cloud_viewer_origin->addLine(pt1, pt2, "upper2");

			pt2.x = newx2 ;
			pt2.y = newy1 ;
			pt2.z = newz;
			cloud_viewer_origin->addLine(pt1, pt2, "upper3");

			pt1.x = newx1 ;
			pt1.y = newy1 ;
			pt1.z = newz;
			cloud_viewer_origin->addLine(pt1, pt2, "upper4");
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
			pt1.x = min(newx1 , newx2) ;
			pt1.y = min(newy1 , newy2) ;
			pt1.z = newz;
			pt2.x = max(newx1 , newx2) ;
			pt2.y = max(newy1 , newy2) ;
			pt2.z = newz;
			memset(linename, 0 , 20);
			sprintf(linename , "lat%02d" , i);
			cloud_viewer_origin->addLine(pt1, pt2, linename);
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
			pt1.x = min(newx1 , newx2) ;
			pt1.y = min(newy1 , newy2) ;
			pt1.z = newz;
			pt2.x = max(newx1 , newx2) ;
			pt2.y = max(newy1 , newy2) ;
			pt2.z = newz;
			memset(linename, 0 , 20);
			sprintf(linename , "lng%02d" , i);
			cloud_viewer_origin->addLine(pt1, pt2, linename);
		}
	}
#endif //VIEWER_ORI
}
int main(int argc, char** argv)
{
	ros::init(argc, argv, "stiff_depth");
	ros::NodeHandle nh;

	google::ParseCommandLineFlags(&argc, &argv, true);
	google::InitGoogleLogging(argv[0]);
	google::InstallFailureSignalHandler();
	FLAGS_colorlogtostderr = true; //设置输出到屏幕的日志显示相应颜色

	ros::param::get("~visulization",visual_on);
	visual_on = true;
	for(int j=0;j<870;j++){
		double angle=(j*360.0/870);
		sintable[j]=std::sin(angle*PI/180);
		costable[j]=std::cos(angle*PI/180);
	}
#ifdef VIEWER
	prepare_viewer();
#endif //VIEWER
	pubStiffwaterOgm=nh.advertise<stiff_msgs::stiffwater> ("stiffwaterogm",20);
#ifdef TOYOTA
	double zangle = 0,yangle =0.3288*PI/180,xangle = -2.6254*PI/180;//0
	//		double zangle = -88.4676*PI/180,yangle =-3.2*PI/180,xangle =-1*PI/180;//my
	//EulerAngles to RotationMatrix
	::Eigen::Vector3d ea0(zangle,yangle,xangle);

	::Eigen::Matrix3d R;

	R = ::Eigen::AngleAxisd(ea0[0], ::Eigen::Vector3d::UnitZ())
	* ::Eigen::AngleAxisd(ea0[1], ::Eigen::Vector3d::UnitY())
	* ::Eigen::AngleAxisd(ea0[2], ::Eigen::Vector3d::UnitX());


	T.rotate ( R );                                        // 按照rotation_vector进行旋转
	T.pretranslate ( Eigen::Vector3d (0,1, 2.14) );
	if(visual_on){
		ros::Subscriber subLaserCloudFullRes_ = nh.subscribe("lidar_cloud_calibrated", 1,showpointcloud);
	}
	ros::Subscriber subHeightMap = nh.subscribe("lidar_height_map",1,callback);
	ros::Subscriber subwatersend = nh.subscribe("topology_global_path",1,sendcallback);
	//	ros::Subscriber subGpsdata=nh.subscribe("gpsdata",10,gpsdatacllbak);
	std::thread measurement_process{process};
#endif

#ifdef SIXT
	double zangle = -92.6382*PI/180,yangle =  -0.8687*PI/180,xangle =  0.5211*PI/180;//0
	double zangle1 = 179.9995*PI/180,yangle1 =-1.1592*PI/180,xangle1 =0.6678*PI/180;//1
	//EulerAngles to RotationMatrix
	::Eigen::Vector3d ea0(zangle,yangle,xangle);
	::Eigen::Vector3d ea01(zangle1,yangle1,xangle1);

	::Eigen::Matrix3d R;
	::Eigen::Matrix3d R1;

	R = ::Eigen::AngleAxisd(ea0[0], ::Eigen::Vector3d::UnitZ())
	* ::Eigen::AngleAxisd(ea0[1], ::Eigen::Vector3d::UnitY())
	* ::Eigen::AngleAxisd(ea0[2], ::Eigen::Vector3d::UnitX());

	R1= ::Eigen::AngleAxisd(ea01[0], ::Eigen::Vector3d::UnitZ())
	* ::Eigen::AngleAxisd(ea01[1], ::Eigen::Vector3d::UnitY())
	* ::Eigen::AngleAxisd(ea01[2], ::Eigen::Vector3d::UnitX());

	T.rotate ( R );                                        // 按照rotation_vector进行旋转
	T.pretranslate ( Eigen::Vector3d (-1.3790,1.4331,1.4548) );//0
	T1.rotate(R1);
	T1.pretranslate ( Eigen::Vector3d ( 1.3749,1.4186, 1.4810) );//1

	ros::Subscriber subLaserCloudFullRes_ = nh.subscribe("lidar_cloud_calibrated", 1,showpointcloud);
	ros::Subscriber subHeightMap = nh.subscribe("lidar_height_map",1,callback);
	ros::Subscriber subGpsdata=nh.subscribe("gpsdata",10,gpsdatacllbak);
#endif
#ifdef SIXT64
	double zangle = 0,yangle =-0.0180*PI/180,xangle = -1.1237*PI/180;//0
	//EulerAngles to RotationMatrix
	::Eigen::Vector3d ea0(zangle,yangle,xangle);
	::Eigen::Matrix3d R;
	R = ::Eigen::AngleAxisd(ea0[0], ::Eigen::Vector3d::UnitZ())
	* ::Eigen::AngleAxisd(ea0[1], ::Eigen::Vector3d::UnitY())
	* ::Eigen::AngleAxisd(ea0[2], ::Eigen::Vector3d::UnitX());
	T.rotate ( R );                                        // 按照rotation_vector进行旋转
	T.pretranslate ( Eigen::Vector3d (0,1.8, 1.7465) );
	if(visual_on){
		ros::Subscriber subLaserCloudFullRes_ = nh.subscribe("lidar_cloud_calibrated", 1,showpointcloud);
	}
	ros::Subscriber subHeightMap = nh.subscribe("lidar_height_map",1,callback);
	ros::Subscriber subwatersend = nh.subscribe("topology_global_path",1,sendcallback);
	//	ros::Subscriber subGpsdata=nh.subscribe("gpsdata",10,gpsdatacllbak);
	std::thread measurement_process{process};
#endif
#ifdef BEIQI
	double zangle = -0.9557*PI/180, yangle = 0.7721*PI/180,xangle = -3.1274*PI/180;//0
	//EulerAngles to RotationMatrix
	::Eigen::Vector3d ea0(zangle,yangle,xangle);
	::Eigen::Matrix3d R;
	R = ::Eigen::AngleAxisd(ea0[0], ::Eigen::Vector3d::UnitZ())
	* ::Eigen::AngleAxisd(ea0[1], ::Eigen::Vector3d::UnitY())
	* ::Eigen::AngleAxisd(ea0[2], ::Eigen::Vector3d::UnitX());
	T.rotate ( R );                                        // 按照rotation_vector进行旋转
	T.pretranslate (Eigen::Vector3d(0,1.80,2.2));//0

	ros::Subscriber subLaserCloudFullRes_ = nh.subscribe("lidar_cloud_calibrated", 1,showpointcloud);
	ros::Subscriber subHeightMap = nh.subscribe("lidar_height_map",1,callback);
	ros::Subscriber subGpsdata=nh.subscribe("gpsdata",10,gpsdatacllbak);
#endif
#ifdef SIXT_RS
	double zangle = -1*PI/180, yangle = 0.3266*PI/180,xangle = -1.9278*PI/180;//0
	//EulerAngles to RotationMatrix
	::Eigen::Vector3d ea0(zangle,yangle,xangle);
	::Eigen::Matrix3d R;
	R = ::Eigen::AngleAxisd(ea0[0], ::Eigen::Vector3d::UnitZ())
	* ::Eigen::AngleAxisd(ea0[1], ::Eigen::Vector3d::UnitY())
	* ::Eigen::AngleAxisd(ea0[2], ::Eigen::Vector3d::UnitX());
	T.rotate ( R );                                        // 按照rotation_vector进行旋转
	T.pretranslate (Eigen::Vector3d(0,1.80,1.64));//0



	ros::Subscriber subLaserCloudFullRes_ = nh.subscribe("lidar_cloud_calibrated", 1,showpointcloud);
	ros::Subscriber subHeightMap = nh.subscribe("lidar_height_map",1,callback);
	ros::Subscriber subGpsdata=nh.subscribe("gpsdata",10,gpsdatacllbak);
#endif
#ifdef TOYOTA_RS
	double zangle = 0*PI/180, yangle = 0.4596*PI/180,xangle = -2.7206*PI/180;//0
	//EulerAngles to RotationMatrix
	::Eigen::Vector3d ea0(zangle,yangle,xangle);

	::Eigen::Matrix3d R;

	R = ::Eigen::AngleAxisd(ea0[0], ::Eigen::Vector3d::UnitZ())
	* ::Eigen::AngleAxisd(ea0[2], ::Eigen::Vector3d::UnitX())
	* ::Eigen::AngleAxisd(ea0[1], ::Eigen::Vector3d::UnitY());


	T.rotate ( R );                                        // 按照rotation_vector进行旋转
	T.pretranslate (Eigen::Vector3d(0,1.15,2.331));//0
	std::cout << std::endl << "main=======================================================" << std::endl;
	std::cout << "=======================================================" << std::endl;
	std::cout << "=======================================================" << std::endl;
	std::cout << R << std::endl;

	ros::Subscriber subLaserCloudFullRes_ = nh.subscribe("lidar_cloud_calibrated", 1,lidar_handler);
	ros::Subscriber subHeightMap = nh.subscribe("lidar_height_map",1,callback);
	ros::Subscriber subGpsdata=nh.subscribe("gpsdata",10,gpsdatacllbak);
#endif
	ros::spin();
	std::cout << "after spin() " << std::endl;
}
