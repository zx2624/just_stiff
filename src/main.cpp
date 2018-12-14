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
#define TOYOTA

//是否开启可视化
bool visual_on=true;
//是否发送水的检测结果
bool send_water=false;
//是否显示绿线---主要为了去掉绿线可视化
#define showgreen
//#define USENEIGHBER
#define CONDITION(x) if(0)
#define PI 3.141592653
#define TESTSHOW
//#define COUNTTHRESH 6
//栅格地图的尺寸和分辨率
#define HEIGHTTHRESH 0.2
#define GRIDWH 351
//64线激光雷达的内参--垂直角度
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

//用于根据激光雷达点的距离和垂直角度以及水平角度反算xyz坐标的sintable、costable
double sintable[870];
double costable[870];
//膨胀腐蚀的元素的形状大小
cv::Mat elementero = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
cv::Mat elementdil = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
//用于向车体坐标转换的变换矩阵
Eigen::Isometry3d T=Eigen::Isometry3d::Identity();
//用于存储惯导和激光雷达深度图消息的队列
common::BlockingQueue<sensor_driver_msgs::GpswithHeadingConstPtr> qgwithhmsgs_;
common::BlockingQueue<depth_image_utils::HeightMapConstPtr> qheightmap_;
//用于发送检测结果的publisher
ros::Publisher pubStiffwaterOgm;
//点云线程锁
std::mutex mtx_cloud;
//是否显示点云
#define VIEWER
#ifdef VIEWER
boost::shared_ptr<PCLVisualizer> 	cloud_viewer_(new PCLVisualizer ("zx Cloud"));
#endif

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

#ifdef TOYOTA
//深度图callback
void callback(const depth_image_utils::HeightMapConstPtr& height_msg){
	qheightmap_.Push(height_msg);
	if(qheightmap_.Size()>1){
		qheightmap_.Pop();
	}
}
//惯导callback
void gpsdatacllbak(const sensor_driver_msgs::GpswithHeadingConstPtr& msg){
	qgwithhmsgs_.Push(msg);
	if(qgwithhmsgs_.Size()>10){
		qgwithhmsgs_.Pop();
	}
}
//整体处理线程
void process(){
	while(ros::ok()){
		const depth_image_utils::HeightMapConstPtr heightmapmsg=qheightmap_.PopWithTimeout(common::FromSeconds(0.1));
		if(heightmapmsg==nullptr) continue;
		double timeheightmap=heightmapmsg->header.stamp.toSec();
		if(visual_on){
#ifdef TESTSHOW
			cv::namedWindow("grid",CV_WINDOW_NORMAL);
			cv::namedWindow("justshow",CV_WINDOW_NORMAL);
#endif
		}
		cv::Mat heightmat_0 = cv::Mat::zeros(64,870,CV_32F);
		cv::Mat dismat_0 = cv::Mat::zeros(64,870,CV_32F);
		//将消息里的深度图取出放入dismat_0
		for(int i=0;i<64;++i){
			for(int j =0;j<870;++j){
				if(abs(heightmapmsg->data[i*870+j]+100)>0.0001){//原始值为-100赋值为0
					dismat_0.at<float>(i,j)=heightmapmsg->data[i*870+j+64*870];
				}
			}
		}
		//gridshow_0用于画悬崖和费悬崖区域
		cv::Mat gridshow_0 = cv::Mat::zeros(GRIDWH,GRIDWH,CV_8UC3);
		//justshow 拥有显示
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
		//车辆中间区域
		int rightend=(int)38/0.2;
		int leftend=(int)32/0.2;

		for(int j=0;j<870;j++){
			//用于记录第一条有低到高的有效雷达线，用于特殊处理
			int countlaser=0;
			for(int i=58;i>15;i--){
				//当前激光雷达点和下一个点的索引
				int index=i*870+j;
				int indexnext=(i-1)*870+j;
				//当前点的距离
				double dis=(double)dismat_0.at<float>(i,j);
				//半径
				double radius=dis*std::cos(pitchrad64[i]*PI/180);
				//雷达坐标系该点的坐标
				double originx=radius*costable[j];
				double originy=radius*sintable[j];
				double originz=dis*std::sin(pitchrad64[i]*PI/180);

				//计算起始点及前一点高度
				double height=heightmat_0.at<float>(i,j);
				double heightbefore=heightmat_0.at<float>(i+1,j);
				//当前线雷达与上一线雷达距离差
				double diff2=(double)dismat_0.at<float>(i,j)-(double)dismat_0.at<float>(i+1,j);
				double disnext=(double)dismat_0.at<float>(i-1,j);
				//跳过无效点并计数
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

				//下一个点的半径以及雷达坐标系坐标
				double radiusnext=disnext*std::cos(pitchrad64[63-i+1]*PI/180);
				double originxnext=radiusnext*costable[j];
				double originynext=radiusnext*sintable[j];
				double originznext=disnext*std::sin(pitchrad64[63-i+1]*PI/180);



				Eigen::Matrix<double, 3, 1> originp;
				Eigen::Matrix<double, 3, 1> originpnext;
				Eigen::Matrix<double, 3, 1> originpvirtual;

				originp<<originx,originy,originz;
				originpnext<<originxnext,originynext,originznext;

				//将当前点和下一点转换到车体坐标系
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



				//虚拟半径，---主要用于第一条线，用虚拟半径作为上一条线
				double radiusvirtual=4.0;
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
				//只用高度进行判断
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


				//下一个点的栅格坐标
				int colnext=boost::math::round((xnext+35)/0.2);
				int rownext=boost::math::round((ynext+20)/0.2);
				int gridindex=0;
				unsigned char* ptr=gridshow_0.ptr<unsigned char>(GRIDWH-1-row);


				//下面是为了查看从当前点到下一个点之间（在栅格地图上）是否有障碍物
				//在栅格地图上悬崖的末端在通常高度通常为负值，而水的反射点在栅格地图上通常为正值
				//目前用的就是下一个点，可以延长一段，水的倒影有可能在后面一点
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

				//正式开始检测特征
				//当前点在栅格范围内时
				if(col<GRIDWH&&row<GRIDWH&&col>=0&&row>=0){
					//半径不同设置不同的高度，超过该高度停止检测
					//经过矫正后的高度可以设置为固定值
					if(radius<3)break;
					if(radius<8){if(height>0.3)break;}
					else if(radius<13){if(height>0.5)break;}
					else{if(height>0.8)break;}
					unsigned char* ptrshow=justshow.ptr<unsigned char>(GRIDWH-1-row);
					//					if(col==35/0.2){
					//						ptrshow[3*col]=255;
					//						ptrshow[3*col+1]=255;
					//						ptrshow[3*col+2]=255;
					//					}

					//				针对远距离误检不同阈值
					if(1){
						//短距离用高度差和距离突变来定义
						if(radius<10){
							//可以增加动态阈值
							int thresh=6;
							if(col>155&&col<195) thresh=15;
							//高度差大于0.5m，并且距离差之比大于阈值
							if(heightnext-height<-0.5&&diff1/diff2>thresh){//
								//把当前这个点标红125，代表候选悬崖的边缘
								ptr[3*col]=0;
								ptr[3*col+1]=0;
								ptr[3*col+2]=125;
								//							cout<<radius<<"==================>"<<diff1/diff2<<endl;
								//排除尺寸比较小的，通常悬崖都会比较大
								if((colnext-col)*(colnext-col)*0.04+(rownext-row)*(rownext-row)*0.04<1){
#ifdef showgreen
									//绿色150代表非悬崖
									cv::line(gridshow_0,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext),cvScalar(0,150,0));
#endif
									continue;
								}
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
								//排除水的误检
								//遍历当前到下一点之间有无较高的障碍物
								cv::LineIterator it(heightgridmax_0,cvPoint(col,GRIDWH-1-row),cvPoint(colwater,GRIDWH-1-rowwater));
								double hm=-100;
								for(int k=0;k<it.count;++k,it++){
									int x=it.pos().x;
									int y=it.pos().y;
									y=GRIDWH-1-y;
									//不止遍历一个栅格，遍历每个该条线附近的多个栅格
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
								//从当前点到下一点栅格地图上没有较高障碍物的话，
								//认为不是由水造成的误检，则认为是悬崖区域
								//将收尾标红125，代表悬崖区域
								//todo：可以缩短距离，防止范围过大误伤非悬崖区域
								if(hm<0.4){
									cv::line(gridshow_0,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext),cvScalar(0,0,125));
								}
							}
							//如果不满足阈值条件则标绿150
							else{//对应的为条件：heightnext-height<-0.5&&diff1/diff2>thresh
#ifdef showgreen
								cv::line(gridshow_0,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext),cvScalar(0,150,0));
#endif
							}
						}
						//对于10m之外的设置不同阈值
						else if(radius<30){
							int thresh=5;
							//这里指对于路中间的区域，是否阈值过大呢？待检测
							if(col>155&&col<195) thresh=15;
							if(1){//&&height>-0.4 &&yprdiff<7
								//					if((heightmat_0.at<float>(i-1,j)-heightmat_0.at<float>(i,j))<-1&&diff1/diff2>3){
								if((height-heightnext)/(radiusnext-radius)>0.07
										&&heightnext-height<-1&&diff1/diff2>thresh){//
									//可能的边缘点
									ptr[3*col]=0;
									ptr[3*col+1]=0;
									ptr[3*col+2]=125;

									//排除径向距离比较小的
									if((colnext-col)*(colnext-col)*0.04+(rownext-row)*(rownext-row)*0.04<1){
#ifdef showgreen
										cv::line(gridshow_0,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext),cvScalar(0,150,0));
#endif
										continue;
									}

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
									}
									//									cout<<hm<<endl;
									if(hm<0.4){
										cv::line(justshow,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext),cvScalar(0,0,125));
										cv::line(gridshow_0,cvPoint(col,GRIDWH-1-row),cvPoint(colnext,GRIDWH-1-rownext),cvScalar(0,0,125));
									}

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
				//这里找的水区域是只在近距离内丢失点比较多的区域，
				//这要求激光雷达点比较密集，目前只在64线效果比较好
				cv::LineIterator it(heightgridmax_0,cvPoint(col,GRIDWH-1-row),cvPoint(colwater,GRIDWH-1-rowwater));
				//count是只在循环开始时跳过的不处理的点数，如果跳过比较多的话就标记出来
				if(count>2){
					double hm=-100;
					//这里也需要遍历排除水的倒影
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
		//用于将之前用颜色标识的悬崖区域转换为单通道的黑白图像
		//用于膨胀腐蚀
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
		//膨胀腐蚀，消除单独的误检
		cv::dilate(gridall,gridall,elementdil);
		cv::erode(gridall,gridall,elementero);
		//将70*70的栅格地图投影到40*70
		int max_cnt=-10;
		for(int row=0;row<351;row++){
			unsigned char* ptr=gridall.ptr<unsigned char>(GRIDWH-1-row);
			for(int col=0;col<201;col++){
				int index=row*201+col;
				if(ptr[col+75]==255){
					//5代表悬崖
					msg_send.data.push_back(5);
				}
				else if(ptr[col+75]==200){
					if(send_water){
						//6代表水
						msg_send.data.push_back(6);
					}
					else{
						//未知
						msg_send.data.push_back(0);
					}
				}
				else if(ptr[col+75]==100){
					//1为非悬崖和非水
					msg_send.data.push_back(1);
				}
				else{
					//未知
					msg_send.data.push_back(0);
				}

				//如果车前有比较多的水的话发送一个标识
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
		//发送消息
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

//接收消息，是否发送水检测结果
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
//点云显示
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
	//将360度分成870份作为每个经过下采样的激光点的横向角度
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
	//雷达到车辆坐标系的姿态角
	double zangle = 0,yangle =0.3288*PI/180,xangle = -2.6254*PI/180;//0
	::Eigen::Vector3d ea0(zangle,yangle,xangle);

	::Eigen::Matrix3d R;

	R = ::Eigen::AngleAxisd(ea0[0], ::Eigen::Vector3d::UnitZ())
	* ::Eigen::AngleAxisd(ea0[1], ::Eigen::Vector3d::UnitY())
	* ::Eigen::AngleAxisd(ea0[2], ::Eigen::Vector3d::UnitX());


	T.rotate ( R );                                        // 按照rotation_vector进行旋转
	T.pretranslate ( Eigen::Vector3d (0,1, 2.14) );

	ros::Subscriber subHeightMap = nh.subscribe("lidar_height_map",1,callback);
	ros::Subscriber subwatersend = nh.subscribe("topology_global_path",1,sendcallback);
	//	ros::Subscriber subGpsdata=nh.subscribe("gpsdata",10,gpsdatacllbak);
	std::thread measurement_process{process};
#endif


	ros::spin();
	std::cout << "after spin() " << std::endl;
}
