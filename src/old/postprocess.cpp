

#include <pcl/io/boost.h>
#include <boost/version.hpp>
#include <boost/foreach.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/math/special_functions.hpp>
#include <boost/lambda/lambda.hpp>
#include "postprocess.h"

#define LOCAL_IP "192.168.0.112"
//#define LOCAL_IP "127.0.0.1"
#define FROMLADAR_LOCAL_PORT 9906



void PostProcess::init(const std::string &correctionsfile)
{


	//		subLaserOdometry_ = nodehandle_.subscribe<nav_msgs::Odometry>
	//										 ("lidar_odometry_to_init", 5, boost::bind(&PostProcess::laserOdometryHandler,this,_1));//需要雷达里程计信息时需要，否则可以注释掉

	subLaserCloudFullRes_ = nodehandle_.subscribe<sensor_msgs::PointCloud2>
	("lidar_cloud_calibrated", 1, boost::bind(&PostProcess::laserCloudHandler,this,_1));//经过筛选且转换之后的点云

	subLaserOdometry_ = nodehandle_.subscribe<sensor_driver_msgs::OdometrywithGps>
	("lidar_odometry_to_earth", 10, boost::bind(&PostProcess::laserOdometryHandler,this,_1));


	//file_.open("/home/jkj/catkin_ws/result.txt",std::ios::out);

	loadCorrectionsFile (correctionsfile);
	for(int index=0;index<HDL_MAX_NUM_LASERS;index++)
	{
		if(index<LASER_LAYER)
		{
			double angle=laser_corrections_[index].verticalCorrection;
			//double tan_angle=tan(HDL_Grabber_toRadians(angle));
			map_tanangle_index[angle]=index;     //build mapping
		}
	}
	//="<<map_tanangle_index.size()<<std::endl;

	int index=0;
	for(std::map<double,int>::iterator iter=map_tanangle_index.begin() ; iter!=map_tanangle_index.end(); iter++)
	{

		indexmaptable[index].number=iter->second;
		indexmaptable[index].angle=iter->first;
		//cout<<index<<"\tlaserindex="<<iter->second<<"\tangle="<<iter->first<<"\ttanangle="<<tan(HDL_Grabber_toRadians(iter->first))<<endl;
		index++;
	}
	//		  if(1)
	//		      {
	//		          boost::function0<void> fdisplay = boost::bind(&PostProcess::displayPointCloud,this);
	//		          thread_displayPointCloud=new boost::thread(fdisplay);
	//		      }

	for(int i=0;i<LASER_LAYER;i++)
	{
		theorydis[i]=-z_offset_h/std::tan(indexmaptable[i].angle*M_PI/180);
		//		         if(theorydis[i]>0&&theorydis[i]<60)
		//		             anglerange[i]=2.5/theorydis[i]*180/M_PI;
		//		         cout<<"theorydis["<<i<<"]="<< theorydis[i]<<endl;
	}
	stiffcloud_= Cloud::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
	normalcloud_ = Cloud::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
	SetViewerPCL( cloud_viewer_);
	SetViewerEgoVehicleModel( cloud_viewer_);
	processthread_ = new boost::thread(boost::bind(&PostProcess::process,this));

	//		    cloud_viewer_->addCoordinateSystem (3.0);
	//		    cloud_viewer_->setBackgroundColor (0, 0, 0);
	//		    cloud_viewer_->initCameraParameters ();
	//		    cloud_viewer_->setCameraPosition (0.0, 0.0, 30.0, 0.0, 1.0, 0.0, 0);
	//		    cloud_viewer_->setCameraClipDistances (0.0, 100.0);
	//	        float x1 = -1 , x2 = 1 , y1 = -1 , y2 = 3, z = 0;
	//	        float newx1, newx2, newy1, newy2, newz;
	//	        char linename[20];
	//	        pcl::PointXYZI pt1, pt2, pt3, pt4;
	//	        for(int i = 0 ; i < 100 ; i++)
	//	        {
	//	            x1 = -20 ;
	//	            x2 = 20 ;
	//	            y1 = (i - 2) * 1 ;
	//	            y2 = (i - 2) * 1;
	//	            z = 0;
	//	            coordinate_from_vehicle_to_velodyne(x1,y1,z,newx1,newy1,newz);
	//	            coordinate_from_vehicle_to_velodyne(x2,y2,z,newx2,newy2,newz);
	//	            pt1.x = min(newx1 , newx2) ;
	//	            pt1.y = min(newy1 , newy2) ;
	//	            pt1.z = newz;
	//	            pt2.x = max(newx1 , newx2) ;
	//	            pt2.y = max(newy1 , newy2) ;
	//	            pt2.z = newz;
	//	            memset(linename, 0 , 20);
	//	            sprintf(linename , "lat%02d" , i);
	//	            cloud_viewer_->addLine(pt1, pt2, linename);
	//	        }
	//
	//	        for(int i = 0 ; i < 50 ; i++)
	//	        {
	//	            x1 = i * 1 - 20;
	//	            x2 = i * 1 - 20;
	//	            y1 = -20 ;
	//	            y2 = 70 ;
	//	            z = 0;
	//	            coordinate_from_vehicle_to_velodyne(x1,y1,z,newx1,newy1,newz);
	//	            coordinate_from_vehicle_to_velodyne(x2,y2,z,newx2,newy2,newz);
	//	            pt1.x = min(newx1 , newx2) ;
	//	            pt1.y = min(newy1 , newy2) ;
	//	            pt1.z = newz;
	//	            pt2.x = max(newx1 , newx2) ;
	//	            pt2.y = max(newy1 , newy2) ;
	//	            pt2.z = newz;
	//	            memset(linename, 0 , 20);
	//	            sprintf(linename , "lng%02d" , i);
	//	            cloud_viewer_->addLine(pt1, pt2, linename);
	//	        }

}
void PostProcess::coordinate_from_vehicle_to_velodyne(float x, float y , float z, float& newx, float& newy, float& newz)
{
	newz = z;
	newx = x;
	newy = y;
}
void PostProcess::keyboard_callback (const KeyboardEvent& event, void* cookie)
{
	if (event.keyUp ())
	{
		if(event.getKeyCode() ==  '1')
			freeze_ = 1;
		else if(event.getKeyCode() ==  '2')
			freeze_ = 0;
		return;
	}
}
void PostProcess::mouse_callback (const MouseEvent& mouse_event, void* cookie)
{
	if (mouse_event.getType () == MouseEvent::MouseButtonPress &&
			mouse_event.getButton () == MouseEvent::LeftButton)
	{
		cout << mouse_event.getX () << " , " << mouse_event.getY () << endl;
	}
}

void PostProcess::SetViewerPCL(boost::shared_ptr<PCLVisualizer> cloud_viewer_)
{
	//��ʼ��PCL��ʾ��������ز���
	cloud_viewer_->addCoordinateSystem (3.0);
	cloud_viewer_->setBackgroundColor(0,0,0);//(1.0,1.0,1.0);// (255, 0, 0);
	cloud_viewer_->initCameraParameters ();
	cloud_viewer_->setCameraPosition (0.0, 0.0, 30.0, 0.0, 1.0, 0.0, 0);
	cloud_viewer_->setCameraClipDistances (1.0, 120.0);
	cloud_viewer_->registerMouseCallback(&PostProcess::mouse_callback, *this);
	cloud_viewer_->registerKeyboardCallback (&PostProcess::keyboard_callback, *this);
	std::cout<<"........................................................"<<std::endl;

}
void PostProcess::SetViewerEgoVehicleModel(boost::shared_ptr<PCLVisualizer> cloud_viewer_)
{
	//�Զ���ʻ����Ѳ������//��ʾ���ڻ�����
	if(1)
	{
		//������
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

		pt1.x = 0 ;
		pt1.y = 3 ;
		pt1.z = 0;
		pt2.x = -1 ;
		pt2.y = 1.5 ;
		pt2.z = 0;
		cloud_viewer_->addLine(pt1, pt2, "body5");

		pt1.x = 0 ;
		pt1.y = 3 ;
		pt1.z = 0;
		pt2.x = 1 ;
		pt2.y = 1.5 ;
		pt2.z = 0;
		cloud_viewer_->addLine(pt1, pt2, "body6");

		pt1.x = -1 ;
		pt1.y = 1.5 ;
		pt1.z = 0;
		pt2.x = 1 ;
		pt2.y = 1.5 ;
		pt2.z = 0;
		cloud_viewer_->addLine(pt1, pt2, "body7");

		//���Ϸ�Χ
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

		//��������
		if (0)
		{
			char linename[20];
			for(int i = 0 ; i < 10 ; i++)
			{
				x1 = -20 ;
				x2 = 20 ;
				y1 = (i - 2) * 10 ;
				y2 = (i - 2) * 10;
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
				sprintf(linename , "lat%02d", i);
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
		if (1)
		{
			for (int i=1; i<5; i++)
			{
				pcl::ModelCoefficients coefficients/*(new pcl::ModelCoefficients)*/;
				coefficients.values.resize(3);
				coefficients.values[0] = 0;
				coefficients.values[1] = 0;
				coefficients.values[2] = i*10;


				char name[20];
				sprintf(name, "circle%d",i);
				cloud_viewer_->addCircle(coefficients,name,0);

			}
		}

	}
}
void PostProcess::ShowViewerCloudPoints( boost::shared_ptr<PCLVisualizer> cloud_viewer_, vector<Cloud::ConstPtr> cloud_show_,
		char cloudname[], double color_red_, double color_green_, double color_blue_ )
{
	cloud_viewer_->removeAllPointClouds();
	if(cloud_show_.size()==3){


		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> cloudHandler( cloud_show_[0], 255, 0, 0);
		if (!cloud_viewer_->updatePointCloud(cloud_show_[0],cloudHandler, cloudname))
		{
			//				std::cout<<"cloudgeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeet"<<std::endl;
			cloud_viewer_->addPointCloud(cloud_show_[0], cloudHandler, cloudname);
		}
		//					pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> cloudHandler1( cloud_show_[1], 0,255,0);
		//					if (!cloud_viewer_->updatePointCloud(cloud_show_[1],cloudHandler1, cloudname))
		//					{
		//						//				std::cout<<"cloudgeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeet"<<std::endl;
		//						cloud_viewer_->addPointCloud(cloud_show_[1], cloudHandler1, cloudname);
		//					}
		//					pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> cloudHandler2( cloud_show_[2], 0,0,255);
		//					if (!cloud_viewer_->updatePointCloud(cloud_show_[2],cloudHandler2, cloudname))
		//					{
		//						//				std::cout<<"cloudgeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeet"<<std::endl;
		//						cloud_viewer_->addPointCloud(cloud_show_[2], cloudHandler2, cloudname);
		//					}


	}

}
void PostProcess::ShowViewerCloudPoints( boost::shared_ptr<PCLVisualizer> cloud_viewer_, Cloud::ConstPtr cloud_show_,
		char cloudname[], double color_red_, double color_green_, double color_blue_ )
{
	cloud_viewer_->removeAllPointClouds();


	if ( cloud_show_->size()>0 )
	{

		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> cloudHandler( cloud_show_, color_red_, color_green_, color_blue_ );
		if (!cloud_viewer_->updatePointCloud(cloud_show_,cloudHandler, cloudname))
		{
			//				std::cout<<"cloudgeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeet"<<std::endl;
			cloud_viewer_->addPointCloud(cloud_show_, cloudHandler, cloudname);
		}
	}

}
void PostProcess::displayPointCloud(Cloud::Ptr cloud_show)//
{
	boost::shared_ptr<PCLVisualizer> cloud_viewer_(new PCLVisualizer ("stiff Cloud"));

	cloud_viewer_->addCoordinateSystem (3.0);
	cloud_viewer_->setBackgroundColor (0, 0, 0);
	cloud_viewer_->initCameraParameters ();
	cloud_viewer_->setCameraPosition (0.0, 0.0, 30.0, 0.0, 1.0, 0.0, 0);
	cloud_viewer_->setCameraClipDistances (0.0, 100.0);
	cloud_viewer_->registerKeyboardCallback (&PostProcess::keyboard_callback, *this);
	//cloud_viewer_->registerMouseCallback(&PostProcess::mouse_callback,*this);
	{
		//画
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
		//	        char linename[20];
		//	        for(int i = 0 ; i < 10 ; i++)
		//	        {
		//	            x1 = -20 ;
		//	            x2 = 20 ;
		//	            y1 = (i - 2) * 10 ;
		//	            y2 = (i - 2) * 10;
		//	            z = 0;
		//	            coordinate_from_vehicle_to_velodyne(x1,y1,z,newx1,newy1,newz);
		//	            coordinate_from_vehicle_to_velodyne(x2,y2,z,newx2,newy2,newz);
		//	            pt1.x = min(newx1 , newx2) ;
		//	            pt1.y = min(newy1 , newy2) ;
		//	            pt1.z = newz;
		//	            pt2.x = max(newx1 , newx2) ;
		//	            pt2.y = max(newy1 , newy2) ;
		//	            pt2.z = newz;
		//	            memset(linename, 0 , 20);
		//	            sprintf(linename , "lat%02d" , i);
		//	            cloud_viewer_->addLine(pt1, pt2, linename);
		//	        }
		//
		//	        for(int i = 0 ; i < 5 ; i++)
		//	        {
		//	            x1 = i * 10 - 20;
		//	            x2 = i * 10 - 20;
		//	            y1 = -20 ;
		//	            y2 = 70 ;
		//	            z = 0;
		//	            coordinate_from_vehicle_to_velodyne(x1,y1,z,newx1,newy1,newz);
		//	            coordinate_from_vehicle_to_velodyne(x2,y2,z,newx2,newy2,newz);
		//	            pt1.x = min(newx1 , newx2) ;
		//	            pt1.y = min(newy1 , newy2) ;
		//	            pt1.z = newz;
		//	            pt2.x = max(newx1 , newx2) ;
		//	            pt2.y = max(newy1 , newy2) ;
		//	            pt2.z = newz;
		//	            memset(linename, 0 , 20);
		//	            sprintf(linename , "lng%02d" , i);
		//	            cloud_viewer_->addLine(pt1, pt2, linename);
		//	        }
	}

	while (!cloud_viewer_->wasStopped ())//&&!flag_close)
	{

		std::cout<<"getclouuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuud"<<std::endl;
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> rigid_nopassablecloudHandler (cloud_show, 255, 0, 0);
		if (!cloud_viewer_->updatePointCloud (cloud_show, rigid_nopassablecloudHandler, "normalcloud"))
			cloud_viewer_->addPointCloud (cloud_show, rigid_nopassablecloudHandler, "normalcloud");
		if(1)
		{
			//	            if(freeze_==1)
			//	                cloud_viewer_->spinOnce();
			//	            else
			{
				//display
				// if(velodyne_pointcloud->points.size()>0)
				if(cloudupdate)
				{

					cloudupdate=false;
					//  boost::mutex::scoped_lock lock(displaymutex);

					cloud_viewer_->removeAllPointClouds();
					//��ͨ������
					if(stiffcloud_->points.size()>0)
					{
						//��ɫ

						pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> passablecloudHandler (stiffcloud_, 0, 255, 0);
						if (!cloud_viewer_->updatePointCloud (stiffcloud_, passablecloudHandler, "stiffcloud")){
							cloud_viewer_->addPointCloud (stiffcloud_, passablecloudHandler, "stiffcloud");
						}
					}

					if(cloud_show->points.size()>0)
					{
						//	                        std::cout<<"getclouuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuud"<<std::endl;
						//	                        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> rigid_nopassablecloudHandler (cloud_show, 255, 0, 0);
						//	                        if (!cloud_viewer_->updatePointCloud (cloud_show, rigid_nopassablecloudHandler, "normalcloud")){
						//	                            cloud_viewer_->addPointCloud (cloud_show, rigid_nopassablecloudHandler, "normalcloud");
					}
				}


				//       if(velodyne_pointcloud)
				cloud_viewer_->spinOnce();



			}
			//	                if (!grabber_H_.isRunning())
			//	                    cloud_viewer_->spin ();


			////��ʾ********************/

#ifdef SIMULATION
			boost::this_thread::sleep (boost::posix_time::microseconds (50000));
#else
			boost::this_thread::sleep (boost::posix_time::microseconds (100));
#endif

		}
	}
	//    if (replay_ == 1)
	//        grabber_H_.resume();

	//flag_close=true;
	cloud_viewer_->close();
}

void PostProcess::SendData(OGMData<unsigned char>& ogmdata)  //udp通信 发送端例程
{
	BoostUdp sendogmdata(LOCAL_IP,FROMLADAR_LOCAL_PORT);
	sendogmdata.connectRemoteEndpoint("192.168.0.111",9905);
	//	sendogmdata.connectRemoteEndpoint("127.0.0.1",9905);
	int lenth=2000;
	int package = ogmdata.ogmcell_size/lenth;
	int package_total;

	char ogm_total_data[ogmdata.ogmcell_size];
	int headernum = 3;
	char header[headernum];
	//memcpy(ogm_total_data,ogmdata.ogm,1);
	if(ogmdata.ogmcell_size - lenth * package>0)
	{
		package_total = package + 1;
	}
	for(int i =0;i < package;i++)
	{
		int ogmstart = lenth*i;
		char senddata[lenth+headernum];
		header[0]='a';
		header[1]=i;
		header[2]=package_total;
		memcpy(senddata,header,headernum);
		memcpy(senddata+headernum,ogmdata.ogm+ogmstart,lenth);
		sendogmdata.send(senddata,sizeof(senddata));
	}
	if(ogmdata.ogmcell_size - lenth * package>0)
	{
		int lenth_new = ogmdata.ogmcell_size - lenth * package;
		char senddata[lenth_new];
		header[0] = 0x88;
		header[1] = package;
		header[2] = package_total;
		memcpy(senddata,header,headernum);
		memcpy(senddata+headernum,ogmdata.ogm+lenth*package,lenth_new);
		sendogmdata.send(senddata,sizeof(senddata));
	}
}

void PostProcess::laserOdometryHandler(const sensor_driver_msgs::OdometrywithGps::ConstPtr& laserOdometry)  //雷达里程计
{
	double timeOdometry = laserOdometry->odometry.header.stamp.toSec();
	static double last_stamp = -1;
	//  static geometry_msgs::Quaternion last_geoQuat;
	static transform::Rigid3d lasttransformodometry;
	//  static float last_trans[6];
	//  double roll, pitch, yaw;
	geometry_msgs::Quaternion geoQuat = laserOdometry->odometry.pose.pose.orientation;

	Eigen::Quaterniond roatation(geoQuat.w,geoQuat.x,geoQuat.y,geoQuat.z);
	Eigen::Vector3d translation(laserOdometry->odometry.pose.pose.position.x,
			laserOdometry->odometry.pose.pose.position.y,
			laserOdometry->odometry.pose.pose.position.z);

	transform::Rigid3d transformodometry(translation,roatation);

	lidarOdoms_.Push(common::make_unique<TimePosePair>(timeOdometry,transformodometry));

}



void PostProcess::laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudFullRes2) //点云数据
{
	double timeLaserCloudFullRes = laserCloudFullRes2->header.stamp.toSec();
	timestamp_=laserCloudFullRes2->header.stamp;
	//		std::cout<<"lasertime" << fixed << setprecision(9)<<timestamp_<<std::endl;

	//		LOG(INFO)<<std::fixed<<std::setprecision(3)<<"cloudtime:"<<timeLaserCloudFullRes;
	//		LOG(INFO)<<"starttime"<<ros::Time::now().toSec() - timeLaserCloudFullRes;
	lidarCloudMsgs_.Push(laserCloudFullRes2);
	if(lidarCloudMsgs_.Size()>1)
		lidarCloudMsgs_.Pop();

}

void PostProcess::analysisCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr inputcloud,
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

void PostProcess::loadCorrectionsFile (const std::string& correctionsFile)
{
	boost::property_tree::ptree pt;
	try
	{
		read_xml (correctionsFile, pt, boost::property_tree::xml_parser::trim_whitespace);
	}
	catch (boost::exception const&)
	{
		PCL_ERROR ("[pcl::MyHDLGrabber::loadCorrectionsFile] Error reading calibration file %s!\n", correctionsFile.c_str ());
		return;
	}

	BOOST_FOREACH (boost::property_tree::ptree::value_type &v, pt.get_child ("boost_serialization.DB.points_"))
	{
		if (v.first == "item")
		{
			boost::property_tree::ptree points = v.second;
			BOOST_FOREACH(boost::property_tree::ptree::value_type &px, points)
			{
				if (px.first == "px")
				{
					boost::property_tree::ptree calibrationData = px.second;
					int index = -1;
					double azimuth = 0, vertCorrection = 0, distCorrection = 0,
							vertOffsetCorrection = 0, horizOffsetCorrection = 0;

					BOOST_FOREACH (boost::property_tree::ptree::value_type &item, calibrationData)
					{
						if (item.first == "id_")
							index = atoi (item.second.data ().c_str ());
						if (item.first == "rotCorrection_")
							azimuth = atof (item.second.data ().c_str ());
						if (item.first == "vertCorrection_")
							vertCorrection = atof (item.second.data ().c_str ());
						if (item.first == "distCorrection_")
							distCorrection = atof (item.second.data ().c_str ());
						if (item.first == "vertOffsetCorrection_")
							vertOffsetCorrection = atof (item.second.data ().c_str ());
						if (item.first == "horizOffsetCorrection_")
							horizOffsetCorrection = atof (item.second.data ().c_str ());
					}
					if (index != -1)
					{
						laser_corrections_[index].azimuthCorrection = azimuth;
						laser_corrections_[index].verticalCorrection = vertCorrection;
						laser_corrections_[index].distanceCorrection = distCorrection / 100.0;
						laser_corrections_[index].verticalOffsetCorrection = vertOffsetCorrection / 100.0;
						laser_corrections_[index].horizontalOffsetCorrection = horizOffsetCorrection / 100.0;

						laser_corrections_[index].cosVertCorrection = std::cos (HDL_Grabber_toRadians(laser_corrections_[index].verticalCorrection));
						laser_corrections_[index].sinVertCorrection = std::sin (HDL_Grabber_toRadians(laser_corrections_[index].verticalCorrection));

					}
				}
			}
		}
	}
}
void PostProcess::countogmpoints(vector<pcl::PointCloud<pcl::PointXYZI>::ConstPtr> cloud)
{
	cout<<"new frame coming.................................................................in"<<endl;
	memset(point_count_ogm_.ogm , 0 , point_count_ogm_.ogmcell_size*sizeof(int));//每次都要置零,避免填充错乱
	memset(point_count_ogm_big_.ogm , 0 , point_count_ogm_big_.ogmcell_size*sizeof(int));
	for(int i=0;i<maxz_ogm_. ogmcell_size;i++){
		maxz_ogm_.ogm[i]=-11.11;
	}
	for(int i=0;i<maxz_ogm_big_.ogmcell_size;i++){
		maxz_ogm_big_.ogm[i]=-11.11;
	}
	//		memset(ogm_msg_.ogm,0,ogm_msg_.ogmcell_size*sizeof(unsigned int));
	memset(ogm_msg_.ogm,0,ogm_msg_.ogmcell_size*sizeof(unsigned char));

	float ogm_y_offset = 20.0f;
	//为每个栅格赋值点云数量
	int framecount=0;
	for(int cloudi=0;cloudi<cloud.size();cloudi++){
		//			cout<<"cloudi sieze if "<<cloud[cloudi]->points.size()<<"    ";
		for (int i = 0; i < cloud[cloudi]->points.size(); i++)
		{

			float x = cloud[cloudi]->points[i].x,
					y = cloud[cloudi]->points[i].y,
					z = cloud[cloudi]->points[i].z;


			float newy = y + ogm_y_offset;//ogm_y_offset
			//					if(cloud->points[i].intensity < 15.0f)//排除噪声干扰，只考虑反射率较高的点
			//					                continue;
			if(x>-1&&x<1&&y>-1&&y<3){		//排除车身周围
				continue;
			}
			if(framecount==0||framecount==1){																			//第一帧全部都投影到小栅格，部分投影到大栅格,
				if((x >=-point_count_ogm_.ogmwidth / 2  && x <= point_count_ogm_.ogmwidth / 2) &&//这里的条件是否需要再考虑一下
						(newy >=0 && newy < point_count_ogm_.ogmheight) &&
						( z <=  Z_MAX))
				{
					if(x>-2&&x<2&&y<3.5&&y>-1) continue;
					int col = boost::math::round(x / point_count_ogm_.ogmresolution) + ( point_count_ogm_.ogmwidth_cell - 1 ) / 2;
					int row = boost::math::round(newy / point_count_ogm_.ogmresolution) ;





					if((row >=0 && row < point_count_ogm_.ogmheight_cell)
							&& (col >=0 && col < point_count_ogm_.ogmwidth_cell))
					{
						int index = row * point_count_ogm_.ogmwidth_cell + col;
						point_count_ogm_.ogm[index]++;
						if( maxz_ogm_.ogm[index] < z)
						{
							maxz_ogm_.ogm[index] = z;
						}
					}
				}
				if(x*x+y*y<RADIUS){continue;}
				else if(x*x+y*y<625||(x>-5&&x<5&&y<50))//这里把大栅格较检测区域扩大了一圈，
				{
					//						cout<<"..............."<<endl;
					if((x >=-point_count_ogm_big_.ogmwidth / 2  && x <= point_count_ogm_big_.ogmwidth / 2) &&//这里的条件是否需要再考虑一下
							(newy >=0 && newy < point_count_ogm_big_.ogmheight) &&
							( z <=  Z_MAX))
					{
						int col = boost::math::round(x / point_count_ogm_big_.ogmresolution) + ( point_count_ogm_big_.ogmwidth_cell - 1 ) / 2;
						int row = boost::math::round(newy / point_count_ogm_big_.ogmresolution) ;





						if((row >=0 && row < point_count_ogm_big_.ogmheight_cell)
								&& (col >=0 && col < point_count_ogm_big_.ogmwidth_cell))
						{
							int index = row * point_count_ogm_big_.ogmwidth_cell + col;
							point_count_ogm_big_.ogm[index]++;
							if(maxz_ogm_big_.ogm[index]<z){
								maxz_ogm_big_.ogm[index]=z;
							}
						}
					}
				}

			}
			else{
				if(x*x+y*y<RADIUS){continue;}														//这里只拼接半径之外的到大栅格
				else if(x*x+y*y<625||(x>-5&&x<5&&y<50)){
					if((x >=-point_count_ogm_big_.ogmwidth / 2  && x <= point_count_ogm_big_.ogmwidth / 2) &&
							(newy >=0 && newy < point_count_ogm_big_.ogmheight) &&
							( z <=  Z_MAX))
					{
						int col = boost::math::round(x / point_count_ogm_big_.ogmresolution) + ( point_count_ogm_big_.ogmwidth_cell - 1 ) / 2;
						int row = boost::math::round(newy / point_count_ogm_big_.ogmresolution) ;





						if((row >=0 && row < point_count_ogm_big_.ogmheight_cell)
								&& (col >=0 && col < point_count_ogm_big_.ogmwidth_cell))
						{
							int index = row * point_count_ogm_big_.ogmwidth_cell + col;
							point_count_ogm_big_.ogm[index]++;
							if(maxz_ogm_big_.ogm[index]<z){
								maxz_ogm_big_.ogm[index]=z;
							}
						}

					}

				}
			}
		}
		framecount++;

		//test
		//		for(int i=point_count_ogm_.ogmwidth_cell/2;i<point_count_ogm_.ogmwidth_cell/2+6/point_count_ogm_.ogmresolution;i++){
		//			int index=20/point_count_ogm_.ogmresolution*point_count_ogm_.ogmwidth_cell+i;
		//			std::cout<<point_count_ogm_.ogm[index]<<" ";
		//		}
		//		std::cout<<std::endl;
		/////////////////////////////////
		//为每个栅格赋值maxz


		///test
		//		for(int i=maxz_ogm_.ogmwidth_cell/2;i<maxz_ogm_.ogmwidth_cell/2+15/maxz_ogm_.ogmresolution;i++){
		//				int index=35/maxz_ogm_.ogmresolution*maxz_ogm_.ogmwidth_cell+i;
		//				std::cout<<maxz_ogm_.ogm[index]<<" ";
		//			}
		//			std::cout<<std::endl;
		//测试代码,看每个格子的点数/高度
		//		for (int i=20;i<30;i++)
		//		{
		//			for(int j=20;j<40;j++)//车体右侧
		//				{
		////				if(maxz_ogm_.ogm[i*point_count_ogm_.ogmwidth_cell+j]>1){
		//				std::cout<<".."<<maxz_ogm_.ogm[i*point_count_ogm_.ogmwidth_cell+j];
		////				cout<<".."<<j<<".."<<i<<endl;
		////				}
		//			}
		//			std::cout<<std::endl;
		//		}
		//		for(int i=13;i<21;i++){
		//			std::cout<<point_count_ogm_.ogm[35*point_count_ogm_.ogmwidth_cell+i]<<"  ";
		//		}
		//		std::cout<<std::endl;
	}



	//下面将考虑通过检测比较大片的无点区域判断是否为悬崖zx
	//		1、首先检测小区域内有没有悬崖区域等。
	for(int i=10/point_count_ogm_.ogmresolution;i<30/point_count_ogm_.ogmresolution;i++)//
	{
		float thresh;
		if (i>15/point_count_ogm_.ogmresolution){thresh=GRID_THRESH;}
		else {thresh=GRID_THRESH+3;}
		//			std::cout<<"......11111111111111.................."<<std::endl;
		int end_right=point_count_ogm_.ogmwidth_cell/2+16/point_count_ogm_.ogmresolution;
		for(int j=point_count_ogm_.ogmwidth_cell/2+4/point_count_ogm_.ogmresolution;j<end_right;j++)//车体右侧
		{

			float actual_x=(j-20/point_count_ogm_.ogmresolution)*0.2;
			float actual_y=(i-20/point_count_ogm_.ogmresolution)*0.2;
			if((actual_x*actual_x+actual_y*actual_y)>RADIUS){
				continue;
			}
			int count=0;
			int index=i*point_count_ogm_.ogmwidth_cell+j;
			//用于记录连续没有点云的栅格数
			float bound=0;
			while(point_count_ogm_.ogm[index]<POINT_COUNT_THRESH&&bound<RADIUS)//0221这里的10之前是16zx
			{
				//									std::cout<<"................"<<count+i<<std::endl;
				count++;
				index++;
				bound=(j-20/point_count_ogm_.ogmresolution+count)*0.2*(j-20/point_count_ogm_.ogmresolution+count)*0.2+(i-20/point_count_ogm_.ogmresolution)*0.2
						*(i-20/point_count_ogm_.ogmresolution)*0.2;

			}
			//								int tempindex=index;
			//								int index_img=(point_count_ogm_.ogmheight_cell-i-1)*point_count_ogm_.ogmwidth_cell+j;//这是在图像上的索引，用于测试
			if(count>thresh)								//可调参数
			{
				float maxz1=maxz_ogm_.ogm[i*maxz_ogm_.ogmwidth_cell+j-1];
				float maxz2=maxz_ogm_.ogm[i*maxz_ogm_.ogmwidth_cell+j+count];
				if(abs(maxz1+11.11)<0.01){
					for(int hi=1;hi<5;hi++){
						for(int sign=0;sign<2;sign++){
							hi=-hi;
							for(int wj=j-1;wj>j-11;wj--){
								int index=(i+hi)*maxz_ogm_.ogmwidth_cell+wj;
								if(abs(maxz_ogm_.ogm[index]-1111)>0.01){
									maxz1=maxz_ogm_.ogm[index];
									break;
								}
							}
							if(abs(maxz1+11.11)>0.01) break;
						}
						if(abs(maxz1+11.11)>0.01) break;
					}
				}
				//maxz2
				if(abs(maxz2+11.11)<0.01){
					for(int hi=1;hi<5;hi++){
						for(int sign=0;sign<2;sign++){
							hi=-hi;
							for(int wj=j+count;wj<j+count+10;wj++){
								int index=(i+hi)*maxz_ogm_.ogmwidth_cell+wj;
								if(abs(maxz_ogm_.ogm[index]-1111)>0.01){
									maxz2=maxz_ogm_.ogm[index];
									break;
								}
							}
							if(abs(maxz2+11.11)>0.01) break;
						}
						if(abs(maxz2+11.11)>0.01) break;
					}
				}
				bool heightstate=true;
				if(abs(maxz1+11.11)>0.01&&abs(maxz2+11.11)>0.01&&abs(maxz1-maxz2)<0.4){
					heightstate=false;
				}
				if(maxz1>1.4){
					heightstate=false;
				}
				if(!heightstate) continue;
				int msgindex=i*ogm_msg_.ogmwidth_cell+j;
				for(int kk=0;kk<count;kk++){
					if(pathClear(i,j+kk)){

						vec2side_.push_back(i);
						vec2side_.push_back(j+kk);
						ogm_msg_.ogm[msgindex]=5;
						msgindex+=1;
					}
				}
			}
			if(count>0)
				j+=count-1;
		}

		for(int j=point_count_ogm_.ogmwidth_cell/2-20/point_count_ogm_.ogmresolution;j<point_count_ogm_.ogmwidth_cell/2-4/point_count_ogm_.ogmresolution;j++)//车体左侧
		{
			float actual_x=(j-20/point_count_ogm_.ogmresolution)*0.2;
			float actual_y=(i-20/point_count_ogm_.ogmresolution)*0.2;
			if((actual_x*actual_x+actual_y*actual_y)>RADIUS)
			{
				continue;
			}
			int count=0;
			int index=i*point_count_ogm_.ogmwidth_cell+j;
			//用于记录连续没有点云的栅格数
			float bound=0;
			while(point_count_ogm_.ogm[index]<POINT_COUNT_THRESH&&bound<RADIUS&&count<point_count_ogm_.ogmwidth_cell/2-4/point_count_ogm_.ogmresolution-j)//0221这里的10之前是16zx
			{
				//									std::cout<<"................"<<count+i<<std::endl;
				count++;
				index++;
				bound=(j-20/point_count_ogm_.ogmresolution+count)*0.2*(j-20/point_count_ogm_.ogmresolution+count)*0.2+(i-20/point_count_ogm_.ogmresolution)*0.2
						*(i-20/point_count_ogm_.ogmresolution)*0.2;

			}
			int tempindex=index;
			//								int index_img=(point_count_ogm_.ogmheight_cell-i-1)*point_count_ogm_.ogmwidth_cell+j;//这是在图像上的索引，用于测试
			index=i*ogm_msg_.ogmwidth_cell+j;//栅格地图上悬崖起始点索引
			int index2=(4*i+1)*ogm_msg_.ogmwidth_cell+4*j;
			int index3=(4*i+2)*ogm_msg_.ogmwidth_cell+4*j;
			int index4=(4*i+3)*ogm_msg_.ogmwidth_cell+4*j;
			if(count>thresh)								//可调参数
			{
				float maxz1=maxz_ogm_.ogm[i*maxz_ogm_.ogmwidth_cell+j-1];
				float maxz2=maxz_ogm_.ogm[i*maxz_ogm_.ogmwidth_cell+j+count];
				if(abs(maxz1+11.11)<0.01){
					for(int hi=1;hi<5;hi++){
						for(int sign=0;sign<2;sign++){
							hi=-hi;
							for(int wj=j-1;wj>j-11;wj--){
								int index=(i+hi)*maxz_ogm_.ogmwidth_cell+wj;
								if(abs(maxz_ogm_.ogm[index]-1111)>0.01){
									maxz1=maxz_ogm_.ogm[index];
									break;
								}
							}
							if(abs(maxz1+11.11)>0.01) break;
						}
						if(abs(maxz1+11.11)>0.01) break;
					}
				}
				//maxz2
				if(abs(maxz2+11.11)<0.01){
					for(int hi=1;hi<5;hi++){
						for(int sign=0;sign<2;sign++){
							hi=-hi;
							for(int wj=j+count;wj<j+count+10;wj++){
								int index=(i+hi)*maxz_ogm_.ogmwidth_cell+wj;
								if(abs(maxz_ogm_.ogm[index]-1111)>0.01){
									maxz2=maxz_ogm_.ogm[index];
									break;
								}
							}
							if(abs(maxz2+11.11)>0.01) break;
						}
						if(abs(maxz2+11.11)>0.01) break;
					}
				}
				bool heightstate=true;
				if(abs(maxz1+11.11)>0.01&&abs(maxz2+11.11)>0.01&&abs(maxz1-maxz2)<0.4){
					heightstate=false;
				}
				if(maxz2>1.4){
					heightstate=false;
				}
				if(!heightstate) continue;
				int msgindex=i*ogm_msg_.ogmwidth_cell+j;
				for(int kk=0;kk<count;kk++){
					if(pathClear(i,j+kk)){
						vec2side_.push_back(i);
						vec2side_.push_back(j+kk);
						ogm_msg_.ogm[msgindex]=5;
						msgindex+=1;
					}
				}
			}
			if(count>0)
				j+=count-1;
		}
		//
	}
	//test
	//		for(int i=13;i<21;i++){
	//			int index=35*point_count_ogm_.ogmwidth_cell+i;
	//			int count=0;
	//			while(point_count_ogm_.ogm[index]==0){
	//				count++;
	//				index++;
	//			}
	//			if(count>2){
	//				if(pathClear(35,i)){
	//					std::cout<<"yesyesyes"<<std::endl;
	//				}
	//				else cout<<"nonono"<<std::endl;
	//			}
	//		}
	//test done
	for(int j=point_count_ogm_.ogmwidth_cell/2-4/point_count_ogm_.ogmresolution;j<point_count_ogm_.ogmwidth_cell/2+4/point_count_ogm_.ogmresolution;j++)//车体上下范围
	{

		//			for(int i=0;i<20/point_count_ogm_.ogmresolution;i++)//-3/point_count_ogm_.ogmresolution
		//			{
		//				int count=0;
		//				//排除车体周围范围
		//				if(j>point_count_ogm_.ogmwidth_cell/2-3/point_count_ogm_.ogmresolution&&j<point_count_ogm_.ogmwidth_cell/2+3/point_count_ogm_.ogmresolution
		//						&&i<point_count_ogm_.ogmheight_cell/2+2/point_count_ogm_.ogmresolution&&i>point_count_ogm_.ogmheight_cell/2-4/point_count_ogm_.ogmresolution)
		//					continue;
		//				int index=i*point_count_ogm_.ogmwidth_cell+j;
		//				while(point_count_ogm_.ogm[index]==0&&count<6&&count<point_count_ogm_.ogmheight_cell/2-4/point_count_ogm_.ogmresolution-i)//-point_count_ogm_.ogmheight_cell/2//&&count<point_count_ogm_.ogmheight_cell-i
		//				{
		//					count++;
		//					index+=point_count_ogm_.ogmwidth_cell;
		//				}
		//				index=i*2*ogm_msg_.ogmwidth_cell+j*2;//这是在栅格坐标系下的索引zx
		////				int index_img=(point_count_ogm_.ogmheight_cell-i-1)*point_count_ogm_.ogmwidth_cell+j;//这是在图像上的索引，用于测试
		//				if(count>GRID_THRESH)								//可调参数
		//				{
		//					if(pathClear(i,j))	//判断一下由雷达到无点区域中点之间有无障碍  pathClear(j,i+count/2)
		//					{
		//								for(int k=0;k<count;k++)
		//								{
		//									memset(&ogm_msg_.ogm[index],5,2*sizeof(unsigned char));
		//									memset(&ogm_msg_.ogm[index+ogm_msg_.ogmwidth_cell],5,2*sizeof(unsigned char));
		//		//							std::cout<<ogm_msg_.ogm[index]<<" "<<ogm_msg_.ogm[index+ogm_msg_.ogmwidth_cell]<<std::endl;
		//		//							ogm_msg_.ogm[index]=5;;
		//		//							ogm_msg_.ogm[index+1]=5;
		//									index+=2*ogm_msg_.ogmwidth_cell;			//在图像和栅格坐标系下分别是+= 和-=
		//								}
		////										std::cout<<"........................"<<std::endl;
		//						vecup_.push_back(point_count_ogm_.ogmheight_cell-i-1);
		//						vecup_.push_back(j);
		//						vecup_.push_back(count);
		//
		//					}
		//				}
		//				if(count>0) i+=count-1;
		//
		//			}
		for(int i=20/point_count_ogm_.ogmresolution+6/point_count_ogm_.ogmresolution;i<40/point_count_ogm_.ogmresolution;i++)
		{
			float actual_x=(j-20/point_count_ogm_.ogmresolution)*0.2;
			float actual_y=(i-20/point_count_ogm_.ogmresolution)*0.2;
			if((actual_x*actual_x+actual_y*actual_y)>RADIUS)
			{
				continue;
			}
			int thresh;
			if(i<40/point_count_ogm_.ogmresolution) thresh=GRID_THRESH;
			else if(i<50/point_count_ogm_.ogmresolution) thresh=GRID_THRESH2;
			else thresh=GRID_THRESH3;
			int count=0;
			if(j>point_count_ogm_.ogmwidth_cell/2-3/point_count_ogm_.ogmresolution&&j<point_count_ogm_.ogmwidth_cell/2+3/point_count_ogm_.ogmresolution
					&&i<20/point_count_ogm_.ogmresolution+2/point_count_ogm_.ogmresolution&&i>20/point_count_ogm_.ogmresolution-4/point_count_ogm_.ogmresolution)
				continue;
			int index=i*point_count_ogm_.ogmwidth_cell+j;
			float bound=0;

			while(point_count_ogm_.ogm[index]<POINT_COUNT_THRESH&&bound<RADIUS)//&&count<point_count_ogm_.ogmheight_cell-i
			{
				//					cout<<"the point count is  "<<i<<"  "<<j<<endl;
				//					cout<<"the index is "<<index<<endl;
				bound=(j-20/point_count_ogm_.ogmresolution)*0.2*(j-20/point_count_ogm_.ogmresolution)*0.2+(i-20/point_count_ogm_.ogmresolution+count)*0.2
						*(i-20/point_count_ogm_.ogmresolution+count)*0.2;

				count++;
				index+=point_count_ogm_.ogmwidth_cell;
			}
			//				cout<<"the count is "<<count<<endl;
			//							index=i*point_count_ogm_.ogmwidth_cell+j;//复位zx
			index=i*ogm_msg_.ogmwidth_cell+j;
			//							int index_img=(point_count_ogm_.ogmheight_cell-i-1)*point_count_ogm_.ogmwidth_cell+j;//这是在图像上的索引，用于测试
			if(count>GRID_THRESH+2)								//可调参数
			{
				if(pathClear(i,j))	//判断一下由雷达到无点区域中点之间有无障碍  pathClear(j,i+count/2)
				{

					float maxz1=maxz_ogm_.ogm[(i-1)*maxz_ogm_.ogmwidth_cell+j];
					float maxz2=maxz_ogm_.ogm[(i+count)*maxz_ogm_.ogmwidth_cell+j];
					if(abs(maxz1+11.11)<0.01){
						for(int hi=i-2;hi>i-7;hi--){
							int index=hi*maxz_ogm_.ogmwidth_cell+j;
							if(abs(maxz_ogm_.ogm[index]-1111)>0.01){
								maxz1=maxz_ogm_.ogm[index];
								break;
							}
						}
					}
					if(abs(maxz2+11.11)<0.01){
						for(int hi=i+count;hi<i+count+5;hi++){
							int index=hi*maxz_ogm_.ogmwidth_cell+j;
							if(abs(maxz_ogm_.ogm[index]-1111)>0.01){
								maxz2=maxz_ogm_.ogm[index];
								break;
							}
						}
					}
					bool heightstate=true;
					if(abs(maxz1+11.11)>0.01&&abs(maxz2+11.11)>0.01&&(maxz1-maxz2)<0.4){
						heightstate=false;
					}
					if(heightstate){
						for(int k=0;k<count;k++)
						{
							ogm_msg_.ogm[index]=5;
							index+=point_count_ogm_.ogmwidth_cell; //在图像和栅格坐标系下分别是+= 和-=


						}
						//						std::cout<<"............jj is............"<<j<<std::endl;
						vecup_.push_back(i);
						vecup_.push_back(j);
						vecup_.push_back(count);
					}

				}
			}
			if(count>0) i+=count-1;

		}
	}
	//		2、检测10m之外
	//		2.1、左右两侧
	for(int i=0;i<41/point_count_ogm_big_.ogmresolution;i++)//
	{

		//			std::cout<<"......11111111111111.................."<<std::endl;
		float thresh;
		if(i<40/point_count_ogm_big_.ogmresolution){thresh=GRID_THRESH_BIG;}
		else{thresh=int(GRID_THRESH_BIG*1.5);}
		int end_right=point_count_ogm_big_.ogmwidth_cell/2+20/point_count_ogm_big_.ogmresolution;
		for(int j=point_count_ogm_big_.ogmwidth_cell/2+4/point_count_ogm_big_.ogmresolution;j<end_right;j++)//车体右侧
		{

			float actual_x=(j-20/point_count_ogm_big_.ogmresolution)*0.4;
			float actual_y=(i-20/point_count_ogm_big_.ogmresolution)*0.4;
			if((actual_x*actual_x+actual_y*actual_y)<RADIUS||((actual_x*actual_x+actual_y*actual_y)>400)){
				continue;
			}
			int count=0;
			int index=i*point_count_ogm_big_.ogmwidth_cell+j;
			//用于记录连续没有点云的栅格数
			float bound=(j-20/point_count_ogm_big_.ogmresolution)*0.4*(j-20/point_count_ogm_big_.ogmresolution)*0.4+(i-20/point_count_ogm_big_.ogmresolution)*0.4
					*(i-20/point_count_ogm_big_.ogmresolution)*0.4;
			while(point_count_ogm_big_.ogm[index]<POINT_COUNT_THRESH&&bound<400)//
			{
				//									std::cout<<"................"<<count+i<<std::endl;
				count++;
				index++;
				bound=(j-20/point_count_ogm_big_.ogmresolution+count)*0.4*(j-20/point_count_ogm_big_.ogmresolution+count)*0.4+(i-20/point_count_ogm_big_.ogmresolution)*0.4
						*(i-20/point_count_ogm_big_.ogmresolution)*0.4;



			}
			//								int tempindex=index;
			//								int index_img=(point_count_ogm_big_.ogmheight_cell-i-1)*point_count_ogm_big_.ogmwidth_cell+j;//这是在图像上的索引，用于测试
			index=2*i*ogm_msg_.ogmwidth_cell+j*2;//栅格地图上悬崖起始点索引
			int index2=(2*i+1)*ogm_msg_.ogmwidth_cell+2*j;
			int index3=(4*i+2)*ogm_msg_.ogmwidth_cell+4*j;
			int index4=(4*i+3)*ogm_msg_.ogmwidth_cell+4*j;
			if(count>thresh)								//可调参数
			{


				bool heightstate=true;
				float maxz1=maxz_ogm_big_.ogm[i*maxz_ogm_big_.ogmwidth_cell+j-1];
				float maxz2=maxz_ogm_big_.ogm[i*maxz_ogm_big_.ogmwidth_cell+j+count];
				//					cout<<"before is "<<maxz1<<"  "<<maxz2<<endl;
				if(abs(maxz1+11.11)<0.01){
					for(int hi=1;hi<5;hi++){
						for(int sign=0;sign<2;sign++){
							hi=-hi;
							for(int wj=j-1;wj>j-11;wj--){
								int index=(i+hi)*maxz_ogm_big_.ogmwidth_cell+wj;
								if(abs(maxz_ogm_big_.ogm[index]+11.11)>0.01){
									maxz1=maxz_ogm_big_.ogm[index];
									break;
								}
							}
							if(abs(maxz1+11.11)>0.01) break;
						}
						if(abs(maxz1+11.11)>0.01) break;
					}
				}
				if(abs(maxz2+11.11)<0.01){
					for(int hi=1;hi<5;hi++){
						for(int sign=0;sign<2;sign++){
							hi=-hi;
							for(int wj=j+count;wj<j+count+10;wj++){
								int index=(i+hi)*maxz_ogm_big_.ogmwidth_cell+wj;
								if(abs(maxz_ogm_big_.ogm[index]+11.11)>0.01){
									maxz2=maxz_ogm_big_.ogm[index];
									break;
								}
							}
							if(abs(maxz2+11.11)>0.01) break;
						}
						if(abs(maxz2+11.11)>0.01) break;
					}
				}
				//					cout<<"after is "<<maxz1<<"  "<<maxz2<<endl;
				if(abs(maxz1+11.11)>0.1&&abs(maxz2+11.11)>0.1&&(maxz1-maxz2)<0.4){
					heightstate=false;
				}
				if(maxz1>1.4){
					heightstate=false;
				}
				if(!heightstate){
					continue;
				}
				int msgindex=2*i*ogm_msg_.ogmwidth_cell+2*j;
				for(int kk=0;kk<count;kk++){
					if(pathClear(2*i,2*(j+kk))){
						vec4side_.push_back(i);
						vec4side_.push_back(j+kk);
						ogm_msg_.ogm[msgindex]=5;
						ogm_msg_.ogm[msgindex+1]=5;
						ogm_msg_.ogm[msgindex+ogm_msg_.ogmwidth_cell]=5;
						ogm_msg_.ogm[msgindex+ogm_msg_.ogmwidth_cell+1]=5;
						msgindex+=2;
					}
				}

				//											if((pathClear(2*i,j*2)&&pathClear((i)*2,2*(j+count/2)))&&pathClear((i)*2,2*(j+count)))

			}
			if(count>0)
				j+=count-1;
		}

		for(int j=point_count_ogm_big_.ogmwidth_cell/2-20/point_count_ogm_big_.ogmresolution;j<point_count_ogm_big_.ogmwidth_cell/2-4/point_count_ogm_big_.ogmresolution;j++)//车体左侧
		{
			float actual_x=(j-20/point_count_ogm_big_.ogmresolution)*0.4;
			float actual_y=(i-20/point_count_ogm_big_.ogmresolution)*0.4;
			if((actual_x*actual_x+actual_y*actual_y)<RADIUS||((actual_x*actual_x+actual_y*actual_y)>400)){
				continue;
			}
			int count=0;
			int index=i*point_count_ogm_big_.ogmwidth_cell+j;
			//用于记录连续没有点云的栅格数

			float bound=(j-20/point_count_ogm_big_.ogmresolution)*0.4*(j-20/point_count_ogm_big_.ogmresolution)*0.4+(i-20/point_count_ogm_big_.ogmresolution)*0.4
					*(i-20/point_count_ogm_big_.ogmresolution)*0.4;
			while(point_count_ogm_big_.ogm[index]<POINT_COUNT_THRESH&&count<point_count_ogm_big_.ogmwidth_cell/2-4/point_count_ogm_big_.ogmresolution-j&&bound<400&&bound>100)//0221这里的10之前是16zx
			{
				//									std::cout<<"................"<<count+i<<std::endl;
				bound=(j-20/point_count_ogm_big_.ogmresolution+count)*0.4*(j-20/point_count_ogm_big_.ogmresolution+count)*0.4+(i-20/point_count_ogm_big_.ogmresolution)*0.4
						*(i-20/point_count_ogm_big_.ogmresolution)*0.4;
				count++;
				index++;


			}
			int tempindex=index;
			//								int index_img=(point_count_ogm_big_.ogmheight_cell-i-1)*point_count_ogm_big_.ogmwidth_cell+j;//这是在图像上的索引，用于测试
			index=2*i*ogm_msg_.ogmwidth_cell+j*2;//栅格地图上悬崖起始点索引
			int index2=(2*i+1)*ogm_msg_.ogmwidth_cell+2*j;
			int index3=(4*i+2)*ogm_msg_.ogmwidth_cell+4*j;
			int index4=(4*i+3)*ogm_msg_.ogmwidth_cell+4*j;
			if(count>thresh)								//可调参数
			{

				bool heightstate=true;
				float maxz1=maxz_ogm_big_.ogm[i*maxz_ogm_big_.ogmwidth_cell+j-1];
				float maxz2=maxz_ogm_big_.ogm[i*maxz_ogm_big_.ogmwidth_cell+j+count];
				//					cout<<"before is "<<maxz1<<"  "<<maxz2<<endl;
				if(abs(maxz1+11.11)<0.01){
					for(int hi=1;hi<5;hi++){
						for(int sign=0;sign<2;sign++){
							hi=-hi;
							for(int wj=j-1;wj>j-11;wj--){
								int index=(i+hi)*maxz_ogm_big_.ogmwidth_cell+wj;
								if(abs(maxz_ogm_big_.ogm[index]+11.11)>0.01){
									maxz1=maxz_ogm_big_.ogm[index];
									break;
								}
							}
							if(abs(maxz1+11.11)>0.01) break;
						}
						if(abs(maxz1+11.11)>0.01) break;
					}
				}
				if(abs(maxz2+11.11)<0.01){
					for(int hi=1;hi<5;hi++){
						for(int sign=0;sign<2;sign++){
							hi=-hi;
							for(int wj=j+count;wj<j+count+10;wj++){
								int index=(i+hi)*maxz_ogm_big_.ogmwidth_cell+wj;
								if(abs(maxz_ogm_big_.ogm[index]+11.11)>0.01){
									maxz2=maxz_ogm_big_.ogm[index];
									break;
								}
							}
							if(abs(maxz2+11.11)>0.01) break;
						}
						if(abs(maxz2+11.11)>0.01) break;
					}
				}
				//					cout<<"after is "<<maxz1<<"  "<<maxz2<<endl;
				if(abs(maxz1+11.11)>0.1&&abs(maxz2+11.11)>0.1&&(maxz1-maxz2)<0.4){
					heightstate=false;
				}
				if(maxz2>1.4){
					heightstate=false;
				}
				if(!heightstate){
					continue;
				}
				int msgindex=2*i*ogm_msg_.ogmwidth_cell+2*j;
				for(int kk=0;kk<count;kk++){
					if(pathClear(2*i,2*(j+kk))){
						vec4side_.push_back(i);
						vec4side_.push_back(j+kk);
						ogm_msg_.ogm[msgindex]=5;
						ogm_msg_.ogm[msgindex+1]=5;
						ogm_msg_.ogm[msgindex+ogm_msg_.ogmwidth_cell]=5;
						ogm_msg_.ogm[msgindex+ogm_msg_.ogmwidth_cell+1]=5;
						msgindex+=2;
					}
				}
			}
			if(count>0)
				j+=count-1;
		}
		//
	}
	//		2.2上下侧
	for(int j=point_count_ogm_big_.ogmwidth_cell/2-4/point_count_ogm_big_.ogmresolution;j<point_count_ogm_big_.ogmwidth_cell/2+4/point_count_ogm_big_.ogmresolution;j++)//车体上下范围
	{

		//			for(int i=0;i<20/point_count_ogm_.ogmresolution;i++)//-3/point_count_ogm_.ogmresolution
		//			{
		//				int count=0;
		//				//排除车体周围范围
		//				if(j>point_count_ogm_.ogmwidth_cell/2-3/point_count_ogm_.ogmresolution&&j<point_count_ogm_.ogmwidth_cell/2+3/point_count_ogm_.ogmresolution
		//						&&i<point_count_ogm_.ogmheight_cell/2+2/point_count_ogm_.ogmresolution&&i>point_count_ogm_.ogmheight_cell/2-4/point_count_ogm_.ogmresolution)
		//					continue;这里记得要改，不是i/2了
		//				int index=i*point_count_ogm_.ogmwidth_cell+j;
		//				while(point_count_ogm_.ogm[index]==0&&count<6&&count<point_count_ogm_.ogmheight_cell/2-4/point_count_ogm_.ogmresolution-i)//-point_count_ogm_.ogmheight_cell/2//&&count<point_count_ogm_.ogmheight_cell-i
		//				{
		//					count++;
		//					index+=point_count_ogm_.ogmwidth_cell;
		//				}
		//				index=i*2*ogm_msg_.ogmwidth_cell+j*2;//这是在栅格坐标系下的索引zx
		////				int index_img=(point_count_ogm_.ogmheight_cell-i-1)*point_count_ogm_.ogmwidth_cell+j;//这是在图像上的索引，用于测试
		//				if(count>GRID_THRESH)								//可调参数
		//				{
		//					if(pathClear(i,j))	//判断一下由雷达到无点区域中点之间有无障碍  pathClear(j,i+count/2)
		//					{
		//						for(int k=0;k<count;k++)
		//						{
		//							memset(&ogm_msg_.ogm[index],5,2*sizeof(unsigned char));
		//							memset(&ogm_msg_.ogm[index+ogm_msg_.ogmwidth_cell],5,2*sizeof(unsigned char));
		////							std::cout<<ogm_msg_.ogm[index]<<" "<<ogm_msg_.ogm[index+ogm_msg_.ogmwidth_cell]<<std::endl;
		////							ogm_msg_.ogm[index]=5;;
		////							ogm_msg_.ogm[index+1]=5;
		//							index+=2*ogm_msg_.ogmwidth_cell;			//在图像和栅格坐标系下分别是+= 和-=
		//						}
		////										std::cout<<"........................"<<std::endl;
		//						vecup_.push_back(point_count_ogm_.ogmheight_cell-i-1);
		//						vecup_.push_back(j);
		//						vecup_.push_back(count);
		//
		//					}
		//				}
		//				if(count>0) i+=count-1;
		//
		//			}
		for(int i=30/point_count_ogm_big_.ogmresolution;i<40/point_count_ogm_big_.ogmresolution;i++)
		{
			float actual_x=(j-20/point_count_ogm_big_.ogmresolution)*0.4;
			float actual_y=(i-20/point_count_ogm_big_.ogmresolution)*0.4;
			//								if((actual_x*actual_x+actual_y*actual_y)<RADIUS||((actual_x*actual_x+actual_y*actual_y)>400))
			//								{
			//									continue;
			//								}
			int thresh;
			if(i<40/point_count_ogm_big_.ogmresolution) thresh=GRID_THRESH_BIG;
			else if(i<50/point_count_ogm_big_.ogmresolution) thresh=5/point_count_ogm_big_.ogmresolution;
			//								else thresh=GRID_THRESH3;
			int count=0;

			int index=i*point_count_ogm_big_.ogmwidth_cell+j;
			//						float bound=0;

			float bound=(j-20/point_count_ogm_big_.ogmresolution)*0.4*(j-20/point_count_ogm_big_.ogmresolution)*0.4+(i-20/point_count_ogm_big_.ogmresolution)*0.4
					*(i-20/point_count_ogm_big_.ogmresolution)*0.4;
			while(point_count_ogm_big_.ogm[index]<3&&count<45/point_count_ogm_big_.ogmresolution-i)//&&count<40/point_count_ogm_big_.ogmresolution-i
			{

				bound=(j-20/point_count_ogm_big_.ogmresolution)*0.4*(j-20/point_count_ogm_big_.ogmresolution)*0.4+(i-20/point_count_ogm_big_.ogmresolution+count)*0.4
						*(i-20/point_count_ogm_big_.ogmresolution+count)*0.4;

				count++;
				index+=point_count_ogm_big_.ogmwidth_cell;
			}
			//				cout<<"the count is "<<count<<endl;
			//							index=i*point_count_ogm_big_.ogmwidth_cell+j;//复位zx
			index=2*i*ogm_msg_.ogmwidth_cell+j*2;
			//							int index_img=(point_count_ogm_big_.ogmheight_cell-i-1)*point_count_ogm_big_.ogmwidth_cell+j;//这是在图像上的索引，用于测试
			if(count>20)								//可调参数
			{
				//									if(pathClear(4*i,4*j))	//判断一下由雷达到无点区域中点之间有无障碍  pathClear(j,i+count/2)
				if(pathClear(2*i,2*j))//pathClear(2*i,2*j)
				{
					float maxz1=maxz_ogm_big_.ogm[(i-1)*maxz_ogm_big_.ogmwidth_cell+j];
					float maxz2=maxz_ogm_big_.ogm[(i+count)*maxz_ogm_big_.ogmwidth_cell+j];
					cout<<"beforeis "<<maxz1<<"   "<<maxz2<<endl;
					if(abs(maxz1+11.11)<0.01){
						for(int hi=i-2;hi>i-7;hi--){
							int index=hi*maxz_ogm_big_.ogmwidth_cell+j;
							if(abs(maxz_ogm_big_.ogm[index]+11.11)>0.01){
								maxz1=maxz_ogm_big_.ogm[index];
								break;
							}
						}
					}
					if(abs(maxz2+11.11)<0.01){
						for(int hi=i+count;hi<hi+count+5;hi++){
							int index=index=hi*maxz_ogm_big_.ogmwidth_cell+j;
							if(abs(maxz_ogm_big_.ogm[index]+11.11)>0.01){
								maxz2=maxz_ogm_big_.ogm[index];
								break;
							}
						}
					}
					cout<<"after is "<<maxz1<<"   "<<maxz2<<endl;
					bool heightstate=true;
					if(abs(maxz1+11.11)>0.01&&abs(maxz2+11.11)>0.01&&(maxz1-maxz2)<0.4){
						heightstate=false;
					}
					if(heightstate){
						for(int k=0;k<count;k++)

						{
							//																memset(&ogm_msg_.ogm[index],5,4*sizeof(unsigned char));
							//																memset(&ogm_msg_.ogm[index+ogm_msg_.ogmwidth_cell],5,4*sizeof(unsigned char));
							//																memset(&ogm_msg_.ogm[index+2*ogm_msg_.ogmwidth_cell],5,4*sizeof(unsigned char));
							//																memset(&ogm_msg_.ogm[index+3*ogm_msg_.ogmwidth_cell],5,4*sizeof(unsigned char));
							//																index+=4*ogm_msg_.ogmwidth_cell;
							memset(&ogm_msg_.ogm[index],5,2*sizeof(unsigned char));
							memset(&ogm_msg_.ogm[index+ogm_msg_.ogmwidth_cell],5,2*sizeof(unsigned char));
							index+=2*ogm_msg_.ogmwidth_cell;

						}
						//						std::cout<<"............jj is............"<<j<<std::endl;
						vecup_big_.push_back(i);
						vecup_big_.push_back(j);
						vecup_big_.push_back(count);
					}

				}
			}
			if(count>0) i+=count-1;

		}
	}
	//	通过窗口内悬崖点数量判断是否为悬崖
	//	for(int i=0;i<vecright_.size()/3;i++){
	//		int originindex=(ogm_msg_.ogmheight_cell-vecright_[3*i]-1)*ogm_msg_.ogmwidth_cell+vecright_[3*i+1];
	//		int index=0;
	//		int count=0;
	//		for(int windowi=ogm_msg_.ogmheight_cell-vecright_[3*i]-5;windowi<ogm_msg_.ogmheight_cell-vecright_[3*i]+4;windowi++){
	//			for(int windowj=vecright_[3*i+1];windowj<vecright_[3*i+1]+5;windowj++){
	//				index=windowi*ogm_msg_.ogmwidth_cell+windowj;
	//				if(ogm_msg_.ogm[index]==5)
	//				{
	//					count++;
	//				}
	//			}
	//		}
	//		if(count<15){
	//			memset(&ogm_msg_.ogm[originindex],0,vecright_[3*i+2]*sizeof(unsigned char));
	//		}
	//	}
	//
	//	for(int i=0;i<vecup_.size()/3;i++){
	//		int originindex=(ogm_msg_.ogmheight_cell-vecup_[3*i]-1)*ogm_msg_.ogmwidth_cell+vecup_[3*i+1];
	//		int index=0;
	//		int count=0;
	//		for(int windowi=ogm_msg_.ogmheight_cell-vecup_[3*i]-1;windowi<ogm_msg_.ogmheight_cell-vecup_[3*i]+6;windowi++){
	//			for(int windowj=vecup_[3*i+1]-2;windowj<vecup_[3*i+1]+3;windowj++){
	//				index=windowi*ogm_msg_.ogmwidth_cell+windowj;
	//				if(ogm_msg_.ogm[index]==5)
	//				{
	//					count++;
	//				}
	//			}
	//		}
	//		if(count<12){
	//			for(int k=0;k<vecup_[3*i+2];k++){
	//				ogm_msg_.ogm[originindex]=0;
	//				originindex+=ogm_msg_.ogmwidth_cell;
	//			}
	//		}
	//	}
	//	for(int i=0;i<vecup_big_.size()/3;i++){
	//		int originindex=2*(point_count_ogm_big_.ogmheight_cell-vecup_big_[3*i]-1)*ogm_msg_.ogmwidth_cell+2*vecup_big_[3*i+1];
	//		int index=0;
	//		int count=0;
	//		for(int windowi=2*(point_count_ogm_big_.ogmheight_cell-vecup_big_[3*i]-1);windowi<2*(point_count_ogm_big_.ogmheight_cell-vecup_big_[3*i]-1)+7;windowi++){
	//			for(int windowj=2*(vecup_big_[3*i+1])-2;windowj<2*(vecup_big_[3*i+1])+4;windowj++){
	//				index=windowi*ogm_msg_.ogmwidth_cell+windowj;
	//				if(ogm_msg_.ogm[index]==5)
	//				{
	//					count++;
	//				}
	//			}
	//		}
	//		if(count<24){
	//			for(int k=0;k<vecup_big_[3*i+2];k++){
	//				ogm_msg_.ogm[originindex]=0;
	//				ogm_msg_.ogm[originindex+1]=0;
	//				ogm_msg_.ogm[originindex+ogm_msg_.ogmwidth_cell]=0;
	//				ogm_msg_.ogm[originindex+ogm_msg_.ogmwidth_cell+1]=0;
	//				originindex+=2*ogm_msg_.ogmwidth_cell;
	//			}
	//		}
	//	}
	//	for(int i=0;i<vecright_big_.size()/3;i++){
	//		int originindex=2*(point_count_ogm_big_.ogmheight_cell-vecright_big_[3*i]-1)*ogm_msg_.ogmwidth_cell+2*vecright_big_[3*i+1];
	//		int index=0;
	//		int count=0;
	//		for(int windowi=2*(point_count_ogm_big_.ogmheight_cell-vecright_big_[3*i]-1)-4;windowi<2*(point_count_ogm_big_.ogmheight_cell-vecright_big_[3*i]-1)+5;windowi++){
	//			for(int windowj=2*vecright_big_[3*i+1];windowj<2*vecright_big_[3*i+1]+5;windowj++){
	//				index=windowi*ogm_msg_.ogmwidth_cell+windowj;
	//				if(ogm_msg_.ogm[index]==5)
	//				{
	//					count++;
	//				}
	//			}
	//		}
	//		if(count<24){
	//			memset(&ogm_msg_.ogm[originindex],0,vecright_big_[3*i+2]*2*sizeof(unsigned char));
	//			memset(&ogm_msg_.ogm[originindex+ogm_msg_.ogmwidth_cell],0,vecright_big_[3*i+2]*2*sizeof(unsigned char));
	//		}
	//	}


	for(int i=0;i<40/ogm_msg_.ogmresolution;){
		for(int j=0;j<ogm_msg_.ogmwidth_cell;){
			//				float actual_x=(j-20/ogm_msg_.ogmresolution)*0.2;
			//				float actual_y=(i-20/ogm_msg_.ogmresolution)*0.2;
			int count=0;
			vector<int> vecindex;
			vecindex.clear();
			vecindex.reserve(15);
			int window;
			for(int windowi=i;windowi<i+5;windowi++){
				for(int windowj=j;windowj<j+3;windowj++){
					int index=windowi*ogm_msg_.ogmwidth_cell+windowj;
					if(ogm_msg_.ogm[index]==5){
						count++;
						vecindex.push_back(index);
					}
				}
			}
			if(count<7){
				for(auto it=vecindex.begin();it!=vecindex.end();it++){
					ogm_msg_.ogm[*it]=0;
				}
			}
			j+=3;
		}
		i+=5;
	}
	//		膨胀
	for(int i=0;i<40/ogm_msg_.ogmresolution;){
		for(int j=0;j<ogm_msg_.ogmwidth_cell;){
			vector<int> vecindex;
			vecindex.clear();
			vecindex.reserve(6);
			for(int windowi=i;windowi<i+3;windowi++){
				for(int windowj=j;windowj<j+2;windowj++){
					int index=windowi*ogm_msg_.ogmwidth_cell+windowj;
					vecindex.push_back(index);
				}
			}
			bool flag=false;
			for(int windowi=i;windowi<i+3;windowi++){
				for(int windowj=j;windowj<j+2;windowj++){
					int index=windowi*ogm_msg_.ogmwidth_cell+windowj;
					vecindex.push_back(index);
					if(ogm_msg_.ogm[index]==5){
						flag=true;
						break;
					}
				}
				if(flag) break;
			}
			if(flag){
				for(auto it=vecindex.begin();it!=vecindex.end();it++){
					ogm_msg_.ogm[*it]=5;
				}
			}
			j+=2;
		}
		i+=3;
	}

}
#ifndef NOOLD
void PostProcess::countogmpoints(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
	memset(point_count_ogm_.ogm , 0 , point_count_ogm_.ogmcell_size*sizeof(int));//每次都要置零,避免填充错乱
	memset(point_count_ogm_big_.ogm , 0 , point_count_ogm_big_.ogmcell_size*sizeof(int));//每次都要置零,避免填充错乱
	memset(maxz_ogm_.ogm , 0 , maxz_ogm_.ogmcell_size*sizeof(int));
	//		memset(ogm_msg_.ogm,0,ogm_msg_.ogmcell_size*sizeof(unsigned int));
	memset(ogm_msg_.ogm,0,ogm_msg_.ogmcell_size*sizeof(unsigned char));

	float ogm_y_offset = 20.0f;
	//为每个栅格赋值点云数量
	for (int i = 0; i < cloud->points.size(); i++)
	{

		float x = cloud->points[i].x,
				y = cloud->points[i].y,
				z = cloud->points[i].z;
		float newy = y + ogm_y_offset;//ogm_y_offset
		if((x >=-point_count_ogm_.ogmwidth / 2  && x <= point_count_ogm_.ogmwidth / 2) &&//这里的条件是否需要再考虑一下
				(newy >=0 && newy < point_count_ogm_.ogmheight) &&
				( z <=  Z_MAX))
		{
			int col = boost::math::round(x / point_count_ogm_.ogmresolution) + ( point_count_ogm_.ogmwidth_cell - 1 ) / 2;
			int row = boost::math::round(newy / point_count_ogm_.ogmresolution) ;





			if((row >=0 && row < point_count_ogm_.ogmheight_cell)
					&& (col >=0 && col < point_count_ogm_.ogmwidth_cell))
			{
				int index = row * point_count_ogm_.ogmwidth_cell + col;
				point_count_ogm_.ogm[index]++;
			}
		}
		if(x*x+y*y<RADIUS){continue;}
		else if(x*x+y*y<400||(x>-5&&x<5&&y<50))//||(x>-5&&x<5&&y<50)
		{
			//						cout<<"..............."<<endl;
			if((x >=-point_count_ogm_big_.ogmwidth / 2  && x <= point_count_ogm_big_.ogmwidth / 2) &&//这里的条件是否需要再考虑一下
					(newy >=0 && newy < point_count_ogm_big_.ogmheight) &&
					( z <=  Z_MAX))
			{
				int col = boost::math::round(x / point_count_ogm_big_.ogmresolution) + ( point_count_ogm_big_.ogmwidth_cell - 1 ) / 2;
				int row = boost::math::round(newy / point_count_ogm_big_.ogmresolution) ;





				if((row >=0 && row < point_count_ogm_big_.ogmheight_cell)
						&& (col >=0 && col < point_count_ogm_big_.ogmwidth_cell))
				{
					int index = row * point_count_ogm_big_.ogmwidth_cell + col;
					point_count_ogm_big_.ogm[index]++;
				}
			}
		}

	}


	//test
	//		for(int i=point_count_ogm_.ogmwidth_cell/2;i<point_count_ogm_.ogmwidth_cell/2+6/point_count_ogm_.ogmresolution;i++){
	//			int index=20/point_count_ogm_.ogmresolution*point_count_ogm_.ogmwidth_cell+i;
	//			std::cout<<point_count_ogm_.ogm[index]<<" ";
	//		}
	//		std::cout<<std::endl;
	/////////////////////////////////
	//为每个栅格赋值maxz
	for (int i = 0; i < cloud->points.size(); i++)
	{
		float x = cloud->points[i].x,
				y = cloud->points[i].y,
				z = cloud->points[i].z;
		float newy = y + ogm_y_offset;
		//			if(cloud->points[i].intensity < 15.0f)//排除噪声干扰，只考虑反射率较高的点
		//			                continue;
		if(x>-1.5&&x<1.5&&y>-1&&y<4){		//排除车身周围------------------------------------zx
			continue;
		}

		//			            newy = y + maxz_ogm_.ogmheight/2;
		if((x >=-maxz_ogm_.ogmwidth / 2  && x <= maxz_ogm_.ogmwidth / 2) &&
				(newy >=0 && newy < maxz_ogm_.ogmheight) &&
				(z >= - 2 && z <=  Z_MAX))
		{
			int col = boost::math::round(x / maxz_ogm_.ogmresolution) + ( maxz_ogm_.ogmwidth_cell - 1 ) / 2;
			int row = boost::math::round(newy / maxz_ogm_.ogmresolution) ;

			if((row >=0 && row < maxz_ogm_.ogmheight_cell)					//加了_cell的才是栅格zx
					&& (col >=0 && col < maxz_ogm_.ogmwidth_cell))
			{
				int index = row * maxz_ogm_.ogmwidth_cell + col;
				if( maxz_ogm_.ogm[index] < z)
					maxz_ogm_.ogm[index] = z;
			}
		}
	}
	///test
	//		for(int i=maxz_ogm_.ogmwidth_cell/2;i<maxz_ogm_.ogmwidth_cell/2+15/maxz_ogm_.ogmresolution;i++){
	//				int index=35/maxz_ogm_.ogmresolution*maxz_ogm_.ogmwidth_cell+i;
	//				std::cout<<maxz_ogm_.ogm[index]<<" ";
	//			}
	//			std::cout<<std::endl;
	//测试代码,看每个格子的点数/高度
	//		for (int i=20;i<30;i++)
	//		{
	//			for(int j=20;j<40;j++)//车体右侧
	//				{
	////				if(maxz_ogm_.ogm[i*point_count_ogm_.ogmwidth_cell+j]>1){
	//				std::cout<<".."<<maxz_ogm_.ogm[i*point_count_ogm_.ogmwidth_cell+j];
	////				cout<<".."<<j<<".."<<i<<endl;
	////				}
	//			}
	//			std::cout<<std::endl;
	//		}
	//		for(int i=13;i<21;i++){
	//			std::cout<<point_count_ogm_.ogm[35*point_count_ogm_.ogmwidth_cell+i]<<"  ";
	//		}
	//		std::cout<<std::endl;
	//下面将考虑通过检测比较大片的无点区域判断是否为悬崖zx
	//		1、首先检测小区域内有没有悬崖区域等。
	for(int i=10/point_count_ogm_.ogmresolution;i<30/point_count_ogm_.ogmresolution;i++)//
	{
		float thresh;
		if (i>15/point_count_ogm_.ogmresolution){thresh=GRID_THRESH;}
		else {thresh=GRID_THRESH+3;}
		//			std::cout<<"......11111111111111.................."<<std::endl;
		int end_right=point_count_ogm_.ogmwidth_cell/2+16/point_count_ogm_.ogmresolution;
		for(int j=point_count_ogm_.ogmwidth_cell/2+4/point_count_ogm_.ogmresolution;j<end_right;j++)//车体右侧
		{

			float actual_x=(j-20/point_count_ogm_.ogmresolution)*0.2;
			float actual_y=(i-20/point_count_ogm_.ogmresolution)*0.2;
			if((actual_x*actual_x+actual_y*actual_y)>RADIUS){
				continue;
			}
			int count=0;
			int index=i*point_count_ogm_.ogmwidth_cell+j;
			//用于记录连续没有点云的栅格数
			float bound=0;
			while(point_count_ogm_.ogm[index]<POINT_COUNT_THRESH&&bound<RADIUS)//0221这里的10之前是16zx
			{
				//									std::cout<<"................"<<count+i<<std::endl;
				count++;
				index++;
				bound=(j-20/point_count_ogm_.ogmresolution+count)*0.2*(j-20/point_count_ogm_.ogmresolution+count)*0.2+(i-20/point_count_ogm_.ogmresolution)*0.2
						*(i-20/point_count_ogm_.ogmresolution)*0.2;

			}
			//								int tempindex=index;
			//								int index_img=(point_count_ogm_.ogmheight_cell-i-1)*point_count_ogm_.ogmwidth_cell+j;//这是在图像上的索引，用于测试
			index=i*ogm_msg_.ogmwidth_cell+j;//栅格地图上悬崖起始点索引
			int index2=(4*i+1)*ogm_msg_.ogmwidth_cell+4*j;
			int index3=(4*i+2)*ogm_msg_.ogmwidth_cell+4*j;
			int index4=(4*i+3)*ogm_msg_.ogmwidth_cell+4*j;
			if(count>thresh)								//可调参数
			{
				//									float thresh_flat=0,maxz1=-10,maxz2=-10;//用来排除地面点云间隙造成的误检
				//									if(j+count==end_right-1){thresh_flat=-1;}//如果无点区域直接出界
				//									else{
				//										for(int k=4*i;k<4*i+4;k++){
				//											for(int l=4*j-4;l<4*j;l++){
				//												int small_i=k*maxz_ogm_.ogmwidth_cell+l;
				//												if(maxz_ogm_.ogm[small_i]>maxz1){
				//													maxz1=maxz_ogm_.ogm[small_i];
				//													if(abs(maxz1)>0.1) break;
				//												}
				//											}
				//											if(abs(maxz1)>0.1) break;
				//										}
				//										for(int k=4*i;k<4*i+4;k++){
				//											for(int l=4*(j+count);l<4*(j+count+1);l++){
				//												int small_i=k*maxz_ogm_.ogmwidth_cell+l;
				//												if(maxz_ogm_.ogm[small_i]>maxz2){
				//													maxz2=maxz_ogm_.ogm[small_i];
				//													if(abs(maxz2)>0.1) break;
				//												}
				//											}
				//											if(abs(maxz2)>0.1) break;
				//										}
				//										thresh_flat=(maxz1!=0&&maxz2!=0)?FLAT_THRESH:(-1);
				//									}

#ifdef SMALLGRID
				if((pathClear(i,j)&&pathClear(i,j+int(count/3)))&&pathClear(i,j+int(0.66*count))&&pathClear(i,j+count))
					//									if((pathClear(2*i,2*j)&&pathClear(2*i,2*(j+int(count/3))))&&pathClear(2*i,2*(j+int(0.66*count)))&&pathClear(2*i,2*(j+count)))
#else
					if((pathClear(4*i,j*4)&&pathClear((i+count/2)*4,4*j))&&pathClear((i+count)*4+3,4*j)&&abs(maxz1-maxz2)>thresh_flat)	//判断一下由雷达到无点区域中点之间有无障碍
#endif
					{
						//										std::cout<<"<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>"<<std::endl;
						memset(&ogm_msg_.ogm[index],5,count*sizeof(unsigned char));
						//										memset(&ogm_msg_.ogm[index2],5,count*4*sizeof(unsigned char));
						//										memset(&ogm_msg_.ogm[index3],5,count*4*sizeof(unsigned char));
						//										memset(&ogm_msg_.ogm[index4],5,count*4*sizeof(unsigned char));

						//										for(int d=0;d<count;d++)
						//										{
						//											std::cout<<ogm_msg_.ogm[index+d]<<"  ";
						//										}
						//										std::cout<<"........................"<<std::endl;
						vecright_.push_back(point_count_ogm_.ogmheight_cell-i-1);
						vecright_.push_back(j);
						vecright_.push_back(count);

					}
			}
			if(count>0)
				j+=count-1;
		}

		for(int j=point_count_ogm_.ogmwidth_cell/2-20/point_count_ogm_.ogmresolution;j<point_count_ogm_.ogmwidth_cell/2-4/point_count_ogm_.ogmresolution;j++)//车体左侧
		{
			float actual_x=(j-20/point_count_ogm_.ogmresolution)*0.2;
			float actual_y=(i-20/point_count_ogm_.ogmresolution)*0.2;
			if((actual_x*actual_x+actual_y*actual_y)>RADIUS)
			{
				continue;
			}
			int count=0;
			int index=i*point_count_ogm_.ogmwidth_cell+j;
			//用于记录连续没有点云的栅格数
			float bound=0;
			while(point_count_ogm_.ogm[index]<POINT_COUNT_THRESH&&bound<RADIUS&&count<point_count_ogm_.ogmwidth_cell/2-4/point_count_ogm_.ogmresolution-j)//0221这里的10之前是16zx
			{
				//									std::cout<<"................"<<count+i<<std::endl;
				count++;
				index++;
				bound=(j-20/point_count_ogm_.ogmresolution+count)*0.2*(j-20/point_count_ogm_.ogmresolution+count)*0.2+(i-20/point_count_ogm_.ogmresolution)*0.2
						*(i-20/point_count_ogm_.ogmresolution)*0.2;

			}
			int tempindex=index;
			//								int index_img=(point_count_ogm_.ogmheight_cell-i-1)*point_count_ogm_.ogmwidth_cell+j;//这是在图像上的索引，用于测试
			index=i*ogm_msg_.ogmwidth_cell+j;//栅格地图上悬崖起始点索引
			int index2=(4*i+1)*ogm_msg_.ogmwidth_cell+4*j;
			int index3=(4*i+2)*ogm_msg_.ogmwidth_cell+4*j;
			int index4=(4*i+3)*ogm_msg_.ogmwidth_cell+4*j;
			if(count>thresh)								//可调参数
			{
				//									float thresh_flat=0,maxz1=-10,maxz2=-10;//用来排除地面点云间隙造成的误检
				//									if(j==point_count_ogm_.ogmwidth_cell/2-20/point_count_ogm_.ogmresolution){thresh_flat=-1;}
				//									else{
				//										for(int k=4*i;k<4*i+4;k++){
				//											for(int l=4*j-1;l>4*j-5;l--){
				//												int small_i=k*maxz_ogm_.ogmwidth_cell+l;
				//												if(maxz_ogm_.ogm[small_i]>maxz1){
				//													maxz1=maxz_ogm_.ogm[small_i];
				//													if(abs(maxz1)>0.1) break;
				//												}
				//											}
				//											if(abs(maxz1)>0.1) break;
				//										}
				//										for(int k=4*i;k<4*i+4;k++){
				//											for(int l=4*(j+count);l<4*(j+count+1);l++){
				//												int small_i=k*maxz_ogm_.ogmwidth_cell+l;
				//												if(maxz_ogm_.ogm[small_i]>maxz2){
				//													maxz2=maxz_ogm_.ogm[small_i];
				//													if(abs(maxz2)>0.1) break;
				//												}
				//											}
				//											if(abs(maxz2)>0.1) break;
				//										}
				//										thresh_flat=(maxz1!=0&&maxz2!=0)?FLAT_THRESH:(-1);
				//									}
#ifdef SMALLGRID
				if((pathClear(i,j)&&pathClear(i,j+int(count/3)))&&pathClear(i,j+int(0.66*count))&&pathClear(i,j+count))
					//									if((pathClear(2*i,2*j)&&pathClear(2*i,2*(j+int(count/3))))&&pathClear(2*i,2*(j+int(0.66*count)))&&pathClear(2*i,2*(j+count)))
#else
					if((pathClear(4*i,j*4)&&pathClear((i+count/2)*4,4*j))&&pathClear((i+count)*4+3,4*j)&&abs(maxz1-maxz2)>thresh_flat)	//判断一下由雷达到无点区域中点之间有无障碍
#endif
					{
						//										cout<<"the maxz is  "<<maxz2<<"   "<<maxz1<<endl;
						//										cout<<"absolute value is "<<abs(maxz1-maxz2)<<endl;
						memset(&ogm_msg_.ogm[index],5,count*sizeof(unsigned char));
						//										memset(&ogm_msg_.ogm[index2],5,count*4*sizeof(unsigned char));
						//										memset(&ogm_msg_.ogm[index3],5,count*4*sizeof(unsigned char));
						//										memset(&ogm_msg_.ogm[index4],5,count*4*sizeof(unsigned char));

						//										for(int d=0;d<count;d++)
						//										{
						//											std::cout<<ogm_msg_.ogm[index+d]<<"  ";
						//										}
						//										std::cout<<"........................"<<std::endl;
						vecright_.push_back(point_count_ogm_.ogmheight_cell-i-1);
						vecright_.push_back(j);
						vecright_.push_back(count);

					}
			}
			if(count>0)
				j+=count-1;


		}
		//
	}
	//test
	//		for(int i=13;i<21;i++){
	//			int index=35*point_count_ogm_.ogmwidth_cell+i;
	//			int count=0;
	//			while(point_count_ogm_.ogm[index]==0){
	//				count++;
	//				index++;
	//			}
	//			if(count>2){
	//				if(pathClear(35,i)){
	//					std::cout<<"yesyesyes"<<std::endl;
	//				}
	//				else cout<<"nonono"<<std::endl;
	//			}
	//		}
	//test done
	for(int j=point_count_ogm_.ogmwidth_cell/2-4/point_count_ogm_.ogmresolution;j<point_count_ogm_.ogmwidth_cell/2+4/point_count_ogm_.ogmresolution;j++)//车体上下范围
	{

		//			for(int i=0;i<20/point_count_ogm_.ogmresolution;i++)//-3/point_count_ogm_.ogmresolution
		//			{
		//				int count=0;
		//				//排除车体周围范围
		//				if(j>point_count_ogm_.ogmwidth_cell/2-3/point_count_ogm_.ogmresolution&&j<point_count_ogm_.ogmwidth_cell/2+3/point_count_ogm_.ogmresolution
		//						&&i<point_count_ogm_.ogmheight_cell/2+2/point_count_ogm_.ogmresolution&&i>point_count_ogm_.ogmheight_cell/2-4/point_count_ogm_.ogmresolution)
		//					continue;
		//				int index=i*point_count_ogm_.ogmwidth_cell+j;
		//				while(point_count_ogm_.ogm[index]==0&&count<6&&count<point_count_ogm_.ogmheight_cell/2-4/point_count_ogm_.ogmresolution-i)//-point_count_ogm_.ogmheight_cell/2//&&count<point_count_ogm_.ogmheight_cell-i
		//				{
		//					count++;
		//					index+=point_count_ogm_.ogmwidth_cell;
		//				}
		//				index=i*2*ogm_msg_.ogmwidth_cell+j*2;//这是在栅格坐标系下的索引zx
		////				int index_img=(point_count_ogm_.ogmheight_cell-i-1)*point_count_ogm_.ogmwidth_cell+j;//这是在图像上的索引，用于测试
		//				if(count>GRID_THRESH)								//可调参数
		//				{
		//					if(pathClear(i,j))	//判断一下由雷达到无点区域中点之间有无障碍  pathClear(j,i+count/2)
		//					{
		//						for(int k=0;k<count;k++)
		//						{
		//							memset(&ogm_msg_.ogm[index],5,2*sizeof(unsigned char));
		//							memset(&ogm_msg_.ogm[index+ogm_msg_.ogmwidth_cell],5,2*sizeof(unsigned char));
		////							std::cout<<ogm_msg_.ogm[index]<<" "<<ogm_msg_.ogm[index+ogm_msg_.ogmwidth_cell]<<std::endl;
		////							ogm_msg_.ogm[index]=5;;
		////							ogm_msg_.ogm[index+1]=5;
		//							index+=2*ogm_msg_.ogmwidth_cell;			//在图像和栅格坐标系下分别是+= 和-=
		//						}
		////										std::cout<<"........................"<<std::endl;
		//						vecup_.push_back(point_count_ogm_.ogmheight_cell-i-1);
		//						vecup_.push_back(j);
		//						vecup_.push_back(count);
		//
		//					}
		//				}
		//				if(count>0) i+=count-1;
		//
		//			}
		for(int i=20/point_count_ogm_.ogmresolution+6/point_count_ogm_.ogmresolution;i<50/point_count_ogm_.ogmresolution;i++)
		{
			float actual_x=(j-20/point_count_ogm_.ogmresolution)*0.2;
			float actual_y=(i-20/point_count_ogm_.ogmresolution)*0.2;
			if((actual_x*actual_x+actual_y*actual_y)>RADIUS)
			{
				continue;
			}
			int thresh;
			if(i<40/point_count_ogm_.ogmresolution) thresh=GRID_THRESH;
			else if(i<50/point_count_ogm_.ogmresolution) thresh=GRID_THRESH2;
			else thresh=GRID_THRESH3;
			int count=0;
			if(j>point_count_ogm_.ogmwidth_cell/2-3/point_count_ogm_.ogmresolution&&j<point_count_ogm_.ogmwidth_cell/2+3/point_count_ogm_.ogmresolution
					&&i<point_count_ogm_.ogmheight_cell/2+2/point_count_ogm_.ogmresolution&&i>point_count_ogm_.ogmheight_cell/2-4/point_count_ogm_.ogmresolution)
				continue;
			int index=i*point_count_ogm_.ogmwidth_cell+j;
			float bound=0;

			while(point_count_ogm_.ogm[index]<POINT_COUNT_THRESH&&bound<RADIUS)//&&count<point_count_ogm_.ogmheight_cell-i
			{
				//					cout<<"the point count is  "<<i<<"  "<<j<<endl;
				//					cout<<"the index is "<<index<<endl;
				bound=(j-20/point_count_ogm_.ogmresolution)*0.2*(j-20/point_count_ogm_.ogmresolution)*0.2+(i-20/point_count_ogm_.ogmresolution+count)*0.2
						*(i-20/point_count_ogm_.ogmresolution+count)*0.2;

				count++;
				index+=point_count_ogm_.ogmwidth_cell;
			}
			//				cout<<"the count is "<<count<<endl;
			//							index=i*point_count_ogm_.ogmwidth_cell+j;//复位zx
			index=i*ogm_msg_.ogmwidth_cell+j;
			//							int index_img=(point_count_ogm_.ogmheight_cell-i-1)*point_count_ogm_.ogmwidth_cell+j;//这是在图像上的索引，用于测试
			if(count>GRID_THRESH)								//可调参数
			{
				if(pathClear(i,j))	//判断一下由雷达到无点区域中点之间有无障碍  pathClear(j,i+count/2)
				{
					for(int k=0;k<count;k++)
					{
						ogm_msg_.ogm[index]=5;
						index+=point_count_ogm_.ogmwidth_cell; //在图像和栅格坐标系下分别是+= 和-=


					}
					//						std::cout<<"............jj is............"<<j<<std::endl;
					vecup_.push_back(point_count_ogm_.ogmheight_cell-i-1);
					vecup_.push_back(j);
					vecup_.push_back(count);

				}
			}
			if(count>0) i+=count-1;

		}
	}
	//		2、检测10m之外
	//		2.1、左右两侧
	for(int i=30/point_count_ogm_big_.ogmresolution;i<41/point_count_ogm_big_.ogmresolution;i++)//
	{

		//			std::cout<<"......11111111111111.................."<<std::endl;
		float thresh;
		if(i<40/point_count_ogm_big_.ogmresolution){thresh=GRID_THRESH_BIG;}
		else{thresh=int(GRID_THRESH_BIG*1.5);}
		int end_right=point_count_ogm_big_.ogmwidth_cell/2+20/point_count_ogm_big_.ogmresolution;
		for(int j=point_count_ogm_big_.ogmwidth_cell/2+4/point_count_ogm_big_.ogmresolution;j<end_right;j++)//车体右侧
		{

			float actual_x=(j-20/point_count_ogm_big_.ogmresolution)*0.4;
			float actual_y=(i-20/point_count_ogm_big_.ogmresolution)*0.4;
			if((actual_x*actual_x+actual_y*actual_y)<RADIUS||((actual_x*actual_x+actual_y*actual_y)>400)){
				continue;
			}
			int count=0;
			int index=i*point_count_ogm_big_.ogmwidth_cell+j;
			//用于记录连续没有点云的栅格数
			float bound=(j-20/point_count_ogm_big_.ogmresolution)*0.4*(j-20/point_count_ogm_big_.ogmresolution)*0.4+(i-20/point_count_ogm_big_.ogmresolution)*0.4
					*(i-20/point_count_ogm_big_.ogmresolution)*0.4;
			while(point_count_ogm_big_.ogm[index]<POINT_COUNT_THRESH&&bound<400)//0221这里的10之前是16zx
			{
				//									std::cout<<"................"<<count+i<<std::endl;
				count++;
				index++;
				bound=(j-20/point_count_ogm_big_.ogmresolution+count)*0.4*(j-20/point_count_ogm_big_.ogmresolution+count)*0.4+(i-20/point_count_ogm_big_.ogmresolution)*0.4
						*(i-20/point_count_ogm_big_.ogmresolution)*0.4;



			}
			//								int tempindex=index;
			//								int index_img=(point_count_ogm_big_.ogmheight_cell-i-1)*point_count_ogm_big_.ogmwidth_cell+j;//这是在图像上的索引，用于测试
			index=4*i*ogm_msg_.ogmwidth_cell+j*4;//栅格地图上悬崖起始点索引
			int index2=(4*i+1)*ogm_msg_.ogmwidth_cell+4*j;
			int index3=(4*i+2)*ogm_msg_.ogmwidth_cell+4*j;
			int index4=(4*i+3)*ogm_msg_.ogmwidth_cell+4*j;
			if(count>thresh)								//可调参数
			{
				//											float thresh_flat=0,maxz1=-10,maxz2=-10;//用来排除地面点云间隙造成的误检
				//											if(j+count==end_right-1){thresh_flat=-1;}//如果无点区域直接出界
				//											else{
				//												for(int k=4*i;k<4*i+4;k++){
				//													for(int l=4*j-4;l<4*j;l++){
				//														int small_i=k*maxz_ogm_.ogmwidth_cell+l;
				//														if(maxz_ogm_.ogm[small_i]>maxz1){
				//															maxz1=maxz_ogm_.ogm[small_i];
				//															if(abs(maxz1)>0.1) break;
				//														}
				//													}
				//													if(abs(maxz1)>0.1) break;
				//												}
				//												for(int k=4*i;k<4*i+4;k++){
				//													for(int l=4*(j+count);l<4*(j+count+1);l++){
				//														int small_i=k*maxz_ogm_.ogmwidth_cell+l;
				//														if(maxz_ogm_.ogm[small_i]>maxz2){
				//															maxz2=maxz_ogm_.ogm[small_i];
				//															if(abs(maxz2)>0.1) break;
				//														}
				//													}
				//													if(abs(maxz2)>0.1) break;
				//												}
				//												thresh_flat=(maxz1!=0&&maxz2!=0)?FLAT_THRESH:(-1);
				//											}

#ifdef SMALLGRID
				//									if((pathClear(i,j)&&pathClear(i,j+int(count/3)))&&pathClear(i,j+int(0.66*count))&&pathClear(i,j+count))
				//									if((pathClear(2*i,2*j)&&pathClear(2*i,2*(j+int(count/3))))&&pathClear(2*i,2*(j+int(0.66*count)))&&pathClear(2*i,2*(j+count)))

				//											if((pathClear(4*i,j*4)&&pathClear((i)*4,4*(j+count/2)))&&pathClear((i)*4,4*(j+count))&&abs(maxz1-maxz2)>thresh_flat)	//判断一下由雷达到无点区域中点之间有无障碍
				bool state=false;
				for(int kk=0;kk<count;kk++){
					state=state||pathClear(2*i,2*(j+kk));

					if(state) {

						break;
					}
				}
				//											if((pathClear(2*i,j*2)&&pathClear((i)*2,2*(j+count/2)))&&pathClear((i)*2,2*(j+count)))
				if(state)//
#endif
				{
					//										std::cout<<"<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>"<<std::endl;
					//												memset(&ogm_msg_.ogm[index],5,4*count*sizeof(unsigned char));
					//												memset(&ogm_msg_.ogm[index2],5,count*4*sizeof(unsigned char));
					//												memset(&ogm_msg_.ogm[index3],5,count*4*sizeof(unsigned char));
					//												memset(&ogm_msg_.ogm[index4],5,count*4*sizeof(unsigned char));

					//										for(int d=0;d<count;d++)
					//										{
					//											std::cout<<ogm_msg_.ogm[index+d]<<"  ";
					//										}
					//										std::cout<<"........................"<<std::endl;
					vecright_big_.push_back(point_count_ogm_big_.ogmheight_cell-i-1);
					vecright_big_.push_back(j);
					vecright_big_.push_back(count);

				}
			}
			if(count>0)
				j+=count-1;
		}

		for(int j=point_count_ogm_big_.ogmwidth_cell/2-20/point_count_ogm_big_.ogmresolution;j<point_count_ogm_big_.ogmwidth_cell/2-4/point_count_ogm_big_.ogmresolution;j++)//车体左侧
		{
			float actual_x=(j-20/point_count_ogm_big_.ogmresolution)*0.4;
			float actual_y=(i-20/point_count_ogm_big_.ogmresolution)*0.4;
			if((actual_x*actual_x+actual_y*actual_y)<RADIUS||((actual_x*actual_x+actual_y*actual_y)>400)){
				continue;
			}
			int count=0;
			int index=i*point_count_ogm_big_.ogmwidth_cell+j;
			//用于记录连续没有点云的栅格数

			float bound=(j-20/point_count_ogm_big_.ogmresolution)*0.4*(j-20/point_count_ogm_big_.ogmresolution)*0.4+(i-20/point_count_ogm_big_.ogmresolution)*0.4
					*(i-20/point_count_ogm_big_.ogmresolution)*0.4;
			while(point_count_ogm_big_.ogm[index]<POINT_COUNT_THRESH&&count<point_count_ogm_big_.ogmwidth_cell/2-4/point_count_ogm_big_.ogmresolution-j&&bound<400)//0221这里的10之前是16zx
			{
				//									std::cout<<"................"<<count+i<<std::endl;
				bound=(j-20/point_count_ogm_big_.ogmresolution+count)*0.4*(j-20/point_count_ogm_big_.ogmresolution+count)*0.4+(i-20/point_count_ogm_big_.ogmresolution)*0.4
						*(i-20/point_count_ogm_big_.ogmresolution)*0.4;
				count++;
				index++;


			}
			int tempindex=index;
			//								int index_img=(point_count_ogm_big_.ogmheight_cell-i-1)*point_count_ogm_big_.ogmwidth_cell+j;//这是在图像上的索引，用于测试
			index=4*i*ogm_msg_.ogmwidth_cell+4*j;//栅格地图上悬崖起始点索引
			int index2=(4*i+1)*ogm_msg_.ogmwidth_cell+4*j;
			int index3=(4*i+2)*ogm_msg_.ogmwidth_cell+4*j;
			int index4=(4*i+3)*ogm_msg_.ogmwidth_cell+4*j;
			if(count>thresh)								//可调参数
			{
				//									float thresh_flat=0,maxz1=-10,maxz2=-10;//用来排除地面点云间隙造成的误检
				//									if(j==point_count_ogm_big_.ogmwidth_cell/2-20/point_count_ogm_big_.ogmresolution){thresh_flat=-1;}
				//									else{
				//										for(int k=4*i;k<4*i+4;k++){
				//											for(int l=4*j-1;l>4*j-5;l--){
				//												int small_i=k*maxz_ogm_.ogmwidth_cell+l;
				//												if(maxz_ogm_.ogm[small_i]>maxz1){
				//													maxz1=maxz_ogm_.ogm[small_i];
				//													if(abs(maxz1)>0.1) break;
				//												}
				//											}
				//											if(abs(maxz1)>0.1) break;
				//										}
				//										for(int k=4*i;k<4*i+4;k++){
				//											for(int l=4*(j+count);l<4*(j+count+1);l++){
				//												int small_i=k*maxz_ogm_.ogmwidth_cell+l;
				//												if(maxz_ogm_.ogm[small_i]>maxz2){
				//													maxz2=maxz_ogm_.ogm[small_i];
				//													if(abs(maxz2)>0.1) break;
				//												}
				//											}
				//											if(abs(maxz2)>0.1) break;
				//										}
				//										thresh_flat=(maxz1!=0&&maxz2!=0)?FLAT_THRESH:(-1);
				//									}
#ifdef SMALLGRID
				//									if((pathClear(i,j)&&pathClear(i,j+int(count/3)))&&pathClear(i,j+int(0.66*count))&&pathClear(i,j+count))
				//									if((pathClear(2*i,2*j)&&pathClear(2*i,2*(j+int(count/3))))&&pathClear(2*i,2*(j+int(0.66*count)))&&pathClear(2*i,2*(j+count)))
				//#else
				//											if((pathClear(4*i,j*4)&&pathClear((i)*4,4*(j+count/2)))&&pathClear((i)*4,4*(j+count)))	//判断一下由雷达到无点区域中点之间有无障碍
				bool state=false;
				for(int kk=0;kk<count;kk++){
					state=state||pathClear(2*i,2*(j+kk));

					if(state) {

						break;
					}
				}
				//											if((pathClear(2*i,j*2)&&pathClear((i)*2,2*(j+count/2)))&&pathClear((i)*2,2*(j+count)))
				if(state)//
#endif
				{
					//										cout<<"the maxz is  "<<maxz2<<"   "<<maxz1<<endl;
					//										cout<<"absolute value is "<<abs(maxz1-maxz2)<<endl;
					//												memset(&ogm_msg_.ogm[index],5,count*4*sizeof(unsigned char));
					//												memset(&ogm_msg_.ogm[index2],5,count*4*sizeof(unsigned char));
					//												memset(&ogm_msg_.ogm[index3],5,count*4*sizeof(unsigned char));
					//												memset(&ogm_msg_.ogm[index4],5,count*4*sizeof(unsigned char));

					//										for(int d=0;d<count;d++)
					//										{
					//											std::cout<<ogm_msg_.ogm[index+d]<<"  ";
					//										}
					//										std::cout<<"........................"<<std::endl;
					vecright_big_.push_back(point_count_ogm_big_.ogmheight_cell-i-1);
					vecright_big_.push_back(j);
					vecright_big_.push_back(count);

				}
			}
			if(count>0)
				j+=count-1;


		}
		//
	}
	//		2.2上下侧
	for(int j=point_count_ogm_big_.ogmwidth_cell/2-4/point_count_ogm_big_.ogmresolution;j<point_count_ogm_big_.ogmwidth_cell/2+4/point_count_ogm_big_.ogmresolution;j++)//车体上下范围
	{

		//			for(int i=0;i<20/point_count_ogm_.ogmresolution;i++)//-3/point_count_ogm_.ogmresolution
		//			{
		//				int count=0;
		//				//排除车体周围范围
		//				if(j>point_count_ogm_.ogmwidth_cell/2-3/point_count_ogm_.ogmresolution&&j<point_count_ogm_.ogmwidth_cell/2+3/point_count_ogm_.ogmresolution
		//						&&i<point_count_ogm_.ogmheight_cell/2+2/point_count_ogm_.ogmresolution&&i>point_count_ogm_.ogmheight_cell/2-4/point_count_ogm_.ogmresolution)
		//					continue;
		//				int index=i*point_count_ogm_.ogmwidth_cell+j;
		//				while(point_count_ogm_.ogm[index]==0&&count<6&&count<point_count_ogm_.ogmheight_cell/2-4/point_count_ogm_.ogmresolution-i)//-point_count_ogm_.ogmheight_cell/2//&&count<point_count_ogm_.ogmheight_cell-i
		//				{
		//					count++;
		//					index+=point_count_ogm_.ogmwidth_cell;
		//				}
		//				index=i*2*ogm_msg_.ogmwidth_cell+j*2;//这是在栅格坐标系下的索引zx
		////				int index_img=(point_count_ogm_.ogmheight_cell-i-1)*point_count_ogm_.ogmwidth_cell+j;//这是在图像上的索引，用于测试
		//				if(count>GRID_THRESH)								//可调参数
		//				{
		//					if(pathClear(i,j))	//判断一下由雷达到无点区域中点之间有无障碍  pathClear(j,i+count/2)
		//					{
		//						for(int k=0;k<count;k++)
		//						{
		//							memset(&ogm_msg_.ogm[index],5,2*sizeof(unsigned char));
		//							memset(&ogm_msg_.ogm[index+ogm_msg_.ogmwidth_cell],5,2*sizeof(unsigned char));
		////							std::cout<<ogm_msg_.ogm[index]<<" "<<ogm_msg_.ogm[index+ogm_msg_.ogmwidth_cell]<<std::endl;
		////							ogm_msg_.ogm[index]=5;;
		////							ogm_msg_.ogm[index+1]=5;
		//							index+=2*ogm_msg_.ogmwidth_cell;			//在图像和栅格坐标系下分别是+= 和-=
		//						}
		////										std::cout<<"........................"<<std::endl;
		//						vecup_.push_back(point_count_ogm_.ogmheight_cell-i-1);
		//						vecup_.push_back(j);
		//						vecup_.push_back(count);
		//
		//					}
		//				}
		//				if(count>0) i+=count-1;
		//
		//			}
		for(int i=30/point_count_ogm_big_.ogmresolution;i<50/point_count_ogm_big_.ogmresolution;i++)
		{
			float actual_x=(j-20/point_count_ogm_big_.ogmresolution)*0.4;
			float actual_y=(i-20/point_count_ogm_big_.ogmresolution)*0.4;
			//								if((actual_x*actual_x+actual_y*actual_y)<RADIUS||((actual_x*actual_x+actual_y*actual_y)>400))
			//								{
			//									continue;
			//								}
			int thresh;
			if(i<40/point_count_ogm_big_.ogmresolution) thresh=GRID_THRESH_BIG;
			else if(i<50/point_count_ogm_big_.ogmresolution) thresh=5/point_count_ogm_big_.ogmresolution;
			//								else thresh=GRID_THRESH3;
			int count=0;
			if(j>point_count_ogm_big_.ogmwidth_cell/2-3/point_count_ogm_big_.ogmresolution&&j<point_count_ogm_big_.ogmwidth_cell/2+3/point_count_ogm_big_.ogmresolution
					&&i<point_count_ogm_big_.ogmheight_cell/2+2/point_count_ogm_big_.ogmresolution&&i>point_count_ogm_big_.ogmheight_cell/2-4/point_count_ogm_big_.ogmresolution)
				continue;
			int index=i*point_count_ogm_big_.ogmwidth_cell+j;
			//						float bound=0;
			if(j==92||j==93||j==94){

			}
			float bound=(j-20/point_count_ogm_big_.ogmresolution)*0.4*(j-20/point_count_ogm_big_.ogmresolution)*0.4+(i-20/point_count_ogm_big_.ogmresolution)*0.4
					*(i-20/point_count_ogm_big_.ogmresolution)*0.4;
			while(point_count_ogm_big_.ogm[index]<POINT_COUNT_THRESH&&count<50/point_count_ogm_big_.ogmresolution-i)//
			{
				//					cout<<"the point count is  "<<i<<"  "<<j<<endl;
				//					cout<<"the index is "<<index<<endl;
				bound=(j-20/point_count_ogm_big_.ogmresolution)*0.4*(j-20/point_count_ogm_big_.ogmresolution)*0.4+(i-20/point_count_ogm_big_.ogmresolution+count)*0.4
						*(i-20/point_count_ogm_big_.ogmresolution+count)*0.4;

				count++;
				index+=point_count_ogm_big_.ogmwidth_cell;
			}
			//				cout<<"the count is "<<count<<endl;
			//							index=i*point_count_ogm_big_.ogmwidth_cell+j;//复位zx
			index=4*i*ogm_msg_.ogmwidth_cell+j*4;
			//							int index_img=(point_count_ogm_big_.ogmheight_cell-i-1)*point_count_ogm_big_.ogmwidth_cell+j;//这是在图像上的索引，用于测试
			if(count>thresh)								//可调参数
			{
				//									if(pathClear(4*i,4*j))	//判断一下由雷达到无点区域中点之间有无障碍  pathClear(j,i+count/2)
				if(pathClear(2*i,2*j))
				{
					//										for(int k=0;k<count;k++)
					//										{
					//											memset(&ogm_msg_.ogm[index],5,4*sizeof(unsigned char));
					//											memset(&ogm_msg_.ogm[index+ogm_msg_.ogmwidth_cell],5,4*sizeof(unsigned char));
					//											memset(&ogm_msg_.ogm[index+2*ogm_msg_.ogmwidth_cell],5,4*sizeof(unsigned char));
					//											memset(&ogm_msg_.ogm[index+3*ogm_msg_.ogmwidth_cell],5,4*sizeof(unsigned char));
					//											index+=4*ogm_msg_.ogmwidth_cell;
					//										}
					//						std::cout<<"............jj is............"<<j<<std::endl;
					vecup_big_.push_back(point_count_ogm_big_.ogmheight_cell-i-1);
					vecup_big_.push_back(j);
					vecup_big_.push_back(count);

				}
			}
			if(count>0) i+=count-1;

		}
	}

}
#endif
void PostProcess::radialDetection(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud){
	memset(point_count_ogm_.ogm , 0 , point_count_ogm_.ogmcell_size*sizeof(int));//每次都要置零,避免填充错乱
	for(int i=0;i<maxz_ogm_. ogmcell_size;i++){
		maxz_ogm_.ogm[i]=-11.11;
		minz_ogm_.ogm[i]=1111;
	}
	float ogm_y_offset = 20.0f;
	//为每个栅格赋值点云数量
	for (int i = 0; i < cloud->points.size(); i++)
	{

		float x = cloud->points[i].x,
				y = cloud->points[i].y,
				z = cloud->points[i].z;
		if(x>-1.5&&x<1.5&&y>-1&&y<4){		//排除车身周围
			continue;
		}
		float newy = y + ogm_y_offset;//ogm_y_offset
		if((x >=-point_count_ogm_.ogmwidth / 2  && x <= point_count_ogm_.ogmwidth / 2) &&//这里的条件是否需要再考虑一下
				(newy >=0 && newy < point_count_ogm_.ogmheight) &&
				( z <=  Z_MAX))
		{
			int col = boost::math::round(x / point_count_ogm_.ogmresolution) + ( point_count_ogm_.ogmwidth_cell - 1 ) / 2;
			int row = boost::math::round(newy / point_count_ogm_.ogmresolution) ;





			if((row >=0 && row < point_count_ogm_.ogmheight_cell)
					&& (col >=0 && col < point_count_ogm_.ogmwidth_cell))
			{
				int index = row * point_count_ogm_.ogmwidth_cell + col;
				point_count_ogm_.ogm[index]++;
				if(maxz_ogm_.ogm[index]<z){
					maxz_ogm_.ogm[index]=z;
				}
				if(minz_ogm_.ogm[index]>z){
					minz_ogm_.ogm[index]=z;
				}
			}

		}
	}
	//	for(int i=20/point_count_ogm_.ogmresolution-5;i<20/point_count_ogm_.ogmresolution+5;i++){
	//		for(int j=20/point_count_ogm_.ogmresolution-5;j<20/point_count_ogm_.ogmresolution+5;j++){
	//			int index =i*point_count_ogm_.ogmwidth_cell+j;
	//			cout<<"point count is "<<point_count_ogm_.ogm[index]<<endl;
	//		}
	//	}
	for(int anglei=0;anglei<359;){
//		if(anglei==90) continue;
		int height0=20/point_count_ogm_.ogmresolution;//+(int)(4*std::sin(anglei*PI/180))/point_count_ogm_.ogmresolution;
		int width0=20/point_count_ogm_.ogmresolution;//+ (int)(4*std::cos(anglei*PI/180))/point_count_ogm_.ogmresolution;
		float k=std::tan(anglei*PI/180);
		vector<int> vecindex;
		int result=radiusCount(k,height0,width0,vecindex,anglei);
		if(result==11){
			for(int i=0;i<vecindex.size();i++){
				vecindex_.push_back(vecindex[i]);
			}
		}
		anglei+=1;
	}
}
float PostProcess::caldis2(int beginh,int beginw,int endh,int endw){
	return (endh-beginh)*0.2*(endh-beginh)*0.2+(endw-beginw)*0.2*(endw-beginw)*0.2;
}
int  PostProcess::radiusCount(const float k,int& height0,int& width0,vector<int>& vecindex,int& anglei)
{
	float d=-0.5;
	int flag=0;
	int index=height0*point_count_ogm_.ogmwidth_cell+width0;
	if(anglei>=0&&anglei<45){flag=1;}
	if(anglei>=45&&anglei<90){flag=2;}
	if(anglei>=90&&anglei<135){flag=3;}
	if(anglei>=135&&anglei<180){flag=4;}
	if(anglei>=180&&anglei<225){flag=5;}
	if(anglei>=225&&anglei<270){flag=6;}
	if(anglei>=270&&anglei<315){flag=7;}
	if(anglei>=315&&anglei<360){flag=8;}
	float maxz1=-11.11;
	float maxz2=-11.11;
	//	if(width0<=width&&height0<=height&&k>1){ flag=2;}
	//	if(width0>=width&&height0<=height&&k<=-1){flag=3;}
	//	if(width0>=width&&height0<=height&&k>-1&&k<0){flag=4;}
	//	if(width0>=width&&height0>=height&&k>0&&k<1){flag=5;}
	//	if(width0>=width&&height0>=height&&k>=1){ flag=6;}
	//	if(width0<=width&&height0>height&&k<=-1){flag=7;}
	//	if(width0<=width&&height0>height&&k>-1&&k<0) {flag=8;}
	//	if(width<width0&&k==0){flag=9;}
	//	if(width>width0&&k==0){flag=10;}
	//		std::cout<<"....."<<flag<<"..."<<height0<<"..."<<height<<"..."<<width0<<"..."<<width<<"...."<<k<<std::endl;
	int height_next=0;
	int width_next=0;
	switch (flag){

	case 1:
		//先走到盲区边上
		while(point_count_ogm_.ogm[index]==0&&width0<point_count_ogm_.ogmwidth_cell&&height0<point_count_ogm_.ogmheight_cell&&caldis2(height0,width0,100,100)<900){
			//			vecindex_.push_back(height0);
			//			vecindex_.push_back(width0);
			width0++;
			d+=k;
			if(d>0) {height0++;d-=1;}
			index=height0*point_count_ogm_.ogmwidth_cell+width0;
		}
		while(width0<point_count_ogm_.ogmwidth_cell&&height0<point_count_ogm_.ogmheight_cell&&caldis2(height0,width0,100,100)<900)
		{
			//			cout<<"<<<<<<<<<<<<"<<endl;
			vecindex.clear();
#ifdef TEST
			vecindex_.push_back(height0);
			vecindex_.push_back(width0);
#endif
			//如果前方有障碍物后续不用判断
			index=height0*point_count_ogm_.ogmwidth_cell+width0;
			if(maxz_ogm_.ogm[index]>PATHTHRESH)			{return -11;}

			if(abs(maxz_ogm_.ogm[index]+11.11)>0.01){
				maxz1=maxz_ogm_.ogm[index];
			}
			int count=0;
			int beginh=height0;
			int beginw=width0;
			while(point_count_ogm_.ogm[index]==0&&width0<point_count_ogm_.ogmwidth_cell&&height0<point_count_ogm_.ogmheight_cell&&caldis2(height0,width0,100,100)<900){
				count++;
				vecindex.push_back(height0);
				vecindex.push_back(width0);
				width0++;
				d+=k;
				if(d>0) {height0++;d-=1;}
				index=height0*point_count_ogm_.ogmwidth_cell+width0;

			}
			int endh=height0;
			int endw=width0;
			if(count>3){
				if(abs(maxz_ogm_.ogm[index]+11.11)>0.01){
					maxz1=maxz_ogm_.ogm[index];
				}
				bool heightstate=true;
				if(abs(maxz1+11.11)>0.01&&abs(maxz2+11.11)>0.01&&(maxz1-maxz2)<0.5){
					heightstate=false;
				}
				float beginradius_2=caldis2(beginh,beginw,100,100);
				float diff_2=caldis2(beginh,beginw,endh,endw);
				//盲区内排除
				float thresh_2=0;
				if(beginradius_2<100){thresh_2=DIFFTHRESH1;}
				else if(beginradius_2<400){thresh_2=DIFFTHRESH2;}
				else{thresh_2=DIFFTHRESH3;}
				//				cout<<"the radius is "<<beginradius_2<<endl;
				//				cout<<"the thresh is "<<thresh_2<<endl;
				if(heightstate&&caldis2(beginh,beginw,100,100)>25&&caldis2(beginh,beginw,100,100)<900&&diff_2>thresh_2){//
					//cout<<"maxzs are"<<maxz1<<"  "<<maxz2<<endl;
					return 11;
				}

				//			判断下一个有点处是否为高度障碍物
				index=height0*point_count_ogm_.ogmwidth_cell+width0;
				if(maxz_ogm_.ogm[index]>PATHTHRESH)			{return -11;}
			}
			width0++;
			d+=k;
			if(d>0) {height0++;d-=1;}

		}return -1;
		//				return true;
	case 2:
		//先走到盲区边上
		while(point_count_ogm_.ogm[index]==0&&width0<point_count_ogm_.ogmwidth_cell&&height0<point_count_ogm_.ogmheight_cell&&caldis2(height0,width0,100,100)<900){
#ifdef TEST
			vecindex_.push_back(height0);
			vecindex_.push_back(width0);
#endif
			height0++;
			d+=1/k;
			if(d>0){width0++;d-=1;}
			index=height0*point_count_ogm_.ogmwidth_cell+width0;

		}
		while(width0<point_count_ogm_.ogmwidth_cell&&height0<point_count_ogm_.ogmheight_cell&&caldis2(height0,width0,100,100)<900)
		{
			vecindex.clear();

			//			cout<<"maxz1 is "<<maxz_ogm_.ogm[index]<<endl;
			//			vecindex_.push_back(height0);
			//			vecindex_.push_back(width0);
			index=height0*point_count_ogm_.ogmwidth_cell+width0;
			//如果前方有障碍物后续不用判断
			if(maxz_ogm_.ogm[index]>PATHTHRESH)	{return -11;}
			if(abs(maxz_ogm_.ogm[index]+11.11)>0.01){
				maxz1=maxz_ogm_.ogm[index];
			}
			int count=0;
			int beginh=height0;
			int beginw=width0;

			while(point_count_ogm_.ogm[index]==0&&width0<point_count_ogm_.ogmwidth_cell&&height0<point_count_ogm_.ogmheight_cell&&caldis2(height0,width0,100,100)<900){
				count++;
				vecindex.push_back(height0);
				vecindex.push_back(width0);
				height0++;
				d+=1/k;
				if(d>0){width0++;d-=1;}
				index=height0*point_count_ogm_.ogmwidth_cell+width0;
			}

			int endh=height0;
			int endw=width0;
			if(count>3){
				if(abs(maxz_ogm_.ogm[index]+11.11)>0.01){
					maxz1=maxz_ogm_.ogm[index];
				}
				bool heightstate=true;
				if(abs(maxz1+11.11)>0.01&&abs(maxz2+11.11)>0.01&&(maxz1-maxz2)<0.5){
					heightstate=false;
				}
				float beginradius_2=caldis2(beginh,beginw,100,100);
				float diff_2=caldis2(beginh,beginw,endh,endw);
				//盲区内排除
				//				if(caldis2(endh,endw,100,100)<25)continue;
				float thresh_2=0;
				if(beginradius_2<100){thresh_2=DIFFTHRESH1;}
				else if(beginradius_2<400){thresh_2=DIFFTHRESH2;}
				else{thresh_2=DIFFTHRESH3;}
				//				cout<<"the radius is "<<beginradius_2<<endl;
				//				cout<<"the thresh is "<<thresh_2<<endl;
				if(heightstate&&caldis2(beginh,beginw,100,100)>25&&caldis2(beginh,beginw,100,100)<900&&diff_2>thresh_2){//
					//cout<<"maxzs are"<<maxz1<<"  "<<maxz2<<endl;
					return 11;
				}

				//			判断下一个有点处是否为高度障碍物
				index=height0*point_count_ogm_.ogmwidth_cell+width0;
				if(maxz_ogm_.ogm[index]>PATHTHRESH)			{return -11;}
			}
			height0++;
			d+=1/k;
			if(d>0){width0++;d-=1;}
		}return -1;
	case 3:
		//先走到盲区边上
		while(point_count_ogm_.ogm[index]==0&&width0>0&&height0<point_count_ogm_.ogmheight_cell&&caldis2(height0,width0,100,100)<900){
#ifdef TEST
			vecindex_.push_back(height0);
			vecindex_.push_back(width0);
#endif
			height0++;
			d-=1/k;
			if(d>0) {width0--;d-=1;}
			index=height0*point_count_ogm_.ogmwidth_cell+width0;
		}
		while(width0>0&&height0<point_count_ogm_.ogmheight_cell&&caldis2(height0,width0,100,100)<900)
		{
			vecindex.clear();

			//			cout<<"maxz1 is "<<maxz_ogm_.ogm[index]<<endl;
			//			vecindex_.push_back(height0);
			//			vecindex_.push_back(width0);
			index=height0*point_count_ogm_.ogmwidth_cell+width0;
			//如果前方有障碍物后续不用判断
			if(maxz_ogm_.ogm[index]>PATHTHRESH)	{return -11;}
			if(abs(maxz_ogm_.ogm[index]+11.11)>0.01){
				maxz1=maxz_ogm_.ogm[index];
			}
			int count=0;
			int beginh=height0;
			int beginw=width0;

			while(point_count_ogm_.ogm[index]==0&&width0>0&&height0<point_count_ogm_.ogmheight_cell&&caldis2(height0,width0,100,100)<900){
				count++;
				vecindex.push_back(height0);
				vecindex.push_back(width0);
				height0++;
				d-=1/k;
				if(d>0) {width0--;d-=1;}
				index=height0*point_count_ogm_.ogmwidth_cell+width0;
			}

			int endh=height0;
			int endw=width0;
			if(count>3){
				if(abs(maxz_ogm_.ogm[index]+11.11)>0.01){
					maxz1=maxz_ogm_.ogm[index];
				}
				bool heightstate=true;
				if(abs(maxz1+11.11)>0.01&&abs(maxz2+11.11)>0.01&&(maxz1-maxz2)<0.5){
					heightstate=false;
				}
				float beginradius_2=caldis2(beginh,beginw,100,100);
				float diff_2=caldis2(beginh,beginw,endh,endw);
				//盲区内排除
				//				if(caldis2(endh,endw,100,100)<25)continue;
				float thresh_2=0;
				if(beginradius_2<100){thresh_2=DIFFTHRESH1;}
				else if(beginradius_2<400){thresh_2=DIFFTHRESH2;}
				else{thresh_2=DIFFTHRESH3;}
				//				cout<<"the radius is "<<beginradius_2<<endl;
				//				cout<<"the thresh is "<<thresh_2<<endl;
				if(heightstate&&caldis2(beginh,beginw,100,100)>25&&caldis2(beginh,beginw,100,100)<900&&diff_2>thresh_2){//
					//cout<<"maxzs are"<<maxz1<<"  "<<maxz2<<endl;
					return 11;
				}

				//			判断下一个有点处是否为高度障碍物
				index=height0*point_count_ogm_.ogmwidth_cell+width0;
				if(maxz_ogm_.ogm[index]>PATHTHRESH)			{return -11;}
			}
			height0++;
			d-=1/k;
			if(d>0) {width0--;d-=1;}
		}return -1;
		//		while(height0<point_count_ogm_.ogmheight_cell)
		//		{
		//			if(maxz_ogm_.ogm[height0*maxz_ogm_.ogmwidth_cell+width0]>PATHTHRESH||maxz_ogm_.ogm[height0*maxz_ogm_.ogmwidth_cell+width0+1]>PATHTHRESH||maxz_ogm_.ogm[height0*maxz_ogm_.ogmwidth_cell+width0-1]>PATHTHRESH){
		//				return false;
		//			}
		//			height0++;
		//			d-=1/k;
		//			if(d>0) {width0--;d-=1;}
		//		}return true;
	case 4:
		//先走到盲区边上
		while(point_count_ogm_.ogm[index]==0&&width0>0&&height0<point_count_ogm_.ogmheight_cell&&caldis2(height0,width0,100,100)<900){
#ifdef TEST
			vecindex_.push_back(height0);
			vecindex_.push_back(width0);
#endif
			width0--;
			d-=k;
			if(d>0) {height0++;d-=1;}
			index=height0*point_count_ogm_.ogmwidth_cell+width0;

		}
		while(width0>0&&height0<point_count_ogm_.ogmheight_cell&&caldis2(height0,width0,100,100)<900)
		{
			vecindex.clear();

			//			cout<<"maxz1 is "<<maxz_ogm_.ogm[index]<<endl;
			//			vecindex_.push_back(height0);
			//			vecindex_.push_back(width0);
			index=height0*point_count_ogm_.ogmwidth_cell+width0;
			//如果前方有障碍物后续不用判断
			if(maxz_ogm_.ogm[index]>PATHTHRESH)	{return -11;}
			if(abs(maxz_ogm_.ogm[index]+11.11)>0.01){
				maxz1=maxz_ogm_.ogm[index];
			}
			int count=0;
			int beginh=height0;
			int beginw=width0;

			while(point_count_ogm_.ogm[index]==0&&width0>0&&height0<point_count_ogm_.ogmheight_cell&&caldis2(height0,width0,100,100)<900){
				count++;
				vecindex.push_back(height0);
				vecindex.push_back(width0);
				width0--;
				d-=k;
				if(d>0) {height0++;d-=1;}
				index=height0*point_count_ogm_.ogmwidth_cell+width0;
			}

			int endh=height0;
			int endw=width0;
			if(count>3){
				if(abs(maxz_ogm_.ogm[index]+11.11)>0.01){
					maxz1=maxz_ogm_.ogm[index];
				}
				bool heightstate=true;
				if(abs(maxz1+11.11)>0.01&&abs(maxz2+11.11)>0.01&&(maxz1-maxz2)<0.5){
					heightstate=false;
				}
				float beginradius_2=caldis2(beginh,beginw,100,100);
				float diff_2=caldis2(beginh,beginw,endh,endw);
				//盲区内排除
				//				if(caldis2(endh,endw,100,100)<25)continue;
				float thresh_2=0;
				if(beginradius_2<100){thresh_2=DIFFTHRESH1;}
				else if(beginradius_2<400){thresh_2=DIFFTHRESH2;}
				else{thresh_2=DIFFTHRESH3;}
				//				cout<<"the radius is "<<beginradius_2<<endl;
				//				cout<<"the thresh is "<<thresh_2<<endl;
				if(heightstate&&caldis2(beginh,beginw,100,100)>25&&caldis2(beginh,beginw,100,100)<900&&diff_2>thresh_2){//
					//cout<<"maxzs are"<<maxz1<<"  "<<maxz2<<endl;
					return 11;
				}

				//			判断下一个有点处是否为高度障碍物
				index=height0*point_count_ogm_.ogmwidth_cell+width0;
				if(maxz_ogm_.ogm[index]>PATHTHRESH)			{return -11;}
			}
			width0--;
			d-=k;
			if(d>0) {height0++;d-=1;}
		}return -1;
		//		while(width0>0)
		//		{
		//
		//			if(maxz_ogm_.ogm[height0*maxz_ogm_.ogmwidth_cell+width0]>PATHTHRESH||maxz_ogm_.ogm[(height0+1)*maxz_ogm_.ogmwidth_cell+width0]>PATHTHRESH||maxz_ogm_.ogm[(height0-1)*maxz_ogm_.ogmwidth_cell+width0]>PATHTHRESH){
		//				return false;
		//			}
		//			width0--;
		//			d-=k;
		//			if(d>0) {height0++;d-=1;}
		//
		//		}return true;
	case 5:
		//先走到盲区边上
		while(point_count_ogm_.ogm[index]==0&&width0>0&&height0>0&&caldis2(height0,width0,100,100)<100){
#ifdef TEST
			vecindex_.push_back(height0);
			vecindex_.push_back(width0);
#endif
			width0--;
			d+=k;
			if(d>0){height0--;d-=1;}
			index=height0*point_count_ogm_.ogmwidth_cell+width0;

		}
		while(width0>0&&height0>0&&caldis2(height0,width0,100,100)<100)
		{
			vecindex.clear();

			//			cout<<"maxz1 is "<<maxz_ogm_.ogm[index]<<endl;
			//			vecindex_.push_back(height0);
			//			vecindex_.push_back(width0);
			index=height0*point_count_ogm_.ogmwidth_cell+width0;
			//如果前方有障碍物后续不用判断
			if(maxz_ogm_.ogm[index]>PATHTHRESH)	{return -11;}
			if(abs(maxz_ogm_.ogm[index]+11.11)>0.01){
				maxz1=maxz_ogm_.ogm[index];
			}
			int count=0;
			int beginh=height0;
			int beginw=width0;

			while(point_count_ogm_.ogm[index]==0&&width0>0&&height0>0&&caldis2(height0,width0,100,100)<100){
				count++;
				vecindex.push_back(height0);
				vecindex.push_back(width0);
				width0--;
				d+=k;
				if(d>0){height0--;d-=1;}
				index=height0*point_count_ogm_.ogmwidth_cell+width0;
			}

			int endh=height0;
			int endw=width0;
			if(count>3){
				if(abs(maxz_ogm_.ogm[index]+11.11)>0.01){
					maxz1=maxz_ogm_.ogm[index];
				}
				bool heightstate=true;
				if(abs(maxz1+11.11)>0.01&&abs(maxz2+11.11)>0.01&&(maxz1-maxz2)<0.5){
					heightstate=false;
				}
				float beginradius_2=caldis2(beginh,beginw,100,100);
				float diff_2=caldis2(beginh,beginw,endh,endw);
				//盲区内排除
				//				if(caldis2(endh,endw,100,100)<25)continue;
				float thresh_2=0;
				if(beginradius_2<100){thresh_2=DIFFTHRESH1;}
				else if(beginradius_2<400){thresh_2=DIFFTHRESH2;}
				else{thresh_2=DIFFTHRESH3;}
				//				cout<<"the radius is "<<beginradius_2<<endl;
				//				cout<<"the thresh is "<<thresh_2<<endl;
				if(heightstate&&caldis2(beginh,beginw,100,100)>25&&caldis2(beginh,beginw,100,100)<900&&diff_2>thresh_2){//
					//cout<<"maxzs are"<<maxz1<<"  "<<maxz2<<endl;
					return 11;
				}

				//			判断下一个有点处是否为高度障碍物
				index=height0*point_count_ogm_.ogmwidth_cell+width0;
				if(maxz_ogm_.ogm[index]>PATHTHRESH)			{return -11;}
			}
			width0--;
			d+=k;
			if(d>0){height0--;d-=1;}
		}return -1;
		//		while(width0>0)
		//		{
		//
		//			if(maxz_ogm_.ogm[height0*maxz_ogm_.ogmwidth_cell+width0]>PATHTHRESH||maxz_ogm_.ogm[(height0+1)*maxz_ogm_.ogmwidth_cell+width0]>PATHTHRESH||maxz_ogm_.ogm[(height0-1)*maxz_ogm_.ogmwidth_cell+width0]>PATHTHRESH){
		//				return false;}
		//			width0--;
		//			d+=k;
		//			if(d>0){height0--;d-=1;}
		//		}return true;
	case 6:
		//先走到盲区边上
		while(point_count_ogm_.ogm[index]==0&&width0>0&&height0>0&&caldis2(height0,width0,100,100)<100){
#ifdef TEST
			vecindex_.push_back(height0);
			vecindex_.push_back(width0);
#endif
			height0--;
			d+=1/k;
			if(d>0) {width0--;d-=1;}
			index=height0*point_count_ogm_.ogmwidth_cell+width0;

		}
		while(width0>0&&height0>0&&caldis2(height0,width0,100,100)<100)
		{
			vecindex.clear();

			//			cout<<"maxz1 is "<<maxz_ogm_.ogm[index]<<endl;
			//			vecindex_.push_back(height0);
			//			vecindex_.push_back(width0);
			index=height0*point_count_ogm_.ogmwidth_cell+width0;
			//如果前方有障碍物后续不用判断
			if(maxz_ogm_.ogm[index]>PATHTHRESH)	{return -11;}
			if(abs(maxz_ogm_.ogm[index]+11.11)>0.01){
				maxz1=maxz_ogm_.ogm[index];
			}
			int count=0;
			int beginh=height0;
			int beginw=width0;

			while(point_count_ogm_.ogm[index]==0&&width0>0&&height0>0&&caldis2(height0,width0,100,100)<100){
				count++;
				vecindex.push_back(height0);
				vecindex.push_back(width0);
				height0--;
				d+=1/k;
				if(d>0) {width0--;d-=1;}
				index=height0*point_count_ogm_.ogmwidth_cell+width0;
			}

			int endh=height0;
			int endw=width0;
			if(count>3){
				if(abs(maxz_ogm_.ogm[index]+11.11)>0.01){
					maxz1=maxz_ogm_.ogm[index];
				}
				bool heightstate=true;
				if(abs(maxz1+11.11)>0.01&&abs(maxz2+11.11)>0.01&&(maxz1-maxz2)<0.5){
					heightstate=false;
				}
				float beginradius_2=caldis2(beginh,beginw,100,100);
				float diff_2=caldis2(beginh,beginw,endh,endw);
				//盲区内排除
				//				if(caldis2(endh,endw,100,100)<25)continue;
				float thresh_2=0;
				if(beginradius_2<100){thresh_2=DIFFTHRESH1;}
				else if(beginradius_2<400){thresh_2=DIFFTHRESH2;}
				else{thresh_2=DIFFTHRESH3;}
				//				cout<<"the radius is "<<beginradius_2<<endl;
				//				cout<<"the thresh is "<<thresh_2<<endl;
				if(heightstate&&caldis2(beginh,beginw,100,100)>25&&caldis2(beginh,beginw,100,100)<900&&diff_2>thresh_2){//
					//cout<<"maxzs are"<<maxz1<<"  "<<maxz2<<endl;
					return 11;
				}

				//			判断下一个有点处是否为高度障碍物
				index=height0*point_count_ogm_.ogmwidth_cell+width0;
				if(maxz_ogm_.ogm[index]>PATHTHRESH)			{return -11;}
			}
			height0--;
			d+=1/k;
			if(d>0) {width0--;d-=1;}
		}return -1;
//		while(height0>0)
//		{
//			if(maxz_ogm_.ogm[height0*maxz_ogm_.ogmwidth_cell+width0]>PATHTHRESH||maxz_ogm_.ogm[height0*maxz_ogm_.ogmwidth_cell+width0+1]>PATHTHRESH||maxz_ogm_.ogm[height0*maxz_ogm_.ogmwidth_cell+width0-1]>PATHTHRESH){
//				return false;}
//			height0--;
//			d+=1/k;
//			if(d>0) {width0--;d-=1;}
//
//		}return true;
	case 7:
		//先走到盲区边上
		while(point_count_ogm_.ogm[index]==0&&width0<point_count_ogm_.ogmwidth_cell&&height0>0&&caldis2(height0,width0,100,100)<100){
#ifdef TEST
			vecindex_.push_back(height0);
			vecindex_.push_back(width0);
#endif
			height0--;
			d-=1/k;
			if(d>0){width0++;d-=1;}
			index=height0*point_count_ogm_.ogmwidth_cell+width0;

		}
		while(width0<point_count_ogm_.ogmwidth_cell&&height0>0&&caldis2(height0,width0,100,100)<100)
		{
			vecindex.clear();

			//			cout<<"maxz1 is "<<maxz_ogm_.ogm[index]<<endl;
			//			vecindex_.push_back(height0);
			//			vecindex_.push_back(width0);
			index=height0*point_count_ogm_.ogmwidth_cell+width0;
			//如果前方有障碍物后续不用判断
			if(maxz_ogm_.ogm[index]>PATHTHRESH)	{return -11;}
			if(abs(maxz_ogm_.ogm[index]+11.11)>0.01){
				maxz1=maxz_ogm_.ogm[index];
			}
			int count=0;
			int beginh=height0;
			int beginw=width0;

			while(point_count_ogm_.ogm[index]==0&&width0<point_count_ogm_.ogmwidth_cell&&height0>0&&caldis2(height0,width0,100,100)<100){
				count++;
				vecindex.push_back(height0);
				vecindex.push_back(width0);
				height0--;
				d-=1/k;
				if(d>0){width0++;d-=1;}
				index=height0*point_count_ogm_.ogmwidth_cell+width0;
			}

			int endh=height0;
			int endw=width0;
			if(count>3){
				if(abs(maxz_ogm_.ogm[index]+11.11)>0.01){
					maxz1=maxz_ogm_.ogm[index];
				}
				bool heightstate=true;
				if(abs(maxz1+11.11)>0.01&&abs(maxz2+11.11)>0.01&&(maxz1-maxz2)<0.5){
					heightstate=false;
				}
				float beginradius_2=caldis2(beginh,beginw,100,100);
				float diff_2=caldis2(beginh,beginw,endh,endw);
				//盲区内排除
				//				if(caldis2(endh,endw,100,100)<25)continue;
				float thresh_2=0;
				if(beginradius_2<100){thresh_2=DIFFTHRESH1;}
				else if(beginradius_2<400){thresh_2=DIFFTHRESH2;}
				else{thresh_2=DIFFTHRESH3;}
				//				cout<<"the radius is "<<beginradius_2<<endl;
				//				cout<<"the thresh is "<<thresh_2<<endl;
				if(heightstate&&caldis2(beginh,beginw,100,100)>25&&caldis2(beginh,beginw,100,100)<900&&diff_2>thresh_2){//
					//cout<<"maxzs are"<<maxz1<<"  "<<maxz2<<endl;
					return 11;
				}

				//			判断下一个有点处是否为高度障碍物
				index=height0*point_count_ogm_.ogmwidth_cell+width0;
				if(maxz_ogm_.ogm[index]>PATHTHRESH)			{return -11;}
			}
			height0--;
			d-=1/k;
			if(d>0){width0++;d-=1;}
		}return -1;
		//				std::cout<<"..."<<"this is case8"<<std::endl;
//		while(height0>0)
//		{
//			int index=height0*maxz_ogm_.ogmwidth_cell+width0;
//			//				std::cout<<maxz_ogm_.ogm[index]<<" ";
//			if(maxz_ogm_.ogm[height0*maxz_ogm_.ogmwidth_cell+width0]>PATHTHRESH||maxz_ogm_.ogm[height0*maxz_ogm_.ogmwidth_cell+width0+1]>PATHTHRESH||maxz_ogm_.ogm[height0*maxz_ogm_.ogmwidth_cell+width0-1]>PATHTHRESH){
//				return false;}
//			height0--;
//			d-=1/k;
//			if(d>0){width0++;d-=1;}
//
//		}return true;//std::cout<<std::endl;
	case 8:
		//先走到盲区边上
		while(point_count_ogm_.ogm[index]==0&&width0<point_count_ogm_.ogmwidth_cell&&height0>0&&caldis2(height0,width0,100,100)<100){
#ifdef TEST
			vecindex_.push_back(height0);
			vecindex_.push_back(width0);
#endif
			width0++;
			d-=k;
			if(d>0){height0--;d-=1;}
			index=height0*point_count_ogm_.ogmwidth_cell+width0;

		}
		while(width0<point_count_ogm_.ogmwidth_cell&&height0>0&&caldis2(height0,width0,100,100)<100)
		{
			vecindex.clear();

			//			cout<<"maxz1 is "<<maxz_ogm_.ogm[index]<<endl;
			//			vecindex_.push_back(height0);
			//			vecindex_.push_back(width0);
			index=height0*point_count_ogm_.ogmwidth_cell+width0;
			//如果前方有障碍物后续不用判断
			if(maxz_ogm_.ogm[index]>PATHTHRESH)	{return -11;}
			if(abs(maxz_ogm_.ogm[index]+11.11)>0.01){
				maxz1=maxz_ogm_.ogm[index];
			}
			int count=0;
			int beginh=height0;
			int beginw=width0;

			while(point_count_ogm_.ogm[index]==0&&width0<point_count_ogm_.ogmwidth_cell&&height0>0&&caldis2(height0,width0,100,100)<100){
				count++;
				vecindex.push_back(height0);
				vecindex.push_back(width0);
				width0++;
				d-=k;
				if(d>0){height0--;d-=1;}
				index=height0*point_count_ogm_.ogmwidth_cell+width0;
			}

			int endh=height0;
			int endw=width0;
			if(count>3){
				if(abs(maxz_ogm_.ogm[index]+11.11)>0.01){
					maxz1=maxz_ogm_.ogm[index];
				}
				bool heightstate=true;
				if(abs(maxz1+11.11)>0.01&&abs(maxz2+11.11)>0.01&&(maxz1-maxz2)<0.5){
					heightstate=false;
				}
				float beginradius_2=caldis2(beginh,beginw,100,100);
				float diff_2=caldis2(beginh,beginw,endh,endw);
				//盲区内排除
				//				if(caldis2(endh,endw,100,100)<25)continue;
				float thresh_2=0;
				int i=0;
//				while(beginradius_2>theoryr2table_[i]){i++;}
//				//				cout<<"the thresh is "<<thresh_2<<endl;
//				if(theoryr2table_[i]-beginradius_2>2*(beginradius_2-theoryr2table_[i-1])){
//					thresh_2=FACTOR*theorydiff2table_[i-1];
//				}
//				else{
//					thresh_2=FACTOR*theorydiff2table_[i];
//				}
				if(beginradius_2<100){thresh_2=DIFFTHRESH1;}
				else if(beginradius_2<400){thresh_2=DIFFTHRESH2;}
				else{thresh_2=DIFFTHRESH3;}

				//				cout<<"the radius is "<<beginradius_2<<endl;
				//				cout<<"the thresh is "<<thresh_2<<endl;
				if(heightstate&&caldis2(beginh,beginw,100,100)>25&&caldis2(beginh,beginw,100,100)<900&&diff_2>thresh_2){//
					//cout<<"maxzs are"<<maxz1<<"  "<<maxz2<<endl;
					return 11;
				}

				//			判断下一个有点处是否为高度障碍物
				index=height0*point_count_ogm_.ogmwidth_cell+width0;
				if(maxz_ogm_.ogm[index]>PATHTHRESH)			{return -11;}
			}
			width0++;
			d-=k;
			if(d>0){height0--;d-=1;}
		}return -1;
//		while(width0<point_count_ogm_.ogmwidth_cell)
//		{
//
//			if(maxz_ogm_.ogm[height0*maxz_ogm_.ogmwidth_cell+width0]>PATHTHRESH||maxz_ogm_.ogm[(height0+1)*maxz_ogm_.ogmwidth_cell+width0]>PATHTHRESH||maxz_ogm_.ogm[(height0-1)*maxz_ogm_.ogmwidth_cell+width0]>PATHTHRESH){
//				return false;}
//			width0++;
//			d-=k;
//			if(d>0){height0--;d-=1;}
//
//
//		}return true;
		//	case 9:
		//		while(width<width0)
		//		{
		//			if(maxz_ogm_.ogm[height*maxz_ogm_.ogmwidth_cell+width]>PATHTHRESH){
		//				return false;}
		//			width++;
		//		}return true;
		//	case 10:
		//		while(width>width0)
		//		{
		//			if(maxz_ogm_.ogm[height*maxz_ogm_.ogmwidth_cell+width]>PATHTHRESH){
		//				return false;}
		//			width--;
		//		}return true;
//	case 3:
//		return 100;
	}

}
bool  PostProcess::pathClear(int height,int width)
{
	int height0=20/maxz_ogm_.ogmresolution,width0=20/maxz_ogm_.ogmresolution;//起点坐标zx
	float k=(float)(height-height0)/(width-width0);
	float d=-0.5;
	int flag=0;
	//test
	//		int heighttest=55,widthtest=32;
	//		float ktest=(float)(heighttest-height0)/(widthtest-width0);
	//		while(height0<heighttest)
	//		{
	////			if(maxz_ogm_.ogm[height0*maxz_ogm_.ogmwidth_cell+width0]>PATHTHRESH||maxz_ogm_.ogm[height0*maxz_ogm_.ogmwidth_cell+width0+1]>PATHTHRESH||maxz_ogm_.ogm[height0*maxz_ogm_.ogmwidth_cell+width0-1]>PATHTHRESH){
	////				return false;
	////			}
	//			std::cout<<maxz_ogm_.ogm[height0*maxz_ogm_.ogmwidth_cell+width0]<<" ";
	//			height0++;
	//			d+=1/ktest;
	//			if(d>0){width0++;d-=1;}
	//		}
	//		std::cout<<endl;
	//test done
	//if(width0>width||height0>height||k>1) return false;//暂时先只判断车体右侧
	if(width0<=width&&height0<=height&&0<k&&k<=1){flag=1;}
	if(width0<=width&&height0<=height&&k>1){ flag=2;}
	if(width0>=width&&height0<=height&&k<=-1){flag=3;}
	if(width0>=width&&height0<=height&&k>-1&&k<0){flag=4;}
	if(width0>=width&&height0>=height&&k>0&&k<1){flag=5;}
	if(width0>=width&&height0>=height&&k>=1){ flag=6;}
	if(width0<=width&&height0>height&&k<=-1){flag=7;}
	if(width0<=width&&height0>height&&k>-1&&k<0) {flag=8;}
	if(width<width0&&k==0){flag=9;}
	if(width>width0&&k==0){flag=10;}
	//		std::cout<<"....."<<flag<<"..."<<height0<<"..."<<height<<"..."<<width0<<"..."<<width<<"...."<<k<<std::endl;
	int height_next=0;
	int width_next=0;
	switch (flag){

	case 1:
		while(width0<width)
		{

			if(maxz_ogm_.ogm[height0*maxz_ogm_.ogmwidth_cell+width0]>PATHTHRESH||maxz_ogm_.ogm[(height0+1)*maxz_ogm_.ogmwidth_cell+width0]>PATHTHRESH||maxz_ogm_.ogm[(height0-1)*maxz_ogm_.ogmwidth_cell+width0]>PATHTHRESH)
			{return false;}//判断无数点是否由于障碍物产生zx,此处阈值可更改
			width0++;
			d+=k;
			if(d>0) {height0++;d-=1;}
		}return true;
		//				return true;
	case 2:
		//			std::cout<<"..."<<"this is case2"<<std::endl;
		while(height0+2<height)
		{
			if(maxz_ogm_.ogm[height0*maxz_ogm_.ogmwidth_cell+width0]>PATHTHRESH||maxz_ogm_.ogm[height0*maxz_ogm_.ogmwidth_cell+width0+1]>PATHTHRESH||maxz_ogm_.ogm[height0*maxz_ogm_.ogmwidth_cell+width0-1]>PATHTHRESH){
				return false;
			}
			height0++;
			d+=1/k;
			if(d>0){width0++;d-=1;}
		}return true;
	case 3:
		while(height0<height)
		{
			if(maxz_ogm_.ogm[height0*maxz_ogm_.ogmwidth_cell+width0]>PATHTHRESH||maxz_ogm_.ogm[height0*maxz_ogm_.ogmwidth_cell+width0+1]>PATHTHRESH||maxz_ogm_.ogm[height0*maxz_ogm_.ogmwidth_cell+width0-1]>PATHTHRESH){
				return false;
			}
			height0++;
			d-=1/k;
			if(d>0) {width0--;d-=1;}
		}return true;
	case 4:
		while(width<width0)
		{

			if(maxz_ogm_.ogm[height0*maxz_ogm_.ogmwidth_cell+width0]>PATHTHRESH||maxz_ogm_.ogm[(height0+1)*maxz_ogm_.ogmwidth_cell+width0]>PATHTHRESH||maxz_ogm_.ogm[(height0-1)*maxz_ogm_.ogmwidth_cell+width0]>PATHTHRESH){
				return false;
			}
			width0--;
			d-=k;
			if(d>0) {height0++;d-=1;}

		}return true;
	case 5:
		while(width<width0)
		{

			if(maxz_ogm_.ogm[height0*maxz_ogm_.ogmwidth_cell+width0]>PATHTHRESH||maxz_ogm_.ogm[(height0+1)*maxz_ogm_.ogmwidth_cell+width0]>PATHTHRESH||maxz_ogm_.ogm[(height0-1)*maxz_ogm_.ogmwidth_cell+width0]>PATHTHRESH){
				return false;}
			width0--;
			d+=k;
			if(d>0){height0--;d-=1;}
		}return true;
	case 6:
		while(height<height0)
		{
			if(maxz_ogm_.ogm[height0*maxz_ogm_.ogmwidth_cell+width0]>PATHTHRESH||maxz_ogm_.ogm[height0*maxz_ogm_.ogmwidth_cell+width0+1]>PATHTHRESH||maxz_ogm_.ogm[height0*maxz_ogm_.ogmwidth_cell+width0-1]>PATHTHRESH){
				return false;}
			height0--;
			d+=1/k;
			if(d>0) {width0--;d-=1;}

		}return true;
	case 7:
		//				std::cout<<"..."<<"this is case8"<<std::endl;
		while(height<height0)
		{
			int index=height0*maxz_ogm_.ogmwidth_cell+width0;
			//				std::cout<<maxz_ogm_.ogm[index]<<" ";
			if(maxz_ogm_.ogm[height0*maxz_ogm_.ogmwidth_cell+width0]>PATHTHRESH||maxz_ogm_.ogm[height0*maxz_ogm_.ogmwidth_cell+width0+1]>PATHTHRESH||maxz_ogm_.ogm[height0*maxz_ogm_.ogmwidth_cell+width0-1]>PATHTHRESH){
				return false;}
			height0--;
			d-=1/k;
			if(d>0){width0++;d-=1;}

		}return true;//std::cout<<std::endl;
	case 8:
		while(width0<width)
		{

			if(maxz_ogm_.ogm[height0*maxz_ogm_.ogmwidth_cell+width0]>PATHTHRESH||maxz_ogm_.ogm[(height0+1)*maxz_ogm_.ogmwidth_cell+width0]>PATHTHRESH||maxz_ogm_.ogm[(height0-1)*maxz_ogm_.ogmwidth_cell+width0]>PATHTHRESH){
				return false;}
			width0++;
			d-=k;
			if(d>0){height0--;d-=1;}


		}return true;
	case 9:
		while(width<width0)
		{
			if(maxz_ogm_.ogm[height*maxz_ogm_.ogmwidth_cell+width]>PATHTHRESH){
				return false;}
			width++;
		}return true;
	case 10:
		while(width>width0)
		{
			if(maxz_ogm_.ogm[height*maxz_ogm_.ogmwidth_cell+width]>PATHTHRESH){
				return false;}
			width--;
		}return true;
	}

}
void PostProcess::showOGM(const char* windowname,const OGMData<int>& ogmdata,const vector<int>& vecindex)
{
	cvNamedWindow(windowname,0);
	IplImage *slopemat = cvCreateImage(cvSize(ogmdata.ogmwidth_cell,ogmdata.ogmheight_cell),IPL_DEPTH_8U,3);
	cvZero(slopemat);
	int heightnum = ogmdata.ogmheight_cell;
	int widthnum = ogmdata.ogmwidth_cell;
	for(int j=0;j<ogmdata.ogmheight_cell;j++)
	{
		unsigned char* pdata = (unsigned char*)(slopemat->imageData + (ogmdata.ogmheight_cell - 1 - j)* slopemat->widthStep);
		for(int i=0 ;i < ogmdata.ogmwidth_cell ; i++)
		{
			unsigned char val = ogmdata.ogm[i + j*ogmdata.ogmwidth_cell];//val为每个栅格的点云数量
			if(val > 0)
			{
				pdata[3*i]=0;//以val×10作为像素值
				pdata[3*i+1]=abs(val*100);
				pdata[3*i+2]=0;
			}

		}

	}

	cvLine(slopemat,cvPoint(0,ogmdata.ogmheight_cell-1-20/ogmdata.ogmresolution),cvPoint(ogmdata.ogmwidth_cell,ogmdata.ogmheight_cell-1-20/ogmdata.ogmresolution),cvScalar(255));
	cvLine(slopemat,cvPoint(ogmdata.ogmwidth_cell/2,0),cvPoint(ogmdata.ogmwidth_cell/2,ogmdata.ogmheight_cell),cvScalar(255));

	cvLine(slopemat,cvPoint(ogmdata.ogmwidth_cell/2-4/ogmdata.ogmresolution,0),cvPoint(ogmdata.ogmwidth_cell/2-4/ogmdata.ogmresolution,70/ogmdata.ogmresolution),cvScalar(255));
	cvLine(slopemat,cvPoint(ogmdata.ogmwidth_cell/2+4/ogmdata.ogmresolution,0),cvPoint(ogmdata.ogmwidth_cell/2+4/ogmdata.ogmresolution,70/ogmdata.ogmresolution),cvScalar(255));


	for(int i=0;i<vecindex.size()/2;i++)
	{
		int height=ogmdata.ogmheight_cell-1-vecindex[2*i];
		int width=vecindex[2*i+1];
		unsigned char* p=(unsigned char*)slopemat->imageData+height * slopemat->widthStep;
		p[3*width]=0;
		p[3*width+1]=0;
		p[3*width+2]=255;
	}

	cvShowImage(windowname,slopemat);
	cvWaitKey(10);
	cvReleaseImage(&slopemat);

}
//	std::condition_variable con;
void PostProcess::showOGM(const char* windowname ,const OGMData<int>& ogmdata,vector<int> vecside,vector<int> vecupdown)
{
	cvNamedWindow(windowname,0);
	IplImage *slopemat = cvCreateImage(cvSize(ogmdata.ogmwidth_cell,ogmdata.ogmheight_cell),IPL_DEPTH_8U,1);
	cvZero(slopemat);
	int heightnum = ogmdata.ogmheight_cell;
	int widthnum = ogmdata.ogmwidth_cell;
	for(int j=0;j<ogmdata.ogmheight_cell;j++)
	{
		unsigned char* pdata = (unsigned char*)(slopemat->imageData + (ogmdata.ogmheight_cell - 1 - j)* slopemat->widthStep);
		for(int i=0 ;i < ogmdata.ogmwidth_cell ; i++)
		{
			unsigned char val = ogmdata.ogm[i + j*ogmdata.ogmwidth_cell];//val为每个栅格的点云数量
			if(val > 0)
			{
				pdata[i]=abs(val*10);//以val×10作为像素值

			}

		}

	}

	//		cvCircle(slopemat,cvPoint(20/ogmdata.ogmresolution,ogmdata.ogmheight_cell-20/ogmdata.ogmresolution-1),50,cvScalar(255,0,0));
	////test
	//		int height=20/ogmdata.ogmresolution;
	//		int width=20/ogmdata.ogmresolution;
	//		for(int j=0;j<ogmdata.ogmheight_cell;j++)
	//		{
	//			unsigned char* pdata = (unsigned char*)(slopemat->imageData + (ogmdata.ogmheight_cell - 1 - j)* slopemat->widthStep);
	//			for(int i=0 ;i < ogmdata.ogmwidth_cell ; i++)
	//			{
	//				float r=(i-height)*0.2*(i-height)*0.2+(j-width)*0.2*(j-width)*0.2;
	//				unsigned char val = ogmdata.ogm[i + j*ogmdata.ogmwidth_cell];//val为每个栅格的点云数量
	//				if(r<100)
	//				{
	//					pdata[i]=255;//以val×10作为像素值
	//
	//				}
	//
	//			}
	//
	//		}

	//		for(int i=0;i<6;i++){
	//			cvLine(slopemat,cvPoint(0,63-i*10),cvPoint(50,63-i*10),cvScalar(255));
	//		}
	//		矩形
	//			cvRectangle(slopemat,cvPoint(30,30),cvPoint(35,35),cvScalar(255));
	//		矩形
	//		cvLine(slopemat,cvPoint(0,87-45),cvPoint(50,87-45),cvScalar(255));
	//			int height0=25,width0=25;
	//			int height=64,width=15;
	////			cvLine(slopemat,cvPoint(width0,height0),cvPoint(width,height),cvScalar(255));
	//			float k=(float)(height-height0)/(width-width0);
	//			float d=-0.5;
	//			while(height>height0)
	//			{
	//				unsigned char* pdata = (unsigned char*)(slopemat->imageData + (87-height0-1)* slopemat->widthStep);
	//				pdata[width0]=255;
	////				if(maxz_ogm_.ogm[height0*maxz_ogm_.ogmwidth_cell+width0]>PATHTHRESH||maxz_ogm_.ogm[height0*maxz_ogm_.ogmwidth_cell+width0+1]>PATHTHRESH||maxz_ogm_.ogm[height0*maxz_ogm_.ogmwidth_cell+width0-1]>PATHTHRESH){
	////					return false;}
	//				height0++;
	//				d-=1/k;
	//				if(d>0){width0--;d-=1;}
	//
	//			}
	////test done
	//			std::cout<<ogmdata.ogmwidth_cell/2<<std::endl;
	cvLine(slopemat,cvPoint(0,ogmdata.ogmheight_cell-1-20/ogmdata.ogmresolution),cvPoint(ogmdata.ogmwidth_cell,ogmdata.ogmheight_cell-1-20/ogmdata.ogmresolution),cvScalar(255));
	cvLine(slopemat,cvPoint(ogmdata.ogmwidth_cell/2,0),cvPoint(ogmdata.ogmwidth_cell/2,ogmdata.ogmheight_cell),cvScalar(255));

	cvLine(slopemat,cvPoint(ogmdata.ogmwidth_cell/2-4/ogmdata.ogmresolution,0),cvPoint(ogmdata.ogmwidth_cell/2-4/ogmdata.ogmresolution,70/ogmdata.ogmresolution),cvScalar(255));
	cvLine(slopemat,cvPoint(ogmdata.ogmwidth_cell/2+4/ogmdata.ogmresolution,0),cvPoint(ogmdata.ogmwidth_cell/2+4/ogmdata.ogmresolution,70/ogmdata.ogmresolution),cvScalar(255));
	//	    	cvLine(slopemat,cvPoint(ogmdata.ogmwidth_cell/2+3/ogmdata.ogmresolution,0),cvPoint(ogmdata.ogmwidth_cell/2+3/ogmdata.ogmresolution,99),cvScalar(255));
	//	    	cvLine(slopemat,cvPoint(ogmdata.ogmwidth_cell/2-3/ogmdata.ogmresolution,0),cvPoint(ogmdata.ogmwidth_cell/2-3/ogmdata.ogmresolution,99),cvScalar(255));
	//	    	cvLine(slopemat,cvPoint(0,ogmdata.ogmwidth_cell/2+6/ogmdata.ogmresolution),cvPoint(99,ogmdata.ogmwidth_cell/2+6/ogmdata.ogmresolution),cvScalar(255));
	//	    	cvLine(slopemat,cvPoint(0,ogmdata.ogmwidth_cell/2-5/ogmdata.ogmresolution),cvPoint(99,ogmdata.ogmwidth_cell/2-5/ogmdata.ogmresolution),cvScalar(255));
	//test
	//	for(int i=0;i<vecside.size();)
	//	{
	//		//	    	std::cout<<".............................."<<std::endl;
	//		cvLine(slopemat,cvPoint(vecside[i+1],vecside[i]),cvPoint(vecside[i+1]+vecside[i+2],vecside[i]),cvScalar(255));
	//		i+=3;
	//	}
	for(int i=0;i<vecupdown.size();)
	{
		//	    	std::cout<<".............................."<<std::endl;
		int height=ogmdata.ogmheight_cell-vecupdown[i]-1;
		int width=vecupdown[i+1];
		int count=vecupdown[i+2];
		cvLine(slopemat,cvPoint(width,height),cvPoint(width,height-count),cvScalar(255));
		i+=3;
	}
	for(int i=0;i<vecside.size()/2;i++)
	{
		unsigned char* p=(unsigned char*)slopemat->imageData+(ogmdata.ogmheight_cell - 1 - vecside[2*i])* slopemat->widthStep;
		p[vecside[2*i+1]]=255;
	}

	cvShowImage(windowname,slopemat);
	cvWaitKey(10);
	cvReleaseImage(&slopemat);

}
void PostProcess::radiusDetection(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
	//		std::cout<<"radiusdetectionenterrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrr"<<std::endl;
	int col_count = cloud->size()/LASER_LAYER;
	//		float x_thresh;
	//		float y_thresh;
	float z_thresh=-0.1;//TODO:根据不同距离设置不同到阈值
	float x_diff=0,y_diff=0,z_diff=0;//这个声明在外面会更好一些吗？zx
	for(int i=0;i<col_count;i++)
	{
		//			std::cout<<"radiusdetectionenterrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrr"<<std::endl;
		for(int j=0;j<40;j++)//一组到64或32根线处理
		{
			//				std::cout<<"radiusdetectionenterrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrr"<<std::endl;
			int actual_num=indexmaptable[j].number;
			int actual_num_next=indexmaptable[j+1].number;//获取点对的index方便索引
			int actual_index=i*LASER_LAYER+actual_num;
			int actual_index_next=i*LASER_LAYER+actual_num_next;
			float x1=cloud->points[actual_index].x;
			float x2=cloud->points[actual_index_next].x;
			//				x_diff=x1-x2;
			y_diff=(cloud->points[actual_index_next].y-cloud->points[actual_index].y);
			x_diff=(cloud->points[actual_index_next].x-cloud->points[actual_index].x);
			z_diff=(cloud->points[actual_index_next].z-cloud->points[actual_index].z);//获取点三个坐标的变化
			//				std::cout<<".........."<<y_diff<<".........."<<x_diff<<".........."<<z_diff<<".........."<<std::endl;
			//				printf("................%f",x_diff);
			if(/*x_diff>x_thresh||y_diff>y_thresh||*/z_diff<z_thresh)
			{
				//					std::cout<<"radiusdetectionenterrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrr"<<std::endl;
				stiffcloud_->points.push_back(cloud->points[actual_index]);
			}
		}
	}
}
void PostProcess::radiusPointpair(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
	int col_count = cloud->size() / LASER_LAYER;
	float x_thresh=6;
	float y_thresh=6;
	float z_thresh=1.2;
	float x1,x2,y1,y2,z1,z2,x_diff,y_diff,z_diff;
	for(int i=0;i<55;i++)	//防止溢出，另外也用不了64根线
	{
		int actual_i=indexmaptable[i].number;
		for(int j=0;j<col_count;j++)
		{
			int index1=j*LASER_LAYER+actual_i;
			int index2=index1+LASER_LAYER;
			x1=cloud->points[index1].x;
			x2=cloud->points[index2].x;
			y1=cloud->points[index1].y;
			y2=cloud->points[index2].y;
			z1=cloud->points[index1].z;
			z2=cloud->points[index2].z;
			x_diff=abs(x2-x1);
			y_diff=abs(y2-y1);
			z_diff=abs(z2-z1);

			if(x_diff>x_thresh&&y_diff>y_thresh&&z_diff>z_thresh)
			{
				stiffcloud_->points.push_back(cloud->points[index1]);
				stiffcloud_->points.push_back(cloud->points[index2]);
			}
		}
	}
}
//void PostProcess::circleradiusDetection(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
//{
//
//	int col_count = cloud->size() / LASER_LAYER;
//	float delta_dis_thresh = 2;//adjustable param
//	float delta_dis_thresh2=delta_dis_thresh/2;
//	int max_min_counter_thresh=2;
//	int cloud_window_size = 10;//adjustable param cloud_window_size
//	//	    for(int i=0;i<64;i++)
//	//	    {
//	//	    	for(int j=0;j<64;j++)
//	//	    	{
//	//	    		stiffcloud_->points.push_back(cloud->points[i*64+j]);
//	//	    	}
//	//	    }
//
//	for(int dis_i = 0 ; dis_i < LASER_LAYER ; dis_i++)
//	{
//		if(theorydis[dis_i] > 40 || theorydis[dis_i]<0)				//理论值吗?zx
//			continue;
//		//float verticalangle = laserverticalangle[i] * M_PI / 180 + beta_h;
//		//float verticalangle=indexmaptable[dis_i].angle*M_PI/180+beta_h;			//incexmaptable中的angle是否是垂直角度?
//		int i=indexmaptable[dis_i].number;
//
//		float temptheorydis=theorydis[dis_i];
//		float temptheoryz=0;
//		bool flag_matchtheory=false;
//		// if(verticalangle < verticalangle_thresh)
//		{
//
//			float disminold=100;
//			float dismaxold=-100;
//			int disminposold=0;
//			int dismaxposold=0;
//			for(int j = 0 ; j < col_count - cloud_window_size; j+=2)								//这个应该是要在每一圈内做文章?
//			{
//				int ori_index = (j) * LASER_LAYER + i;
//				if(cloud->points[ori_index].y<4&&cloud->points[ori_index].y>-4&&cloud->points[ori_index].x>-2.5&&cloud->points[ori_index].x<2.5) //back exist false detection , so need delete it
//					continue;
//
//				if(cloud->points[ori_index].y< -20||cloud->points[ori_index].x< -30||cloud->points[ori_index].x > 30) //back exist false detection , so need delete it
//					continue;
//
//
//
//				float dismin=100;
//				float dismax=-100;
//				int disminpos=0;
//				int dismaxpos=0;
//				int window_maxz=-100;
//				int window_minz=100;
//				//get min max
//				for(int window_j=0;window_j<cloud_window_size;window_j+=1)
//				{
//					int index = (j+ window_j) * LASER_LAYER + i;
//					if(cloud->points[index].y<4&&cloud->points[index].y>-4&&cloud->points[index].x>-2.5&&cloud->points[index].x<2.5) //back exist false detection , so need delete it
//						continue;
//					float temprange=cloud->points[index].range;
//					float tempz=cloud->points[index].z;
//					if(temprange < 0.5)
//						continue;
//
//					if(temprange<dismin)
//					{
//						dismin=temprange;
//						disminpos=window_j;
//					}
//					if(temprange>dismax)
//					{
//						dismax=temprange;
//						dismaxpos=window_j;
//					}
//
//					if(tempz<window_minz)
//					{
//						window_minz=tempz;
//					}
//
//					if(tempz>window_maxz)
//					{
//						window_maxz=tempz;
//					}
//
//				}
//				//count num
//				if(dismax-dismin>delta_dis_thresh&&dismax-dismin>delta_dis_thresh*0.1*dismin||dismax-dismin>delta_dis_thresh*3)//dismax-dismin>delta_dis_thresh????????zx
//				{
//					int mincounter=0;
//					int maxcounter=0;
//					int zerocounter=0;
//					for(int window_j=0;window_j<cloud_window_size;window_j++)
//					{
//						int index = (j+ window_j) * LASER_LAYER + i;
//						if(cloud->points[index].y<4&&cloud->points[index].y>-4&&cloud->points[index].x>-2.5&&cloud->points[index].x<2.5) //back exist false detection , so need delete it
//						{
//							zerocounter++;
//							continue;
//						}
//						float temprange=cloud->points[index].range;
//						if(temprange < 0.5)
//						{
//							zerocounter++;
//							continue;
//
//						}
//
//						if(temprange-dismin<delta_dis_thresh2)
//						{
//							mincounter++;
//						}
//						if(dismax-temprange<delta_dis_thresh2)
//						{
//							maxcounter++;
//						}
//
//
//
//					}
//
//					if(mincounter>=max_min_counter_thresh&&maxcounter>=max_min_counter_thresh&&zerocounter<max_min_counter_thresh*2)
//					{
//						for(int window_j=0;window_j<cloud_window_size;window_j++)
//						{
//							int index = (j+ window_j) * LASER_LAYER + i;
//							float temprange=cloud->points[index].range;
//							if(temprange-dismin<delta_dis_thresh2)//加了z方向的条件zx20180130
//							{
//#ifdef USE_OMP
//								omp_set_lock(&omplock); //获得互斥器
//#endif
//								cloud->points[index].passibility = 0.0;
//								stiffcloud_->points.push_back(cloud->points[index]);
//#ifdef USE_OMP
//								omp_unset_lock(&omplock); //释放互斥器
//#endif
//							}
//						}
//					}
//				}
//<<<<<<< HEAD
//=======
////				MyTime mytime;
////				mytime.start();
//				countogmpoints(vtotalcloud_);
//				showOGM("grid",point_count_ogm_,vecright_,vecup_);
//				vecright_.clear();							//TODO:检测下size
//				vecup_.clear();
////				mytime.show_s();
//			}
//>>>>>>> 49b1c9702ea15b78c7fe024f2443d7be49c17d41
//
//
//			}
//		}
//	}
//	cloudupdate=true;
//}
void PostProcess::process()
{

	while(!processthreadfinished_)
	{


		const sensor_msgs::PointCloud2ConstPtr cloudmsg = lidarCloudMsgs_.PopWithTimeout(common::FromSeconds(0.1));
		if(cloudmsg == nullptr)
			continue;
		double timeLaserCloudFullRes = cloudmsg->header.stamp.toSec();
		pcl::PointCloud<pcl::PointXYZI>::Ptr tempcloud(new pcl::PointCloud<pcl::PointXYZI>);//当前帧点云（雷达里程计坐标系）
		pcl::fromROSMsg(*cloudmsg, *tempcloud);//获取当前帧点云数据
		std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> outputclouds;
		std::vector<pcl::PointXYZI> lidarpropertys;
		char cloud_name[50];
		memset( cloud_name, 0 , 50);
		sprintf( cloud_name, "passablecloud");
		ShowViewerCloudPoints(cloud_viewer_, tempcloud,
				cloud_name, 0,255, 0);
		cloud_viewer_->spinOnce();
		//			analysisCloud(tempcloud,outputclouds,lidarpropertys);

		//			对齐时间戳 拼接点云
#ifdef FUSE
		if(lworldclouds_.size()>=5)
			lworldclouds_.pop_front();
		vtotalcloud_.clear();
		vtotalcloud_.push_back(tempcloud);//将当前点云赋给totalcloud_
		//			totalclouds_->clear();
		//			*totalclouds_+=(*tempcloud);
		LOG(INFO)<<"wait lidarodom";
		auto timeposepair = lidarOdoms_.Pop();
		if(timeposepair==nullptr)
			continue;
		if(timeposepair->first-timeLaserCloudFullRes>0.005)
		{
			lidarOdoms_.Push_Front(std::move(timeposepair));
			continue;
		}

		while((timeposepair->first-timeLaserCloudFullRes)<-0.005)
			timeposepair = lidarOdoms_.Pop();

		LOG(INFO)<<"got lidarodom";
		if(timeposepair==nullptr)
			continue;
		if(fabs(timeposepair->first-timeLaserCloudFullRes)<0.005){
			transform::Rigid3d transformodometry = timeposepair->second;
			transform::Rigid3d transformodometry_inverse = transformodometry.inverse();
			int n=1;
			for(std::list<pcl::PointCloud<pcl::PointXYZI>::Ptr>::iterator it = lworldclouds_.begin();it != lworldclouds_.end();it++)
			{
				n++;
				pcl::PointCloud<pcl::PointXYZI>::Ptr tempcloud(new pcl::PointCloud<pcl::PointXYZI>);
				pcl::transformPointCloud(*(*it),*tempcloud,transformodometry_inverse.translation()
						,transformodometry_inverse.rotation());
				vtotalcloud_.push_back(tempcloud) ;
				//					*totalclouds_+=(*tempcloud);
			}
			//				cout<<"size now is "<<vtotalcloud_.size()<<endl;
			pcl::PointCloud<pcl::PointXYZI>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZI>);
			pcl::transformPointCloud(*tempcloud,*tmp,transformodometry.translation()
					,transformodometry.rotation());
			//				pcl::PointCloud<pcl::PointXYZI>::Ptr ttt=tmp;
			//				cout<<"n now is "<<n<<endl;
			lworldclouds_.push_back(tmp);
			if(0){
				char cloud_name[50];
				memset( cloud_name, 0 , 50);
				sprintf( cloud_name, "passablecloud");
				ShowViewerCloudPoints(cloud_viewer_, vtotalcloud_,
						cloud_name, 255, 0, 0);
				cloud_viewer_->spinOnce();
			}
			//				MyTime mytime;
			//				mytime.start();
			//				cout<<"dddddddddddddddddddd"<<endl;
			countogmpoints(vtotalcloud_);
			//				cout<<"dddddddddddddddddddd"<<endl;
			showOGM("0.2grid",point_count_ogm_,vec2side_,vecup_);

			showOGM("0.4grid",point_count_ogm_big_,vec4side_,vecup_big_);
			vec2side_.clear();							//TODO:检测下size
			vecup_.clear();
			vec4side_.clear();
			vecup_big_.clear();
			//				mytime.show_s();
		}

		//			对齐时间戳 拼接点云

		//			LOG(INFO)<<"cloud num:"<<lidarpropertys.size();
		//			for(int i=0;i<lidarpropertys.size();i++)
		//			{
		//				LOG(INFO)<<i<<"\tpoint num:"<<outputclouds[i]->size();
		//				LOG(INFO)<<i<<"\tlidar pos:"<<lidarpropertys[i].x<<" "<<lidarpropertys[i].y<<" "<<lidarpropertys[i].z<<" ";
		//				LOG(INFO)<<i<<"\tlidar layernum:"<<int(lidarpropertys[i].intensity);
		//			}
		//			lidarOdoms_.Pop(); //需要雷达里程计信息时需要，否则可以注释掉
		//就用64线的先

#else
		{
			if(tempcloud)
			{

				//					  	  std::cout<<"the point size is  "<<outputclouds[0]->points.size()<<std::endl;
				//                                timer t;

				char cloud_name[50];
				memset( cloud_name, 0 , 50);
				sprintf( cloud_name, "passablecloud");
				//                                ShowViewerCloudPoints(cloud_viewer_, tempcloud,
				//                                                      cloud_name, 255, 0, 0);



				//				            circleradiusDetection(outputclouds[0]);
				//				            radiusDetection(outputclouds[0]);
				//					  	  	radiusPointpair(outputclouds[0]);
				//				            char stiff_name[50];
				//				            memset( stiff_name, 0 , 50);
				//		                    sprintf( stiff_name, "stiffcloud");
				//				            ShowViewerCloudPoints(cloud_viewer_, stiffcloud_,
				//				            stiff_name, 255, 0, 0);
				//                                MyTime mytime;
				//                                mytime.start();
				//				countogmpoints(tempcloud);
				//				showOGM("grid",point_count_ogm_,vecright_,vecup_);
				//				showOGM("biggrid",point_count_ogm_big_,vecright_big_,vecup_big_);
				//				vecright_.clear();							//TODO:检测下size
				//				vecup_.clear();
				//				vecright_big_.clear();
				//				vecup_.clear();
				radialDetection(tempcloud);
				showOGM("radialdetecton",point_count_ogm_,vecindex_);
				vecindex_.clear();


				//                                cloud_viewer_->spinOnce();//这个一定记得取消注释

				//                                mytime.show_s ();

			}
		}
#endif



	}
}



