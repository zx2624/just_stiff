#define left
if(lidarCloudMsgs_ != nullptr){
		//		cout<<"got lidar"<<endl;
		mtx_cloud.lock();
		double timeLaserCloudFullRes = lidarCloudMsgs_->header.stamp.toSec();
		pcl::PointCloud<pcl::PointXYZI>::Ptr tempcloud(new pcl::PointCloud<pcl::PointXYZI>);//当前帧点云（雷达里程计坐标系）
		pcl::fromROSMsg(*lidarCloudMsgs_, *tempcloud);//获取当前帧点云数据
		mtx_cloud.unlock();
		std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> outputclouds;
		std::vector<pcl::PointXYZI> lidarpropertys;
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
				//排除车身周围
				if(j>grid_width/2-3/res&&j<grid_width/2+3/res
						&&i<20/res+3/res&&i>20/2-4/res)
					continue;
				if(maxz.at<float>(i, j) > 0.5) break;//前方有较高障碍物时break
				int index=i*grid_width+j;

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
				if(count>4)								//可调参数
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
				//下面这个主要是水
				if(0)//count>2 && i < 35 / res
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
					if(abs(maxz1 + 11.11) > 0.001 && abs(maxz2 + 11.11) > 0.001&& abs(maxz1 - maxz2) < 0.3) state  = true ;
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
			double radius=dis*std::cos(pitchrad_toyota[31-i]*PI/180);

			double originx=radius*costable[j];
			double originy=radius*sintable[j];
			double originz=dis*std::sin(pitchrad_toyota[31-i]*PI/180);

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
					if(i == 30 && y > 0 && x > -4 && x < 4){
						//						cout <<" the height is "<<originp.z()<<endl;
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
			double radius=dis*std::cos(pitchrad_toyota[31-i]*PI/180);
			double originx=radius*costable[j];
			double originy=radius*sintable[j];
			double originz=dis*std::sin(pitchrad_toyota[31-i]*PI/180);

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
			if((i == 31 || i == 30)){// && (x < -4 || x > 4)
				colvirtual=boost::math::round((xvirtual+35)/0.2);
				rowvirtual=boost::math::round((yvirtual+20)/0.2);
				if(height < -1){// &&(colvirtual<32/0.2||colvirtual>38/0.2)
					//					cout<<"======================== "<<i<<endl;
					//					if(y > 0){
					//						cout<<"the height is "<<height<<endl;
					//					}

					cv::line(gridshow_0, cvPoint(colvirtual, GRIDWH - 1 -rowvirtual), cvPoint(col, GRIDWH - 1 - row),
							cvScalar(0, 0, 125));
					cv::line(justshow, cvPoint(colvirtual, GRIDWH - 1 -rowvirtual), cvPoint(col, GRIDWH - 1 - row),
							cvScalar(0, 125, 125));
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
						int thresh=4;
						if(col>155&&col<195) thresh=5;
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
#endif //left
