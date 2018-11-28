
//判断图像上始末点之间有无点，主要用于排除水的干扰。！！注意这里是在图像坐标系！！
//bool line(int height0,int width0,int height,int width,cv::Mat mat)
//{
//	float k=(float)(height-height0)/(width-width0);
//	float d=-0.5;
//	int flag=0;
//	//test
//	//		int heighttest=55,widthtest=32;
//	//		float ktest=(float)(heighttest-height0)/(widthtest-width0);
//	//		while(height0<heighttest)
//	//		{
//	////			if(maxz_ogm_.ogm[height0*maxz_ogm_.ogmwidth_cell+width0]>PATHTHRESH||maxz_ogm_.ogm[height0*maxz_ogm_.ogmwidth_cell+width0+1]>PATHTHRESH||maxz_ogm_.ogm[height0*maxz_ogm_.ogmwidth_cell+width0-1]>PATHTHRESH){
//	////				return false;
//	////			}
//	//			std::cout<<maxz_ogm_.ogm[height0*maxz_ogm_.ogmwidth_cell+width0]<<" ";
//	//			height0++;
//	//			d+=1/ktest;
//	//			if(d>0){width0++;d-=1;}
//	//		}
//	//		std::cout<<endl;
//	//test done
//	//if(width0>width||height0>height||k>1) return false;//暂时先只判断车体右侧
//	if(width0<=width&&height0<=height&&0<k&&k<=1){flag=1;}
//	if(width0<=width&&height0<=height&&k>1){ flag=2;}
//	if(width0>width&&height0<=height&&k<=-1){flag=3;}
//	if(width0>width&&height0<=height&&k>-1&&k<0){flag=4;}
//	if(width0>width&&height0>height&&k>0&&k<1){flag=5;}
//	if(width0>width&&height0>height&&k>=1){ flag=6;}
//	if(width0<=width&&height0>height&&k<=-1){flag=7;}
//	if(width0<=width&&height0>height&&k>-1&&k<0) {flag=8;}
//	if(width<width0&&k==0){flag=9;}
//	if(width>width0&&k==0){flag=10;}
//	int height_next=0;
//	int width_next=0;
//	switch (flag){
//
//	case 1:
//		while(width0<width)
//		{
//			unsigned char* ptr=mat.ptr<unsigned char>(height0);
//			if(ptr[3*width0+1]==255)
//			{return false;}
//			width0++;
//			d+=k;
//			if(d>0) {height0++;d-=1;}
//		}return true;
//		//				return true;
//	case 2:
//		//			std::cout<<"..."<<"this is case2"<<std::endl;
//		while(height0<height)
//		{
//			unsigned char* ptr=mat.ptr<unsigned char>(height0);
//			if(ptr[3*width0+1]==255)
//			{return false;}
//			height0++;
//			d+=1/k;
//			if(d>0){width0++;d-=1;}
//		}return true;
//	case 3:
//		while(height0<height)
//		{
//			unsigned char* ptr=mat.ptr<unsigned char>(height0);
//			if(ptr[3*width0+1]==255)
//			{return false;}
//			height0++;
//			d-=1/k;
//			if(d>0) {width0--;d-=1;}
//		}return true;
//	case 4:
//		while(width<width0)
//		{
//			unsigned char* ptr=mat.ptr<unsigned char>(height0);
//			if(ptr[3*width0+1]==255)
//			{return false;}
//			width0--;
//			d-=k;
//			if(d>0) {height0++;d-=1;}
//
//		}return true;
//	case 5:
//		while(width<width0)
//		{
//			unsigned char* ptr=mat.ptr<unsigned char>(height0);
//			if(ptr[3*width0+1]==255)
//			{return false;}
//			width0--;
//			d+=k;
//			if(d>0){height0--;d-=1;}
//		}return true;
//	case 6:
//		while(height<height0)
//		{
//			unsigned char* ptr=mat.ptr<unsigned char>(height0);
//			if(ptr[3*width0+1]==255)
//			{return false;}
//			height0--;
//			d+=1/k;
//			if(d>0) {width0--;d-=1;}
//
//		}return true;
//	case 7:
//		//				std::cout<<"..."<<"this is case8"<<std::endl;
//		while(height<height0)
//		{
//			unsigned char* ptr=mat.ptr<unsigned char>(height0);
//			if(ptr[3*width0+1]==255)
//			{return false;}
//			height0--;
//			d-=1/k;
//			if(d>0){width0++;d-=1;}
//
//		}return true;//std::cout<<std::endl;
//	case 8:
//		while(width0<width)
//		{
//			unsigned char* ptr=mat.ptr<unsigned char>(height0);
//			if(ptr[3*width0+1]==255)
//			{return false;}
//			width0++;
//			d-=k;
//			if(d>0){height0--;d-=1;}
//
//
//		}return true;
//	case 9:
//		while(width<width0)
//		{
//			unsigned char* ptr=mat.ptr<unsigned char>(height0);
//			if(ptr[3*width0+1]==255)
//			{return false;}
//			width++;
//		}return true;
//	case 10:
//		while(width>width0)
//		{
//			unsigned char* ptr=mat.ptr<unsigned char>(height0);
//			if(ptr[3*width0+1]==255)
//			{return false;}
//			width--;
//		}return true;
//	}
//
//}
