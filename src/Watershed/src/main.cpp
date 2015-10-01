#include "../../../lib/camera.hpp"

int main(int argc, char *argv[])
{
  int x;
  int y;
  int a;
  
  cv::Mat frame = cv::imread(argv[1], CV_LOAD_IMAGE_GRAYSCALE);
  cv::Mat src_img;
  if(!frame.data) return -1;
  cv::resize(frame,src_img,cv::Size(W,H),0,0);

  cv::Mat work_img=cv::Mat::zeros(H,W,CV_8UC1);
  cv::Mat hsv;
  cv::Mat work=cv::Mat(H,W,CV_8UC1);;

  //距離地図作成 loop1
  for(y=1; y<=H-1;y++){
	for(x=1; x<=W-1; x++){
	  a = src_img.step*y+x;
	  if( src_img.data[a]>150){
		
		work_img.data[a] = work_img.data[a-work_img.step-1] + 1;
		if( work_img.data[a] > work_img.data[a-1] + 1  )
		  work_img.data[a] = work_img.data[a-1] + 1;
		if( work_img.data[a] > work_img.data[a-work_img.step] + 1  )
		  work_img.data[a] = work_img.data[a-work_img.step] + 1;
		
	  }
	  else  work_img.data[a]=0;
	}
  }
  
  //距離地図作成 loop2
  for(y=H-1; y>=1;y--){
	for(x=W-1; x>=1; x--){
	  a = work_img.step*y+x;
	  if( src_img.data[a]>150){
		if( work_img.data[a] > work_img.data[a+work_img.step+1] + 1 )
		  work_img.data[a] = work_img.data[a+work_img.step+1] + 1;
		if( work_img.data[a] > work_img.data[a+1] + 1  )
		  work_img.data[a] = work_img.data[a+1] + 1;
		if( work_img.data[a] > work_img.data[a+work_img.step] + 1  )
		  work_img.data[a] = work_img.data[a+work_img.step] + 1;
	  }
	}
  }

  int i=0;
  int pointset_x[500];
  int pointset_y[500];
  //極限点探索 loop
  for(y=1; y<=H-1;y++){
	for(x=1; x<=W-1; x++){
	  a = work_img.step*y+x;
	  if( (work_img.data[a] >= work_img.data[a+work_img.step+1]) &&
		  (work_img.data[a] >= work_img.data[a+work_img.step+0]) &&
		  (work_img.data[a] >= work_img.data[a+work_img.step-1]) &&
		  (work_img.data[a] >= work_img.data[a+1]) &&
		  (work_img.data[a] >= work_img.data[a-1]) &&
		  (work_img.data[a] >= work_img.data[a-work_img.step+1]) &&
		  (work_img.data[a] >= work_img.data[a-work_img.step+0]) &&
		  (work_img.data[a] >= work_img.data[a-work_img.step-1]) ){
		pointset_x[i]=x;
		pointset_y[i]=y;
		i++;
		//cv::circle(work, cv::Point(x, y), 5, cv::Scalar(0,200,0), 8, 8);
	  }
	}
  }
  
  //show image
  //=======================================
  cv::imshow("exclode_data_red",work_img);
  cv::imshow("src",src_img);
  //======================================
  
  cv::waitKey(0);
}
