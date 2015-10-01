#include "../../../lib/camera.hpp"

typedef struct{
  int x;
  int y;
  int value;
  int flag;
}dataset;



void mean_shift(dataset* set, cv::Mat& output, int num,double h){
  int i;
  int j;
  int flag = 1;
  int a;
  int loop = 0;
  const double threshold = 0.5;
  const int max_loop = 5;

  for(j=0;j<num;j++){
	
	float average_x = set[j].x;
	float average_y = set[j].y;
	float average_x_pre = set[j].x;
	float average_y_pre = set[j].y;
	
	if( set[j].flag == 0 ){
	  while(1){
		loop++;
		float moment_x = 0;
		float moment_y = 0;
		int moment_number = 0;
		
		for(i=0;i<num;i++){
		  if( /* (set[i].flag == 0) &&*/ (pow( pow(average_x-set[i].x,2) + pow(average_y-set[i].y,2 ),0.5) < h)  ){
			moment_x += (set[i].x - average_x) * set[i].value;
			moment_y += (set[i].y - average_y) * set[i].value;
			moment_number += set[i].value;
		  }
		}

		if(moment_number>0){
		  average_x += moment_x / moment_number;
		  average_y += moment_y / moment_number;
		}
		
		if( (((average_x_pre-average_x) < threshold) && ((average_y_pre-average_y) < threshold )) ){//収束条件
		  a = output.step*(int)average_y + (int)average_x;
		  for(i=0;i<num;i++){
			if( set[i].flag == 0 ){
			  if( (pow( pow(average_x-set[i].x,2) + pow(average_y-set[i].y,2 ),0.5) < h)  ){
				set[i].flag = flag;
				output.data[a] = 255;
				set[j].flag = flag;
			  }
			}else{
			  set[j].flag = set[i].flag;
			}
		  }
		  loop = 0;
		  flag++;
		  break;
		}else if( loop > max_loop ){
		  loop=0;
		  break;
		}else{
		  average_x_pre = average_x;
		  average_y_pre = average_y;
		  moment_number = 0;
		}
	  }
	}
	//printf("%d\n",flag);
  }
}


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
	cv::Mat work=cv::Mat::zeros(H,W,CV_8UC1);
	cv::Mat mean=cv::Mat::zeros(H,W,CV_8UC1);
	cv::Mat dilation;
	cv::Mat blur = cv::Mat::zeros( W, H, CV_8UC1);

	cv::GaussianBlur(src_img, src_img, cv::Size(5, 5), 3.0, 3.0);

	//距離地図作成 loop0
	for(y=1; y<=H-1;y++){
	  for(x=1; x<=W-1; x++){
		a = src_img.step*y+x;
		if( src_img.data[a]>150){		
		  work_img.data[a] = 1;
		}
		else  work_img.data[a]=0;
	  }
	}

  
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


	int i=0;
	dataset set[H*W]; 
  
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

		  set[i].x = x;
		  set[i].y = y;
		  set[i].flag = 0;
		  set[i].value = work_img.data[a];
		  i++;
		  work.data[a]=255;
		
		}
		else  work_img.data[a] = 0;
	  }
	}
  
	//printf("%d\n",i);
	//mean_shift(set,work,i,H,W,5.0);

  
	// int i=0;
	// dataset set[H*W];
	// //極限点探索 loop
	// for(y=1; y<=H-1;y++){
	//   for(x=1; x<=W-1; x++){
	// 	a = work_img.step*y+x;
	// 	if ( work_img.data[a] >= 2){
	// 	  if( (work_img.data[a] >= work_img.data[a+work_img.step+1]) &&
	// 		  (work_img.data[a] >= work_img.data[a+work_img.step+0]) &&
	// 		  (work_img.data[a] >= work_img.data[a+work_img.step-1]) &&
	// 		  (work_img.data[a] >= work_img.data[a+1]) &&
	// 		  (work_img.data[a] >= work_img.data[a-1]) &&
	// 		  (work_img.data[a] >= work_img.data[a-work_img.step+1]) &&
	// 		  (work_img.data[a] >= work_img.data[a-work_img.step+0]) &&
	// 		  (work_img.data[a] >= work_img.data[a-work_img.step-1]) )
	// 		{
	  
	// 		  set[i].x = x;
	// 		  set[i].y = y;
	// 		  set[i].flag = 0;
	// 		  set[i].value = work_img.data[a];
	// 		  i++;
	// 		  work.data[a]=255;
	// 		}
	// 	}
	//   }
	// }
  
	mean_shift(set,mean,i,20.0);

  
	//show image
	//=======================================
	cv::imshow("mean",mean);
	cv::imshow("work",work);
	cv::imshow("exclode_data_red",work_img);
	cv::imshow("src",src_img);
	//======================================
  
	cv::waitKey(0);
  }
