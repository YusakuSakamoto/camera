#include "../../../lib/camera.hpp"

typedef struct{
  int x;
  int y;
  int value;
  int flag;
}dataset;

/*mean shift
HOW to use
arg 1 = Type dataset array
arg 2 = Mat output (gray scale) 
arg 3 = Mat output's array number
arg 4 = mean shift circle radius
arg 5 = epsilon
arg 6 = max loop number
====================================================*/
void mean_shift(dataset* set, cv::Mat& output, const int num,const int h  ,const double threshold, const int max_loop){
  
  //変数宣言
  //===========================================
  int a,i,j;
  int flag = 1;
  int loop = 0;
  //===========================================

  for(j=0;j<num;j++){
	//mean shift による中心保存用変数と収束確認用変数の初期化
	//==============================================
	loop = 0;
	float average_x = set[j].x;
	float average_y = set[j].y;
	float average_x_pre = set[j].x;
	float average_y_pre = set[j].y;
	//==============================================
	
	if( set[j].flag == 0 ){//まだ触れていない注目画素であれば
	  while(1){//収束が確認できるまでの無限ループ
		loop++;//loop counter

		//for moment calc
		//===============================================
		int rec_n=0;
		int rec[ (int)(pow(4*h,2)) ] = {0};
		float moment_x = 0;
		float moment_y = 0;
		int moment_number = 0;
		
		for(i=0;i<num;i++){
		  if( (pow( pow(average_x-set[i].x,2) + pow(average_y-set[i].y,2 ),0.5)) < h  ){
			moment_x += (set[i].x - average_x) * set[i].value;
			moment_y += (set[i].y - average_y) * set[i].value;
			moment_number += set[i].value;
			rec[rec_n++] = i;
		  }
		}
		average_x += moment_x / moment_number;
		average_y += moment_y / moment_number;
		//==================================================

		//繰り返し判定
		//=====================================================
		if( ((abs(average_x_pre-average_x) < threshold) && (abs(average_y_pre-average_y) < threshold )) ){//収束条件
		  a = output.step*(int)average_y + (int)average_x;
		  output.data[a] = 255;
		  for(i=0;i<rec_n;i++){
			if( set[ rec[i] ].flag == 0 ){
			  set[ rec[i] ].flag = flag;
			  set[j].flag = flag;
			}else{
			  set[j].flag = set[ rec[i] ].flag;
			}
		  }
		  flag++;
		  break;
		}else if( loop > max_loop ){//収束しなければ
		  break;
		}else{//繰り返し計算の途中
		  average_x_pre = average_x;
		  average_y_pre = average_y;
		  moment_number = 0;
		}
	  }
	  //===========================================================
	}
	printf("%d\n",flag);
  }
}


int  make_EDM(const int height,const int width,cv::Mat& input,cv::Mat& output,dataset* set){
  int a,i,x,y;
  //距離地図作成 loop0
  for(y=1; y<=height-1;y++){
	for(x=1; x<=width-1; x++){
	  a = input.step*y+x;
	  if( input.data[a]>150){		
		output.data[a] = 1;
	  }
	  else  output.data[a]=0;
	}
  }

  //距離地図作成 loop1
  for(y=1; y<=height-1;y++){
	for(x=1; x<=width-1; x++){
	  a = input.step*y+x;
	  if( input.data[a]>150){
		
		output.data[a] = output.data[a-output.step-1] + 1;
		if( output.data[a] > output.data[a-1] + 1  )
		  output.data[a] = output.data[a-1] + 1;
		if( output.data[a] > output.data[a-output.step] + 1  )
		  output.data[a] = output.data[a-output.step] + 1;
	  }
	  else  output.data[a]=0;
	}
  }

  i=0;
  //距離地図作成 loop2
  for(y=height-1; y>=1;y--){
	for(x=width-1; x>=1; x--){
	  a = output.step*y+x;
	  if( input.data[a]>150){
		if( output.data[a] > output.data[a+output.step+1] + 1 )
		  output.data[a] = output.data[a+output.step+1] + 1;
		if( output.data[a] > output.data[a+1] + 1  )
		  output.data[a] = output.data[a+1] + 1;
		if( output.data[a] > output.data[a+output.step] + 1  )
		  output.data[a] = output.data[a+output.step] + 1;

		set[i].x = x;
		set[i].y = y;
		set[i].flag = 0;
		set[i].value = output.data[a];
		i++;
	  }
	  else  output.data[a] = 0;
	}
  }
  return i;
}




int main(int argc, char *argv[])
{
  cv::Mat frame = cv::imread(argv[1], CV_LOAD_IMAGE_GRAYSCALE);
  cv::Mat src_img;
  if(!frame.data) return -1;
  cv::resize(frame,src_img,cv::Size(W/2,H/2),0,0);

  cv::Mat work_img=cv::Mat::zeros(H/2,W/2,CV_8UC1);
  cv::Mat mean=cv::Mat::zeros(H/2,W/2,CV_8UC1);
  //mean shift
  //=====================================
  int i=0;
  dataset set[H/2*W/2]; 
  i = make_EDM(H/2,W/2,src_img,work_img,set);
  mean_shift(set,mean,i,10,0.05,10);
  //=====================================
  
  //show image
  //=======================================
  cv::imshow("mean",mean);
  cv::imshow("src",src_img);
  //======================================
  
  cv::waitKey(0);
}
