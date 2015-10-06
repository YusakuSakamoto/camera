#include "../../../../lib/camera.hpp"

using namespace std;
using namespace cv;

myMutex cameramutex;

void *Watershed(void *arg){
  int ch;
  string filedirec="data/";
  string png=".jpg";
  
  cv::Mat frame = cv::imread("../../../data/16.jpg", 0);
  cv::Mat src_img =cv::Mat::zeros(frame.size(),CV_8UC1);
  cv::Mat work_img=cv::Mat::zeros(H/2,W/2,CV_8UC1);
  cv::Mat mean=cv::Mat::zeros(H/2,W/2,CV_8UC1);
  cv::Mat gray = cv::Mat::zeros(H/2,W/2,CV_8UC1);
  if(!frame.data) return 0;
  cv::resize(frame,gray,cv::Size(W/2,H/2),0,0);

  //mean shift
  //=====================================
  int i=0;
  dataset set[H/2*W/2]; 
  i = make_EDM(H/2,W/2,gray,work_img,set);
  mean_shift(set,mean,i,10,0.01,3);
  //=====================================
  
  i=0;
  while(1){
	//show image
	//=======================================
	cv::imshow("mean",mean);
	cv::imshow("gray",gray);
	ch=cv::waitKey(0);
	//======================================
	if( cameramutex.a == 1 ) break;
	else if( cameramutex.a == 2 ) {
	  cameramutex.a = 0;
	  
	  std::ostringstream file;
	  file << filedirec << i++ << png;
	  cv::imwrite(file.str(), mean );
	  
	  std::ostringstream file1;
	  file1 << filedirec << i++ << png;
	  cv::imwrite(file1.str(), gray );
	  cout << "write image!" << endl;
	}
  }
  return 0;  
}


void *myKey(void *arg)
{
  startup();
  char pwd[PATH_SIZE];
  char input[500] = {"\0"};

  getcwd(pwd,PATH_SIZE);
  while(1){
	cout << NAME_COLOR1 << "arc-->";
	printf(" $ \x1b[0m");
	fgets(input,sizeof(input),stdin) ;
	if( input[0] == 'q' ){ cameramutex.a=1;break;}
	else if( input[0] == 's' ){ cameramutex.a = 2;}
	else if( input[0] == 'h' ){ help(); }
	else if( input[0] == 'c' ) { system("clear"); }
	else if( input[0] == ' '){ cameramutex.a = 3;}
  }
  end();
  return NULL;  
}



int main(int argc, char *argv[])
{
  int status;
  pthread_t thread_a;
  pthread_t thread_c;
  void *thread_return;
  //calibrate();
	
  status = pthread_create(&thread_a, NULL, Watershed, NULL);
  if(status != 0 )exit(1);
  status=pthread_create(&thread_c, NULL, myKey,NULL);
  if(status != 0 )exit(1);  
  status = pthread_join(thread_c, &thread_return);
  if(status != 0 )exit(1);
  
  return 0;
}
