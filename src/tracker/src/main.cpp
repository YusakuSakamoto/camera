#include "../../../lib/camera.hpp"

using namespace std;
using namespace cv;


void *tracker(void *arg){
  //video setteing
  int i;
  int idx = 0;
  cv::Mat frame;
  cv::Mat out = cv::Mat::zeros( W, H, CV_8UC3);
  cv::VideoCapture cap;
  string file_save_directory="./data/";
  string png=".jpg";
  if (!cap.open(idx)) return 0;
  else{
	cap.set(CV_CAP_PROP_FRAME_WIDTH, W);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, H);
  }

  //kalman filter setting
  double ticks = 0;
  bool found = false;
  int notFoundCount = 0;
  int contrSize = 0;
  cv::Mat res;
  cv::KalmanFilter kf(stateSize, measSize, contrSize, CV_32F);
  cv::Mat state(stateSize, 1, CV_32F);  // [x,y,v_x,v_y,w,h]
  cv::Mat meas(measSize, 1, CV_32F);    // [z_x,z_y,z_w,z_h]
  kalman_setting(kf);

  i=0;
  while (1){
	//video capture
	cv::waitKey(1);
	cap >> frame;
	cv::Mat blur = cv::Mat::zeros( W, H, CV_8UC3);
	cv::GaussianBlur(frame, blur, cv::Size(5, 5), 3.0, 3.0);
	rotateCW90( blur, out, W, H);

	//loop time recorder
	double precTick = ticks;
	ticks = (double) cv::getTickCount();
	double dT = (ticks - precTick) / cv::getTickFrequency();
	kf.transitionMatrix.at<float>(2) = dT;
	kf.transitionMatrix.at<float>(9) = dT;
	//cout << "dT:" << endl << dT << endl;

		
	//kalman filtering
	vector<vector<cv::Point> > balls;
	vector<cv::Rect> ballsBox;
	cv::Mat rangeRes = cv::Mat::zeros(out.size(), CV_8UC1);
	out.copyTo( res );
	if (found) kalman_if_found(kf,state,res);
	kalman_process(out,rangeRes,res,balls,ballsBox);
	if (balls.size() == 0)
	  {
		if( ++notFoundCount >= 100 ) found = false;
	  }
	else
	  {
		notFoundCount = 0;
		kalman_find(meas,ballsBox,found,kf,state);
	  }
	cv::imshow("Tracking", res);



	if( cameramutex.a == 1 ) break;
	else if( cameramutex.a == 2 ) {
	  cameramutex.lock();
	  cameramutex.a = 0;
	  
	  std::ostringstream file;
	  file << file_save_directory << i << png;
	  cv::imwrite(file.str(), res );
	  
	  cameramutex.unlock();
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
	else if( input[0] == 's' ){ cameramutex.a=2;}
	else if( input[0] == 'h' ){ help(); }
	else if( input[0] == 'c' ) { system("clear"); }
	else if( input[0] == ' '){ cameramutex.a=3;}
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

  status = pthread_create(&thread_a, NULL, tracker, NULL);
  if(status != 0 )exit(1);
  status=pthread_create(&thread_c, NULL, myKey,NULL);
  if(status != 0 )exit(1);
  status = pthread_join(thread_c, &thread_return);
  if(status != 0 )exit(1);
  
  return 0;
}
