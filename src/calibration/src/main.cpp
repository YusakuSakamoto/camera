#include "../../../lib/camera.hpp"

using namespace std;
using namespace cv;

void *mycalibration(void *arg)
{
  cout << endl << "Reading: " << endl;
  FileStorage fs;
  fs.open(camera_param, FileStorage::READ);

  Mat CM1 = Mat(3, 3, CV_32FC1);
  Mat D1;
  Mat CM2 = Mat(3, 3, CV_32FC1);
  Mat D2;
  Mat R, T, E, F;
  Mat R1, R2, P1, P2, Q;
  Mat map1x;
  Mat map1y;
  Mat map2x;
  Mat map2y;
  Rect roi1;
  Rect roi2;


  fs["CM1"] >> CM1;
  fs["D1"] >> D1;
  fs["CM2"] >> CM2;
  fs["D2"] >> D2;
  fs["R"] >> R;
  fs["T"] >> T;
  fs["E"] >> E;
  fs["F"] >> F;
  fs["R1"] >> R1;
  fs["R2"] >> R2;
  fs["P1"] >> P1;
  fs["P2"] >> P2;
  fs["Q"] >> Q;
  fs["map1x"] >> map1x;
  fs["map1y"] >> map1y;
  fs["map2x"] >> map2x;
  fs["map2y"] >> map2y;
  fs["roi1"] >> roi1;
  fs["roi2"] >> roi2;


  
  int i=0;
  cv::Mat frame1;
  cv::Mat frame2;
  cv::Mat out1 = cv::Mat::zeros( W, H, CV_8UC3);
  cv::Mat out2 = cv::Mat::zeros( W, H, CV_8UC3);
  cv::Mat imgU1;
  cv::Mat imgU2;
  cv::Mat disparity = Mat(480,640,CV_16S);
  cv::Mat vdisparity;
  cv::Mat disp8;
  cv::VideoCapture cap1(LEFT);
  cv::VideoCapture cap2(RIGHT);
  string fileleft="../data/left0";
  string fileright="../data/right0";
  string png=".jpg";

  CvStereoBMState *BMState = cvCreateStereoBMState();
  BMState->preFilterSize = 41;
  BMState->preFilterCap = 31;
  BMState->SADWindowSize = 41;
  BMState->minDisparity = -64;
  BMState->numberOfDisparities = 128;
  BMState->textureThreshold = 10;
  BMState->uniquenessRatio=15;

  
  while(1)
	{
	  //90CW rotation
	  //======================
	  cap1 >> frame1;
	  cap2 >> frame2;
	  rotateCW90(frame1, out1, W, H);
	  rotateCW90(frame2, out2, W, H);	
	  //==========================
	  
	  remap(out1, imgU1, map1x, map1y, INTER_LINEAR, BORDER_CONSTANT, Scalar());
	  remap(out2, imgU2, map2x, map2y, INTER_LINEAR, BORDER_CONSTANT, Scalar());

	  cv::Mat grayU1;
	  cv::Mat grayU2;
	  
	  cvtColor(imgU1, grayU1,CV_RGB2GRAY);
	  cvtColor(imgU2, grayU2,CV_RGB2GRAY);
	  

	  imshow("undistort1", grayU1);
	  imshow("undistort2", grayU2);
	  //imshow("disp", disparity);

	  waitKey(5);
	  if( cameramutex.a == 1 ) break;
	  else if( cameramutex.a == 2 ) {
		cameramutex.lock();
		cameramutex.a = 0;
	  
		std::ostringstream file;
		file << fileright << i << png;
		cv::imwrite(file.str(), out1 );
	  
		std::ostringstream file2;
		file2 << fileleft << i++ << png;
		cv::imwrite(file2.str(), out2 );
		cameramutex.unlock();
	  }
	}
  cap1.release();
  cap2.release();
  return NULL;
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
  //calibrate();
	
  status = pthread_create(&thread_a, NULL, mycalibration, NULL);
  if(status != 0 )exit(1);
  status=pthread_create(&thread_c, NULL, myKey,NULL);
  if(status != 0 )exit(1);
  
  status = pthread_join(thread_c, &thread_return);
  if(status != 0 )exit(1);
  
  return 0;
}
