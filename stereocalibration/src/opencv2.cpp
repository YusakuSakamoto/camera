#include <iostream>
#include <stdio.h>
#include <pthread.h>
#include <stdlib.h>
#include <unistd.h>
#include <vector>
#include <cmath>
#include <sstream>
#include <Like_terminal.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include <string>

#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"

using namespace std;
using namespace cv;

class myMutex {
public:
  int a;
  myMutex() {
	pthread_mutex_init( &m_mutex, NULL );
  }
  void lock() {
	pthread_mutex_lock( &m_mutex );
  }
  void unlock() {
	pthread_mutex_unlock( &m_mutex );
  }
private:
  pthread_mutex_t m_mutex;
};

void *myThread(void *arg);
void *myKey(void *arg);
myMutex cameramutex;

void rotateCW90(unsigned char *buffer, const unsigned int width, const unsigned int height)
{
  const unsigned int sizeBuffer = width * height * 3; 
  unsigned char *tempBuffer = new unsigned char[sizeBuffer];

  for (int y = 0, destinationColumn = height - 1; y < height; ++y, --destinationColumn)
    {
	  int offset = y * width;

	  for (int x = 0; x < width; x++)
        {
		  for (int i = 0; i < 3; i++) { // RGB
			tempBuffer[(x * height + destinationColumn) * 3 + i] = buffer[(offset + x) * 3 + i];
		  }
        }
    }

  memcpy(buffer, tempBuffer, sizeBuffer);
  delete[] tempBuffer;
}








void *myThread(void *arg)
{
  bool found1 = false;
  bool found2 = false;
  int success = 0;
  int k = 0;
  int i;
  int numBoards = 100;
  int board_w = 10;
  int board_h = 7;
  int board_n = board_w*board_h;
  unsigned char *pImg1;
  unsigned char *pImg2;
  const int W = 640;
  const int H = 480;
  Size board_sz1 = Size(board_w, board_h);
  Size board_sz2 = Size(board_w, board_h);
  vector<vector<Point3f> > object_points;
  vector<vector<Point2f> > image_points1;
  vector<vector<Point2f> > image_points2;
  vector<Point2f> corners1;
  vector<Point2f> corners2;
  vector<Point3f> obj;

  cv::Mat gray1;
  cv::Mat gray2;
  cv::Mat frame1;
  cv::Mat frame2;
  cv::Mat out1 = cv::Mat::zeros( W, H, CV_8UC3);
  cv::Mat out2 = cv::Mat::zeros( W, H, CV_8UC3);
  cv::VideoCapture cap1(0);
  cv::VideoCapture cap2(1);

  for (int j=0; j<board_n; j++)
    {
	  obj.push_back(Point3f(j/board_w, j%board_w, 0.0f));
    }

  cap1.set(CV_CAP_PROP_FRAME_WIDTH, 640);
  cap1.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
  cap2.set(CV_CAP_PROP_FRAME_WIDTH, 640);
  cap2.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
  cv::namedWindow("Capture1", CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);
  cv::namedWindow("Capture2", CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);

  i=0;
  while(success < numBoards) {
	//=============================
	string fileleft="./data/left0";
	string fileright="./data/right0";
	string png=".jpg";
	  
	cap1 >> frame1;
	cap2 >> frame2;
	
	pImg1 = frame1.data;
	pImg2 = frame2.data;
	
	rotateCW90(pImg1, W,H);
	rotateCW90(pImg2, W,H);
	
	out1.data = pImg1;
	out2.data = pImg2;
	
	cv::imshow("Capture1", out1);
	cv::imshow("Capture2", out2);
	
	cvWaitKey(1);
	//===================================>

	cvtColor(out1, gray1, CV_BGR2GRAY);
	cvtColor(out2, gray2, CV_BGR2GRAY);
	
	found1 = findChessboardCorners(gray1, board_sz1, corners1, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
	found2 = findChessboardCorners(gray2, board_sz2, corners2, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
	//===================================

	
	if (found1)
	  {
		cornerSubPix(gray1, corners1, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
		drawChessboardCorners(gray1, board_sz1, corners1, found1);
	  }
	if (found2)
	  {
		cornerSubPix(gray2, corners2, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
		drawChessboardCorners(gray2, board_sz2, corners2, found2);
	  }

	imshow("corners1", gray1);
	imshow("corners2", gray2);
	
	k = waitKey(10);
	if (found1 && found2)
	  {
		k = waitKey(0);
	  }
	if (k == 27)
	  {
		break;
	  }
	if (k == ' ' && found1 !=0 && found2 != 0)
	  {
		image_points1.push_back(corners1);
		image_points2.push_back(corners2);
		object_points.push_back(obj);
		printf ("Corners stored\n");
		success++;

		if (success >= numBoards)
		  {
			break;
		  }
	  }
  }
  destroyAllWindows();
  printf("Starting calibration\n");

  Mat CM1 = Mat(3, 3, CV_64FC1);
  Mat CM2 = Mat(3, 3, CV_64FC1);
  Mat D1, D2;
  Mat R, T, E, F;

  stereoCalibrate(object_points, image_points1, image_points2, CM1, D1, CM2, D2, out1.size(), R, T, E, F, CALIB_FIX_INTRINSIC,TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 100, 1e-6) );

  FileStorage fs1("mystereocalib.yml", FileStorage::WRITE);
  fs1 << "CM1" << CM1;
  fs1 << "CM2" << CM2;
  fs1 << "D1" << D1;
  fs1 << "D2" << D2;
  fs1 << "R" << R;
  fs1 << "T" << T;
  fs1 << "E" << E;
  fs1 << "F" << F;

  printf("Done Calibration\n");

  printf("Starting Rectification\n");

  Mat R1, R2, P1, P2, Q;
  stereoRectify(CM1, D1, CM2, D2, out1.size(), R, T, R1, R2, P1, P2, Q);
  fs1 << "R1" << R1;
  fs1 << "R2" << R2;
  fs1 << "P1" << P1;
  fs1 << "P2" << P2;
  fs1 << "Q" << Q;

  printf("Done Rectification\n");

  printf("Applying Undistort\n");

  Mat map1x, map1y, map2x, map2y;
  Mat imgU1, imgU2;

  initUndistortRectifyMap(CM1, D1, R1, P1, out1.size(), CV_32FC1, map1x, map1y);
  initUndistortRectifyMap(CM2, D2, R2, P2, out2.size(), CV_32FC1, map2x, map2y);

  printf("Undistort complete\n");

  while(1)
    {
	  cap1 >> frame1;
	  cap2 >> frame2;

	  pImg1 = frame1.data;
	  pImg2 = frame2.data;
	
	  rotateCW90(pImg1, W,H);
	  rotateCW90(pImg2, W,H);
	
	  out1.data = pImg1;
	  out2.data = pImg2;
	  
	  remap(out1, imgU1, map1x, map1y, INTER_LINEAR, BORDER_CONSTANT, Scalar());
	  remap(out2, imgU2, map2x, map2y, INTER_LINEAR, BORDER_CONSTANT, Scalar());
	  

	  imshow("image1", out1);
	  imshow("undistort1", imgU1);
	  imshow("image2", out2);
	  imshow("undistort2", imgU2);

	  waitKey(1);
	  if( cameramutex.a == 1 ) break;
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
	if( input[0] == 'a' ){ cameramutex.a=1;break;}
	else if( input[0] == 'b' ){ cameramutex.a=2;}
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

	status = pthread_create(&thread_a, NULL, myThread, NULL);
	if(status != 0 )exit(1);
	status=pthread_create(&thread_c, NULL, myKey,NULL);
	if(status != 0 )exit(1);
	status = pthread_join(thread_c, &thread_return);
	if(status != 0 )exit(1);
  
	return 0;
  }
