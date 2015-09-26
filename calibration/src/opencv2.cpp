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
  bool found0 = false;
  bool found1 = false;
  int success = 0;
  int k = 0;
  int i;
  int numBoards = 10;
  int board_w = 10;
  int board_h = 7;
  int board_n = board_w*board_h;
  unsigned char *pImg0;
  unsigned char *pImg1;
  const int W = 640;
  const int H = 480;
  Size board_sz = Size(board_w, board_h);
  vector<vector<Point3f> > object_points0;
  vector<vector<Point2f> > image_points0;
  vector<vector<Point3f> > object_points1;
  vector<vector<Point2f> > image_points1;
  vector<Point2f> corners0;
  vector<Point2f> corners1;
  vector<Point3f> obj0;
  vector<Point3f> obj1;

  cv::Mat gray0;
  cv::Mat gray1;
  cv::Mat frame0;
  cv::Mat frame1;
  cv::Mat out0 = cv::Mat::zeros( W, H, CV_8UC3);
  cv::Mat out1 = cv::Mat::zeros( W, H, CV_8UC3);
  cv::VideoCapture cap0(0);
  cv::VideoCapture cap1(1);

  for (int j=0; j<board_n; j++)
    {
	  obj0.push_back(Point3f(j/board_w, j%board_w, 0.0f));
	  obj1.push_back(Point3f(j/board_w, j%board_w, 0.0f));
    }

  cap0.set(CV_CAP_PROP_FRAME_WIDTH, 640);
  cap0.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
  cap1.set(CV_CAP_PROP_FRAME_WIDTH, 640);
  cap1.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
  cv::namedWindow("Capture0", CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);
  cv::namedWindow("Capture1", CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);

  i=0;
  while(success < numBoards) {
	//=============================
	string fileleft="./data/left0";
	string fileright="./data/right0";
	string png=".jpg";
	  
	cap0 >> frame0;
	cap1 >> frame1;
	
	pImg0 = frame0.data;
	pImg1 = frame1.data;
	
	rotateCW90(pImg0, W,H);
	rotateCW90(pImg1, W,H);
	
	out0.data = pImg0;
	out1.data = pImg1;
	
	cv::imshow("Capture0", out0);
	cv::imshow("Capture1", out1);
	
	cvWaitKey(1);
	//===================================>

	cvtColor(out0, gray0, CV_BGR2GRAY);
	cvtColor(out1, gray1, CV_BGR2GRAY);
	
	found0 = findChessboardCorners(gray0, board_sz, corners0, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
	found1 = findChessboardCorners(gray1, board_sz, corners1, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
	//===================================

	
	if (found0)
	  {
		cornerSubPix(gray0, corners0, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
		drawChessboardCorners(gray0, board_sz, corners0, found0);
	  }
	if (found1)
	  {
		cornerSubPix(gray1, corners1, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
		drawChessboardCorners(gray0, board_sz, corners1, found1);
	  }
	
	imshow("corners0", gray0);
	imshow("corners1", gray1);
	
	
	if( cameramutex.a == 1 ) break;
	else if( cameramutex.a == 2 ) {
	  cameramutex.lock();
	  cameramutex.a = 0;
	  
	  std::ostringstream file;
	  file << fileright << i << png;
	  cv::imwrite(file.str(), out0 );
	  
	  std::ostringstream file1;
	  file1 << fileleft << i++ << png;
	  cv::imwrite(file1.str(), out1 );
	  cameramutex.unlock();
	}	else if( cameramutex.a == 3 ){
	  if(found0 != 0 && found1 != 0) {
	
		image_points0.push_back(corners0);
		object_points0.push_back(obj0);
		image_points1.push_back(corners1);
		object_points1.push_back(obj1);
	 
		printf ("Corners stored\n");
		success++;

		if (success >= numBoards)
		  {
			break;
		  }
	  }
	  else{
		cout << "not found!" << endl;
	  }
	  cameramutex.a = 0;
	}
  }
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
