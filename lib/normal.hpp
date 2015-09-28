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
#include "opencv2/core/utility.hpp"

#define W 640
#define H 480
#define type CV_32F
#define stateSize 6
#define measSize 4
#define MINIMUM_SQUARE 1600
#define MINIMUM_TOMATO_RATIO 0.75
#define NUMBOARD 20
#define BOARD_W 10
#define BOARD_H 7
#define LEFT 1
#define RIGHT 0

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

typedef struct {
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
} MY_THREAD_ARG;


void rotateCW90(unsigned char *buffer, const unsigned int width, const unsigned int height);
void *myThread(void *arg);
void *myKey(void *arg);
void exclode_clr(cv::Mat&, cv::Mat&);
void Detection_result(cv::Mat&, vector<cv::Rect>&,	  vector<vector<cv::Point> >&);
void Filtering(vector<vector<cv::Point> > &,vector<vector<cv::Point> >&,vector<cv::Rect>&);
int kalman_find(cv::Mat&,vector<cv::Rect>&,bool&,cv::KalmanFilter&,cv::Mat&);
int kalman_process(cv::Mat&,cv::Mat&,cv::Mat&,vector< vector<cv::Point> >&,vector<cv::Rect>&);
int kalman_if_found(cv::KalmanFilter&,cv::Mat&,cv::Mat&);
void kalman_setting(cv::KalmanFilter& kf);

void calibrate();
void *mycalibration(void *arg);
