#include <iostream>
#include <stdio.h>
#include <pthread.h>
#include <stdlib.h>
#include <unistd.h>
#include <vector>
#include <cmath>
#include <sstream>
#include <arc.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include <string>
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
//#include "opencv2/core/utility.hpp"


#ifndef __CAMERA_H__
#define __CAMERA_H__
//#define __RASPBERRY__
#define __NORMAL__
#endif

using namespace std;
using namespace cv;

#if defined(__RASPBERRY__) && !defined(__RASPBERRY_2__)
#define __RASPBERRY_2__
#define W 320
#define H 240
#define stateSize 6
#define measSize 4
#define MINIMUM_SQUARE 1600
#define MINIMUM_TOMATO_RATIO 0.75
#define NUMBOARD 40
#define BOARD_W 10
#define BOARD_H 7
#define LEFT 1
#define RIGHT 0
#define KALMAN_MIN_SQUARE 400
#define KALMAN_MIN_RATIO 0.75

typedef struct{
  int x;
  int y;
  int value;
  int flag;
}dataset;

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
#endif

#if defined(__NORMAL__) && !defined(__NORMAL_2__)
#define __NORMAL_2__
#define W 640
#define H 480
#define stateSize 6
#define measSize 4
#define MINIMUM_SQUARE 1600
#define MINIMUM_TOMATO_RATIO 0.75
#define NUMBOARD 40
#define BOARD_W 10
#define BOARD_H 7
#define LEFT 0  //cappture 1
#define RIGHT 1 //capture 2
#define KALMAN_MIN_SQUARE 200
#define KALMAN_MIN_RATIO 0.75

typedef struct{
  int x;
  int y;
  int value;
  int flag;
}dataset;

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
#endif


void rotateCW90( cv::Mat& input, cv::Mat& output, const unsigned int width, const unsigned int height);
void *myThread(void *arg);
void *myKey(void *arg);

void exclode_clr(cv::Mat&, cv::Mat&);
void exclode_clr_green(cv::Mat&, cv::Mat&);
void Detection_result(cv::Mat&, vector<cv::Rect>&,	  vector<vector<cv::Point> >&);
void Filtering(vector<vector<cv::Point> > &,vector<vector<cv::Point> >&,vector<cv::Rect>&);
int kalman_find(cv::Mat&,vector<cv::Rect>&,bool&,cv::KalmanFilter&,cv::Mat&);
int kalman_process(cv::Mat&,cv::Mat&,cv::Mat&,vector< vector<cv::Point> >&,vector<cv::Rect>&);
int kalman_if_found(cv::KalmanFilter&,cv::Mat&,cv::Mat&);
void kalman_setting(cv::KalmanFilter& kf);
void calibrate();
void *mycalibration(void *arg);
void *video_finder(void *arg);
void *tracker(void *arg);
void *image_finder(void *arg);
void stereoMatching(cv::Mat&,cv::Mat&,cv::Mat&);
void *Watershed(void *arg);
void mean_shift(dataset* set, cv::Mat& output, const int num,const int h  ,const double threshold, const int max_loop);
int  make_EDM(const int height,const int width,cv::Mat& input,cv::Mat& output,dataset* set);
