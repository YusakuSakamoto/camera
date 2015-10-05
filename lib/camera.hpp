#include <iostream>
#include <stdio.h>
#include <pthread.h>
#include <stdlib.h>
#include <unistd.h>
#include <vector>
#include <cmath>
#include <sstream>
#include <stack>
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

class RAList {
public:
  int label;
  RAList *next;
  RAList( void );
  ~RAList( void );
  int Insert( RAList* );
  
private:
  RAList *cur, *prev;
  unsigned char exists;
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
#define color_radius 6.5
#define spatial_radius 10

#define DECLARE_TIMING(s)  int64 timeStart_##s; double timeDiff_##s; double timeTally_##s = 0; int countTally_##s = 0
#define START_TIMING(s) timeStart_##s = 0
#define STOP_TIMING(s) timeDiff_##s = (double)(cvGetTickCount() - timeStart_##s); timeTally_##s += timeDiff_##s; countTally_##s++

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

class RAList {
public:
  int label;
  RAList *next;
  RAList( void );
  ~RAList( void );
  int Insert( RAList* );
  
private:
  RAList *cur, *prev;
  unsigned char exists;
};
#endif


void rotateCW90( cv::Mat& input, cv::Mat& output, const unsigned int width, const unsigned int height);
void exclode_clr(cv::Mat&, cv::Mat&);
void exclode_clr_green(cv::Mat&, cv::Mat&);
void Detection_result(cv::Mat&, vector<cv::Rect>&,	  vector<vector<cv::Point> >&);
void Filtering(vector<vector<cv::Point> > &,vector<vector<cv::Point> >&,vector<cv::Rect>&);
int kalman_find(cv::Mat&,vector<cv::Rect>&,bool&,cv::KalmanFilter&,cv::Mat&);
int kalman_process(cv::Mat&,cv::Mat&,cv::Mat&,vector< vector<cv::Point> >&,vector<cv::Rect>&);
int kalman_if_found(cv::KalmanFilter&,cv::Mat&,cv::Mat&);
void kalman_setting(cv::KalmanFilter& kf);
void calibrate();
void stereoMatching(cv::Mat&,cv::Mat&,cv::Mat&);
void mean_shift(dataset* set, cv::Mat& output, const int num,const int h  ,const double threshold, const int max_loop);
int  make_EDM(const int height,const int width,cv::Mat& input,cv::Mat& output,dataset* set);
int MeanShift(cv::Mat&,int**);
void meanshift_step_ONE( cv::Mat& img, cv::Mat& result );


inline float color_distance( const float* a, const float* b){
  float l = a[0]-b[0];
  float u = a[1]-b[1];
  float v = a[2]-b[2];

  return l*l+u*u+v*v;
}


inline float color_distance( cv::Mat& img, int x1,int y1,int x2,int y2){
  int a1 = img.step*x1 + y1*3;
  int a2 = img.step*x2 + y2*3;
  
  int r = img.data[a1+0] - img.data[a2+0];
  int g = img.data[a1+1] - img.data[a2+1];
  int b = img.data[a1+2] - img.data[a2+1];
  
  return r*r+g*g+b*b;
}
