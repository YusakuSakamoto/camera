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

#define NUMBOARD 10
#define BOARD_W 10
#define BOARD_H 7

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
  Mat intrinsic1 = Mat(3, 3, CV_32FC1);
  Mat distcoeffs1;
  Mat intrinsic2 = Mat(3, 3, CV_32FC1);
  Mat distcoeffs2;
} MY_THREAD_ARG;

void *myThread(void *arg);
void *myKey(void *arg);
void rotateCW90(unsigned char *buffer, const unsigned int width, const unsigned int height);
void calibrate(MY_THREAD_ARG* thread_message);
