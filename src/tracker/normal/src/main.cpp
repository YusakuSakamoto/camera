//#define __NORMAL_CAMERA__
//#define __KALMAN_TRACK_MODE__
#include "../include/tomato.hpp"

using namespace std;

int main()
{
  //video setteing
  char ch = 0;
  int idx = 0;
  cv::Mat frame;
  cv::VideoCapture cap;
  if (!cap.open(idx)) return EXIT_FAILURE;
  else{
	cap.set(CV_CAP_PROP_FRAME_WIDTH, WIDTH);
	cap.set(CV_CAP_PROP_FRAME_WIDTH, HEIGHT);
  }

  //kalman filter setting
  double ticks = 0;
  bool found = false;
  int notFoundCount = 0;
  int contrSize = 0;
  cv::Mat res;
  cv::KalmanFilter kf(stateSize, measSize, contrSize, type);
  cv::Mat state(stateSize, 1, type);  // [x,y,v_x,v_y,w,h]
  cv::Mat meas(measSize, 1, type);    // [z_x,z_y,z_w,z_h]
  kalman_setting(kf);

	
  while (ch != 'q' && ch != 'Q')
    {
	  //video capture
	  cout << "\nHit 'q' to exit...\n";
	  ch = cv::waitKey(1);
	  cap >> frame;
	  //cv::imshow("frame",frame);

	  //loop time recorder

	  double precTick = ticks;
	  ticks = (double) cv::getTickCount();
	  double dT = (ticks - precTick) / cv::getTickFrequency();
	  kf.transitionMatrix.at<float>(2) = dT;
	  kf.transitionMatrix.at<float>(9) = dT;
	  cout << "dT:" << endl << dT << endl;

	  
	  //kalman filtering
	  vector<vector<cv::Point> > balls;
	  vector<cv::Rect> ballsBox;
	  cv::Mat rangeRes = cv::Mat::zeros(frame.size(), CV_8UC1);
	  frame.copyTo( res );
  	  if (found) kalman_if_found(kf,state,res);
	  kalman_process(frame,rangeRes,res,balls,ballsBox);
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
	  //cv::imshow("Threshold", rangeRes);
    }
  return EXIT_SUCCESS;
}
