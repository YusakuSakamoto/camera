//#define __NORMAL_CAMERA__
//#define __KALMAN_TRACK_MODE__
#include "../include/tomato.hpp"

#define MINIMUM_SQUARE 1600
#define MINIMUM_TOMATO_RATIO 0.75
using namespace std;

int main(int argc,char *argv[])
{
  if(argc == 1){
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
  else if (argc == 2){
	//file input;
	char ch;
	string fileneme=argv[1];
	cv::Mat src_frame = cv::imread(fileneme, 1);
	if(!src_frame.data) cout << "input file error" << endl;
	cv::Mat frame;
	cv::resize(src_frame,frame,cv::Size(640,480),0,0);
	cv::namedWindow("image", CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);

	cv::Mat res;
	cv::Mat state(stateSize, 1, type);  // [x,y,v_x,v_y,w,h]
	cv::Mat meas(measSize, 1, type);    // [z_x,z_y,z_w,z_h]

	
	while(ch != 'q' && ch != 'Q'){
	  //show normal image
	  cv::imshow("image", frame);
	  ch = cv::waitKey(1);

	  //kalman filtering
	  vector<vector<cv::Point> > balls;
	  vector<cv::Rect> ballsBox;
	  cv::Mat rangeRes = cv::Mat::zeros(frame.size(), CV_8UC1);
	  frame.copyTo( res );
	  cv::Mat blur;
	  cv::Mat frmHsv;
	  vector<vector<cv::Point> > contours;

	  cv::GaussianBlur(frame, blur, cv::Size(5, 5), 3.0, 3.0);
	  cv::cvtColor(blur, frmHsv, CV_BGR2HSV);
	  int a,x,y;
  
	  for(y=0; y<HEIGHT;y++)
		{
		  for(x=0; x<WIDTH; x++)
			{
			  a = frmHsv.step*y+(x*3);		  
			  if(  (frmHsv.data[a] >=170 || frmHsv.data[a] <=5)  && frmHsv.data[a+1] >=20 && frmHsv.data[a+2] >= 20 )  rangeRes.at<unsigned char>(y,x) = 255;
			  else  rangeRes.at<unsigned char>(y,x) = 0;
			}
		}
	  //cv::imshow("Threshold", rangeRes);
	  cv::erode(rangeRes, rangeRes, cv::Mat(), cv::Point(-1, -1), 2);
	  //cv::imshow("Threshold", rangeRes);
	  cv::dilate(rangeRes, rangeRes, cv::Mat(), cv::Point(-1, -1), 2);
	  cv::imshow("Threshold", rangeRes);
	  cv::findContours(rangeRes, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	  //cv::imshow("Threshold", rangeRes);
	  
	  for (size_t i = 0; i < contours.size(); i++)
		{
		  cv::Rect bBox;
		  bBox = cv::boundingRect(contours[i]);

		  float ratio = (float) bBox.width / (float) bBox.height;
		  if (ratio > 1.0f)
			ratio = 1.0f / ratio;

		  // Searching for a bBox almost square
		  if (ratio > MINIMUM_TOMATO_RATIO && bBox.area() >= MINIMUM_SQUARE)
			{
			  balls.push_back(contours[i]);
			  ballsBox.push_back(bBox);
			}
		}
		
	  cout << "Balls found:" << ballsBox.size() << endl;
	  Detection_result(res,ballsBox,balls);//show result
	  cv::imshow("Tracking", res);
	  //cv::imshow("Threshold", rangeRes);
	}
	return EXIT_SUCCESS;
  }
}
