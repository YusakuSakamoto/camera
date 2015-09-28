#include "./normal.hpp"

using namespace std;
using namespace cv;

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
  unsigned char *pImg0;
  unsigned char *pImg1;
  int i;

  cv::Mat frame0;
  cv::Mat frame1;
  cv::Mat out0 = cv::Mat::zeros( W, H, CV_8UC3);
  cv::Mat out1 = cv::Mat::zeros( W, H, CV_8UC3);
  cv::VideoCapture cap0(0);
  cv::VideoCapture cap1(1);

  cap0.set(CV_CAP_PROP_FRAME_WIDTH, W);
  cap0.set(CV_CAP_PROP_FRAME_HEIGHT,H);
  cap1.set(CV_CAP_PROP_FRAME_WIDTH, W);
  cap1.set(CV_CAP_PROP_FRAME_HEIGHT,H);

  i=0;
  while(1) {
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


void exclode_clr(cv::Mat &input, cv::Mat &output){
  int a,x,y;
  
  for(y=0; y<H;y++)
	{
	  for(x=0; x<W; x++)
		{
		  a = input.step*y+(x*3);		  
		  if(  (input.data[a] >=175 || input.data[a] <=5)  && input.data[a+1] >=50 && input.data[a+2] >= 50 )  output.at<unsigned char>(y,x) = 255;
		  else  output.at<unsigned char>(y,x) = 0;
		}
	}
}

void Detection_result(cv::Mat& res, vector<cv::Rect>& ballsBox,	  vector<vector<cv::Point> >& balls){
  for (size_t i = 0; i < balls.size(); i++)
	{
	  cv::drawContours(res, balls, i, CV_RGB(20,150,20), 1);
	  cv::rectangle(res, ballsBox[i], CV_RGB(0,255,0), 2);

	  cv::Point center;
	  center.x = ballsBox[i].x + ballsBox[i].width / 2;
	  center.y = ballsBox[i].y + ballsBox[i].height / 2;
	  cv::circle(res, center, 2, CV_RGB(20,150,20), -1);

	  stringstream sstr;
	  sstr << "(" << center.x << "," << center.y << ")";
	  cv::putText(res, sstr.str(),
				  cv::Point(center.x + 3, center.y - 3),
				  cv::FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(20,150,20), 2);
	}
}

void Filtering(vector<vector<cv::Point> > &contours,vector<vector<cv::Point> >& balls, 	  vector<cv::Rect> &ballsBox){
  for (size_t i = 0; i < contours.size(); i++)
	{
	  cv::Rect bBox;
	  bBox = cv::boundingRect(contours[i]);

	  float ratio = (float) bBox.width / (float) bBox.height;
	  if (ratio > 1.0f)
		ratio = 1.0f / ratio;

	  // Searching for a bBox almost square
	  if (ratio > 0.75 && bBox.area() >= 400)
		{
		  balls.push_back(contours[i]);
		  ballsBox.push_back(bBox);
		}
	}
}

int kalman_find(cv::Mat& meas,vector<cv::Rect> &ballsBox,bool &found,cv::KalmanFilter& kf,cv::Mat& state){
  meas.at<float>(0) = ballsBox[0].x + ballsBox[0].width / 2;
  meas.at<float>(1) = ballsBox[0].y + ballsBox[0].height / 2;
  meas.at<float>(2) = (float)ballsBox[0].width;
  meas.at<float>(3) = (float)ballsBox[0].height;
  if (!found) // First detection!
	{
	  // >>>> Initialization
	  kf.errorCovPre.at<float>(0) = 1; // px
	  kf.errorCovPre.at<float>(7) = 1; // px
	  kf.errorCovPre.at<float>(14) = 1;
	  kf.errorCovPre.at<float>(21) = 1;
	  kf.errorCovPre.at<float>(28) = 1; // px
	  kf.errorCovPre.at<float>(35) = 1; // px
	  state.at<float>(0) = meas.at<float>(0);
	  state.at<float>(1) = meas.at<float>(1);
	  state.at<float>(2) = 0;
	  state.at<float>(3) = 0;
	  state.at<float>(4) = meas.at<float>(2);
	  state.at<float>(5) = meas.at<float>(3);
	  // <<<< Initialization
	  found = true;
	}
  else	kf.correct(meas); // Kalman Correction
  
  cout << "Measure matrix:" << endl << meas << endl;
  return 0;
}


int kalman_process(cv::Mat& frame,cv::Mat& rangeRes,cv::Mat& res,vector< vector<cv::Point> >& balls, vector<cv::Rect> &ballsBox){
  cv::Mat blur;
  cv::Mat frmHsv;
  vector<vector<cv::Point> > contours;

  cv::GaussianBlur(frame, blur, cv::Size(5, 5), 3.0, 3.0);
  cv::cvtColor(blur, frmHsv, CV_BGR2HSV);
  exclode_clr(frmHsv,rangeRes);
  cv::erode(rangeRes, rangeRes, cv::Mat(), cv::Point(-1, -1), 2);
  cv::dilate(rangeRes, rangeRes, cv::Mat(), cv::Point(-1, -1), 2);
  cv::findContours(rangeRes, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
  Filtering(contours,balls,ballsBox);
  cout << "Balls found:" << ballsBox.size() << endl;
  Detection_result(res,ballsBox,balls);//show result
  
  return 0;  
}

int kalman_if_found(cv::KalmanFilter& kf,cv::Mat& state,cv::Mat& res){
  state = kf.predict();
  cout << "State post:" << endl << state << endl;

  cv::Rect predRect;
  predRect.width = state.at<float>(4);
  predRect.height = state.at<float>(5);
  predRect.x = state.at<float>(0) - predRect.width / 2;
  predRect.y = state.at<float>(1) - predRect.height / 2;

  cv::Point center;
  center.x = state.at<float>(0);
  center.y = state.at<float>(1);
  cv::circle(res, center, 2, CV_RGB(255,0,0), -1);
  cv::rectangle(res, predRect, CV_RGB(255,0,0), 2);

  return 0;
}

void kalman_setting(cv::KalmanFilter& kf){
    // Transition State Matrix A
  // Note: set dT at each processing step!
  // [ 1 0 dT 0  0 0 ]
  // [ 0 1 0  dT 0 0 ]
  // [ 0 0 1  0  0 0 ]
  // [ 0 0 0  1  0 0 ]
  // [ 0 0 0  0  1 0 ]
  // [ 0 0 0  0  0 1 ]
  cv::setIdentity(kf.transitionMatrix);

  // Measure Matrix H
  // [ 1 0 0 0 0 0 ]
  // [ 0 1 0 0 0 0 ]
  // [ 0 0 0 0 1 0 ]
  // [ 0 0 0 0 0 1 ]
  kf.measurementMatrix = cv::Mat::zeros(measSize, stateSize, type);
  kf.measurementMatrix.at<float>(0) = 1.0f;
  kf.measurementMatrix.at<float>(7) = 1.0f;
  kf.measurementMatrix.at<float>(16) = 1.0f;
  kf.measurementMatrix.at<float>(23) = 1.0f;

  // Process Noise Covariance Matrix Q
  // [ Ex   0   0     0     0    0  ]
  // [ 0    Ey  0     0     0    0  ]
  // [ 0    0   Ev_x  0     0    0  ]
  // [ 0    0   0     Ev_y  0    0  ]
  // [ 0    0   0     0     Ew   0  ]
  // [ 0    0   0     0     0    Eh ]
  //cv::setIdentity(kf.processNoiseCov, cv::Scalar(1e-2));
  kf.processNoiseCov.at<float>(0) = 1e-2;
  kf.processNoiseCov.at<float>(7) = 1e-2;
  kf.processNoiseCov.at<float>(14) = 5.0f;
  kf.processNoiseCov.at<float>(21) = 5.0f;
  kf.processNoiseCov.at<float>(28) = 1e-2;
  kf.processNoiseCov.at<float>(35) = 1e-2;

  cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(1e-1));

}



void *mycalibration(void *arg)
{
  string filename = "../data/file.yml";
  cout << endl << "Reading: " << endl;
  FileStorage fs;
  fs.open(filename, FileStorage::READ);

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
  MY_THREAD_ARG *thread_message =(MY_THREAD_ARG*)arg;
  unsigned char *pImg1;
  unsigned char *pImg2;
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
  
  /*
  int ndisparities = 16*5;   
  int SADWindowSize = 11;//奇数でなければならない 
  Ptr<StereoBM> bm = StereoBM::create(ndisparities,SADWindowSize);
  cv::Mat vdisparity = Mat( 480, 640, CV_8UC1 );
  bm->setROI1(roi1);
  bm->setROI2(roi2);
  bm->setPreFilterCap(31);
  bm->setBlockSize(SADWindowSize > 0 ? SADWindowSize : 9);
  bm->setMinDisparity(30);
  bm->setTextureThreshold(1000);
  // bm->setUniquenessRatio(10);
  bm->setSpeckleWindowSize(150);
  bm->setSpeckleRange(2);
  //bm->setDisp12MaxDiff(-1);
	
  /*
    minDisparity – Minimum possible disparity value.
    numDisparities – Maximum disparity minus minimum disparity. This parameter must be divisible by 16.
    SADWindowSize – Matched block size. It must be an odd number >=1 .
    disp12MaxDiff – Maximum allowed difference (in integer pixel units) in the left-right disparity check.
    preFilterCap – Truncation value for the prefiltered image pixels.
    uniquenessRatio – Margin in percentage by which the best (minimum) computed cost function value should “win” the second best value to consider the found match correct. Normally, a value within the 5-15 range is good enough.
    speckleWindowSize – Maximum size of smooth disparity regions to consider their noise speckles and invalidate.
    speckleRange – Maximum disparity variation within each connected component.
  */

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
	  pImg1 = frame1.data;
	  pImg2 = frame2.data;
	  rotateCW90(pImg1, W,H);
	  rotateCW90(pImg2, W,H);	
	  out1.data = pImg1;
	  out2.data = pImg2;
	  //==========================

	  remap(out1, imgU1, map1x, map1y, INTER_LINEAR, BORDER_CONSTANT, Scalar());
	  remap(out2, imgU2, map2x, map2y, INTER_LINEAR, BORDER_CONSTANT, Scalar());

	  cv::Mat grayU1;
	  cv::Mat grayU2;
	  
	  cvtColor(imgU1, grayU1,CV_RGB2GRAY);
	  cvtColor(imgU2, grayU2,CV_RGB2GRAY);

	  /*
		bm->compute(grayU1, grayU2, disparity);

		//-- Check its extreme values
		double minVal; double maxVal;
		minMaxLoc( disparity, &minVal, &maxVal );
		printf("Min disp: %f Max value: %f \n", minVal, maxVal);

		//-- 4. Display it as a CV_8UC1 image
		disparity.convertTo( vdisparity, CV_8UC1, 255/(maxVal - maxVal));
	  */
	  //cvFindStereoCorrespondenceBM(&grayU1,&grayU2,&disparity,BMState);
	  //cvNormalize(&disparity,&vdisparity,0,256,CV_MINMAX);
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

