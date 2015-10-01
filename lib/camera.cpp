#include "./camera.hpp"

using namespace std;
using namespace cv;

myMutex cameramutex;
string camera_param = "../../data/file.yml";

void rotateCW90( cv::Mat& input, cv::Mat& output, const unsigned int width, const unsigned int height)
{
  unsigned char *pImg;
  int i;
  unsigned int y;
  unsigned int x;
  int offset;
  int destinationColumn;
  const unsigned int sizeBuffer = width * height * 3; 
  unsigned char *tempBuffer = new unsigned char[sizeBuffer];

  pImg = input.data;

  for (y = 0, destinationColumn = height - 1; y < height; ++y, --destinationColumn){
	offset = y * width;
	for (x = 0; x < width; x++){
	  for (i = 0; i < 3; i++) { // RGB
		tempBuffer[(x * height + destinationColumn) * 3 + i] = pImg[(offset + x) * 3 + i];
	  }
	}
  }
  memcpy(pImg, tempBuffer, sizeBuffer);
  delete[] tempBuffer;

  output.data = pImg;
  
}


void *myThread(void *arg)
{
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
	
	rotateCW90(frame0, out0, W,H);
	rotateCW90(frame1, out1, W,H);
	
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
		  
		  if(  (input.data[a] >=175 || input.data[a] <= 15 ) )  output.at<unsigned char>(y,x) = 255;
		  else  output.at<unsigned char>(y,x) = 0;
		}
	}
}

void exclode_clr_green(cv::Mat &input, cv::Mat &output){
  int a,x,y;
  
  for(y=0; y<H;y++)
	{
	  for(x=0; x<W; x++)
		{
		  a = input.step*y+(x*3);

		  if( (15 <= input.data[a] && input.data[a] <= 90) || (input.data[a+1] <= 100 && input.data[a+2] >= 100 )) {
			output.at<unsigned char>(y,x) = 130;
		  }
		  else  output.at<unsigned char>(y,x) = 0;
		  
		  if( input.data[a] >=175 || input.data[a] <= 15 )  output.at<unsigned char>(y,x) = 255;
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
		if (ratio > KALMAN_MIN_RATIO && bBox.area() >= KALMAN_MIN_SQUARE)
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
  
	//cout << "Measure matrix:" << endl << meas << endl;
	return 0;
  }


int kalman_process(cv::Mat& frame, cv::Mat& rangeRes, cv::Mat& res, vector< vector<cv::Point> >& balls, vector<cv::Rect> &ballsBox){
  cv::Mat blur = cv::Mat::zeros( W, H, CV_8UC3);
  cv::Mat frmHsv;
  vector<vector<cv::Point> > contours;
  
  frame.copyTo(blur);
  cv::cvtColor(blur, frmHsv, CV_BGR2HSV);
  exclode_clr(frmHsv,rangeRes);
  cv::erode(rangeRes, rangeRes, cv::Mat(), cv::Point(-1, -1), 2);
  cv::erode(rangeRes, rangeRes, cv::Mat(), cv::Point(-1, -1), 2);
  cv::erode(rangeRes, rangeRes, cv::Mat(), cv::Point(-1, -1), 2);
  cv::erode(rangeRes, rangeRes, cv::Mat(), cv::Point(-1, -1), 2);
  cv::erode(rangeRes, rangeRes, cv::Mat(), cv::Point(-1, -1), 2);
  
  cv::dilate(rangeRes, rangeRes, cv::Mat(), cv::Point(-1, -1), 2);
  cv::dilate(rangeRes, rangeRes, cv::Mat(), cv::Point(-1, -1), 2);
  cv::dilate(rangeRes, rangeRes, cv::Mat(), cv::Point(-1, -1), 2);
  cv::dilate(rangeRes, rangeRes, cv::Mat(), cv::Point(-1, -1), 2);
  cv::dilate(rangeRes, rangeRes, cv::Mat(), cv::Point(-1, -1), 2);
  cv::findContours(rangeRes, contours, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);
  Filtering(contours,balls,ballsBox);
  Detection_result(res,ballsBox,balls);//show result
  
  return 0;  
}

  int kalman_if_found(cv::KalmanFilter& kf,cv::Mat& state,cv::Mat& res){
	state = kf.predict();
	//  cout << "State post:" << endl << state << endl;

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
	kf.measurementMatrix = cv::Mat::zeros(measSize, stateSize, CV_32F);
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


  void calibrate(){
	bool found1 = false;
	bool found2 = false;
	int success = 0;
	int k = 0;
	int numBoards = NUMBOARD;
	int board_w = BOARD_W;
	int board_h = BOARD_H;
	int board_n = board_w*board_h;
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
	cv::VideoCapture cap1(LEFT);
	cv::VideoCapture cap2(RIGHT);

	for (int j=0; j<board_n; j++)
	  {
		obj.push_back(Point3f(j/board_w, j%board_w, 0.0f));
	  }

	cap1.set(CV_CAP_PROP_FRAME_WIDTH, 640);
	cap1.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
	cap2.set(CV_CAP_PROP_FRAME_WIDTH, 640);
	cap2.set(CV_CAP_PROP_FRAME_HEIGHT, 480);

	while(success < numBoards) {
	  //=============================
	  string fileleft="../data/left0";
	  string fileright="../data/right0";
	  string png=".jpg";
	  
	  cap1 >> frame1;
	  cap2 >> frame2;
	
	  rotateCW90(frame1, out1, W,H);
	  rotateCW90(frame2, out2, W,H);
	
	  //cv::imshow("Capture1", out1);
	  //cv::imshow("Capture2", out2);
	
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

	vector<Mat> rvecs1;
	vector<Mat> tvecs1;
	vector<Mat> rvecs2;
	vector<Mat> tvecs2;

	Mat CM1 = Mat(3, 3, CV_32FC1);
	Mat D1;
	Mat CM2 = Mat(3, 3, CV_32FC1);
	Mat D2;

	CM1.at<float>(0, 0) = 1;
	CM1.at<float>(1, 1) = 1;
	CM2.at<float>(0, 0) = 1;
	CM2.at<float>(1, 1) = 1;
    
	calibrateCamera(object_points, image_points1,out1.size(), CM1, D1, rvecs1, tvecs1);
  
	calibrateCamera(object_points, image_points2,out2.size(), CM2, D2, rvecs2, tvecs2);

	Mat R, T, E, F;

	stereoCalibrate(object_points, image_points1, image_points2, CM1, D1, CM2, D2, out1.size(), R, T, E, F, CALIB_FIX_INTRINSIC,TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 100, 1e-6) );

	FileStorage fs1(camera_param, FileStorage::WRITE);
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

	Rect roi1, roi2;
	Mat R1, R2, P1, P2, Q;
  
	stereoRectify( CM1, D1, CM2, D2, out1.size(), R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, out1.size(), &roi1, &roi2 );

	fs1 << "R1" << R1;
	fs1 << "R2" << R2;
	fs1 << "P1" << P1;
	fs1 << "P2" << P2;
	fs1 << "Q" << Q;

	printf("Done Rectification\n");

	printf("Applying Undistort\n");

	Mat map1x, map1y, map2x, map2y;

	initUndistortRectifyMap(CM1, D1, R1, P1, out1.size(), CV_32FC1, map1x, map1y);
	initUndistortRectifyMap(CM2, D2, R2, P2, out2.size(), CV_32FC1, map2x, map2y);
	fs1 << "map1x" << map1x;
	fs1 << "map1y" << map1y;
	fs1 << "map2x" << map2x;
	fs1 << "map2y" << map2y;
	fs1 << "roi1" << roi1;
	fs1 << "roi2" << roi2;

	printf("Undistort complete\n");
	fs1.release();

	cap1.release();
	cap2.release();  
  }



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


  void *video_finder(void *arg){
	//video setteing
	int i;
	int idx = 0;
	cv::Mat frame;
	cv::Mat res;
	cv::VideoCapture cap;
	cv::Mat out = cv::Mat::zeros( W, H, CV_8UC3);
	if (!cap.open(idx)) return 0;
	else{
	  cap.set(CV_CAP_PROP_FRAME_WIDTH, W);
	  cap.set(CV_CAP_PROP_FRAME_HEIGHT, H);
	}

	i=0;
	while ( 1 ){
	  string file_save_directory="./data/";
	  string png=".jpg";
	  cv::waitKey(1);
	  cap >> frame;
	  cv::Mat blur = cv::Mat::zeros( W, H, CV_8UC3);
	  cv::GaussianBlur(frame, blur, cv::Size(5, 5), 3.0, 3.0);
	  rotateCW90( blur, out, W, H);

	  
	  vector<vector<cv::Point> > balls;
	  vector<cv::Rect> ballsBox;
	  cv::Mat rangeRes = cv::Mat::zeros(out.size(), CV_8UC1);
	  out.copyTo( res );
	  kalman_process(out,rangeRes,res,balls,ballsBox);
		
	  cv::imshow("Tracking", res);

	  if( cameramutex.a == 1 ) break;
	  else if( cameramutex.a == 2 ) {
		cameramutex.lock();
		cameramutex.a = 0;
	  
		std::ostringstream file;
		file << file_save_directory << i << png;
		cv::imwrite(file.str(), res );
	  
		cameramutex.unlock();
	  }
	}
	return 0;
  }



  void *tracker(void *arg){
	//video setteing
	int i;
	int idx = 0;
	cv::Mat frame;
	cv::Mat out = cv::Mat::zeros( W, H, CV_8UC3);
	cv::VideoCapture cap;
	string file_save_directory="./data/";
	string png=".jpg";
	if (!cap.open(idx)) return 0;
	else{
	  cap.set(CV_CAP_PROP_FRAME_WIDTH, W);
	  cap.set(CV_CAP_PROP_FRAME_HEIGHT, H);
	}

	//kalman filter setting
	double ticks = 0;
	bool found = false;
	int notFoundCount = 0;
	int contrSize = 0;
	cv::Mat res;
	cv::KalmanFilter kf(stateSize, measSize, contrSize, CV_32F);
	cv::Mat state(stateSize, 1, CV_32F);  // [x,y,v_x,v_y,w,h]
	cv::Mat meas(measSize, 1, CV_32F);    // [z_x,z_y,z_w,z_h]
	kalman_setting(kf);

	i=0;
	while (1){
	  //video capture
	  cv::waitKey(1);
	  cap >> frame;
	  cv::Mat blur = cv::Mat::zeros( W, H, CV_8UC3);
	  cv::GaussianBlur(frame, blur, cv::Size(5, 5), 3.0, 3.0);
	  rotateCW90( blur, out, W, H);

	  //loop time recorder
	  double precTick = ticks;
	  ticks = (double) cv::getTickCount();
	  double dT = (ticks - precTick) / cv::getTickFrequency();
	  kf.transitionMatrix.at<float>(2) = dT;
	  kf.transitionMatrix.at<float>(9) = dT;
	  //cout << "dT:" << endl << dT << endl;

		
	  //kalman filtering
	  vector<vector<cv::Point> > balls;
	  vector<cv::Rect> ballsBox;
	  cv::Mat rangeRes = cv::Mat::zeros(out.size(), CV_8UC1);
	  out.copyTo( res );
	  if (found) kalman_if_found(kf,state,res);
	  kalman_process(out,rangeRes,res,balls,ballsBox);
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



	  if( cameramutex.a == 1 ) break;
	  else if( cameramutex.a == 2 ) {
		cameramutex.lock();
		cameramutex.a = 0;
	  
		std::ostringstream file;
		file << file_save_directory << i << png;
		cv::imwrite(file.str(), res );
	  
		cameramutex.unlock();
	  }
	}
	return 0;
  }



void *image_finder(void *arg){
  string filename = "../../data/8.jpg";
  
  cv::Mat src_frame = cv::imread(filename, 1);
  if(!src_frame.data) cout << "input file error" << endl;
  cv::Mat frame;
  cv::resize(src_frame,frame,cv::Size(640,480),0,0);
  cv::namedWindow("image", CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);

  cv::Mat res;
  cv::Mat state(stateSize, 1, CV_32F);  // [x,y,v_x,v_y,w,h]
  cv::Mat meas(measSize, 1, CV_32F);    // [z_x,z_y,z_w,z_h]

	
  while( 1 ){
	//show normal image
	cv::imshow("image", frame);
	cv::waitKey(1);

	//kalman filtering
	vector<vector<cv::Point> > balls;
	vector<cv::Rect> ballsBox;
	cv::Mat rangeRes = cv::Mat::zeros(frame.size(), CV_8UC1);
	frame.copyTo( res );
	kalman_process(frame, rangeRes, res, balls, ballsBox);
	cv::imshow("Tracking", res);
	cv::imshow("corner", rangeRes);
  }
  return 0;
}
