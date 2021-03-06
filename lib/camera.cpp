#include "./camera.hpp"

using namespace std;
using namespace cv;

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



void exclode_clr(cv::Mat &input, cv::Mat &output){
  int a,x,y;
  
  for(y=0; y<H;y++)
	{
	  for(x=0; x<W; x++)
		{
		  a = input.step*y+(x*3);
		  
		  if(  (input.data[a] >=175 || input.data[a] <= 5 ) )  output.at<unsigned char>(y,x) = 255;
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

/*mean shift
  HOW to use
  arg 1 = Type dataset array
  arg 2 = Mat output (gray scale) 
  arg 3 = Mat output's array number
  arg 4 = mean shift circle radius
  arg 5 = epsilon
  arg 6 = max loop number
  ====================================================*/
void mean_shift(dataset* set, cv::Mat& output, const int num,const int h  ,const double threshold, const int max_loop){
  
  //変数宣言
  //===========================================
  int a,i,j;
  int flag = 1;
  int loop = 0;
  //===========================================

  for(j=0;j<num;j++){
	//mean shift による中心保存用変数と収束確認用変数の初期化
	//==============================================
	loop = 0;
	float average_x = set[j].x;
	float average_y = set[j].y;
	float average_x_pre = set[j].x;
	float average_y_pre = set[j].y;
	//==============================================
	
	if( set[j].flag == 0 ){//まだ触れていない注目画素であれば
	  while(1){//収束が確認できるまでの無限ループ
		loop++;//loop counter

		//for moment calc
		//===============================================
		int rec_n=0;
		int rec[ (int)(pow(2*h,2)) ];
		memset(rec,0,sizeof(rec));
		float moment_x = 0;
		float moment_y = 0;
		int moment_number = 0;
		
		for(i=0;i<num;i++){
		  if( (pow( pow(average_x-set[i].x,2) + pow(average_y-set[i].y,2 ),0.5)) < h  ){
			moment_x += (set[i].x - average_x) * set[i].value;
			moment_y += (set[i].y - average_y) * set[i].value;
			moment_number += set[i].value;
			rec[rec_n++] = i;
		  }
		}
		average_x += moment_x / moment_number;
		average_y += moment_y / moment_number;
		//==================================================

		//繰り返し判定
		//=====================================================
		if( ((abs(average_x_pre-average_x) < threshold) && (abs(average_y_pre-average_y) < threshold )) ){//収束条件
		  a = output.step*(int)average_y + (int)average_x;
		  output.data[a] = 255;
		  for(i=0;i<rec_n;i++){
			if( set[ rec[i] ].flag == 0 ){
			  set[ rec[i] ].flag = flag;
			  set[j].flag = flag;
			}else{
			  set[j].flag = set[ rec[i] ].flag;
			}
		  }
		  flag++;
		  break;
		}else if( loop > max_loop ){//収束しなければ
		  break;
		}else{//繰り返し計算の途中
		  // for(int k=0;k<rec_n;k++){
		  // 	set[ rec[k] ].flag = flag;
		  // }
		  average_x_pre = average_x;
		  average_y_pre = average_y;
		  moment_number = 0;
		}
	  }
	  //=====================================================
	}
	printf("(%d/%d)\n",j,num);
  }
}


int  make_EDM(const int height,const int width,cv::Mat& input,cv::Mat& output,dataset* set){
  int a,i,x,y;

  //距離地図作成 loop1
  for(y=1; y<=height-1;y++){
	for(x=1; x<=width-1; x++){
	  a = input.step*y+x;
	  if( input.data[a]>150){
		
		output.data[a] = output.data[a-output.step-1] + 1;
		if( output.data[a] > output.data[a-1] + 1  )
		  output.data[a] = output.data[a-1] + 1;
		if( output.data[a] > output.data[a-output.step] + 1  )
		  output.data[a] = output.data[a-output.step] + 1;
	  }
	  else  output.data[a]=0;
	}
  }

  i=0;
  //距離地図作成 loop2
  for(y=height-1; y>=1;y--){
	for(x=width-1; x>=1; x--){
	  a = output.step*y+x;
	  if( input.data[a]>150){
		if( output.data[a] > output.data[a+output.step+1] + 1 )
		  output.data[a] = output.data[a+output.step+1] + 1;
		if( output.data[a] > output.data[a+1] + 1  )
		  output.data[a] = output.data[a+1] + 1;
		if( output.data[a] > output.data[a+output.step] + 1  )
		  output.data[a] = output.data[a+output.step] + 1;

		set[i].x = x;
		set[i].y = y;
		set[i].flag = 0;
		set[i].value = output.data[a];
		i++;
	  }
	  else  output.data[a] = 0;
	}
  }
  return i;
}
