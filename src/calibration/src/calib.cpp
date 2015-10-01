#include "../../../lib/camera.hpp"

using namespace std;
using namespace cv;

myMutex cameramutex;
string camera_param = "../../data/file.yml";

void stereoMatching(cv::Mat& left,cv::Mat& right,cv::Mat& disp){
  
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
	cv::Mat disp = Mat(480,640,CV_8UC1);
	cv::VideoCapture cap1(LEFT);
	cv::VideoCapture cap2(RIGHT);
	string fileleft="./data/";
	string fileright="./data/";
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
	  
		stereoMatching(grayU1,grayU2,disp);
		
		imshow("undistort1", grayU1);
		imshow("undistort2", grayU2);
		//imshow("disp", disparity);

		waitKey(5);
		if( cameramutex.a == 1 ) break;
		else if( cameramutex.a == 2 ) {
		  cameramutex.lock();
		  cameramutex.a = 0;
	  
		  std::ostringstream file;
		  file << fileright << i++ << png;
		  cv::imwrite(file.str(), grayU1 );
	  
		  std::ostringstream file2;
		  file2 << fileleft << i++ << png;
		  cv::imwrite(file2.str(), grayU2 );
		  cameramutex.unlock();
		}
	  }
	cap1.release();
	cap2.release();
	return NULL;
  }


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
