#include "../include/calib.hpp"

myMutex cameramutex;

void calibrate(MY_THREAD_ARG* thread_message){
  bool found1 = false;
  bool found2 = false;
  int success = 0;
  int k = 0;
  int numBoards = NUMBOARD;
  int board_w = BOARD_W;
  int board_h = BOARD_H;
  int board_n = board_w*board_h;
  unsigned char *pImg1;
  unsigned char *pImg2;
  const int W = 640;
  const int H = 480;
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
  cv::VideoCapture cap1(0);
  cv::VideoCapture cap2(1);

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
	string fileleft="./data/left0";
	string fileright="./data/right0";
	string png=".jpg";
	  
	cap1 >> frame1;
	cap2 >> frame2;
	
	pImg1 = frame1.data;
	pImg2 = frame2.data;
	
	rotateCW90(pImg1, W,H);
	rotateCW90(pImg2, W,H);
	
	out1.data = pImg1;
	out2.data = pImg2;
	
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

  thread_message->intrinsic1.at<float>(0, 0) = 1;
  thread_message->intrinsic1.at<float>(1, 1) = 1;
  thread_message->intrinsic2.at<float>(0, 0) = 1;
  thread_message->intrinsic2.at<float>(1, 1) = 1;
    
  calibrateCamera(object_points, image_points1,out1.size(), thread_message->intrinsic1, thread_message->distcoeffs1, rvecs1, tvecs1);
  calibrateCamera(object_points, image_points2,out2.size(), thread_message->intrinsic2, thread_message->distcoeffs2, rvecs2, tvecs2);

  FileStorage fs1("mycalib1.yml", FileStorage::WRITE);
  fs1 << "CM1" << thread_message->intrinsic1;
  fs1 << "D1" << thread_message->distcoeffs1;

  FileStorage fs2("mycalib2.yml", FileStorage::WRITE);
  fs2 << "CM2" << thread_message->intrinsic2;
  fs2 << "D2" << thread_message->distcoeffs2;

  printf("calibration done\n");

  cap1.release();
  cap2.release();  
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

void rotateCW90(unsigned char *buffer, const unsigned int width, const unsigned int height)
{
  const unsigned int sizeBuffer = width * height * 3; 
  unsigned char *tempBuffer = new unsigned char[sizeBuffer];

  for (unsigned int y = 0, destinationColumn = height - 1; y < height; ++y, --destinationColumn)
    {
	  int offset = y * width;

	  for (unsigned int x = 0; x < width; x++)
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
  int i=0;
  MY_THREAD_ARG *thread_message =(MY_THREAD_ARG*)arg;
  const int W = 640;
  const int H = 480;
  unsigned char *pImg1;
  unsigned char *pImg2;
  cv::Mat frame1;
  cv::Mat frame2;
  cv::Mat out1 = cv::Mat::zeros( W, H, CV_8UC3);
  cv::Mat out2 = cv::Mat::zeros( W, H, CV_8UC3);
  cv::Mat imgU1;
  cv::Mat imgU2;
  cv::VideoCapture cap1(0);
  cv::VideoCapture cap2(1);
  string fileleft="./data/left0";
  string fileright="./data/right0";
  string png=".jpg";

  while(1)
    {
	  cap1 >> frame1;
	  cap2 >> frame2;

	  pImg1 = frame1.data;
	  pImg2 = frame2.data;
	
	  rotateCW90(pImg1, W,H);
	  rotateCW90(pImg2, W,H);
	
	  out1.data = pImg1;
	  out2.data = pImg2;
	  
	  undistort(out1, imgU1, thread_message->intrinsic1, thread_message->distcoeffs1);
	  undistort(out2, imgU2, thread_message->intrinsic2, thread_message->distcoeffs2);
	  

	  //imshow("image1", out1);
	  imshow("undistort1", imgU1);
	  //imshow("image2", out2);
	  imshow("undistort2", imgU2);

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
