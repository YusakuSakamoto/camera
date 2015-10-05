#include "../../../lib/camera.hpp"

using namespace std;
using namespace cv;

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
	if( input[0] == 'q' ){ cameramutex.a=1;break;}
	else if( input[0] == 's' ){ cameramutex.a=2;}
	else if( input[0] == 'h' ){ help(); }
	else if( input[0] == 'c' ) { system("clear"); }
	else if( input[0] == ' '){ cameramutex.a=3;}
  }
  end();
  return NULL;  
}



int main(int argc,char *argv[])
{
  int status;
  pthread_t thread_a;
  pthread_t thread_c;
  void *thread_return;

  
  if(argc == 1){
	
	status = pthread_create(&thread_a, NULL, video_finder, NULL);
	if(status != 0 )exit(1);
	status=pthread_create(&thread_c, NULL, myKey,NULL);
	if(status != 0 )exit(1);
	status = pthread_join(thread_c, &thread_return);
	if(status != 0 )exit(1);
	
	return EXIT_SUCCESS;
  }
  
  else if (argc == 2){

	status = pthread_create(&thread_a, NULL, image_finder, NULL);
	if(status != 0 )exit(1);
	status=pthread_create(&thread_c, NULL, myKey,NULL);
	if(status != 0 )exit(1);
	status = pthread_join(thread_c, &thread_return);
	if(status != 0 )exit(1);
	
	return EXIT_SUCCESS;
  }
}
