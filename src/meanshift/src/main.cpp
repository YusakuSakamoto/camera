#include "../include/Meanshift.h"

using namespace cv;
using namespace std;

int main(int argc, char* argv[])
{
  IplImage *img = cvLoadImage("./data/input.png");
  cv::Mat output = cv::Mat::zeros(img->height,img->width,CV_8UC3);
  
  // Mean shift
  int **ilabels = new int *[img->height];
  for(int i=0;i<img->height;i++){
	ilabels[i] = new int [img->width];
  }
  int regionCount = MeanShift(img, ilabels);

  
  //乱数生成
  vector<int> color(regionCount);
  cv::RNG gen(cv::getTickCount());
  gen.fill(color, cv::RNG::UNIFORM, cv::Scalar(0), cv::Scalar(pow(256,3)));
  
  //ランダム色を代入
  for(int i=0;i<img->height;i++){
	for(int j=0;j<img->width;j++)
	  { 
		int cl = ilabels[i][j];
		int a = output.step*i + j*3;
		output.data[a+0] = (color[cl])&255;
		output.data[a+2] = (color[cl] >> 8)&255;
		output.data[a+2] = (color[cl] >> 16)&255;
	  }
  }

  cvNamedWindow("MeanShift",CV_WINDOW_AUTOSIZE);
  cvShowImage("MeanShift",img);
  cv::imshow("output",output);
  cvWaitKey();
  cvDestroyWindow("MeanShift");
  cvReleaseImage(&img);

  return 0;
}
