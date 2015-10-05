#include "../../../lib/camera.hpp"

using namespace std;
using namespace cv;

int main(int argc,char *argv[]){
  cv::Mat img = cv::imread("./data/input.png",1);
  //Mean shift
  int **ilabels = new int *[ img.cols ];
  for(int i=0;i<img.cols;i++){
	ilabels[i] = new int [img.rows];
  }
  int regionCount = MeanShift(img,ilabels);
  vector<int> color(regionCount);
  CvRNG rng = cvRNG(cvGetTickCount());
  for(int i=0;i<regionCount;i++){
	color[i] = cvRandInt(&rng);
  }
  /*
  for(int i=0;i<img.cols;i++){
	for(int j=0;j<img.rows;j++){
	  
	  int cl = ilabels[i][j];
	  int a = img.step * i + (j*3);
	  
	  img.data[a+0] = (color[cl]) & 255;
	  img.data[a+1] = (color[cl] >> 8) & 255;
	  img.data[a+2] = (color[cl] >> 16) & 255;
	 
	}
  }
  */
  cv::imshow("img",img);
  cv::waitKey(0);
  return 0;
}
