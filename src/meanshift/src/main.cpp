#include "../include/Meanshift.h"

using namespace cv;
using namespace std;

int main(int argc, char* argv[])
{
  cv::Mat input = cv::imread("./data/input.png",1);
  cv::Mat output = cv::Mat::zeros(input.rows,input.cols,CV_8UC3);
  
  // create label container
  int **ilabels = new int *[input.rows];
  for(int i=0;i<input.rows;i++){
	ilabels[i] = new int [input.cols];
  }
  
  // Mean shift
  Meanshift shift(input);
  int regionCount = shift.meanshift(input, ilabels);

  
  //乱数生成
  vector<int> color(regionCount);
  cv::RNG gen(cv::getTickCount());
  gen.fill(color, cv::RNG::UNIFORM, cv::Scalar(0), cv::Scalar(pow(256,3)));
  
  //ランダム色を代入
  for(int i=0;i<output.rows;i++){
	for(int j=0;j<output.cols;j++)
	  { 
		int cl = ilabels[i][j];
		int a = output.step*i + j*3;
		output.data[a+0] = (color[cl])&255;
		output.data[a+2] = (color[cl] >> 8)&255;
		output.data[a+2] = (color[cl] >> 16)&255;
	  }
  }

  // 結果表示
  cv::imshow("input",input);
  cv::imshow("output",output);
  cvWaitKey();

  // メモリ解放
  input.release();
  output.release();

  return 0;
}
