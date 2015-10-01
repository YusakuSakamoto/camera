#include "../../../lib/camera.hpp"

int main(int argc, char *argv[])
{
  cv::Mat frame = cv::imread(argv[1], 1);
  cv::Mat src_img;
  if(!frame.data) return -1;
  cv::resize(frame,src_img,cv::Size(W,H),0,0);


  cv::Mat dst_img;
  cv::Mat work_img=cv::Mat(H,W,CV_8UC1);
  cv::Mat hsv;
  cv::Mat work=cv::Mat(H,W,CV_8UC1);;
  
  dst_img = src_img.clone();

  //change HSV
  //====================================
  cv::cvtColor(src_img,hsv,CV_BGR2HSV);
  //===================================

  //green=gray & red=white
  //=================================
  exclode_clr_green(hsv, work_img);
  cv::erode(work_img, work_img, cv::Mat(), cv::Point(-1, -1), 2);
  cv::dilate(work_img, work_img, cv::Mat(), cv::Point(-1, -1), 2);
  //=================================

  //red=white
  //==================================
  exclode_clr(hsv, work);  
  cv::erode(work, work, cv::Mat(), cv::Point(-1, -1), 2);
  cv::erode(work, work, cv::Mat(), cv::Point(-1, -1), 2);
  cv::erode(work, work, cv::Mat(), cv::Point(-1, -1), 2);
  cv::erode(work, work, cv::Mat(), cv::Point(-1, -1), 2);
  cv::erode(work, work, cv::Mat(), cv::Point(-1, -1), 2);

  cv::dilate(work, work, cv::Mat(), cv::Point(-1, -1), 2);
  cv::dilate(work, work, cv::Mat(), cv::Point(-1, -1), 2);
  cv::dilate(work, work, cv::Mat(), cv::Point(-1, -1), 2);
  cv::dilate(work, work, cv::Mat(), cv::Point(-1, -1), 2);
  cv::dilate(work, work, cv::Mat(), cv::Point(-1, -1), 2);
  //=================================

  //hough circle
  //==================================
  // cv::GaussianBlur(work, work, cv::Size(11,11), 2, 2);
  // std::vector<cv::Vec3f> circles;
  // cv::HoughCircles(work, circles, CV_HOUGH_GRADIENT, 1, 100, 20, 50);
  // std::vector<cv::Vec3f>::iterator it = circles.begin();
  // for(; it!=circles.end(); ++it) {
  //   cv::Point center(cv::saturate_cast<int>((*it)[0]), cv::saturate_cast<int>((*it)[1]));
  //   int radius = cv::saturate_cast<int>((*it)[2]);
  //   cv::circle(dst_img, center, radius, cv::Scalar(0,0,255), 2);
  // }
  //==============================================


  Mat canny_img;
  Canny(src_img, canny_img, 5, 200);

  
  

  //show image
  //=======================================
  cv::imshow("exclode_data_red",work);
  cv::imshow("exclode_data",work_img);
  cv::imshow("HoughCircles", dst_img);
  cv::imshow("Canny", canny_img);
  //======================================
  
  cv::waitKey(0);
}
