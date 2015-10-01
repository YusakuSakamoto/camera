#include "../../../lib/camera.hpp"
 
using namespace cv;
 
int
main (int argc, char **argv)
{
  cv::Mat src_img;
  const char *imagename = argc > 1 ? argv[1] : "../image/bike_sign.png";
  cv::Mat frame = cv::imread(imagename, 0);
  cv::resize(frame,src_img,cv::Size(W,H),0,0);
  if(!src_img.data)
    return -1;

  // (4)implement the Canny algorithm for edge detection
  Mat canny_img;
  Canny(src_img, canny_img, 5, 200);
 
  // (5)show original gray and their edge images respectively, 
  //    and quit when any key pressed
  imshow("Canny", canny_img);
  waitKey(0);
 
  return 0;
}
