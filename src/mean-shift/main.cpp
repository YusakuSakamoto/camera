#include <cv.h>
#include <highgui.h>
#include <stdio.h>

int
main (int argc, char **argv)
{
  int level = 2;
  IplImage *src_img=cvCreateImage( cvSize(640,480), 8, 3 );
  IplImage *dst_img;
  IplImage *src;
  CvRect roi;

  // (1)画像の読み込み
  if (argc >= 2)
    src = cvLoadImage (argv[1], CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);
  else return 0;
  
  cvResize( src, src_img, CV_INTER_LINEAR );

  // (2)領域分割のためにROIをセットする
  printf("set roi");
  roi.x = roi.y = 0;
  roi.width = src_img->width & -(1 << level);
  roi.height = src_img->height & -(1 << level);
  cvSetImageROI (src_img, roi);

  // (3)分割結果画像出力用の画像領域を確保し，領域分割を実行
  dst_img = cvCloneImage (src_img);

  printf("start.");  
  cvPyrMeanShiftFiltering (src_img, dst_img, 30.0, 30.0, level, cvTermCriteria (CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 5, 1));

  // (4)入力画像と分割結果画像の表示
  cvNamedWindow ("Source", CV_WINDOW_AUTOSIZE);
  cvNamedWindow ("MeanShift", CV_WINDOW_AUTOSIZE);
  cvShowImage ("Source", src_img);
  cvShowImage ("MeanShift", dst_img);
  cvWaitKey (0);

  cvDestroyWindow ("Source");
  cvDestroyWindow ("MeanShift");
  cvReleaseImage (&src_img);
  cvReleaseImage (&dst_img);

  return 0;
}
