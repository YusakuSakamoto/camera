#include <cv.h>
#include <highgui.h>
#include <ctype.h>

int main (int argc, char **argv)
{
  CvCapture *capture0 = 0;
  CvCapture *capture1 = 0;
  IplImage *frame0 = 0;
  IplImage *frame1 = 0;
  double w = 640, h = 480;
  int c;

  capture0 = cvCreateCameraCapture (0);
  capture1 = cvCreateCameraCapture (1);
  /* この設定は，利用するカメラに依存する */
  // (2)キャプチャサイズを設定する．
  cvSetCaptureProperty (capture0, CV_CAP_PROP_FRAME_WIDTH, w);
  cvSetCaptureProperty (capture0, CV_CAP_PROP_FRAME_HEIGHT, h);

  cvSetCaptureProperty (capture1, CV_CAP_PROP_FRAME_WIDTH, w);
  cvSetCaptureProperty (capture1, CV_CAP_PROP_FRAME_HEIGHT, h);

  cvNamedWindow ("Capture0", CV_WINDOW_AUTOSIZE);
  cvNamedWindow ("Capture1", CV_WINDOW_AUTOSIZE);

  // (3)カメラから画像をキャプチャする
  while (1) {
    frame0 = cvQueryFrame (capture0);
	frame1 = cvQueryFrame (capture1);
    cvShowImage ("Capture0", frame0);
	cvShowImage ("Capture1", frame1);
	
    c = cvWaitKey (2);
    if (c == '\x1b')
      break;
  }

  cvReleaseCapture (&capture0);
  cvReleaseCapture (&capture1);
  cvDestroyWindow ("Capture0");
  cvDestroyWindow ("Capture1");

  return 0;
}
