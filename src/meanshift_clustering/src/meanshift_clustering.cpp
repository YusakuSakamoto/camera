#include "../../../lib/camera.hpp"

using namespace std;
using namespace cv;

RAList::RAList(void){
  label  = -1;
  next   = 0;
}

RAList::~RAList(void){
}

int RAList::Insert(RAList *entry){
  if(!next){
	next = entry;
	entry->next = 0;
	return 0;
  }
  if( next->label > entry->label ){
	entry->next=next;
	next=entry;
	return 0;
  }
  exists=0;
  cur=next;
  while(cur)
	{
	  if(entry->label == cur->label)
		{
		  exists = 1;
		  break;
		}
	  else if( (!(cur->next)) || (cur->next->label > entry->label) ){
		entry->next = cur->next;
		cur->next = entry;
		break;
	  }
	}
  return (int)exists;
}



int MeanShift(cv::Mat& img,int **ilabels){
  DECLARE_TIMING(timer);
  START_TIMING(timer);

  int level = 1;
  double color_radius2 = color_radius * color_radius;
  int minRegion = 50;

  cv::Mat result = cv::Mat::zeros(cv::Size(img.cols,img.rows),CV_8UC3);
  
  
  return 0;
}
