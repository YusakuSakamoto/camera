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



int MeanShift(cv::Mat& img,int **labels){

  DECLARE_TIMING(timer);
  START_TIMING(timer);

  int level = 1;
  double color_radius2 = color_radius * color_radius;
  int minRegion = 50;

  cv::Mat result = cv::Mat::zeros(cv::Size(img.cols,img.rows),CV_8UC3);
  cv::cvtColor( img, result, CV_RGB2Lab);
  meanshift_step_ONE( img, result);  
  cv::Mat tobeshow;
  cv::cvtColor( result, tobeshow, CV_Lab2RGB);
  cv::imwrite("./data/filtered.png", tobeshow);


  // Step Two. Cluster
  // Connect
  int regionCount = 0;
  int *modePointCounts = new int[img.cols * img.rows];
  memset(modePointCounts,0,img.cols * img.rows*sizeof(int));
  float *mode = new float[img.cols * img.rows * 3];
  {
	int label = -1;
	for(int i=0; i<img.cols; i++)
	  for(int j=0; j<img.rows; j++)
		labels[i][j] = -1;
	
	for(int i=0; i<img.cols; i++)
	  for(int j=0; j<img.rows; j++)
		if( labels[i][j] < 0 ){
		  labels[i][j] = ++label;
		  //cout << label << endl;
		  int a = result.step*i + j*3;
		  float L = (float)result.data[a+0];
		  float A = (float)result.data[a+1];
		  float B = (float)result.data[a+2];

		  mode[ label*3 + 0 ] = L*100/255;
		  mode[ label*3 + 1 ] = 354*A/255-134;
		  mode[ label*3 + 2 ] = 256*B/255-140;

		  //Fill
		  std::stack<CvPoint> neighStack;
		  neighStack.push( cvPoint(i,j) );
		  const int dxdy[][2] = {{-1,-1},{-1,0},{-1,1},{0,-1},{0,1},{1,-1},{1,0},{1,1}};

		  while( !neighStack.empty() ){
			CvPoint p = neighStack.top();
			neighStack.pop();
			
			for(int k=0; k<8; k++){ //8近傍探索
			  int i2 = p.x + dxdy[k][0];
			  int j2 = p.y + dxdy[k][1];
			  if( i2>=0 && j2>=0 && i2<img.cols && j2<img.rows && labels[i2][j2] < 0 && color_distance(result,i,j,i2,j2) < color_radius2 ){
				labels[i2][j2] = label;
				neighStack.push(cvPoint(i2,j2));
				modePointCounts[label]++;
				
				a = result.step*i2 + j2*3;
				L = (float)result.data[a+0];
				A = (float)result.data[a+1];
				B = (float)result.data[a+2];

				mode[label*3+0] += L*100/255;
				mode[label*3+1] += 354*A/255-134;
				mode[label*3+2] += 256*B/255-140;
			  }			  
			}
		  }
		  mode[label*3+0] /= modePointCounts[label];
		  mode[label*3+1] /= modePointCounts[label];
		  mode[label*3+2] /= modePointCounts[label];
		}
	// current Region count
	regionCount = label + 1;
  }
  std::cout<< "Mean Shift(Connect):" << regionCount << std::endl;
  int oldRegionCount = regionCount;
  return 0;
}


void meanshift_step_ONE( cv::Mat& img, cv::Mat& result ){
  //Step one:Filtering stage of meanshift segmentation.
  //please show https://archive.is/GX1YW
  double color_radius2 = color_radius * color_radius;
  for(int i=0; i<img.cols; i++ ){
	for(int j=0; j<img.rows; j++){
	  int ic = i;
	  int jc = j;
	  int icOld, jcOld;
	  int a = result.step*j + i*3;
	  float LOld,AOld,BOld;
	  float L = (float)result.data[a+0];
	  float A = (float)result.data[a+1];
	  float B = (float)result.data[a+2];

	  L = L*100/255;
	  A = A-128;
	  B = B-128;
	  double shift = 5;
	  for( int iters=0; shift > 3 && iters < 100; iters++ ){
		icOld = ic;
		jcOld = jc;
		LOld = L;
		AOld = A;
		BOld = B;
		
		float mi = 0;
		float mj = 0;
		float mL = 0;
		float mA = 0;
		float mB = 0;
		int num = 0;

		int i2from = max(0,i-spatial_radius),i2to = min(img.cols,i+spatial_radius+1);
		int j2from = max(0,j-spatial_radius),j2to = min(img.rows,j+spatial_radius+1);

		for(int i2=i2from; i2 < i2to; i2++){
		  for(int j2=j2from; j2 < j2to; j2++){
			
			int a2 = result.step*j2 + i2*3;
			float L2 = (float)result.data[a2+0];
			float A2 = (float)result.data[a2+1];
			float B2 = (float)result.data[a2+2];

			L2 = L2*100/255;
			A2 = A2-128;
			B2 = B2-128;

			double dL = L2 - L;
			double dA = A2 - A;
			double dB = B2 - B;

			if(dL*dL+dA*dA+dB*dB <= color_radius2){
			  mi += i2;
			  mj += j2;
			  mL += L2;
			  mA += A2;
			  mB += B2;
			  num++;
			}
		  }
		}
		
		float num_ = 1.f/num;
		L = mL*num_;
		A = mA*num_;
		B = mB*num_;
		ic = (int) (mi*num_+0.5);
		jc = (int) (mj*num_+0.5);
		int di = ic-icOld;
		int dj = jc-jcOld;
		double dL = L-LOld;
		double dA = A-AOld;
		double dB = B-BOld;

		shift = di*di + dj*dj + dL+dL + dA*dA + dB*dB;
	  }
	  L = L*255/100;
	  A = A+128;
	  B = B+128;
	  result.data[a+0] = (uchar)L;
	  result.data[a+1] = (uchar)A;
	  result.data[a+2] = (uchar)B;
	}
  }
}
