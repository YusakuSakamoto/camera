#include "../include/Meanshift.h"
#include "../include/RAList.h"

Meanshift::Meanshift(cv::Mat& input):
  regionCount(0)
{
  // Step Zero. prepare to use IplImage & Lab color space.
  // use Lab rather than L*u*v!
  // since Luv may produce noise points
  //======================================================
  input.copyTo( in );
  cv::cvtColor( in, out, CV_RGB2Lab);
  
  height = input.rows;
  width = input.cols;

  imgbody = input;
  img = &imgbody;
  result = cvCreateImage(cvGetSize(img),img->depth,input.channels());
  cvCvtColor(img, result, CV_RGB2Lab);
  
  mode = new float[height*width*3];
  modePointCounts = new int[height*width];
  memset(modePointCounts, 0, width * height * sizeof(int));
  //======================================================
}

Meanshift::~Meanshift(){
  //Step Five. 
  // Output & after treatment
  //======================================================
  delete []mode;
  delete []modePointCounts;
  cvReleaseImage( &img );
  cvReleaseImage( &result );
  //======================================================  
}




void Meanshift::meanshift(int **labels)
{
  DECLARE_TIMING(timer);
  START_TIMING(timer);
  
  // Step One. Filtering stage of meanshift segmentation
  // http://rsbweb.nih.gov/ij/plugins/download/Mean_Shift.java
  //==========================================================
  meanshift_step_one();
  cv::Mat save;
  cv::cvtColor(out,save,CV_Lab2RGB);
  cv::imwrite("./data/filtered.png", save);
  save.release();
  //===========================================================

  // Step Two. Cluster
  // Connect(接合)
  //=============================================================
  meanshift_step_two(labels);
  std::cout<<"Mean Shift(Connect):" << regionCount << std::endl;
  oldRegionCount = regionCount;
  //===========================================================
  
  // Step Three.
  // TransitiveClosure(推移閉包)
  //==========================================================
  meanshift_step_three(labels);
  //==========================================================
    
  // Step Four.
  // Prune(除去(最適化))
  //===========================================================
  meanshift_step_four(labels);
  //==========================================================
  
  //Step Five. 
  // Output & after treatment
  //======================================================
  STOP_TIMING(timer);
  std::cout<<"Mean Shift(ms):"<<GET_TIMING(timer)<<std::endl;
  //======================================================
}




void Meanshift::meanshift_step_one(){
  
  // Step One. Filtering stage of meanshift segmentation
  // http://rsbweb.nih.gov/ij/plugins/download/Mean_Shift.java
  for(int i=0; i < height; i++) 
	for(int j=0; j < width; j++)
	  {
		int ic = i;
		int jc = j;
		int icOld, jcOld;
		float LOld, UOld, VOld;
		
		int a = out.step*i+j*3;
		float L = out.data[a+0];
		float U = out.data[a+1];
		float V = out.data[a+2];

		// in the case of 8-bit and 16-bit images R, G and B are converted to floating-point format and scaled to fit 0 to 1 range
		// http://opencv.willowgarage.com/documentation/c/miscellaneous_image_transformations.html
		L = L*100/255;
		U = U-128;
		V = V-128;
		double shift = 5;
		for (int iters=0; shift > 3 && iters < 100; iters++) 
		  {
			icOld = ic;
			jcOld = jc;
			LOld = L;
			UOld = U;
			VOld = V;

			float mi = 0;
			float mj = 0;
			float mL = 0;
			float mU = 0;
			float mV = 0;
			int num=0;

			int i2from = max(0,i-spatial_radius), i2to = min(height, i+spatial_radius+1);
			int j2from = max(0,j-spatial_radius), j2to = min(width, j+spatial_radius+1);
			for (int i2=i2from; i2 < i2to; i2++) {
			  for (int j2=j2from; j2 < j2to; j2++) {
				int a2 = out.step*i2 + j2*3;
				float L2 = (float)out.data[a2 + 0];
				float U2 = (float)out.data[a2 + 1];
				float V2 = (float)out.data[a2 + 2];
				
				L2 = L2*100/255;
				U2 = U2-128;
				V2 = V2-128;

				double dL = L2 - L;
				double dU = U2 - U;
				double dV = V2 - V;
				if (dL*dL+dU*dU+dV*dV <= color_radius2) {
				  mi += i2;
				  mj += j2;
				  mL += L2;
				  mU += U2;
				  mV += V2;
				  num++;
				}
			  }
			}
			float num_ = 1.f/num;
			L = mL*num_;
			U = mU*num_;
			V = mV*num_;
			ic = (int) (mi*num_+0.5);
			jc = (int) (mj*num_+0.5);
			int di = ic-icOld;
			int dj = jc-jcOld;
			double dL = L-LOld;
			double dU = U-UOld;
			double dV = V-VOld;

			shift = di*di+dj*dj+dL*dL+dU*dU+dV*dV; 
		  }

		L = L*255/100;
		U = U+128;
		V = V+128;

		a = out.step*i + j*3;
		out.data[a + 0] = (uchar)L;
		out.data[a + 1] = (uchar)U;
	    out.data[a + 2] = (uchar)V;

		((uchar *)(result->imageData + i*result->widthStep))[j*result->nChannels + 0] = (uchar)L;
		((uchar *)(result->imageData + i*result->widthStep))[j*result->nChannels + 1] = (uchar)U;
		((uchar *)(result->imageData + i*result->widthStep))[j*result->nChannels + 2] = (uchar)V;
	  }
}



void Meanshift::meanshift_step_two( int **labels){
  // Step Two. Cluster
  // Connect
  int label = -1;
  for(int i=0; i < height; i++) 
	for(int j=0; j < width; j++)
	  labels[i][j] = -1;
  for(int i=0; i < height; i++) 
	for(int j=0; j < width; j++)
	  if(labels[i][j]<0)
		{
		  labels[i][j] = ++label;
		  float L = (float)((uchar *)(result->imageData + i*result->widthStep))[j*result->nChannels + 0],
			U = (float)((uchar *)(result->imageData + i*result->widthStep))[j*result->nChannels + 1],
			V = (float)((uchar *)(result->imageData + i*result->widthStep))[j*result->nChannels + 2];
		  mode[label*3+0] = L*100/255;
		  mode[label*3+1] = 354*U/255-134;
		  mode[label*3+2] = 256*V/255-140;
		  // Fill
		  std::stack<CvPoint> neighStack;
		  neighStack.push(cvPoint(i,j));
		  const int dxdy[][2] = {{-1,-1},{-1,0},{-1,1},{0,-1},{0,1},{1,-1},{1,0},{1,1}};
		  while(!neighStack.empty())
			{
			  CvPoint p = neighStack.top();
			  neighStack.pop();
			  for(int k=0;k<8;k++)
				{
				  int i2 = p.x+dxdy[k][0], j2 = p.y+dxdy[k][1];
				  if(i2>=0 && j2>=0 && i2 < height && j2 < width && labels[i2][j2] < 0 && color_distance(result, i,j,i2,j2)<color_radius2)
					{
					  labels[i2][j2] = label;
					  neighStack.push(cvPoint(i2,j2));
					  modePointCounts[label]++;
					  L = (float)((uchar *)(result->imageData + i2*result->widthStep))[j2*result->nChannels + 0];
					  U = (float)((uchar *)(result->imageData + i2*result->widthStep))[j2*result->nChannels + 1];
					  V = (float)((uchar *)(result->imageData + i2*result->widthStep))[j2*result->nChannels + 2];
					  mode[label*3+0] += L*100/255;
					  mode[label*3+1] += 354*U/255-134;
					  mode[label*3+2] += 256*V/255-140;
					}
				}
			}
		  mode[label*3+0] /= modePointCounts[label];
		  mode[label*3+1] /= modePointCounts[label];
		  mode[label*3+2] /= modePointCounts[label];
		}
  //current Region count
  regionCount = label+1;
}






void Meanshift::meanshift_step_three(int** labels){
  // Step three.
  // TransitiveClosure(推移閉包)
  for(int counter = 0, deltaRegionCount = 1; counter<5 && deltaRegionCount>0; counter++)
	{
	  // 1.Build RAM using classifiction structure
	  RAList *raList = new RAList [regionCount];
	  RAList *raPool = new RAList [10*regionCount];	//10 is hard coded!
	  for(int i = 0; i < regionCount; i++)
		{
		  raList[i].label = i;
		  raList[i].next = NULL;
		}
	  for(int i = 0; i < regionCount*10-1; i++)
		{
		  raPool[i].next = &raPool[i+1];
		}
	  raPool[10*regionCount-1].next = NULL;
	  RAList	*raNode1, *raNode2, *oldRAFreeList, *freeRAList = raPool;
	  for(int i=0; i < height; i++) 
		for(int j=0; j < width; j++)
		  {
			if(i>0 && labels[i][j]!=labels[i-1][j])
			  {
				// Get 2 free node
				raNode1			= freeRAList;
				raNode2			= freeRAList->next;
				oldRAFreeList	= freeRAList;
				freeRAList		= freeRAList->next->next;
				// connect the two region
				raNode1->label	= labels[i][j];
				raNode2->label	= labels[i-1][j];
				if(raList[labels[i][j]].Insert(raNode2))	//already exists!
				  freeRAList = oldRAFreeList;
				else
				  raList[labels[i-1][j]].Insert(raNode1);
			  }
			if(j>0 && labels[i][j]!=labels[i][j-1])
			  {
				// Get 2 free node
				raNode1			= freeRAList;
				raNode2			= freeRAList->next;
				oldRAFreeList	= freeRAList;
				freeRAList		= freeRAList->next->next;
				// connect the two region
				raNode1->label	= labels[i][j];
				raNode2->label	= labels[i][j-1];
				if(raList[labels[i][j]].Insert(raNode2))
				  freeRAList = oldRAFreeList;
				else
				  raList[labels[i][j-1]].Insert(raNode1);
			  }
		  }

	  // 2.Treat each region Ri as a disjoint set
	  for(int i = 0; i < regionCount; i++)
		{
		  RAList *neighbor = raList[i].next;
		  while(neighbor)
			{
			  if(color_distance(&mode[3*i], &mode[3*neighbor->label])<color_radius2)
				{
				  int iCanEl = i, neighCanEl	= neighbor->label;
				  while(raList[iCanEl].label != iCanEl) iCanEl = raList[iCanEl].label;
				  while(raList[neighCanEl].label != neighCanEl) neighCanEl = raList[neighCanEl].label;
				  if(iCanEl<neighCanEl)
					raList[neighCanEl].label = iCanEl;
				  else
					{
					  //raList[raList[iCanEl].label].label = iCanEl;
					  raList[iCanEl].label = neighCanEl;
					}
				}
			  neighbor = neighbor->next;
			}
		}
	  // 3. Union Find
	  for(int i = 0; i < regionCount; i++)
		{
		  int iCanEl	= i;
		  while(raList[iCanEl].label != iCanEl) iCanEl	= raList[iCanEl].label;
		  raList[i].label	= iCanEl;
		}
	  // 4. Traverse joint sets, relabeling image.
	  int *modePointCounts_buffer = new int[regionCount];
	  memset(modePointCounts_buffer, 0, regionCount*sizeof(int));
	  float *mode_buffer = new float[regionCount*3];
	  int	*label_buffer = new int[regionCount];

	  for(int i=0; i < regionCount; i++)
		{
		  label_buffer[i]	= -1;
		  mode_buffer[i*3+0] = 0;
		  mode_buffer[i*3+1] = 0;
		  mode_buffer[i*3+2] = 0;
		}
	  for(int i=0;i<regionCount; i++)
		{
		  int iCanEl	= raList[i].label;
		  modePointCounts_buffer[iCanEl] += modePointCounts[i];
		  for(int k=0;k<3;k++)
			mode_buffer[iCanEl*3+k] += mode[i*3+k]*modePointCounts[i];
		}
	  int	label = -1;
	  for(int i = 0; i < regionCount; i++)
		{
		  int iCanEl	= raList[i].label;
		  if(label_buffer[iCanEl] < 0)
			{
			  label_buffer[iCanEl]	= ++label;

			  for(int k = 0; k < 3; k++)
				mode[label*3+k]	= (mode_buffer[iCanEl*3+k])/(modePointCounts_buffer[iCanEl]);

			  modePointCounts[label]	= modePointCounts_buffer[iCanEl];
			}
		}
	  regionCount = label+1;
	  for(int i = 0; i < height; i++)
		for(int j = 0; j < width; j++)
		  labels[i][j]	= label_buffer[raList[labels[i][j]].label];

	  delete [] mode_buffer;
	  delete [] modePointCounts_buffer;
	  delete [] label_buffer;

	  //Destroy RAM
	  delete[] raList;
	  delete[] raPool;

	  deltaRegionCount = oldRegionCount - regionCount;
	  oldRegionCount = regionCount;
	  std::cout << "Mean Shift(TransitiveClosure):" << regionCount << std::endl;
	}
}



void Meanshift::meanshift_step_four(int** labels){
  int *modePointCounts_buffer = new int[regionCount];
  float *mode_buffer = new float[regionCount*3];
  int	*label_buffer = new int [regionCount];
  int minRegionCount;

  do{
	minRegionCount = 0;
	// Build RAM again
	RAList *raList = new RAList [regionCount];
	RAList *raPool = new RAList [10*regionCount];	//10 is hard coded!
	for(int i = 0; i < regionCount; i++)
	  {
		raList[i].label = i;
		raList[i].next = NULL;
	  }
	for(int i = 0; i < regionCount*10-1; i++)
	  {
		raPool[i].next = &raPool[i+1];
	  }
	raPool[10*regionCount-1].next = NULL;
	RAList *raNode1;
	RAList *raNode2;
	RAList *oldRAFreeList;
	RAList *freeRAList = raPool;
	for(int i=0; i < height; i++) 
	  for(int j=0; j < width; j++)
		{
		  if(i>0 && labels[i][j]!=labels[i-1][j])
			{
			  // Get 2 free node
			  raNode1			= freeRAList;
			  raNode2			= freeRAList->next;
			  oldRAFreeList	    = freeRAList;
			  freeRAList		= freeRAList->next->next;
			  // connect the two region
			  raNode1->label	= labels[i][j];
			  raNode2->label	= labels[i-1][j];
			  if(raList[labels[i][j]].Insert(raNode2))	//already exists!
				freeRAList = oldRAFreeList;
			  else
				raList[labels[i-1][j]].Insert(raNode1);
			}
		  if(j>0 && labels[i][j]!=labels[i][j-1])
			{
			  // Get 2 free node
			  raNode1			= freeRAList;
			  raNode2			= freeRAList->next;
			  oldRAFreeList	= freeRAList;
			  freeRAList		= freeRAList->next->next;
			  // connect the two region
			  raNode1->label	= labels[i][j];
			  raNode2->label	= labels[i][j-1];
			  if(raList[labels[i][j]].Insert(raNode2))
				freeRAList = oldRAFreeList;
			  else
				raList[labels[i][j-1]].Insert(raNode1);
			}
		}
	// Find small regions
	for(int i = 0; i < regionCount; i++)
	  if(modePointCounts[i] < minRegion)
		{
		  minRegionCount++;
		  RAList *neighbor = raList[i].next;
		  int candidate = neighbor->label;
		  float minDistance = color_distance(&mode[3*i], &mode[3*candidate]);
		  neighbor = neighbor->next;
		  while(neighbor)
			{
			  float minDistance2 = color_distance(&mode[3*i], &mode[3*neighbor->label]);
			  if(minDistance2<minDistance)
				{
				  minDistance = minDistance2;
				  candidate = neighbor->label;
				}
			  neighbor = neighbor->next;
			}
		  int iCanEl = i, neighCanEl	= candidate;
		  while(raList[iCanEl].label != iCanEl) iCanEl = raList[iCanEl].label;
		  while(raList[neighCanEl].label != neighCanEl) neighCanEl = raList[neighCanEl].label;
		  if(iCanEl < neighCanEl)
			raList[neighCanEl].label	= iCanEl;
		  else
			{
			  //raList[raList[iCanEl].label].label	= neighCanEl;
			  raList[iCanEl].label = neighCanEl;
			}
		}
	for(int i = 0; i < regionCount; i++)
	  {
		int iCanEl	= i;
		while(raList[iCanEl].label != iCanEl)
		  iCanEl = raList[iCanEl].label;
		raList[i].label	= iCanEl;
	  }
	memset(modePointCounts_buffer, 0, regionCount*sizeof(int));
	for(int i = 0; i < regionCount; i++)
	  {
		label_buffer[i]	= -1;
		mode_buffer[3*i+0]	= 0;
		mode_buffer[3*i+1]	= 0;
		mode_buffer[3*i+2]	= 0;
	  }
	for(int i=0;i<regionCount; i++)
	  {
		int iCanEl	= raList[i].label;
		modePointCounts_buffer[iCanEl] += modePointCounts[i];
		for(int k=0;k<3;k++)
		  mode_buffer[iCanEl*3+k] += mode[i*3+k]*modePointCounts[i];
	  }
	int	label = -1;
	for(int i = 0; i < regionCount; i++)
	  {
		int iCanEl	= raList[i].label;
		if(label_buffer[iCanEl] < 0)
		  {
			label_buffer[iCanEl]	= ++label;

			for(int k = 0; k < 3; k++)
			  mode[label*3+k]	= (mode_buffer[iCanEl*3+k])/(modePointCounts_buffer[iCanEl]);

			modePointCounts[label]	= modePointCounts_buffer[iCanEl];
		  }
	  }
	regionCount = label+1;
	for(int i = 0; i < height; i++)
	  for(int j = 0; j < width; j++)
		labels[i][j]	= label_buffer[raList[labels[i][j]].label];

	//Destroy RAM
	delete[] raList;
	delete[] raPool;
	std::cout << "Mean Shift(Prune):" << regionCount << std::endl;
  }while(minRegionCount > 0);

  delete [] mode_buffer;
  delete [] modePointCounts_buffer;
  delete [] label_buffer;
}
