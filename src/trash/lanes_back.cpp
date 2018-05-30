#include "../include/lane_detector/lanes.hpp"

// typedef std::chrono::high_resolution_clock::time_point TimeVar;	

void imageCb(const sensor_msgs::ImageConstPtr& msg);


int counter = 0;

// int main(int argc, char** argv)
// {
// 	cout<<"running"<<endl;
// 	ros::init(argc, argv, "image_converter");
// 	ros::NodeHandle nh_;

// 	// 	if(argc < 2) return 0;
//     VideoCapture cap(argv[1]); // open the default camera
//     if(!cap.isOpened())  // check if we succeeded
//         return -1;
//     // model = svm_load_model("data.txt.model");
// 	int fr = 0;
// 	while(1)
// 	{
// 		// if(fr == 1000) fr = 0;
// 		//Mat given=imread(argv[1],1);
// 		Mat frame;
// 		cap>>frame;
// 		if(!frame.data) cout<<"null";
// 		Lanes L(frame);     // L --> frame
// 		//L.Intensity_distribution();
// 		fr++;
// 		// if(fr%20 != 0) continue;
	

// 		// L.Intensity_adjust();
// 		// L.remove_grass();

// 		L.Mix_Channel();
// 		L.topview();
// 		// L.display();
// 		// L.superpixels();
// 		L.parabola();
// 		// L.topview(1);
// 			// L.Hough();
// 			// L.display();
// 		// L.Brightest_Pixel_col();
// 		// L.Brightest_Pixel_row();
// 		// L.control_points();
// 		// L.curve_fitting();
// 			// L.control_vanishing();
// 		waitKey(10);
// 	}
// 	// waitKey(0);
// 	return 0;

// }

// void imageCb(const sensor_msgs::ImageConstPtr& msg)
// {
//     Mat img;
// 	cv_bridge::CvImagePtr cv_ptr;
// 	clock_t begin, end;
// 	begin = clock();
//     try
//     {
// 		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
// 		img = cv_ptr->image;
// 	}
// 	catch (cv_bridge::Exception& e)
// 	{
// 		ROS_ERROR("cv_bridge exception: %s", e.what());
// 		return;
// 	}
// 	if(img.rows < 0) return;
// 	counter++;
// 	if(counter > 1000) counter = 0;
// 	// if(counter % 20 != 0) return;
// 	end = clock();;
// 	// cout<<"\nCV_BRIDGE = "<<(end - begin)/1000.0;
	
// 	begin = clock();
// 	Lanes L(img);
// 	end = clock();
// 	// cout<<"\nconstructor = "<<(end - begin)/1000.0;

// 	// cout<<img.rows<<" img rowa"<<endl;
// 	// L.Intensity_adjust();
// 	begin = clock();
// 	L.Mix_Channel();
// 	end = clock();
// 	// cout<<"\nMix channel = "<<(end - begin)/1000.0;

// 	L.remove_grass();
// 	L.topview();
	
// 	begin = clock();
// 	L.parabola();
// 	end = clock();
// 	// cout<<"\nParabola = "<<(end - begin)/1000.0;

// }


// VideoWriter video("outcpp.avi",CV_FOURCC('M','J','P','G'),10, Size(1920, 1200));

int main(int argc, char** argv)
{

	cout<<"running"<<endl;
	ros::init(argc, argv, "image_converter");
	ros::NodeHandle nh_;

	ros::Publisher waypoint_pub = nh_.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1000);

	image_transport::ImageTransport it_(nh_);
	image_transport::Subscriber image_sub_;
	
	image_sub_ = it_.subscribe("/camera/image_color", 10, &imageCb);

    model = svm_load_model("data.txt.model");

	int fr = 0;
	// while(1)
	{
		ros::spin();
	}

	return 0;
}

void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    Mat img;
	cv_bridge::CvImagePtr cv_ptr;
	clock_t begin, end;
	begin = clock();
    try
    {
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		img = cv_ptr->image;
		// resize(img,img,Size(img.rows/2,img.cols/2));
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	if(img.rows < 0) return;
	// video.write(img);
	counter++;
	if(counter > 1000) counter = 0;
	// if(counter % 20 != 0) return;
	end = clock();;
	cout<<"\nCV_BRIDGE = "<<(end - begin)/1000.0;
	
	begin = clock();
	Lanes L(img);
	end = clock();
	cout<<"\nconstructor = "<<(end - begin)/1000.0;

	cout<<img.rows<<" img rowa"<<endl;
	L.Intensity_adjust();
	begin = clock();
	L.Mix_Channel();
	end = clock();
	cout<<"\nMix channel = "<<(end - begin)/1000.0;

	L.remove_grass();
	L.topview();
	
	begin = clock();
	L.parabola();
	end = clock();
	cout<<"\nParabola = "<<(end - begin)/1000.0;
}


Lanes::Lanes(Mat img)
{
	resize(img, img, Size(960, 600));
	// imshow("img", img);
	// cout<<img.rows<<' '<<img.cols<<endl;
	this->img = img;
	top_view_rgb=img.clone();
	cvtColor(this->img, this->img_gray,CV_BGR2GRAY);

}

int Lanes::IsValid(Mat A,int x,int y)
{
	if(x<0||y<0||x>=A.cols||y>=A.rows)
		return 0;
	return 1;
}

void Lanes::Mix_Channel()
{
	// blur(img,img,Size(5,5));
	for(int i = 0; i < img.rows; i++)   // grass removal
		for(int j = 0; j < img.cols; j++)
		{
			int temp = 2 * img.at<Vec3b>(i,j)[0] - img.at<Vec3b>(i,j)[1]; 
			if(temp < 0) temp=0;
			if(temp > 255) temp=255;
			img_gray.at<uchar>(i,j) = (unsigned int)temp;
		}
	top_view=img_gray.clone();
	// imshow("Mix_Chann",img_gray);
	// waitKey(0);
}

void Lanes::display()
{
	imshow("input", img);
	imshow("Mix_Channel", img_gray);
	waitKey(1);
}

void Lanes::Intensity_distribution()
{
	Mat dist(256,img.rows,CV_8UC1,Scalar(0));
	for(int i = 0; i < img.rows; i++)
	{
		int sum = 0;
		for(int j = 0; j < img.cols; j++)
		{
			sum += img.at<Vec3b>(i,j)[0] + img.at<Vec3b>(i,j)[1] + img.at<Vec3b>(i,j)[2];
		}
		sum /= 3;
		sum /= img.cols;
		dist.at<uchar>(256-sum,i) = 255;
	}
	// imshow("Intensity_distribution",dist);
	//waitKey(0);
}

void Lanes::Intensity_adjust()  // the top part of frame may have some intensity variation
{
	Mat filter(img.rows,img.cols,CV_8UC1,Scalar(0));
	for(int i = 0; i <= img.rows/4; i++)
		for(int j = 0; j < img.cols; j++)
			filter.at<uchar>(i,j) = (unsigned int )(img.rows/4 - i) * (SUBSTRACTION_CONSTANT) / (img.rows/4);
	// imshow("filter",filter);
	// waitKey(0);
	for(int i = 0; i < img.rows; i++)
		for(int j = 0; j < img.cols; j++)
			for(int k = 0; k < 3; k++)
			{
				int temp = img.at<Vec3b>(i,j)[k] - filter.at<uchar>(i,j);
				if(temp < 0) temp = 0;
				img.at<Vec3b>(i,j)[k] = (unsigned int )temp;
			}
	// imshow("Filtered Image", img);
	//waitKey(0);
	// imshow("Filtered Image", img);
	// waitKey(0);
}

void Lanes::remove_grass()
{
	// imshow("cv2", img);
	// waitKey(0);
	equalizeHist(img_gray, img_gray);
	Mat non_grass = img.clone();
	for(int i = 0; i < img.rows; i++)
		for(int j = 0; j < img.cols; j++)
		{
			if(img_gray.at<uchar>(i,j) < TH_REM_GRASS)
			{

				non_grass.at<Vec3b>(i,j)[0] = 0;
				non_grass.at<Vec3b>(i,j)[1] = 0;
				non_grass.at<Vec3b>(i,j)[2] = 0;
				img_gray.at<uchar>(i,j) = 0;
			}
			else
			{
				for(int k = 0; k < 3; k++)
					if(non_grass.at<Vec3b>(i,j)[k] < TH_REM_GRASS)
					{
						non_grass.at<Vec3b>(i,j)[0] = 0;
						non_grass.at<Vec3b>(i,j)[1] = 0;
						non_grass.at<Vec3b>(i,j)[2] = 0;
						img_gray.at<uchar>(i,j) = 0;
						break;
					}
			}
		}
	img = non_grass;
	// imshow("remove_grass", non_grass);
	// waitKey(0);
}

void Lanes::topview()
{
	
	Mat h = (Mat_<float>(3, 3)<< 3.665649454142774, 5.249642779023947, -2017.634107745852, 0.1725239771632309, 10.74704553239514, -3191.00361122947, 7.864729235797308e-05, 0.006494804637725546, 1);
	float scale_factor = 3.4;
	cout<<top_view.rows<<' '<<top_view.cols<<endl;
	resize(top_view, top_view, Size(960*2, 1200));
	resize(top_view_rgb, top_view_rgb, Size(960*2, 1200));

	warpPerspective(top_view, top_view, h, Size(480	*scale_factor,300*scale_factor));
	warpPerspective(top_view_rgb, top_view_rgb, h, Size(480*scale_factor,300*scale_factor));
	resize(top_view, top_view, Size(960, 600));
	resize(top_view_rgb, top_view_rgb, Size(960, 600));

	// imshow("top", top_view);
	// waitKey(10);

}

void Lanes::Edge()
{
	Mat Gx, Gy, Ga(img.rows,img.cols,CV_8UC1,Scalar(0)), Gb(img.rows,img.cols,CV_8UC1,Scalar(0)), Gc(img.rows,img.cols,CV_8UC1,Scalar(0)), Gd(img.rows,img.cols,CV_8UC1,Scalar(0));
	// equalizeHist(img_gray, img_gray);     // adjusting the contrast
	Mat temp = img_gray.clone();
	for(int i = 0; i < img.rows; i++)  // pixel
	{
		for(int j = 0; j < img.cols; j++)
		{
			int count=0;
			for(int m = i-1; m < i+2; m++)  // a kernel
			{
				for(int n = j-1; n < j+2; n++)
				{
					if(m<0 || n<0 || m>=img.rows || n>=img.cols) continue;   // isvalid condition
					if(img_gray.at<uchar>(m,n) < INTENSITY_TH) count++;   // erosion condition
				}
			}
			if(count != 0) temp.at<uchar>(i,j)=0;  
		}
	}
	img_gray = temp;
	vector<vector<Point> > contours;
	vector<Vec4i> heirarchy;
	vector<Rect> roi;
	vector<Mat> roi_img;
	Mat t=img_gray.clone();
	
	Mat t1 = img_gray.clone();
	
	cvtColor(t1,t,CV_GRAY2BGR);
	threshold(t1,t1,0,255,THRESH_BINARY+THRESH_OTSU);
	Mat element = getStructuringElement( MORPH_RECT,Size( 2*1 + 1, 2*1+1 ),Point( 1, 1 ) ); 
    int i=5;
    while(i--)
       erode(t1,t1,element);
   	i = 1;
   	// while(i--)
    // dilate(t1,t1,element);
	imshow("thesh",img_gray);
	//Canny(img_gray,img_gray,50,100,3,0);
	findContours(t1,contours,heirarchy,CV_RETR_TREE,CV_CHAIN_APPROX_NONE);
	float resize_factor=1.5;
	for(int i=0;i<contours.size();i++)
	{
		double area=contourArea(contours[i]);
		if(area<100 || area > (0.1*t1.rows*t1.cols)) continue;
		drawContours(t,contours,i,Scalar(255,0,0),2,8,heirarchy,0,Point());
		Rect rect=boundingRect(Mat(contours[i]));
		int width=rect.br().x-rect.tl().x;
		int height=rect.br().y-rect.tl().y;
		Rect roi_latest;
		Point tl=rect.tl(),br=rect.br();
		if((width*1.0)/height>1.5f) {
			tl=rect.tl()-Point(width*0.3*resize_factor,height*1*resize_factor);
			br=rect.br()+Point(width*0.5*resize_factor,height*0.5*resize_factor);
			if(!(tl.x>=0))
				tl.x=0;
			if(!(tl.y>=0))
				tl.y=0;
			if(!(br.x<img.cols))
				br.x=img.cols-1;
			if(!(br.y<img.rows))
				br.y=img.rows-1;
			}
		else {
			// tl=rect.tl()-Point(width*0.3*resize_factor,height*1*resize_factor);
			// br=rect.br()+Point(width*0.5*resize_factor,height*0.5*resize_factor);
			tl=rect.tl();
			br=rect.br();
		}
		rectangle(img,tl,br,Scalar(0,255,0),2,8,0);
		roi_latest=Rect(tl.x,tl.y,br.x-tl.x,br.y-tl.y);
		
		roi.push_back(roi_latest);
		roi_img.push_back(img(roi_latest));

		// imwrite("data/"+to_string(j)+".png",img(roi_latest));
		j++;
	}

	//roi_img now contains all the images in it which are to be predicted for lebelling
	for(int i = 0;i<roi_img.size();i++) {
		Rect rect = roi[i];
		//cout << "Check" << endl;
		Point tl = rect.tl(),br = rect.br();
		rectangle(img,tl,br,Scalar(0,0,0),2,8,0);
	//SVM CODE
	vector< vector < float> > v_descriptorsValues;
 	vector< vector < Point> > v_locations;

 	resize(roi_img[i], roi_img[i], Size(64,128) );

 	HOGDescriptor d( Size(64,128), Size(16,16), Size(8,8), Size(8,8), 9);
  	vector< float> descriptorsValues;
  	vector< Point> locations;
  	d.compute( roi_img[i], descriptorsValues, Size(0,0), Size(0,0), locations);

  	double predict_label;

    //x = (struct svm_node *) malloc(sizeof(struct svm_node));
   	prob.l = 1;
    prob.y = Malloc(double,prob.l); //space for prob.l doubles
	prob.x = Malloc(struct svm_node *, prob.l); //space for prob.l pointers to struct svm_node
	x_space = Malloc(struct svm_node, (3780+1) * prob.l); //memory for pairs of index/value

	int j=0; //counter to traverse x_space[i];
	for (int i=0;i < prob.l; ++i)
	{
		//set i-th element of prob.x to the address of x_space[j]. 
		//elements from x_space[j] to x_space[j+data[i].size] get filled right after next line
		prob.x[i] = &x_space[j];
		for (int k=0; k<3780; ++k, ++j)
		{
			x_space[j].index=k+1; //index of value
			x_space[j].value=descriptorsValues[k]; //value
			// cout<<"x_space["<<j<<"].index = "<<x_space[j].index<<endl;
			// cout<<"x_space["<<j<<"].value = "<<x_space[j].value<<endl;
		}
		x_space[j].index=-1;//state the end of data vector
		x_space[j].value=0;
		// cout<<"x_space["<<j<<"].index = "<<x_space[j].index<<endl;
		// cout<<"x_space["<<j<<"].value = "<<x_space[j].value<<endl;
		j++;

	}


	//set all default parameters for param struct
	param.svm_type = C_SVC;
	param.kernel_type = RBF;
	param.degree = 3;
	param.gamma = 0;	// 1/num_features
	param.coef0 = 0;
	param.nu = 0.5;
	param.cache_size = 100;
	param.C = 1;
	param.eps = 1e-3;
	param.p = 0.1;
	param.shrinking = 1;
	param.probability = 0;
	param.nr_weight = 0;
	param.weight_label = NULL;
	param.weight = NULL;

	double out = svm_predict(model,x_space);

	if(out == 1) {
	rectangle(img_gray,Point(rect.x,rect.y),Point(rect.x + rect.width,rect.y + rect.height),Scalar(0),-1,8,0);
	//cvtColor(img,img_gray,CV_BGR2GRAY);
	}
	}
	rectangle(img_gray,Point(0,img.rows*0.75),Point(img.cols-1,img.rows-1),Scalar(0),-1,8,0);
	line(img,Point(0,img.rows/3),Point(img.cols-1,img.rows/3), Scalar(255,0,0), 2, CV_AA);
	top_view=img_gray.clone();
	
	imshow("erode",img_gray);
	imshow("t",t);
	
}


void Lanes::parabola()
{
	imshow("1", top_view);
	imshow("top_view_rgb", top_view_rgb);
	// waitKey(5);
	Mat temp(top_view.rows, top_view.cols, CV_8UC3, Scalar(0,0,0));
	for(int i = 0; i < top_view.rows; i++)
		for(int j = 0; j < top_view.cols; j++)
			if(top_view.at<uchar>(i,j) > TH_DOT)
				for(int k = 0; k < 3; k++)
					temp.at<Vec3b>(i,j)[k] = 255;
	imshow("temp", temp);
	waitKey(5);
	// IplImage im = temp;
	// IplImage* image = &im;
	// image = cvCreateImage(cvSize(top_view.cols, top_view.rows), 8, 3);

	/*IplImage *lab_image = cvCloneImage(image);
    cvCvtColor(image, lab_image, CV_BGR2Lab);*/
    int w = temp.rows, h = temp.cols;
    int nr_superpixels = 500;
    int nc = 100;

    // double step = sqrt((w * h) / (double) nr_superpixels);
   

    /* Perform the SLIC superpixel algorithm. */
    // Slic slic;
    // slic.generate_superpixels(lab_image, step, nc);
    // slic.create_connectivity(lab_image);
    
    // end = clock();
    // cout<<"\nSlic = "<<(end - start)/1000.0;
    // /* Display the contours and show the result. */
    // slic.display_contours(image, CV_RGB(255,0,0));
    // cvShowImage("slic", image);
    // imshow("top", top_view);
    // waitKey(2);

    /* gpu accelerated super pixel clustering */
	// clock_t start, end;
 //    start = clock();
	
	resize(temp,temp,Size(temp.cols/2,temp.rows/2));

	cvtColor(temp, temp, CV_BGR2BGRA);
	// imshow("image original in top view", temp);
	// cudaFree();
	// waitKey(1);
	FastImgSeg segmenter;
	segmenter.initializeFastSeg(temp.cols, temp.rows, nr_superpixels);
	segmenter.LoadImg(temp.data);
	segmenter.DoSegmentation(RGB_SLIC, nc);
	segmenter.Tool_GetMarkedImg();
	Mat marked(temp.rows, temp.cols, CV_8UC4, segmenter.markedImg);
	Mat mask(temp.rows, temp.cols, CV_32SC1, segmenter.segMask);
	imshow("img super pixel cluster marked", marked);
	waitKey(5);
	// end = clock();
	cout<<"segmented\n";
	// segmenter.~FastImgSeg();

	Mat slic_img = temp.clone();//cvarrToMat(image);
    // imshow("image", img);
    // waitKey(10);

	int max = 0;
	for(int i = 0; i < mask.rows; i++)
		for(int j = 0; j < mask.cols; j++)
			if((int)mask.at<int>(i, j) > max) max = (int)mask.at<int>(i, j);
	// cout<<max;
	int* count_pix = new int[max + 1];
	int* count_col = new int[max + 1];
	for(int i = 0; i <= max; i++)
		count_pix[i] = count_col[i] = 0;
	for(int i = 0; i < top_view.rows; i++)
	{
		for(int j = 0; j < top_view.cols; j++)
		{
			if(i/2 >= mask.rows || j/2 >= mask.cols) continue;
			int idx = mask.at<int>(i/2,j/2);
			count_pix[idx]++;
			if(top_view.at<uchar>(i, j) > TH_DOT) count_col[idx]++;
		}
	}
	cout<<"counted\n";
	// waitKey(10);
	// for(int i = 0; i <= max; i++)
	// {
	// 	cout<<count_col[i]/(count_pix[i]*1.0)<<' ';
	// }
	vector<Point> P;
	// cout<<top_view.rows<<' '<<top_view.cols<<endl;
	Mat dot(top_view.rows, top_view.cols, CV_8UC1, Scalar(0));
	for(int i = 0; i < top_view.rows; i++)
	{
		for(int j = 0; j < top_view.cols; j++)
		{
			// if(!i%10 && !j%10)
			// cout<<i<<' '<<j<<endl;
			// if(i/2 >= mask.rows || j/2 >= mask.cols) continue;
			
			int idx = mask.at<int>(i/2,j/2);
			//cout<<"Idx : "<<idx<<endl;
			if(count_pix[idx] < TH_SMALL_SUPERPIXEL) continue;
			// cout<<(count_col[idx]/(count_pix[idx]*1.0))<<endl;
			if(count_col[idx]/(count_pix[idx]*1.0) < TH_MIN_WHITE_REGION) continue;

			// circle(dot, Point(j, i),3 ,Scalar(255),-1,1,0);
			dot.at<uchar>(i, j) = 255;
			int x = j;
			int y = top_view.rows - i;
			P.push_back(Point(x, y));
			count_pix[idx] = 0;
		}
		// cout<<i<<endl;
	}
	cout<<"P size = "<<P.size()<<endl;

	// for(int i = 0; i < P.size(); i++)
	// 	cout<<P[i].x<<' '<<P[i].y<<endl;
	imshow("dotted_condom", dot);
	waitKey(5);

	if(P.size() < 6)
	{
		cout<<"\nNot enough points";
		return;
	}
	// imshow("undotted", slic_img);
	
	int p1_g, p2_g, p3_g, p4_g;
	int score_gl = 0, comm_count_gl = 0;
	float a_gl, lam_gl, lam2_gl, w_gl;
	for(int i = 0; i < NUM_ITER; i++)
	{
		int p1 = random()%P.size(), p2 = random()%P.size(), p3 = random()%P.size(), p4 = random()%P.size();
		if(p2 == p1) p2 = random()%P.size();
		if(p3 == p1 || p3 == p2) p3 = random()%P.size();
		Point ran_points[4];
		ran_points[0] = P[p1];
		ran_points[1] = P[p2];
		ran_points[2] = P[p3];
		ran_points[3] = P[p4];
		int flag = 0;
		// for(int j = 0; j < 3; j++)
		// 	for(int k = j+1; k < 4; k++)
		// 		if(norm(ran_points[j] - ran_points[k]) < 20)
		// 		{
		// 			flag = 1;
		// 			break;
		// 		}
		// if(flag)
		// {
		// 	i--;
		// 	continue;
		// }
		Point temp;
		for(int m = 0; m < 3; m++)
		{
			for(int n = 0; n < 3 - m; n++)
			{
				if(ran_points[n].x > ran_points[n+1].x) 
				{
					temp = ran_points[n];
					ran_points[n] = ran_points[n+1];
					ran_points[n+1] = temp;
				}
			}	
		}
		// cout<<i<<endl;
		// if(ran_points[2].x - ran_points[1].x < 100)
		// {
		// 	i--;
		// 	continue;
		// }
		// if(abs(ran_points[1].y - ran_points[0].y) < 100)
		// {
		// 	i--;
		// 	continue;
		// }
		// if(abs(ran_points[2].y - ran_points[3].y) < 100)
		// {
		// 	i--;
		// 	continue;
		// }

		int score_loc = 0, comm_count = 0;
		float* param = parabola_params(ran_points);
		float a = param[0], lam = param[1], lam2 = param[2], w = param[3];
		// cout<<"sss "<<w<<endl;
		if(!IsAllowed(lam, a, lam2, w, top_view.rows, top_view.cols)) continue;
		for(int p = 0; p < P.size(); p++)
		{
			int flag = 0;
			for(int x = 0; x < top_view.cols; x++)
			{
				int y_l = sqrt(lam*(x - a)), y_r = sqrt(lam2*(x - a - w));
				// y_l = top_view.rows - y_l;
				// y_r = top_view.rows - y_r;
				if(y_l < 0 && y_r < 0) continue;
				float dist_l = sqrt(pow(x - P[p].x, 2) + pow(y_l - P[p].y, 2));
				if(dist_l < 50)
				{
					flag = 1;
				}
				float dist_r = sqrt(pow(x - P[p].x, 2) + pow(y_r - P[p].y, 2));
				if(dist_r < 50)
				{
					if(!flag) flag = 1;
					else
					{
						flag = 0;
						comm_count++;
					}
				}
			}
			if(flag) score_loc++;
			// cout<<score_loc<<endl;
		}
		if(score_loc>score_gl)
		{
			if(w < 100 && w > -100) continue;
			score_gl = score_loc;
			a_gl = a;
			lam_gl = lam;
			lam2_gl = lam2;
			w_gl = w;
			p1_g = p1;
			p2_g = p2;
			p3_g = p3;
			p4_g = p4;
			cout<<score_gl<<'\t';
			comm_count_gl = comm_count;
		}
	}
	cout<<"w = "<<w_gl<<" a = "<<a_gl<<" lam = "<<lam_gl<<"lam2 = "<<lam2_gl<<endl;
	for(int x = 0; x < top_view.cols; x++)
	{
		int y_l = sqrt(lam_gl*(x - a_gl));
		int y_r = sqrt(lam2_gl*(x - a_gl - w_gl));
		
		circle(dot, Point(x, top_view_rgb.rows - y_l),3,Scalar(255),-1,8,0);
		circle(dot, Point(x, top_view_rgb.rows - y_r),3,Scalar(255),-1,8,0);
	}
	circle(top_view_rgb, Point(P[p1_g].x, top_view.rows - P[p1_g].y),10,Scalar(0,0,255),-1,8,0);
	circle(top_view_rgb, Point(P[p2_g].x, top_view.rows - P[p2_g].y),10,Scalar(0,0,255),-1,8,0);
	circle(top_view_rgb, Point(P[p3_g].x, top_view.rows - P[p3_g].y),10,Scalar(0,0,255),-1,8,0);
	circle(top_view_rgb, Point(P[p4_g].x, top_view.rows - P[p4_g].y),10,Scalar(0,0,255),-1,8,0);

	imshow("top_view_rgb", top_view_rgb);
	// imshow("sampled_points", dot);
	waitKey(2);
	vector<double> way;
    way = generateWaypoint(top_view_rgb, a_gl, lam_gl, lam2_gl, w_gl);
	// way.push_back(0.1);
	// way.push_back(0.1);
	// way.push_back(0.1);
	circle(dot, Point(way[0], dot.rows - way[1]),10,Scalar(255),-1,8,0);
	imshow("dot", dot);
	waitKey(5);


    //set up transform
    tf::StampedTransform transform;
    tf::TransformListener listener;

    // try
    // {
    //   listener.lookupTransform("laser", "odom",  
    //                            ros::Time(0), transform);
    // }
    // catch (tf::TransformException ex){
    //   ROS_ERROR("%s",ex.what());
    //   ros::Duration(1.0).sleep();
    // }

	// geometry_msgs::PoseStamped waypoint;
	//geometry_msgs::PoseStamped waypoint_out;

	// waypoint.header.frame_id = "laser";
	// waypoint.pose.position.x = (way[1]*3.0/(top_view.rows)) + transform.getOrigin().x();
	// waypoint.pose.position.y = (-1 * way[0]*3.0/top_view.rows) + transform.getOrigin().y();
	// waypoint.pose.position.z = 0 + transform.getOrigin().z();
	// float theta = way[2] + tf::getYaw(transform.getRotation());


	geometry_msgs::PoseStamped waypoint;
	waypoint.pose.position.x = way[1]*3.0/top_view.rows;
	waypoint.pose.position.y = -1 * (way[0] - top_view.cols/2)*3.0/top_view.rows;
	waypoint.pose.position.z = 0;
	float theta = way[2] - PI/2;


	tf::Quaternion frame_qt = tf::createQuaternionFromYaw(theta);
	waypoint.pose.orientation.x = 0;//frame_qt.x();
	waypoint.pose.orientation.y = 0;//frame_qt.y();
	waypoint.pose.orientation.z = 0;//frame_qt.z();
	waypoint.pose.orientation.w = 1;//frame_qt.w();


	//make the laser->odom transform
	// listener.transformPoint("odom",waypoint,waypoint_out);



	// tf::Quaternion frame_qt = tf::createQuaternionFromYaw(theta);
	// waypoint.pose.orientation.x = frame_qt.x();
	// waypoint.pose.orientation.y = frame_qt.y();
	// waypoint.pose.orientation.z = frame_qt.z();
	// waypoint.pose.orientation.w = frame_qt.w();
	waypoint.header.frame_id = "base_link";
	ros::NodeHandle nh_;


	ros::Publisher waypoint_pub = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal/", 1000);

	waypoint_pub.publish(waypoint);
	cout<<"Published\n";
	// imshow("Res", dot);
	// imshow("Res2", top_view_rgb);
	// waitKey(5);
}

float* Lanes::parabola_params(Point *points)   //mode: 1-6 => llr,lrl,rll,lrr,rlr,rrl
{
	Point p1,p2,p3,p4,temp;
	float *param;
	param = new float [4];
	
	// float dist1,dist2;
	// dist1 = sqrt(pow(p1.x-p2.x , 2) + pow(p1.y-p2.y, 2));
	// dist2 = sqrt(pow(p4.x-p3.x , 2) + pow(p4.y-p3.y, 2));

	// if(dist1 < 0 || dist2 < 0){
	// 	param[0]=param[1]=param[2]=param[3]=0;
	// 	return param;
	// } 


	p1 = points[0];
	p2 = points[1];
	p3 = points[2];
	p4 = points[3];

	// cout<<p1.x<<"  "<<p1.y<<"  "<<p2.x<<"  "<<p2.y<<"  "<<p3.x<<"  "<<p3.y<<"  "<<p4.x<<"  "<<p4.y<<"  "<<endl;

	param[0] = (pow(p1.y,2) * p2.x - pow(p2.y,2)*p1.x)/((pow(p1.y,2)*1.0 - pow(p2.y,2)*1.0)) ; // a
	param[1] = pow(p1.y,2)/((p1.x*1.0 - param[0]*1.0));                                        // lambda1
	param[3] = (pow(p3.y,2) * p4.x - pow(p4.y,2)*p3.x - param[0] * (pow(p3.y,2) - pow(p4.y,2)))/(pow(p3.y,2) - pow(p4.y,2))*1.0; // w
	if(param[3] > top_view.cols || param[3] < -1*top_view.cols) param[3] = 256;
	param[2] = pow(p3.y,2)/(p3.x - param[0] - param[3])*1.0;                                        // lambda2
	return param;

}


// vector<double> generateWaypoint(Mat img,double a,double lm_1,double lm_2,double w)
// {

// 	// ros::NodeHandle nh_;

// 	// ros::Publisher waypoint_pub = nh_.advertise<geometry_msgs::PoseStamped>("lane_waypoint", 1000);

// 	//returns (x,y,theta)
// 	vector<double> waypoint;
// 	double x,y = img.rows/3,theta;  // put y --> height/3
// 	//case : two parabloas visible
// 	double x11,x12;
// 	x11 = (y*y/lm_1) + a;
// 	x12 = (y*y/lm_2) + a + w;
	
// 	if(x11 < 0)
// 		x11 = 0;
// 	if(x12 > img.cols-1)
// 		x12 = img.cols-1;

// 	x = (x11+x12)/2.0;
//         cout<<"!!!!! "<<lm_1<<" "<<lm_2<<endl;
// 	theta = fabs(atan((lm_1+lm_2)/(4.0*y)));
// 	cout<<"Theta = "<<theta<<endl;
// 	circle(img, Point(x,320),10,Scalar(0,255,0),-1,8,0);
// 	int x_d=(int)(x-100*cos(theta));
// 	int y_d=(int)(img.rows-1-(y+100*sin(theta)));
// 	circle(img, Point(x_d,y_d),10,Scalar(255,0,0),-1,8,0);
// 	//line(img,Point(x,img.rows - 360.0),Point(,, Scalar(255,0,0), 2, CV_AA);
// 	waypoint.push_back(x);
// 	waypoint.push_back(y);
// 	waypoint.push_back(theta);
// 	return waypoint;
// }


vector<double> generateWayPoint_2(Mat img,double a,double lm_1,double lm_2, double w)
{
	vector<double> waypoint_2;

	double x,y = (img.rows*1.5)/3;  // put y --> height/3
        double x11,x12;
        curr_xl_2 = y*y/lm_1 + a;
        curr_xr_2 = y*y/lm_2 + a + w;
        //one lane
        if (w > 600 && w < -600) {
		if (fabs(curr_xl_2 - prev_xl_2) < fabs(curr_xr_2 - prev_xr_2)) {
			x = waypoint_prev_2[0] + curr_xl_2 - prev_xl_2;
			//theta = fabs(atan(lm_1/(2*sqrt(lm_1*(x-a)))));
		} 
		else {
			x = waypoint_prev_2[0] + curr_xr_2 - prev_xr_2;
			//theta = fabs(atan(lm_2/(2*sqrt(lm_2*(x-a-w)))));
		}
                y = waypoint_prev_2[1];
                //theta = waypoint_prev_2[2];
                //circle(img, Point(x,320),10,Scalar(0,255,0),-1,8,0);
                //int x_d=(int)(x-100*cos(theta));
                //int y_d=(int)(img.rows-1-(y+100*sin(theta)));
                //circle(img, Point(x_d,y_d),10,Scalar(255,0,0),-1,8,0);
        }

        //two lanes
        else {
                x11 = (y*y/lm_1) + a;
                x12 = (y*y/lm_2) + a + w;
                
                if(x11 < 0)
                        x11 = 0;
                if(x12 > img.cols-1)
                        x12 = img.cols-1;

                x = (x11+x12)/2.0;
                prev_xl_2 = x11;
                prev_xr_2 = x12;
                //cout<<"!!!!! "<<lm_1<<" "<<lm_2<<endl;
                //theta = fabs(atan((lm_1+lm_2)/(4.0*y)));
                //cout<<"Theta = "<<theta<<endl;
                //circle(img, Point(x,320),10,Scalar(0,255,0),-1,8,0);
                //int x_d=(int)(x-100*cos(theta));
                //int y_d=(int)(img.rows-1-(y+100*sin(theta)));
                //circle(img, Point(x_d,y_d),10,Scalar(255,0,0),-1,8,0);
        }


        /*
        //no lane
        else {
                x = waypoint_prev[0];
                circle(img, Point(x,320),10,Scalar(0,255,0),-1,8,0);
                int x_d=(int)(x-100*cos(theta));
                int y_d=(int)(img.rows-1-(y+100*sin(theta)));
                circle(img, Point(x_d,y_d),10,Scalar(255,0,0),-1,8,0);
                return waypoint_prev;

        }
*/
	//line(img,Point(x,img.rows - 360.0),Point(,, Scalar(255,0,0), 2, CV_AA);
	waypoint_2.push_back(x);
	waypoint_2.push_back(y);
	//waypoint_2.push_back(theta);
    waypoint_prev_2 = waypoint_2;
	return waypoint_2;
}
vector<double> generateWaypoint(Mat img,double a,double lm_1,double lm_2,double w) {


	// waitKey(2000);

        //y2 = lam(x-a)

	//returns (x,y,theta)
	vector<double> waypoint;

	double x,y = img.rows/3,theta;  // put y --> height/3
        double x11,x12;
        curr_xl = y*y/lm_1 + a;
        curr_xr = y*y/lm_2 + a + w;
        Point p1;
        p1.y=320;
        //one lane
        if (w > 600 && w < -600) 
        {
			if (fabs(curr_xl - prev_xl) < fabs(curr_xr - prev_xr)) 
			{
				x = waypoint_prev[0] + curr_xl - prev_xl;
				p1.x=2;
				//theta = fabs(atan(lm_1/(2*sqrt(lm_1*(x-a)))));
			} 
			else 
			{
				x = waypoint_prev[0] + curr_xr - prev_xr;
				p1.x=x;
				theta = fabs(atan(lm_2/(2*sqrt(lm_2*(x-a-w)))));
			}
            y = waypoint_prev[1];
            //theta = waypoint_prev[2];
            circle(img, Point(x,320),10,Scalar(0,255,0),-1,8,0);
            //int x_d=(int)(x-100*cos(theta));
            //int y_d=(int)(img.rows-1-(y+100*sin(theta)));
            // circle(img, Point(x_d,y_d),10,Scalar(255,0,0),-1,8,0);
	    }

        //two lanes
        else {
                x11 = (y*y/lm_1) + a;
                x12 = (y*y/lm_2) + a + w;
                
                if(x11 < 0)
                        x11 = 0;
                if(x12 > img.cols-1)
                        x12 = img.cols-1;

                x = (x11+x12)/2.0;
                prev_xl = x11;
                prev_xr = x12;
                cout<<"!!!!! "<<lm_1<<" "<<lm_2<<endl;
                theta = fabs(atan((lm_1+lm_2)/(4.0*y)));
                cout<<"Theta = "<<theta<<endl;
                circle(img, Point(x,320),10,Scalar(0,255,0),-1,8,0);
                p1.x=x;
                //int x_d=(int)(x-100*cos(theta));
                //int y_d=(int)(img.rows-1-(y+100*sin(theta)));
                // circle(img, Point(x_d,y_d),10,Scalar(255,0,0),-1,8,0);
        }


        /*
        //no lane
        else {
                x = waypoint_prev[0];
                circle(img, Point(x,320),10,Scalar(0,255,0),-1,8,0);
                int x_d=(int)(x-100*cos(theta));
                int y_d=(int)(img.rows-1-(y+100*sin(theta)));
                circle(img, Point(x_d,y_d),10,Scalar(255,0,0),-1,8,0);
                return waypoint_prev;

        }
*/
	//line(img,Point(x,img.rows - 360.0),Point(,, Scalar(0,0,255), 2, CV_AA);
	waypoint.push_back(x);
	waypoint.push_back(y);
	vector<double> waypoint_2=generateWayPoint_2(img,a,lm_1,lm_2,w);
    circle(img, Point(waypoint_2[0],240),10,Scalar(255,0,0),-1,8,0);
    line(img, Point(waypoint_2[0],240),p1,Scalar(0,0,255),2,CV_AA);

   cout<<"!!!!@@@@"<<x<<" "<<y<<" "<<waypoint_2[0]<<" "<<waypoint_2[1]<<endl;
	waypoint.push_back(atan((y-waypoint_2[1])/(x-waypoint_2[0])));
	//waypoint.push_back(theta);
    waypoint_prev = waypoint;
	return waypoint;
}

bool IsAllowed(float lam1,float a,float lam2,float w,int rows,int cols)
{
	if(fabs(w)>200000)return 1;	//Single lane
	else if(abs(w)<100)return 0;	//w is very small
	else
	{
		if(fabs(lam1)>500||fabs(lam2)>500)	//Non horizontal lines
		{
			for(int i=0;i<rows;i++)
			{
				int x1=((i*i)/lam1)+a;
				int x2=((i*i)/lam2)+a+w;
				if(abs(x1-x2)<100)return 0;
			}
		}
		else
		{
			for(int i=0;i<cols;i++)
			{
				int y1=sqrt(lam1*(i-a));
				int y2=sqrt(lam2*(i-a-w));
				if(abs(y1-y2)<100)return 0;
			}
		}
	}
	return 1;
}
