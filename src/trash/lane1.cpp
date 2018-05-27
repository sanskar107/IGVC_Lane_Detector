int main(int argc, char** argv)
{
	cout<<"running"<<endl;
	ros::init(argc, argv, "image_converter");
	ros::NodeHandle nh_;

	ros::Publisher waypoint_pub = nh_.advertise<geometry_msgs::PoseStamped>("lane_waypoint", 1000);

	image_transport::ImageTransport it_(nh_);
	image_transport::Subscriber image_sub_;
	
	image_sub_ = it_.subscribe("/camera/image_color", 1, &imageCb);

    model = svm_load_model("data.txt.model");

	int fr = 0;
	while(1)
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
	counter++;
	if(counter > 1000) counter = 0;
	if(counter % 20 != 0) return;
	end = clock();;
	cout<<"\nCV_BRIDGE = "<<(end - begin)/1000.0;
	
	begin = clock();
	Lanes L(img);
	end = clock();
	cout<<"\nconstructor = "<<(end - begin)/1000.0;

	cout<<img.rows<<" img rowa"<<endl;
	// L.Intensity_adjust();
	begin = clock();
	L.Mix_Channel();
	end = clock();
	cout<<"\nMix channel = "<<(end - begin)/1000.0;

	// L.remove_grass();
	L.topview();
	
	begin = clock();
	L.parabola();
	end = clock();
	cout<<"\nParabola = "<<(end - begin)/1000.0;

}
