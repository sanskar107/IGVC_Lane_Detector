#include <iostream>
#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/LaserScan.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <eigen3/Eigen/Dense>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <math.h>

using namespace cv;
using namespace std;
using namespace Eigen;
#define PI 3.14159265359

Mat obstacles,H,H_inv;
vector<Point> obs_points;
int img_rows = 1200,img_cols = 1920;
int height_to_top = 10;
int height_to_bottom = 10;
int pixel_meter_ratio = 336;

Mat img(img_rows,img_cols,CV_8UC3,Scalar(0,0,0));

Point PM(Point t);

void chatterCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
  obstacles = Mat(img_rows,img_cols, CV_8UC1, Scalar(0));
  obs_points.clear();
  int count = 0;
  float i;
  int x, h = 0, k = 0, y;
  for (i = scan->angle_min, count = 0; i <= scan->angle_max; i += scan->angle_increment, count++) {

          if (scan->ranges[i] >= scan->range_max || scan->ranges[i] <= scan->range_min || i <= -PI/2 || i >= PI/2) continue;

          //shift origin from mid point of last row to normal conventions
    		  x = img_cols/2 - (int)scan->ranges[count]*pixel_meter_ratio*sin(i);
          y = img_rows - (int)scan->ranges[count]*cos(i)*pixel_meter_ratio;

          //this is to calibrate the shift between image centers in front and top view
          x += h;
          y += k;

          if (x >= 0 && y >= 0 && x < img_cols && y < img_rows) {
                  cout << x << "," << y << " :: " << count << endl;
                  obstacles.at<uchar>(y,x) = 255;
                  Point final = PM(Point(x,y));
                  // // obs_points.push_back(final);

                  // height_to_top *= ((1.0/1.0)*(scan->ranges[count]*cos(i)));
                  // height_to_bottom *= ((1.0/1.0)*(scan->ranges[count]*cos(i)));
                  // //draw bounding box
                  line(img,final,Point(final.x,final.y - height_to_top),Scalar(0,0,0), 2, CV_AA);
                  line(img,final,Point(final.x,final.y + height_to_bottom),Scalar(0,0,0), 2, CV_AA);
          }
  }
  
  imshow("IMG",img);
  imshow("I",obstacles);
  waitKey(10);
}

void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImagePtr cv_ptr;

    try
    {
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		img = cv_ptr->image;
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
}

void setH() { //returns the homograph matrix
  float scale_factor=3.36;
  //Mat img = imread("test1.png",1);
  vector<Point2f> pnt_src,pnt_dst;

  //src img coordinate are in pixels
  
  pnt_src.push_back(Point(504,345));
  pnt_src.push_back(Point(629,342));
  pnt_src.push_back(Point(935,338));
  pnt_src.push_back(Point(1059,336));
  pnt_src.push_back(Point(476,397));
  pnt_src.push_back(Point(613,393));
  pnt_src.push_back(Point(951,387));
  pnt_src.push_back(Point(1088,386));
  pnt_src.push_back(Point(373,586));
  pnt_src.push_back(Point(555,583));
  pnt_src.push_back(Point(1008,574));
  pnt_src.push_back(Point(1189,572));
  pnt_src.push_back(Point(308,708));
  pnt_src.push_back(Point(517,705));
  pnt_src.push_back(Point(1042,696));
  pnt_src.push_back(Point(1248,687));

  // dst img coordinates in cm
  pnt_dst.push_back(Point(147.2*scale_factor,53.3*scale_factor));
  pnt_dst.push_back(Point(188.2*scale_factor,53.3*scale_factor));
  pnt_dst.push_back(Point(286.7*scale_factor,53.3*scale_factor));
  pnt_dst.push_back(Point(  327*scale_factor,53.3*scale_factor));
  pnt_dst.push_back(Point(147.2*scale_factor,94.6*scale_factor));
  pnt_dst.push_back(Point(188.2*scale_factor,94.6*scale_factor));
  pnt_dst.push_back(Point(286.7*scale_factor,94.6*scale_factor));
  pnt_dst.push_back(Point(  327*scale_factor,94.6*scale_factor));
  pnt_dst.push_back(Point(147.2*scale_factor,193*scale_factor));
  pnt_dst.push_back(Point(188.2*scale_factor,193*scale_factor));
  pnt_dst.push_back(Point(286.7*scale_factor,193*scale_factor));
  pnt_dst.push_back(Point(  327*scale_factor,193*scale_factor));
  pnt_dst.push_back(Point(147.2*scale_factor,234*scale_factor));
  pnt_dst.push_back(Point(188.2*scale_factor,234*scale_factor));
  pnt_dst.push_back(Point(286.7*scale_factor,234*scale_factor));
  pnt_dst.push_back(Point(  327*scale_factor,234*scale_factor));

  H = findHomography(pnt_dst,pnt_src);
  // cout << H.rows << "," << H.cols << endl;
}

Point PM(Point p) { //prespective maps the point in top view
	MatrixXd P(3,1);
	MatrixXd R(3,1);
	P(0,0) = p.x;
	P(1,0) = p.y;
	P(2,0) = 1;

	MatrixXd X(3,3);
	for(int i = 0;i < 3;i++) {
		for(int j = 0;j < 3;j++) {
			X(i,j) = H.at<double>(i,j);
		}
	}

	R = (X.inverse())*P;
	// cout << X << endl;
	// cout << X.inverse() << endl;

	Point final(100,100);
	double x = R(0,0)/R(2,0);
	double y = R(1,0)/R(2,0);
	// Mat t;
	// warpPerspective(img,t,H_inv,Size(1920,1200));
	// imshow("T",t);
	// waitKey(10);
	cout << x << ":: p :: " << y << endl;
	if(x >= 0 & x < img_cols & y >= 0 & y <img_rows) {
		final.x = x;
		final.y = y;
	}

	return final;
}

int main(int argc, char **argv) 
{
  setH();

  ros::init(argc,argv,"laser_to_mat");
  ros::NodeHandle n;

  image_transport::ImageTransport it_(n);
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  image_sub_ = it_.subscribe("/camera/image_color", 10, &imageCb);
  ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, chatterCallback);

  ros::spin();

  return 0;
}
