#ifndef _lanes_HPP
#define _lanes_HPP_

#include <iostream>
#include <string.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>
#include <time.h>
#include <stdlib.h>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include "slic.cpp"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "geometry_msgs/PoseStamped.h"
#include <sensor_msgs/image_encodings.h>
#include "sensor_msgs/Image.h"
#include <sensor_msgs/LaserScan.h>
#include <opencv2/gpu/gpu.hpp>
#include <tf/transform_datatypes.h>
#include "svm.h"
#include "../src/library/gSLIC/FastImgSeg.h"
#include <tf/transform_listener.h>


#define W 400
#define SUBSTRACTION_CONSTANT 30
#define INTENSITY_TH 180
#define PI 3.14159265

#define TH_REM_GRASS 180
#define TH_DOT 190
#define TH_SMALL_SUPERPIXEL 20
#define TH_MIN_WHITE_REGION 0.7
#define NUM_ITER 1000
#define PPM 133.33

#define THRESHOLD_FOR_ANY_LANE 210
#define LANE_THRESHOLD 210

struct svm_model* model;
struct svm_node *x_space;
struct svm_problem prob;
struct svm_parameter param;

int wide = 700;

#define Malloc(type,n) (type *)malloc((n)*sizeof(type))

using namespace cv;
using namespace Eigen;
using namespace std;

// vector<double> generateWaypoint(Mat img,double a,double lm_1,double lm_2,double w);
// vector<double> generateWayPoint_2(Mat img,double a,double lm_1,double lm_2, double w);
bool IsAllowed(float lam1,float a,float lam2,float w,int rows,int cols);
void imageCb(const sensor_msgs::ImageConstPtr& msg);
double heading (Mat img, double a, double lam);
Point centroid(Mat img, double a, double lam);


sensor_msgs::LaserScan imageConvert(cv::Mat image);

vector<double> waypoint_prev;
double prev_xl;
vector<double> way_avg[3];
// double prev_600/4.2;
double curr_xl;
double curr_xr;
vector<double> waypoint_prev_2;
double prev_xl_2;
double prev_xr_2;
double curr_xl_2;
double curr_xr_2;
bool is_prev_single;
bool is_prev_single_left;
bool first_frame=true;
int left_prev_x;
int right_prev_x;



struct quadratic
{
	double a,b,c;
	int inliers;
};

int j=0,frame_skip=-1;

class Lanes
{
	Mat img,img_gray,bisect,top_view,points,curves;
	vector<Vec2i> L,R;
	vector<Point> left_lane,right_lane;
	int frameWidth;
	int frameHeight;
	quadratic left_fy,right_gy;
	quadratic array_l[5],array_r[5];
	double width_lanes;
	int flag_width;

public:
	Mat top_view_rgb;
	Lanes(Mat);
	void Mix_Channel();
	void Intensity_distribution();
	void display();
	void Intensity_adjust();
	void Brightest_Pixel_col();
	void Brightest_Pixel_row();
	void Edge();
	void Hough();
	void Dilation();
	void topview();
	void control_points();
	void control_vanishing();
	void curve_fitting();
	void plot_quad(quadratic,int);
	quadratic RANSAC(vector<Point>);
	void shift_parabola(quadratic ,int );
	int IsValid(Mat , int, int );
	void remove_grass();
	float* parabola_params(Point*);
	void parabola();
	void superpixels();	
	vector<double> generateWaypoint(Mat img,double a,double lm_1,double lm_2,double w);
	vector<double> generateWayPoint_2(Mat img,double a,double lm_1,double lm_2, double w);

};


#endif 
