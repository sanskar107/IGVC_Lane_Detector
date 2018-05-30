#include <iostream>
#include <bits/stdc++.h>
#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <highgui.h>
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace cv;
using namespace Eigen;

int main ()
{
	float scale_factor=3.4;
	Mat img = imread("test1.png",1);
	vector<Point2f> pnt_src,pnt_dst;

		// src img coordinate are in pixels
	
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

	Mat h= findHomography(pnt_src,pnt_dst);
	// MatrixXd homography(3,3);
	// homography << 
	cout<<h<<endl;

	Mat img_out;
			// size in cms.
	warpPerspective(img,img_out,h,Size(480*scale_factor,300*scale_factor));
	namedWindow("Topview",WINDOW_NORMAL); 
	imshow("Topview",img_out);
	waitKey(0);
}