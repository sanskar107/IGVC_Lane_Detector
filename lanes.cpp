#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>

#define SUBSTRACTION_CONSTANT 30
#define INTENSITY_TH 60
using namespace std;
using namespace cv;

class Lanes
{
	Mat img,img_gray;
public:
	Lanes(string name);
	void Mix_Channel();
	void Intensity_distribution();
	void display();
	void Intensity_adjust();
	void Brightest_Pixel();
};

int main(int argc, char** argv)
{
	if(argc < 2) return 0;
	Lanes L(argv[1]);
	// L.Mix_Channel();
	// L.display();
	// L.Intensity_distribution();
	L.Intensity_adjust();
	// L.Intensity_distribution();
	L.Mix_Channel();
	// L.display();
	L.Brightest_Pixel();
	return 0;

}

Lanes::Lanes(string name)
{
	this->img = imread(name, CV_LOAD_IMAGE_COLOR);
	this->img_gray = imread(name, CV_LOAD_IMAGE_GRAYSCALE);
}

void Lanes::Mix_Channel()
{
	for(int i = 0; i < img.rows; i++)
		for(int j = 0; j < img.cols; j++)
		{
			int temp = 2 * img.at<Vec3b>(i,j)[0] - img.at<Vec3b>(i,j)[1];
			if(temp < 0) temp=0;
			if(temp > 255) temp=255;
			img_gray.at<uchar>(i,j) = temp;
		}
	imshow("Mix_Chann",img_gray);
	waitKey(0);
}

void Lanes::display()
{
	imshow("input", img);
	imshow("Mix_Channel", img_gray);
	waitKey(0);
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
	imshow("Intensity_distribution",dist);
	waitKey(0);
}

void Lanes::Intensity_adjust()
{
	Mat filter(img.rows,img.cols,CV_8UC1,Scalar(0));
	for(int i = 0; i <= img.rows/4; i++)
		for(int j = 0; j < img.cols; j++)
			filter.at<uchar>(i,j) = (img.rows/4 - i) * (SUBSTRACTION_CONSTANT) / (img.rows/4);
	// imshow("filter",filter);
	// waitKey(0);
	for(int i = 0; i < img.rows; i++)
		for(int j = 0; j < img.cols; j++)
			for(int k = 0; k < 3; k++)
			{
				int temp = img.at<Vec3b>(i,j)[k] - filter.at<uchar>(i,j);
				if(temp < 0) temp = 0;
				img.at<Vec3b>(i,j)[k] = temp;
			}
	imshow("Filtered Image", img);
	waitKey(0);
}

void Lanes::Brightest_Pixel()
{
	Mat bisect(img.rows,img.cols,CV_8UC1,Scalar(0));
	for(int i = 0; i < img.rows; i++)
	{
		int l,r,max = 0;
		for(int j = 0; j < img.cols/2; j++)
		{
			if(img_gray.at<uchar>(i,j) < INTENSITY_TH) img_gray.at<uchar>(i,j) = 0;
			else img_gray.at<uchar>(i,j) = 255;
			if(img_gray.at<uchar>(i,j) > max)
			{
				max = img.at<uchar>(i,j);
				l = j;
			}
			
		}
		if(max>100) bisect.at<uchar>(i,l) = 255;
		max = 0;
		for(int j = img.cols/2 + 1; j < img.cols; j++)
		{
			if(img_gray.at<uchar>(i,j) < INTENSITY_TH) img_gray.at<uchar>(i,j) = 0;
			else img_gray.at<uchar>(i,j) = 255;
			if(img_gray.at<uchar>(i,j) > max)
			{
				max = img.at<uchar>(i,j);
				r = j;
			}
			
		}
		if(max > 100) bisect.at<uchar>(i,r) = 255;
	}
	imshow("bisect",bisect);
	imshow("Threshold",img_gray);
	waitKey(0);
}