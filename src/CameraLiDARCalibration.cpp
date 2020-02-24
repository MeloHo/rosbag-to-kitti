/*
This file validates the LiDAR to camera extrinsic parameter by projecting the LiDAR points onto camera frame.
It also provides a trackbar using OpenCV to fine tune the parameters.

Yidong He
Feb 22, 2020

*/

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>

#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>

#include <string>
#include <iostream>
#include <cmath>
#include <algorithm>

static cv::Mat imgSrc;
static cv::Mat P(3, 3, cv::DataType<double>::type); // Intrinsic
static pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

static int rotationX = 50;
static int rotationY = 50;
static int rotationZ = 50;
static int translationX = 50;
static int translationY = 50;
static int translationZ = 50;

void computeRTFromEuler(cv::Mat& RT,
						double initialRX,
						double initialRY,
						double initialRZ,
						double initialTX,
						double initialTY,
						double initialTZ)
{
	std::cout << "Initial : " << initialRX << " " << initialRY << " " << initialRZ << std::endl;
	RT.at<double>(0, 3) = initialTX;
	RT.at<double>(1, 3) = initialTY;
	RT.at<double>(2, 3) = initialTZ;

	RT.at<double>(0, 0) = cos(initialRY) * cos(initialRZ);
	RT.at<double>(0, 1) = -cos(initialRX)*sin(initialRZ) + sin(initialRX)*sin(initialRY)*cos(initialRZ);
	RT.at<double>(0, 2) = sin(initialRX)*sin(initialRZ) + cos(initialRX)*sin(initialRY)*cos(initialRZ);

	RT.at<double>(1, 0) = cos(initialRY) * sin(initialRZ);
	RT.at<double>(1, 1) = cos(initialRX)*cos(initialRZ) + sin(initialRX)*sin(initialRY)*sin(initialRZ);
	RT.at<double>(1, 2) = -sin(initialRX)*cos(initialRZ) + cos(initialRX)*sin(initialRY)*sin(initialRZ);

	RT.at<double>(2, 0) = -sin(initialRY);
	RT.at<double>(2, 1) = sin(initialRX) * cos(initialRY);
	RT.at<double>(2, 2) = cos(initialRX) * cos(initialRY);
	
}


static void trakerCallback(int , void* ){
	cv::Mat visualImage = imgSrc.clone();

	double initialRX = -1.7916222 + 0.028;
	double initialRY = -0.0310489 - 0.05;
	double initialRZ = -3.1384458 - 0.008;
	double initialTX = 0.34 - 0.12;
	double initialTY = 0.18 - 0.06;
	double initialTZ = -0.48 - 0.04;
	initialRX += 0.1 * ((double)rotationX - 50) / 100.0; // 
	initialRY += 0.1 * ((double)rotationY - 50) / 100.0;
	initialRZ += 0.1 * ((double)rotationZ - 50) / 100.0;
	initialTX += 2 * ((double)translationX - 50) / 100.0; // 
	initialTY += 2 * ((double)translationY - 50) / 100.0;
	initialTZ += 2 * ((double)translationZ - 50) / 100.0;

	// Compute the Rotation Matrix from Euler angle.
	cv::Mat RT(3, 4, cv::DataType<double>::type);
	computeRTFromEuler(RT, initialRX, initialRY, initialRZ, initialTX, initialTY, initialTZ);

	// Project LiDAR point:
	cv::Mat lidarHomo(4, 1, cv::DataType<double>::type);
	cv::Mat lidarImage(3, 1, cv::DataType<double>::type);

	std::cout << "P: " << std::endl;
	std::cout << P << std::endl;

	std::cout << "RT: " << std::endl;
	std::cout << RT << std::endl;

	for(int i = 0; i < cloud->points.size(); i++){
		lidarHomo.at<double>(0, 0) = cloud->points[i].x;
		lidarHomo.at<double>(1, 0) = cloud->points[i].y;
		lidarHomo.at<double>(2, 0) = cloud->points[i].z;
		lidarHomo.at<double>(3, 0) = 1;

		lidarImage = P * RT * lidarHomo;

		cv::Point pt;
		pt.x = lidarImage.at<double>(0, 0) / lidarImage.at<double>(2, 0);
		pt.y = lidarImage.at<double>(1, 0) / lidarImage.at<double>(2, 0);

		if(pt.x < 0 || pt.y < 0 || pt.x > 1400 || pt.y > 1000 || lidarImage.at<double>(2, 0) < 0){
			continue;
		}
		
		float color = cloud->points[i].x;
		int red = std::min(255, (int)(255 * abs(color - 20.0) / 20.0));
		int green = std::min(255, (int)(255 * (abs((color - 20.0) / 20.0 ))));
		if(green > 100){
			continue;
		}

		cv::circle(visualImage, pt, 3, cv::Scalar(0, green, red), -1);
	}

	cv::imshow("LiDAR to image frame validation", visualImage);
	
}

int main(int argc, char const *argv[])
{
	// Read Image frame
	imgSrc = cv::imread("/home/yidong/catkin_ws/src/rosbag_to_kitti/src/1.jpg");
	if(imgSrc.empty()){
		std::cout << "Error reading the image. \n" << std::endl;
		return (-1);
	}

	// Read LiDAR frame from .pcd file
	if(pcl::io::loadPCDFile<pcl::PointXYZ>("/home/yidong/catkin_ws/src/rosbag_to_kitti/src/1.pcd", *cloud) == -1){
		PCL_ERROR("Couldn't read pcd file \n");
		return (-1);
	}

	// Initialize camera intrinsic P
	P.at<double>(0, 0) = 1050.993774; P.at<double>(0, 1) = 0.0; P.at<double>(0, 2) = 750.963;
	P.at<double>(1, 0) = 0.0; P.at<double>(1, 1) = 1090.754761; P.at<double>(1, 2) = 594.037146;
	P.at<double>(2, 0) = 0.0; P.at<double>(2, 1) = 0.0; P.at<double>(2, 2) = 1.0;
	
	// Create slider bar
	cv::namedWindow("LiDAR to image frame validation", cv::WINDOW_NORMAL);
	cv::createTrackbar("rotationX", "LiDAR to image frame validation", &rotationX, 100, trakerCallback);
	cv::createTrackbar("rotationY", "LiDAR to image frame validation", &rotationY, 100, trakerCallback);
	cv::createTrackbar("rotationZ", "LiDAR to image frame validation", &rotationZ, 100, trakerCallback);
	cv::createTrackbar("translationX", "LiDAR to image frame validation", &translationX, 100, trakerCallback);
	cv::createTrackbar("translationY", "LiDAR to image frame validation", &translationY, 100, trakerCallback);
	cv::createTrackbar("translationZ", "LiDAR to image frame validation", &translationZ, 100, trakerCallback);
	
	trakerCallback(rotationX, 0);

	cv::waitKey(0);

	return 0;
}
