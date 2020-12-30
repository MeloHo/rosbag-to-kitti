/*
This file project LiDAR points on the image frame in a continuous LiDAR/Image stream.
Corrsponding Image and LiDAR frame:
	Image: X.png
	LiDAR: X.bin	


Yidong He
Feb 23, 2020

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
#include <fstream>

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

	double initialRX = -1.7916222 + 0.028 + 0.14;
	double initialRY = -0.0310489 - 0.05;
	double initialRZ = -3.1384458 - 0.008;
	double initialTX = 0.34 - 0.12 - 0.3 - 0.06;
	double initialTY = 0.18 - 0.06 - 0.03;
	double initialTZ = -0.48 - 0.04 - 0.6 - 0.45;
	initialRX += 2.0 * ((double)rotationX - 50) / 100.0; // 
	initialRY += 2.0 * ((double)rotationY - 50) / 100.0;
	initialRZ += 2.0 * ((double)rotationZ - 50) / 100.0;
	initialTX += 3 * ((double)translationX - 50) / 100.0; // 
	initialTY += 3 * ((double)translationY - 50) / 100.0;
	initialTZ += 3 * ((double)translationZ - 50) / 100.0;

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

		if(cloud->points[i].y > 0){
			continue;
		}

		lidarImage = P * RT * lidarHomo;

		cv::Point pt;
		pt.x = lidarImage.at<double>(0, 0) / lidarImage.at<double>(2, 0);
		pt.y = lidarImage.at<double>(1, 0) / lidarImage.at<double>(2, 0);

		//if(pt.x < 0 || pt.y < 0 || pt.x > 1400 || pt.y > 1000){
		//	continue;
		//}
		//if(pt.x <300 || pt.x > 1000){
		//	continue;
		//}
		//std::cout << "x: " << cloud->points[i].x << std::endl;
		//std::cout << "y: " << cloud->points[i].y << std::endl;
		//std::cout << "z: " << cloud->points[i].z << std::endl;
		int red = std::min(255, (int)(100 * (1 + ((double)cloud->points[i].x+5.4)*10.0)));
		//std::cout << "red: " << red << std::endl;
		int green = std::min(255, (int)(100 * std::abs(((double)cloud->points[i].x+5.4)*10.0)));
		//std::cout << "green: " << green << std::endl;
		if(std::abs(red - green) > 50){
			continue;
		}

		cv::circle(visualImage, pt, 3, cv::Scalar(0, green, red), -1);
	}

	cv::imshow("LiDAR to image frame validation", visualImage);
	
}

int main(int argc, char const *argv[])
{
	// Read Image frame
	imgSrc = cv::imread("/home/yidong/Downloads/2019-12-22-11-34-33/leftCamera/data/163.png");
	if(imgSrc.empty()){
		std::cout << "Error reading the image. \n" << std::endl;
		return (-1);
	}

	// Convert .bin file to PointXYZ
	std::string inputBin = "/home/yidong/Downloads/2019-12-22-11-34-33/lidar_points/data/163.bin";
	std::fstream input(inputBin.c_str(), std::ios::in | std::ios::binary);
	if(!input.good()){
		std::cout << "Could not read .bin file" << std::endl;
		return(-1);
	}
	input.seekg(0, std::ios::beg);

	for(int i = 0; input.good() && !input.eof(); i++){
		pcl::PointXYZI point;
		input.read((char*) &point.x, 3*sizeof(float));
		input.read((char*) &point.intensity, sizeof(float));
		
		cloud->push_back(pcl::PointXYZ(point.x, point.y, point.z));
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

