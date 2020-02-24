/*
This file subscribe to image and LiDAR topic and project LiDAR frame onto image frame.

Yidong He
Feb 23, 2020

*/

#include <iostream>
#include <string>
#include <algorithm>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/opencv.hpp"


#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h> // This is for transforming sensor_msgs::PointCloud2 to pcl::PCLPointCloud2
#include <pcl/point_types.h>

#include <boost/bind.hpp>

static cv::Mat P(3, 3, cv::DataType<double>::type);
static cv::Mat RT(3, 4, cv::DataType<double>::type);


void callback(const sensor_msgs::CompressedImageConstPtr& img,
			  const sensor_msgs::PointCloud2ConstPtr& lidarPtr)
{
	// For image
	cv_bridge::CvImagePtr img_ptr;
	try
	{
		img_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& err)
	{
		ROS_ERROR("cv_bridge exception: %s", err.what());
		return;
	}

	cv::Mat visImg = img_ptr->image.clone();

	// For lidar
	pcl::PCLPointCloud2* pointcloud1 = new pcl::PCLPointCloud2;
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
	pcl_conversions::toPCL(*lidarPtr, *pointcloud1);
	pcl::fromPCLPointCloud2(*pointcloud1, *cloud);

	cv::Mat lidarHomo(4, 1, cv::DataType<double>::type);
	cv::Mat lidarImage(3, 1, cv::DataType<double>::type);

	//std::cout << "size: " << cloud->points.size() << std::endl;

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

		//std::cout << "pt.x: " << pt.x << std::endl;
		//std::cout << "pt.y: " << pt.y << std::endl;

		int red = std::min(255, (int)(50 * (1 + ((double)cloud->points[i].x+5.4)*10.0)));
		int green = std::min(255, (int)(50 * std::abs(((double)cloud->points[i].x+5.4)*10.0)));
		if(std::abs(red - green) > 100){
			continue;
		}

		cv::circle(visImg, pt, 2, cv::Scalar(0, green, red), -1);

	}

	cv::imshow("LiDAR to image frame validation", visImg);
	cv::waitKey(5);

}

int main(int argc, char** argv)
{
	// For P:
	P.at<double>(0, 0) = 1050.993774; P.at<double>(0, 1) = 0.0; P.at<double>(0, 2) = 750.963;
	P.at<double>(1, 0) = 0.0; P.at<double>(1, 1) = 1090.754761; P.at<double>(1, 2) = 594.037146;
	P.at<double>(2, 0) = 0.0; P.at<double>(2, 1) = 0.0; P.at<double>(2, 2) = 1.0;

	// For RT
	double initialRX = -1.7916222 + 0.028 + 0.14;
	double initialRY = -0.0310489 - 0.05;
	double initialRZ = -3.1384458 - 0.008;
	double initialTX = 0.34 - 0.12 - 0.3 - 0.06;
	double initialTY = 0.18 - 0.06 - 0.03;
	double initialTZ = -0.48 - 0.04 - 0.6 - 0.45;

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

	// ROS init
	ROS_INFO_STREAM("Initialize the node.");

	ros::init(argc, argv, "rosbagToKITTI");
	ros::NodeHandle nh("~");

	std::string imageTopic = "/camera_left/camera/image_raw/compressed";
	std::string lidarTopic = "/pandar_points";

	message_filters::Subscriber<sensor_msgs::CompressedImage> img_sub(nh, imageTopic, 3);
	message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_sub(nh, lidarTopic, 3);

	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage,
															sensor_msgs::PointCloud2> syncPolicy;

	message_filters::Synchronizer<syncPolicy> syncObj(syncPolicy(10), img_sub,lidar_sub);
	syncObj.registerCallback(boost::bind(&callback, _1, _2));

	cv::namedWindow("LiDAR to image frame validation", cv::WINDOW_NORMAL);

	while(ros::ok()){
		ROS_INFO_STREAM("ROS Working.");
		ros::spin();
	}

	return 0;
}
