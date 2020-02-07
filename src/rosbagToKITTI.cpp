/*
Use it in command line like:
rosrun rosbag_to_kitti rosbagtokitti

Transform messages in rosbag to KITTI format.
Not thread-safe.
Synchronize with ApproximateTime policy.

Yidong He
Jan 30, 2020
*/

#include <iostream>
#include <string>
#include <fstream>
#define _USE_MATH_DEFINES
#include <cmath>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/opencv.hpp"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

//----- IMU Messages ------//
#include <sensor_msgs/NavSatFix.h> // For topic /piksi/navsatfix_best_fix
#include <sensor_msgs/Imu.h> // For topic /piksi/vel_ned
#include <piksi_rtk_msgs/VelNed.h> // For topic /piksi/vel_ned
//#include <piksi_rtk_msgs/ReceiverState_V2_4_1.h> // For topic /piksi/debug/receiver_state. This may not be useful.
//----- IMU Messages ------//


#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h> // This is for transforming sensor_msgs::PointCloud2 to pcl::PCLPointCloud2
#include <pcl/point_types.h> // pcl::PCLPointCloud2
//#include <pcl/io/pcd_io.h> // pcl::io::save

/*
#include <boost/foreach.hpp> // It just provides a convenient iterator.
							 // Use it like:
							 // BOOST_FOREACH(char& ch, string("Hello")){}
*/

#include <boost/bind.hpp>
#include <boost/filesystem.hpp>
#include "boost/date_time/posix_time/posix_time.hpp" // This is for converting time

namespace FS = boost::filesystem;

static std::string leftCameraDataPath;
static std::string rightCameraDataPath;
static std::string leftCameraInfoPath;
static std::string rightCameraInfoPath;
static std::string lidarDataPath;
static std::string lidarTimePath;
static std::string gpsDataPath;
static std::string gpsTimePath;

static std::ofstream fileWriter1; // For writing left camera timestamps
static std::ofstream fileWriter2; // For writing right camera timestamps
static std::ofstream fileWriter3; // For writing lidar timestamps
static std::ofstream fileWriter4; // For writing gps timestamps

static int l_img_cnt;
static int r_img_cnt;
static int lidar_cnt;
static int gps_cnt;

struct EulerAngles{
	double roll, pitch, yaw;
};

EulerAngles quaternionToEuler(double w, double x, double y, double z) // Code from wiki
{
	EulerAngles euler;

	// roll
	double sinr_cosp = 2 * (w * x + y * z);
	double cosr_cosp = 1 - 2 * (x * x + y * y);
	euler.roll = std::atan2(sinr_cosp, cosr_cosp);

	// pitch
	double sinp = 2 * (w * y - z * x);
	if(std::abs(sinp) >= 1)
		euler.pitch = std::copysign(M_PI / 2, sinp);
	else
		euler.pitch = std::asin(sinp);

	// yaw
	double siny_cosp = 2 * (w * z + x * y) ;
	double cosy_cosp = 1 - 2 * (y * y + z * z);
	euler.yaw = std::atan2(siny_cosp, cosy_cosp);

	return euler;
}

void writeGPSData(std::ofstream& gps_frame_writer,
				  const sensor_msgs::NavSatFixConstPtr& navsatfixPtr,
				  const sensor_msgs::ImuConstPtr& imuPtr,
				  const piksi_rtk_msgs::VelNedConstPtr& velPtr)
{
	EulerAngles euler = quaternionToEuler(imuPtr->orientation.w, imuPtr->orientation.x,
										  imuPtr->orientation.y, imuPtr->orientation.z);
	gps_frame_writer << navsatfixPtr->longitude << " "
					 << navsatfixPtr->latitude  << " "
					 << navsatfixPtr->altitude  << " "
					 << euler.roll  << " "
					 << euler.pitch << " "
					 << euler.yaw   << " "
					 << velPtr->n << " "
					 << velPtr->e << " "
					 << velPtr->d << " "
					 << imuPtr->linear_acceleration.x << " "
					 << imuPtr->linear_acceleration.y << " "
					 << imuPtr->linear_acceleration.z << " "
					 << imuPtr->angular_velocity.x << " "
					 << imuPtr->angular_velocity.y << " "
					 << imuPtr->angular_velocity.z << " "
					 << std::endl;
}

void callback(const sensor_msgs::CompressedImageConstPtr& l_img,
			  const sensor_msgs::CompressedImageConstPtr& r_img,
			  const sensor_msgs::PointCloud2ConstPtr& lidarPtr,
			  const sensor_msgs::NavSatFixConstPtr& navsatfixPtr,
			  const sensor_msgs::ImuConstPtr& imuPtr,
			  const piksi_rtk_msgs::VelNedConstPtr& velPtr)
{
	std::cout << "Callback Funciton is called" << std::endl;
	// For Camera:

	// Convert ROS image to cv image format
	cv_bridge::CvImagePtr l_img_ptr, r_img_ptr;
	try
	{
		l_img_ptr = cv_bridge::toCvCopy(l_img, sensor_msgs::image_encodings::BGR8);
		r_img_ptr = cv_bridge::toCvCopy(r_img, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& err)
	{
		ROS_ERROR("cv_bridge exception: %s", err.what());
		return;
	}
	// Save the image to files
	FS::path l_img_name = FS::path(leftCameraDataPath) / FS::path(std::to_string(l_img_cnt) + ".png");
	cv::imwrite(l_img_name.string(), l_img_ptr->image);
	FS::path r_img_name = FS::path(rightCameraDataPath) / FS::path(std::to_string(r_img_cnt) + ".png");
	cv::imwrite(r_img_name.string(), r_img_ptr->image);

	l_img_cnt++;
	r_img_cnt++;
	// Write the timestamps into timestamps.txt
	if(fileWriter1.is_open())
	{
		boost::posix_time::ptime l_img_time = (l_img->header.stamp).toBoost();
		std::string l_img_time_wall = boost::posix_time::to_simple_string(l_img_time);
		fileWriter1 << l_img_time_wall << std::endl;
	}
	else{
		std::cout << "Error when writing left camera timestamps" << std::endl;
	}
	if(fileWriter2.is_open())
	{
		boost::posix_time::ptime r_img_time = (r_img->header.stamp).toBoost();
		std::string r_img_time_wall = boost::posix_time::to_simple_string(r_img_time);
		fileWriter2 << r_img_time_wall << std::endl;
	}
	else{
		std::cout << "Error when writing right camera timestamps" << std::endl;
	}

	// For LiDAR: 1. Convert sensor_msgs::PointCloud2 into pcl::PCLPointCloud2. 
	//			  2. Convert pcl::PCLPointCloud2 to pcl::PointCloud<pcl::PointXYZI>
	//			  3. Write pcl::PClCloud<pcl::PointXYZI> into .bin file
	pcl::PCLPointCloud2* pointcloud1 = new pcl::PCLPointCloud2;
	pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud2 (new pcl::PointCloud<pcl::PointXYZI>);
	pcl_conversions::toPCL(*lidarPtr, *pointcloud1);
	pcl::fromPCLPointCloud2(*pointcloud1, *pointcloud2);
	FS::path lidar_frame_name = FS::path(lidarDataPath) / FS::path(std::to_string(lidar_cnt) + ".bin");
	
	std::ofstream bin_file_writer(lidar_frame_name.string().c_str(), std::ios::out | std::ios::binary | std::ios::app);
	if(bin_file_writer.good()){
		for(size_t i = 0; i < pointcloud2->points.size(); i++){
			bin_file_writer.write((char*)&pointcloud2->points[i].x, 3*sizeof(float));
			bin_file_writer.write((char*)&pointcloud2->points[i].intensity, sizeof(float));
		}
	}
	else{
		std::cout << "Cannot write lidar data into .bin file" << std::endl;
	}

	bin_file_writer.close();
	lidar_cnt++;

	// Write the lidar timestamps
	if(fileWriter3.is_open())
	{
		boost::posix_time::ptime lidar_time = (lidarPtr->header.stamp).toBoost();
		std::string lidar_time_wall = boost::posix_time::to_simple_string(lidar_time);
		fileWriter3 << lidar_time_wall << std::endl;
	}
	else{
		std::cout << "Error when writing left camera timestamps" << std::endl;
	}

	// For GPS:
	// Write GPS data
	FS::path gps_frame_name = FS::path(gpsDataPath) / FS::path(std::to_string(gps_cnt) + ".txt");
	std::ofstream gps_frame_writer(gps_frame_name.string().c_str(), std::ios::out | std::ios::app);
	if(gps_frame_writer.good()){
		writeGPSData(gps_frame_writer, navsatfixPtr, imuPtr, velPtr);
	}
	else{
		std::cout << "Cannot write gps data to .txt file" << std::endl;
	}
	
	gps_frame_writer.close();
	gps_cnt++;

	// Write GPS timestamp
	if(fileWriter4.is_open())
	{
		boost::posix_time::ptime imu_time = (imuPtr->header.stamp).toBoost(); // Use imu timestamp as gps time.
		std::string imu_time_wall = boost::posix_time::to_simple_string(imu_time);
		fileWriter4 << imu_time_wall << std::endl;
	}
	else{
		std::cout << "Error when writing imu timestamps" << std::endl;
	}
}

int main(int argc, char** argv)
{
	// Initialize the rosnode
	ROS_INFO_STREAM("Initialize the node.");
	ros::init(argc, argv, "rosbagToKITTI");
	ros::NodeHandle nh("~");

	std::string ROSBAGNAME;
	nh.getParam("bagName", ROSBAGNAME);
	std::cout << "ROSBAGNAME: " << ROSBAGNAME << std::endl; 

	l_img_cnt = 0;
	r_img_cnt = 0;
	lidar_cnt = 0;
	gps_cnt = 0;

	// Specify rostopic
	std::string leftCameraImage = "/camera_left/camera/image_raw/compressed";
	std::string rightCameraImage = "/camera_right/camera/image_raw/compressed";
	std::string pointCloudTopic = "/pandar_points";
	std::string navsatfixTopic = "/piksi/navsatfix_best_fix";
	std::string imuTopic = "/piksi/imu";
	std::string velTopic = "/piksi/vel_ned";

	// Specify path
	FS::path leftCameraDataPath_ = FS::path(ROSBAGNAME) / FS::path("leftCamera") / FS::path("data");
	FS::path leftCameraInfoPath_ = FS::path(ROSBAGNAME) / FS::path("leftCamera") / FS::path("timestamps.txt");
	FS::path rightCameraDataPath_ = FS::path(ROSBAGNAME) / FS::path("rightCamera") / FS::path("data");
	FS::path rightCameraInfoPath_ = FS::path(ROSBAGNAME) / FS::path("rightCamera") / FS::path("timestamps.txt");
	FS::path lidarDataPath_ = FS::path(ROSBAGNAME) / FS::path("lidar_points") / FS::path("data");
	FS::path lidarTimePath_ = FS::path(ROSBAGNAME) / FS::path("lidar_points") / FS::path("timestamps.txt");
	FS::path gpsDataPath_ = FS::path(ROSBAGNAME) / FS::path("gps") / FS::path("data");
	FS::path gpsTimePath_ = FS::path(ROSBAGNAME) / FS::path("gps") / FS::path("timestamps.txt");

	std::cout << " leftCameraDataPath_ is: " << leftCameraDataPath_ << std::endl;


	leftCameraDataPath = leftCameraDataPath_.string();
	rightCameraDataPath = rightCameraDataPath_.string();
	leftCameraInfoPath = leftCameraInfoPath_.string();
	rightCameraInfoPath = rightCameraInfoPath_.string();
	lidarDataPath = lidarDataPath_.string();
	lidarTimePath = lidarTimePath_.string();
	gpsDataPath = gpsDataPath_.string();
	gpsTimePath = gpsTimePath_.string();

	// Create files and folders

	// Create folders
	if(FS::create_directories(leftCameraDataPath_)){
		std::cout << "Create " << leftCameraDataPath_ << std::endl;  
	}
	if(FS::create_directories(rightCameraDataPath_)){
		std::cout << "Create " << rightCameraDataPath_ << std::endl;  
	}
	if(FS::create_directories(lidarDataPath_)){
		std::cout << "Create " << lidarDataPath_ << std::endl;
	}
	if(FS::create_directories(gpsDataPath_)){
		std::cout << "Create " << gpsDataPath_ << std::endl;
	}

	// Create files and open files
	fileWriter1.open(leftCameraInfoPath);
	fileWriter2.open(rightCameraInfoPath);
	fileWriter3.open(lidarTimePath);
	fileWriter4.open(gpsTimePath);

	if(fileWriter1.is_open()){
		std::cout << "Create " << leftCameraInfoPath_ << std::endl;
	}
	else{
		std::cout << "Error when creating Camera Timestamps file for left camera." << std::endl;
	}

	if(fileWriter2.is_open()){
		std::cout << "Create " << rightCameraInfoPath_ << std::endl;
	}
	else{
		std::cout << "Error when creating Camera Timestamps file for right camera." << std::endl;
	}

	if(fileWriter3.is_open()){
		std::cout << "Create " << lidarTimePath_ << std::endl;
	}
	else{
		std::cout << "Error when creating LiDAR Timestamps file." << std::endl;
	}

	if(fileWriter4.is_open()){
		std::cout << "Create " << gpsTimePath_ << std::endl;
	}
	else{
		std::cout << "Error when creating gps Timestamps file." << std::endl;
	}
	

	// Initialize the subscribers.


	message_filters::Subscriber<sensor_msgs::CompressedImage> l_img_sub(nh, leftCameraImage, 3);
	message_filters::Subscriber<sensor_msgs::CompressedImage> r_img_sub(nh, rightCameraImage, 3);
	message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_sub(nh, pointCloudTopic, 3);
	message_filters::Subscriber<sensor_msgs::NavSatFix> navsatfix_sub(nh, navsatfixTopic, 3);
	message_filters::Subscriber<sensor_msgs::Imu> imu_sub(nh, imuTopic, 3);
	message_filters::Subscriber<piksi_rtk_msgs::VelNed> vel_sub(nh, velTopic, 3);

	// Initialize synchronizer
	//typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, sensor_msgs::CompressedImage, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> syncPolicy;
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, 
															sensor_msgs::CompressedImage,
															sensor_msgs::PointCloud2,
															sensor_msgs::NavSatFix,
															sensor_msgs::Imu,
															piksi_rtk_msgs::VelNed> syncPolicy;
	message_filters::Synchronizer<syncPolicy> syncObj(syncPolicy(10), l_img_sub, r_img_sub, lidar_sub, navsatfix_sub, imu_sub, vel_sub);

	syncObj.registerCallback(boost::bind(&callback, _1, _2, _3, _4, _5, _6));

	while(ros::ok()){
		ROS_INFO_STREAM("ROS Working.");
		ros::spin();
	}

	fileWriter1.close();
	fileWriter2.close();
	fileWriter3.close();
	fileWriter4.close();

	return 0;

}


