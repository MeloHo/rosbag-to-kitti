/*

Use rosbag API to convert the raw rosbag data into KITTI format.
Use it like:
	rosrun rosbag_to_kitti rosbagAPI _bagName:=2019-12-11-11-34-04

Yidong He
Mar 14, 2020
*/


#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/foreach.hpp>
#include <boost/bind.hpp>
#include <boost/filesystem.hpp>
#include "boost/date_time/posix_time/posix_time.hpp" // This is for converting time

#include <iostream>
#include <string>
#include <fstream>
#define _USE_MATH_DEFINES
#include <cmath>
#include <vector>
#include <stack>

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

/*
Convert frame ,e.g. "4", into "0004".
*/
std::string frameCountToString(const int& cnt)
{
	if(cnt / 1000 > 0) {
		return std::to_string(cnt);
	}
	else if(cnt / 100 > 0) {
		return "0" + std::to_string(cnt);
	}
	else if (cnt / 10 > 0) {
		return "00" + std::to_string(cnt);
	}
	else {
		return "000" + std::to_string(cnt);
	}

	return "";
}


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

}

int main(int argc, char** argv)
{
	// Initialize stacks
	std::stack<sensor_msgs::CompressedImageConstPtr> l_img_st;
	std::stack<sensor_msgs::CompressedImageConstPtr> r_img_st;
	std::stack<sensor_msgs::PointCloud2ConstPtr> lidar_st;
	std::stack<sensor_msgs::NavSatFixConstPtr> nav_st;
	std::stack<sensor_msgs::ImuConstPtr> imu_st;
	std::stack<piksi_rtk_msgs::VelNedConstPtr> vel_st;

	// Initialize the node
	ROS_INFO_STREAM("Initialize the node.");
	ros::init(argc, argv, "rosbagAPI");
	ros::NodeHandle nh("~");

	int l_img_cnt = 0;
	int r_img_cnt = 0;
	int lidar_cnt = 0;
	int gps_cnt = 0;

	std::string ROSBAGNAME;
	nh.getParam("bagName", ROSBAGNAME);
	std::cout << "ROSBAGNAME: " << ROSBAGNAME << std::endl; 

	// Open the rosbag
	rosbag::Bag bag;
	std::string ROSBAGPATH = ROSBAGNAME + ".bag";
	bag.open(ROSBAGPATH, rosbag::bagmode::Read);

	// All the topics
	std::string leftCameraImage = "/camera_left/camera/image_raw/compressed";
	std::string rightCameraImage = "/camera_right/camera/image_raw/compressed";
	std::string pointCloudTopic = "/pandar_points"; // Sometime it's /pandar_points, sometime its /pandar_points2
	std::string navsatfixTopic = "/piksi/navsatfix_best_fix";
	std::string imuTopic = "/piksi/imu";
	std::string velTopic = "/piksi/vel_ned";

	std::vector<std::string> topics;
	topics.push_back(leftCameraImage);
	topics.push_back(rightCameraImage);
	topics.push_back(pointCloudTopic);
	topics.push_back(navsatfixTopic);
	topics.push_back(imuTopic);
	topics.push_back(velTopic);

	rosbag::View view(bag, rosbag::TopicQuery(topics));

	// Specify path
	FS::path leftCameraDataPath_ = FS::path(ROSBAGNAME) / FS::path("leftCamera") / FS::path("data");
	FS::path leftCameraInfoPath_ = FS::path(ROSBAGNAME) / FS::path("leftCamera") / FS::path("timestamps.txt");
	FS::path rightCameraDataPath_ = FS::path(ROSBAGNAME) / FS::path("rightCamera") / FS::path("data");
	FS::path rightCameraInfoPath_ = FS::path(ROSBAGNAME) / FS::path("rightCamera") / FS::path("timestamps.txt");
	FS::path lidarDataPath_ = FS::path(ROSBAGNAME) / FS::path("lidar_points") / FS::path("data");
	FS::path lidarTimePath_ = FS::path(ROSBAGNAME) / FS::path("lidar_points") / FS::path("timestamps.txt");
	FS::path gpsDataPath_ = FS::path(ROSBAGNAME) / FS::path("gps") / FS::path("data");
	FS::path gpsTimePath_ = FS::path(ROSBAGNAME) / FS::path("gps") / FS::path("timestamps.txt");

	leftCameraDataPath = leftCameraDataPath_.string();
	rightCameraDataPath = rightCameraDataPath_.string();
	leftCameraInfoPath = leftCameraInfoPath_.string();
	rightCameraInfoPath = rightCameraInfoPath_.string();
	lidarDataPath = lidarDataPath_.string();
	lidarTimePath = lidarTimePath_.string();
	gpsDataPath = gpsDataPath_.string();
	gpsTimePath = gpsTimePath_.string();

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
		std::cout << "Opening " << leftCameraInfoPath_ << std::endl;
	}
	else{
		std::cout << "Error when creating Camera Timestamps file for left camera." << std::endl;
	}

	if(fileWriter2.is_open()){
		std::cout << "Opening " << rightCameraInfoPath_ << std::endl;
	}
	else{
		std::cout << "Error when creating Camera Timestamps file for right camera." << std::endl;
	}

	if(fileWriter3.is_open()){
		std::cout << "Opening " << lidarTimePath_ << std::endl;
	}
	else{
		std::cout << "Error when creating LiDAR Timestamps file." << std::endl;
	}

	if(fileWriter4.is_open()){
		std::cout << "Opening " << gpsTimePath_ << std::endl;
	}
	else{
		std::cout << "Error when creating gps Timestamps file." << std::endl;
	}

	// Set up fake subscribers

	message_filters::Subscriber<sensor_msgs::CompressedImage> l_img_sub(nh, leftCameraImage, 3);
	message_filters::Subscriber<sensor_msgs::CompressedImage> r_img_sub(nh, rightCameraImage, 3);
	message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_sub(nh, pointCloudTopic, 3);
	message_filters::Subscriber<sensor_msgs::NavSatFix> navsatfix_sub(nh, navsatfixTopic, 3);
	message_filters::Subscriber<sensor_msgs::Imu> imu_sub(nh, imuTopic, 3);
	message_filters::Subscriber<piksi_rtk_msgs::VelNed> vel_sub(nh, velTopic, 3);

	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, 
															sensor_msgs::CompressedImage,
															sensor_msgs::PointCloud2,
															sensor_msgs::NavSatFix,
															sensor_msgs::Imu,
															piksi_rtk_msgs::VelNed> syncPolicy;
	message_filters::Synchronizer<syncPolicy> syncObj(syncPolicy(10), l_img_sub, r_img_sub, lidar_sub, navsatfix_sub, imu_sub, vel_sub);

	syncObj.registerCallback(boost::bind(&callback, _1, _2, _3, _4, _5, _6));

	// Set up time and time threshold
	ros::Duration timeInterval(0.1);
	ros::Time l_img_time_threshold, r_img_time_threshold, lidar_time_threshold;

	BOOST_FOREACH(rosbag::MessageInstance const m, view)
	{
		if ((m.getTopic() == leftCameraImage) || ("/" + m.getTopic() == leftCameraImage))
		{
			sensor_msgs::CompressedImageConstPtr l_img = m.instantiate<sensor_msgs::CompressedImage>();
			l_img_st.push(l_img);
		}
		else if ((m.getTopic() == rightCameraImage) || ("/" + m.getTopic() == rightCameraImage))
		{
			sensor_msgs::CompressedImageConstPtr r_img = m.instantiate<sensor_msgs::CompressedImage>();
			r_img_st.push(r_img);
		}
		else if ((m.getTopic() == pointCloudTopic) || ("/" + m.getTopic() == pointCloudTopic))
		{
			sensor_msgs::PointCloud2ConstPtr lidarPtr = m.instantiate<sensor_msgs::PointCloud2>();
			lidar_st.push(lidarPtr);
		}
		else if ((m.getTopic() == navsatfixTopic) || ("/" + m.getTopic() == navsatfixTopic))
		{
			sensor_msgs::NavSatFixConstPtr navsatfixPtr = m.instantiate<sensor_msgs::NavSatFix>();
			nav_st.push(navsatfixPtr);
		}
		else if ((m.getTopic() == imuTopic) || ("/" + m.getTopic() == imuTopic))
		{
			sensor_msgs::ImuConstPtr imuPtr = m.instantiate<sensor_msgs::Imu>();
			imu_st.push(imuPtr);
		}
		else if ((m.getTopic() == velTopic) || ("/" + m.getTopic() == velTopic))
		{
			piksi_rtk_msgs::VelNedConstPtr velPtr = m.instantiate<piksi_rtk_msgs::VelNed>();
			vel_st.push(velPtr);
		}

		if(!l_img_st.empty() && !r_img_st.empty() && !lidar_st.empty() && !nav_st.empty() && !imu_st.empty() && !vel_st.empty()) {
			// For left Camera
			sensor_msgs::CompressedImageConstPtr l_img = l_img_st.top();
			std::stack<sensor_msgs::CompressedImageConstPtr>().swap(l_img_st);
			cv_bridge::CvImagePtr l_img_ptr;
			try
			{
				l_img_ptr = cv_bridge::toCvCopy(l_img, sensor_msgs::image_encodings::BGR8);
			}
			catch (cv_bridge::Exception& err)
			{
				ROS_ERROR("cv_bridge exception: %s", err.what());
				return 0;
			}
			FS::path l_img_name = FS::path(leftCameraDataPath) / FS::path(frameCountToString(l_img_cnt) + ".png");
			cv::imwrite(l_img_name.string(), l_img_ptr->image);
			l_img_cnt++;

			if(fileWriter1.is_open())
			{
				boost::posix_time::ptime l_img_time = (l_img->header.stamp).toBoost();
				std::string l_img_time_wall = boost::posix_time::to_simple_string(l_img_time);
				fileWriter1 << l_img_time_wall << std::endl;
				std::cout << "Writing timestamps for Left Camera, count is: " << l_img_cnt << std::endl;
			}
			else{
				std::cout << "Error when writing left camera timestamps" << std::endl;
			}

			// For right Camera
			sensor_msgs::CompressedImageConstPtr r_img = r_img_st.top();
			std::stack<sensor_msgs::CompressedImageConstPtr>().swap(r_img_st);

			cv_bridge::CvImagePtr r_img_ptr;
			try
			{
				r_img_ptr = cv_bridge::toCvCopy(r_img, sensor_msgs::image_encodings::BGR8);
			}
			catch (cv_bridge::Exception& err)
			{
				ROS_ERROR("cv_bridge exception: %s", err.what());
				return 0;
			}
			FS::path r_img_name = FS::path(rightCameraDataPath) / FS::path(frameCountToString(r_img_cnt) + ".png");
			cv::imwrite(r_img_name.string(), r_img_ptr->image);
			r_img_cnt++;

			if(fileWriter2.is_open())
			{
				boost::posix_time::ptime r_img_time = (r_img->header.stamp).toBoost();
				std::string r_img_time_wall = boost::posix_time::to_simple_string(r_img_time);
				fileWriter2 << r_img_time_wall << std::endl;
			}
			else{
				std::cout << "Error when writing right camera timestamps" << std::endl;
			}

			// For lidar 
			sensor_msgs::PointCloud2ConstPtr lidarPtr = lidar_st.top();
			std::stack<sensor_msgs::PointCloud2ConstPtr>().swap(lidar_st);
			pcl::PCLPointCloud2* pointcloud1 = new pcl::PCLPointCloud2;
			pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud2 (new pcl::PointCloud<pcl::PointXYZI>);
			pcl_conversions::toPCL(*lidarPtr, *pointcloud1);
			pcl::fromPCLPointCloud2(*pointcloud1, *pointcloud2);
			FS::path lidar_frame_name = FS::path(lidarDataPath) / FS::path(frameCountToString(lidar_cnt) + ".bin");
			
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

			// For GPS data
			sensor_msgs::NavSatFixConstPtr navsatfixPtr = nav_st.top();
			std::stack<sensor_msgs::NavSatFixConstPtr>().swap(nav_st);

			sensor_msgs::ImuConstPtr imuPtr = imu_st.top();
			std::stack<sensor_msgs::ImuConstPtr>().swap(imu_st);

			piksi_rtk_msgs::VelNedConstPtr velPtr = vel_st.top();
			std::stack<piksi_rtk_msgs::VelNedConstPtr>().swap(vel_st);

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
	}

	bag.close();

	fileWriter1.close();
	fileWriter2.close();
	fileWriter3.close();
	fileWriter4.close();

}
