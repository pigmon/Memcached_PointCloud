#include <iostream>
#include <sstream>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/hdl_grabber.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>

#include <libmemcached/memcached.h>
// pcl::PointXYZ, pcl::PointXYZI, pcl::PointXYZRGBA
typedef pcl::PointXYZI PointType;


ros::Publisher Pubber;
memcached_st *memc;
memcached_server_st *server;
memcached_return rc;

uint32_t flags;
int type_id;

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "memc_cloud");

	//ros::Rate r(1);
	memc = memcached_create(NULL);
	server = memcached_server_list_append(NULL, "localhost", 11211, &rc);
	rc = memcached_server_push(memc, server);
	memcached_server_list_free(server);

	ros::NodeHandle nh;
	Pubber = nh.advertise<sensor_msgs::PointCloud2>("mc_testcase", 10);

	std::string ipaddress("192.168.1.201");
	std::string port("2368");

	pcl::PointCloud<PointType>::ConstPtr cloud_;
	const std::type_info &type = typeid(PointType);
	type_id = (type == typeid(pcl::PointXYZ)) ? 0 : 1; // ignore PointXYZRGBA for now

	// Retrieved Point Cloud Callback Function
	boost::mutex mutex;
	boost::function<void(const pcl::PointCloud<PointType>::ConstPtr &)> function =
		[&cloud_, &mutex](const pcl::PointCloud<PointType>::ConstPtr &ptr) {
			boost::mutex::scoped_lock lock(mutex);
			cloud_ = ptr;
			
			double start_time = ros::Time::now().toSec();
			pcl::PCLPointCloud2 pcl_pc2;
			toPCLPointCloud2(*cloud_, pcl_pc2);

			std::ostringstream ss;
			// header
			ss << "#" << pcl_pc2.header.seq << "_";
			pcl_conversions::toPCL(ros::Time::now(), pcl_pc2.header.stamp);
			ss << pcl_pc2.header.stamp << "_";
			ss << "velodyne_";
			// info
			ss << type_id << "_";
			ss << pcl_pc2.width << "_";
			ss << pcl_pc2.height << "_";
			ss << pcl_pc2.point_step << "_";
			ss << pcl_pc2.row_step << "_";
			ss << (int)(pcl_pc2.is_bigendian) << "_";
			ss << (int)(pcl_pc2.is_dense) << "#";

			std::string info_str = ss.str();
			const char* info_data = info_str.c_str();
			ROS_INFO("%s", info_data);

			rc = memcached_set(memc, "pcinfo", 6, info_data, info_str.length(), (time_t)0, flags);
			if (rc != MEMCACHED_SUCCESS)
				ROS_INFO("Cache Failed. %s", memcached_strerror(memc, rc));


			// data
			uint32_t block_count = 20;
			size_t block_size = pcl_pc2.data.size() / block_count;
			size_t remain_size = pcl_pc2.data.size() - block_size * (block_count - 1);
			ROS_INFO("Block Size: %ld, Remain Size: %ld", block_size, remain_size);

			
			for (int i = 0; i < block_count - 1; i++)
			{
				std::ostringstream ss_key;
				ss_key << "data_" << i;
				std::string key = ss_key.str();
				rc = memcached_set(memc, key.c_str(), key.length(), 
					(const char*)(pcl_pc2.data.data() + block_size * i), block_size, (time_t)0, flags);

				if (rc != MEMCACHED_SUCCESS)
					std::cout << key << " store failed with : " << memcached_strerror(memc, rc) << std::endl;
				//else 
				//	std::cout << key << " Stored.\n";

				ss_key.clear();
			}

			std::ostringstream final_key_ss;
			final_key_ss << "data_" << block_count - 1;
			std::string key = final_key_ss.str();
			rc = memcached_set(memc, key.c_str(), key.length(), 
				(const char*)(pcl_pc2.data.data() + block_size * (block_count - 1)), remain_size, (time_t)0, flags);	
			final_key_ss.clear();		
			if (rc != MEMCACHED_SUCCESS)
				std::cout << key << " store failed with : " << memcached_strerror(memc, rc) << std::endl;
			//else 
			//	std::cout << key << " Stored.\n";


			/*
			rc = memcached_set(memc, "data_0", 6, (const char*)(pcl_pc2.data.data()), 
				block_size, (time_t)0, flags);
			rc = memcached_set(memc, "data_1", 6, (const char*)(pcl_pc2.data.data() + block_size), 
				block_size, (time_t)0, flags);
			rc = memcached_set(memc, "data_2", 6, (const char*)(pcl_pc2.data.data() + block_size * 2), 
				block_size, (time_t)0, flags);
			rc = memcached_set(memc, "data_3", 6, (const char*)(pcl_pc2.data.data() + block_size * 3), 
				block_size, (time_t)0, flags);
			rc = memcached_set(memc, "data_4", 6, (const char*)(pcl_pc2.data.data() + block_size * 4), 
				block_size, (time_t)0, flags);
			rc = memcached_set(memc, "data_5", 6, (const char*)(pcl_pc2.data.data() + block_size * 5), 
				block_size, (time_t)0, flags);
			rc = memcached_set(memc, "data_6", 6, (const char*)(pcl_pc2.data.data() + block_size * 6), 
				block_size, (time_t)0, flags);
			rc = memcached_set(memc, "data_7", 6, (const char*)(pcl_pc2.data.data() + block_size * 7), 
				block_size, (time_t)0, flags);
			rc = memcached_set(memc, "data_8", 6, (const char*)(pcl_pc2.data.data() + block_size * 8), 
				block_size, (time_t)0, flags);
			rc = memcached_set(memc, "data_9", 6, (const char*)(pcl_pc2.data.data() + block_size * 9), 
				remain_size, (time_t)0, flags);
			*/
			double step1_time = ros::Time::now().toSec();
			double dur = step1_time - start_time;
			ROS_INFO("Duration: %f ms", dur * 1000);

			//if (rc != MEMCACHED_SUCCESS)
			//	ROS_INFO("Cache Failed. %s", memcached_strerror(memc, rc));
			/*
			sensor_msgs::PointCloud2 msg;

			msg.header.seq = pcl_pc2.header.seq;
			msg.header.frame_id = "velodyne";
			msg.header.stamp = ros::Time::now();
			msg.height = pcl_pc2.height;
			msg.width = pcl_pc2.width;
			pcl_conversions::fromPCL(pcl_pc2.fields, msg.fields);
			msg.is_bigendian = pcl_pc2.is_bigendian;
			msg.point_step = pcl_pc2.point_step;
			msg.row_step = pcl_pc2.row_step;
			msg.is_dense = pcl_pc2.is_dense;
			msg.data = pcl_pc2.data;

			Pubber.publish(msg);
			*/
		};

	// HDL Grabber
	boost::shared_ptr<pcl::HDLGrabber> grabber;
	std::cout << "Capture from Sensor..." << std::endl;
	grabber = boost::shared_ptr<pcl::HDLGrabber>(
		new pcl::HDLGrabber(
			boost::asio::ip::address::from_string(ipaddress),
			boost::lexical_cast<unsigned short>(port)));

	// Register Callback Function
	boost::signals2::connection connection = grabber->registerCallback(function);

	// Start Grabber
	grabber->start();

	while (ros::ok())
	{
	}

	// Stop Grabber
	grabber->stop();

	memcached_free(memc);

	// Disconnect Callback Function
	if (connection.connected())
	{
		connection.disconnect();
	}

	return 0;
}