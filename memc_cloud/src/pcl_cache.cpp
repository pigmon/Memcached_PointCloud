#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/hdl_grabber.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>

#include <libmemcached/memcached.h>
// pcl::PointXYZ, pcl::PointXYZI, pcl::PointXYZRGBA
typedef pcl::PointXYZ PointType;

ros::Publisher Pubber;
memcached_st *memc, *memc2;
memcached_server_st *server, *server2;
memcached_return rc;

uint32_t flags;

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "memc_cloud");

	//ros::Rate r(1);
	memc = memcached_create(NULL);
	server = memcached_server_list_append(NULL, "localhost", 11211, &rc);
	rc = memcached_server_push(memc, server);
	memcached_server_list_free(server);

	memc2 = memcached_create(NULL);
	server2 = memcached_server_list_append(NULL, "localhost", 11312, &rc);
	rc = memcached_server_push(memc2, server2);
	memcached_server_list_free(server2);

	ros::NodeHandle nh;
	Pubber = nh.advertise<sensor_msgs::PointCloud2>("mc_testcase", 10);

	std::string ipaddress("192.168.1.201");
	std::string port("2368");

	pcl::PointCloud<PointType>::ConstPtr cloud_;

	// Retrieved Point Cloud Callback Function
	boost::mutex mutex;
	boost::function<void(const pcl::PointCloud<PointType>::ConstPtr &)> function =
		[&cloud_, &mutex](const pcl::PointCloud<PointType>::ConstPtr &ptr) {
			boost::mutex::scoped_lock lock(mutex);
			cloud_ = ptr;

			pcl::PCLPointCloud2 pcl_pc2;
			toPCLPointCloud2(*cloud_, pcl_pc2);
			//ROS_INFO("Size of PointT: %ld", sizeof(PointType));
			//ROS_INFO("raw data len: %ld", cloud_->points.size());
			//ROS_INFO("data len: %ld", pcl_pc2.data.size());
			double start_time = ros::Time::now().toSec();
			rc = memcached_set(memc, "data_test_0", 11, (const char*)(pcl_pc2.data.data()), pcl_pc2.data.size() / 2, (time_t)0, flags);
			double step1_time = ros::Time::now().toSec();
			double dur = step1_time - start_time;
			ROS_INFO("Duration: %f ms", dur * 1000);

			if (rc != MEMCACHED_SUCCESS)
				ROS_INFO("Cache Failed. %s", memcached_strerror(memc, rc));
			/*
			rc = memcached_set(memc2, "data_test_1", 11, (const char*)(pcl_pc2.data.data() + pcl_pc2.data.size() / 2), pcl_pc2.data.size() - pcl_pc2.data.size() / 2, (time_t)0, flags);
			dur = ros::Time::now().toSec() - step1_time;
			ROS_INFO("Duration 2 : %f ms", dur * 1000);

			if (rc != MEMCACHED_SUCCESS)
				ROS_INFO("Cache Failed. %s", memcached_strerror(memc, rc));
			*/
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
	memcached_free(memc2);

	// Disconnect Callback Function
	if (connection.connected())
	{
		connection.disconnect();
	}

	return 0;
}