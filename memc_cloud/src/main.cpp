#include <string>
#include <vector>
#include <iostream>
#include <sstream>

#include "mc.h"

std::string key = "pc";

bool _quit = false;

// Subscribe Callback func
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &pc2_msg)
{
	//if (_quit)
	//	return;

	MemCached_Agnt::GetInstance()->SetFrame(pc2_msg);
	/*
	std::cout << "-----------------------------------------\n";
	for (size_t i = 0; i < pc2_msg->fields.size(); i++)
	{
		sensor_msgs::PointField msg = pc2_msg->fields[i];
		std::cout << "Name: " << msg.name << " | Offset: " << msg.offset << " | DataType: " << (int)(msg.datatype) << " | Count: " << msg.count << std::endl;	
	}
	*/
	//_quit = true;
}

int main(int argc, char **argv)
{
	// Initialize ROS
	ros::init(argc, argv, "memc_cloud");
	ros::NodeHandle nh;

	ros::Subscriber sub = nh.subscribe("/velodyne_points", 1, cloud_cb);

	// Spin
	ros::spin();

	return 0;
}