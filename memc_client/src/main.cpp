#include <iostream>
#include <vector>
#include <sstream>

#include "mc_reader.h"

void process_test(const ros::TimerEvent &_evt)
{
	McReader::GetInstance()->TestCase();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Test_Memcached_Reader");

	ros::NodeHandle n;
	ros::Timer timer = n.createTimer(ros::Duration(0.02), process_test);

	ros::spin();

	return 0;
}