#include <sstream>
#include <iostream>

#include "mc_pointcloud2.h"

const int BLOCK_SIZE = 1000000;

template <class T>
void FromNumber(const T _num, std::string& _out_string)
{
	std::ostringstream ss;
	ss << _num;
	_out_string = ss.str();

	ss.clear();
}

McPc2::McPc2()
{
	keylen_seq_id = key_seq_id.length(); 
	keylen_stamp = key_stamp.length();
	keylen_frame_id = key_frame_id.length();
	keylen_width = key_width.length();
	keylen_height = key_height.length();
	keylen_is_bigendian = key_is_bigendian.length();
	keylen_point_step = key_point_step.length();
	keylen_row_step = key_row_step.length();
	keylen_is_dense = key_is_dense.length();
	//keylen_data = key_data.length();
}

void McPc2::SetFrame(const sensor_msgs::PointCloud2ConstPtr &pc2_msg)
{
	// header::seq
	FromNumber(pc2_msg->header.seq, val_seq_id);
	vallen_seq_id = val_seq_id.length();
	// header::stamp
	//FromNumber(pc2_msg->header.stamp.toSec(), val_stamp);
	FromNumber(ros::Time::now().toSec(), val_stamp);
	ROS_INFO("Start Time: %f", ros::Time::now().toSec());
	vallen_stamp = val_stamp.length();
	// header::frame_id
	val_frame_id = pc2_msg->header.frame_id;
	vallen_frame_id = val_frame_id.length();
	// width
	FromNumber(pc2_msg->width, val_width);
	vallen_width = val_width.length();
	// height
	FromNumber(pc2_msg->height, val_height);
	vallen_height = val_height.length();
	// is big endian
	//FromNumber(pc2_msg->is_bigendian, val_is_bigendian);
	val_is_bigendian = (pc2_msg->is_bigendian == 0) ? "0" : "1";
	vallen_is_bigendian = val_is_bigendian.length();
	// point_step
	FromNumber(pc2_msg->point_step, val_point_step);
	vallen_point_step = val_point_step.length();
	// row_step
	FromNumber(pc2_msg->row_step, val_row_step);
	vallen_row_step = val_row_step.length();
	// is_dense
	//FromNumber(pc2_msg->is_dense, val_is_dense);
	val_is_dense = (pc2_msg->is_dense == 0) ? "0" : "1";
	vallen_is_dense = val_is_dense.length();
	// data
	key_data.clear();
	keylen_data.clear();
	val_data.clear();
	vallen_data.clear();

	std::cout << "Original Size: " << pc2_msg->data.size() << std::endl;
	size_t total_cnt = pc2_msg->data.size();
	std::cout << (int)(pc2_msg->data[0]) << " " << (int)(pc2_msg->data[total_cnt/2] )<< " " << (int)(pc2_msg->data[total_cnt-1]) << std::endl;

	data_block_cnt = pc2_msg->data.size() / BLOCK_SIZE + 1;
	for (unsigned int i = 0; i < data_block_cnt; i++)
	{
		std::ostringstream ss_key, ss_val;
		unsigned int remain = (i == data_block_cnt - 1) ? (pc2_msg->data.size() % BLOCK_SIZE) : BLOCK_SIZE;
		ss_key << "data" << i;

		//for (unsigned int j = 0; j < remain; j++)
		//{
		//	ss_val << pc2_msg->data[i * BLOCK_SIZE + j];
		//}
		uint8_t* tmp = new uint8_t[remain + 1];
		memcpy((uint8_t*)tmp, (const uint8_t*)(pc2_msg->data.data() + i * BLOCK_SIZE), remain);
		std::vector<uint8_t> tmp_vec(tmp, tmp + remain);
		//ss_val << tmp;

		std::string key = ss_key.str();
		key_data.push_back(key);
		keylen_data.push_back(key.length());
		val_data.push_back(tmp_vec);
		vallen_data.push_back(remain);

		delete [] tmp;
		ss_key.clear();
		ss_val.clear();
	}

	ROS_INFO("Zip Time: %f", ros::Time::now().toSec());
}

void McPc2::Cache()
{
	
}

void McPc2::TestRead()
{

}
