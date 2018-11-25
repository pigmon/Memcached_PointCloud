#pragma once

#include <string>
#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>

struct McPc2
{
// keys:
	// header
	std::string key_seq_id = "header_seq";
	std::string key_stamp = "header_stamp";
	std::string key_frame_id = "header_frame_id";
	// info
	std::string key_width = "width";
	std::string key_height = "height";
	std::string key_is_bigendian = "is_bigendian";
	std::string key_point_step = "point_step";
	std::string key_row_step = "row_step";
	std::string key_is_dense = "is_dense";
	// data
	std::vector<std::string> key_data;
	// fields
	// TODO

// key lengths:
	// header
	size_t keylen_seq_id;
	size_t keylen_stamp;
	size_t keylen_frame_id;
	// info
	size_t keylen_width;
	size_t keylen_height;
	size_t keylen_is_bigendian;
	size_t keylen_point_step;
	size_t keylen_row_step;
	size_t keylen_is_dense;
	// data
	std::vector<size_t> keylen_data;
	uint8_t data_block_cnt = 1;

// values:
	// header
	std::string val_seq_id;
	std::string val_stamp;
	std::string val_frame_id;
	// info
	std::string val_width;
	std::string val_height;
	std::string val_is_bigendian;
	std::string val_point_step;
	std::string val_row_step;
	std::string val_is_dense;
	// data
	std::vector<std::vector<uint8_t>> val_data;
	char* m_tmp_data;

// val lengths:
	// header
	size_t vallen_seq_id;
	size_t vallen_stamp;
	size_t vallen_frame_id;
	// info
	size_t vallen_width;
	size_t vallen_height;
	size_t vallen_is_bigendian;
	size_t vallen_point_step;
	size_t vallen_row_step;
	size_t vallen_is_dense;
	// data
	std::vector<size_t> vallen_data;

// Functions
	McPc2();
	void SetFrame(const sensor_msgs::PointCloud2ConstPtr &pc2_msg);
	void Cache();
	void TestRead();
};