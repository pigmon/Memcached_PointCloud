#pragma once

#include <string>
#include <vector>

#include <libmemcached/memcached.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// -------------------------------------------------
// Data Transfor
// -------------------------------------------------
struct McPointCloud2
{
	// header
	uint32_t m_seq_id;
	double m_stamp_in_sec;
	std::string m_frame_id = "velodyne";
	// info
	uint32_t m_height;
	uint32_t m_width;
	bool m_b_big_endian;
	uint32_t m_point_step;
	uint32_t m_row_step;
	bool m_b_dense;
	// data
	std::vector<uint8_t> m_data;

	sensor_msgs::PointCloud2 ToRosMsg();
};

// -------------------------------------------------
// Memcached Client
// -------------------------------------------------
class McReader
{
private:
	memcached_st* m_memc;
	memcached_server_st *m_server;
	memcached_return m_rc;

	std::string m_last_error;

	static McReader *m_instance;

	McReader();

	bool McGetPc2(McPointCloud2& _out_data);
	bool CheckLastError_Step1(
		const memcached_return _rc1,
		const memcached_return _rc2,
		const memcached_return _rc3,
		const memcached_return _rc4,
		const memcached_return _rc5,
		const memcached_return _rc6,
		const memcached_return _rc7,
		const memcached_return _rc8,
		const memcached_return _rc9);

	// Test Case Use
	ros::Publisher m_publisher;

public:
	~McReader() { memcached_free(m_memc); }
	static McReader *GetInstance();
	const char* LastError() const { return m_last_error.c_str(); }

	bool ReadPointCloud_AsRosMsg(sensor_msgs::PointCloud2& _out_msg);
	void TestCase();
};