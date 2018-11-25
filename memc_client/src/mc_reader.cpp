#include "mc_reader.h"

#define DEBUG

// -------------------------------------------------
// Storage Keys for Ros Message PointCloud2
// -------------------------------------------------
// header
const char* key_seq_id = "header_seq";
const char* key_stamp = "header_stamp";
const char* key_frame_id = "header_frame_id";
// info
const char* key_width = "width";
const char* key_height = "height";
const char* key_is_bigendian = "is_bigendian";
const char* key_point_step = "point_step";
const char* key_row_step = "row_step";
const char* key_is_dense = "is_dense";
// data
const char* key_data_prefix = "data";

// -------------------------------------------------
// struct McPointCloud2
// -------------------------------------------------
sensor_msgs::PointCloud2 McPointCloud2::ToRosMsg()
{
	sensor_msgs::PointCloud2 msg;
	msg.header.seq = m_seq_id;
	msg.header.stamp.fromSec(m_stamp_in_sec);
	msg.header.frame_id = m_frame_id;
	msg.width = m_width;
	msg.height = m_height;
	msg.is_bigendian = m_b_big_endian;
	msg.point_step = m_point_step;
	msg.row_step = m_row_step;
	msg.is_dense = m_b_dense;
	msg.data = m_data;

	// temp test
	/*
	Name: x | Offset: 0 | DataType: 7 | Count: 1
	Name: y | Offset: 4 | DataType: 7 | Count: 1
	Name: z | Offset: 8 | DataType: 7 | Count: 1
	Name: intensity | Offset: 16 | DataType: 7 | Count: 1
	Name: ring | Offset: 20 | DataType: 4 | Count: 1
	*/
	sensor_msgs::PointField msg_x, msg_y, msg_z, msg_intensity, msg_ring;
	msg_x.name = "x";
	msg_x.offset = 0;
	msg_x.datatype = 7;
	msg_x.count = 1;
	msg_y.name = "y";
	msg_y.offset = 4;
	msg_y.datatype = 7;
	msg_y.count = 1;
	msg_z.name = "z";
	msg_z.offset = 8;
	msg_z.datatype = 7;
	msg_z.count = 1;
	msg_intensity.name = "intensity";
	msg_intensity.offset = 16;
	msg_intensity.datatype = 7;
	msg_intensity.count = 1;
	msg_ring.name = "ring";
	msg_ring.offset = 20;
	msg_ring.datatype = 4;
	msg_ring.count = 1;

	msg.fields.push_back(msg_x);
	msg.fields.push_back(msg_y);
	msg.fields.push_back(msg_z);
	msg.fields.push_back(msg_intensity);
	msg.fields.push_back(msg_ring);

	return msg;
}

// -------------------------------------------------
// class McReader
// -------------------------------------------------
McReader *McReader::m_instance = 0;

McReader *McReader::GetInstance()
{
	if (!m_instance)
	{
		m_instance = new McReader();
	}
	return m_instance;
}

McReader::McReader()
{
	m_memc = memcached_create(NULL);
	m_server = memcached_server_list_append(NULL, "localhost", 11211, &m_rc);
	m_rc = memcached_server_push(m_memc, m_server);
	memcached_server_list_free(m_server);

#ifdef DEBUG
	ros::NodeHandle nh;
	m_publisher = nh.advertise<sensor_msgs::PointCloud2>("mc_testcase", 10);
#endif
}

bool McReader::CheckLastError_Step1(
	const memcached_return _rc1,
	const memcached_return _rc2,
	const memcached_return _rc3,
	const memcached_return _rc4,
	const memcached_return _rc5,
	const memcached_return _rc6,
	const memcached_return _rc7,
	const memcached_return _rc8,
	const memcached_return _rc9)
{
	bool ret = false;
	if (_rc1 != MEMCACHED_SUCCESS)
	{
		m_last_error += memcached_strerror(m_memc, _rc1);
		m_last_error += " | ";
		ret = true;
	}
	if (_rc2 != MEMCACHED_SUCCESS)
	{
		m_last_error += memcached_strerror(m_memc, _rc2);
		m_last_error += " | ";
		ret = true;
	}
	if (_rc3 != MEMCACHED_SUCCESS)
	{
		m_last_error += memcached_strerror(m_memc, _rc3);
		m_last_error += " | ";
		ret = true;
	}
	if (_rc4 != MEMCACHED_SUCCESS)
	{
		m_last_error += memcached_strerror(m_memc, _rc4);
		m_last_error += " | ";
		ret = true;
	}
	if (_rc5 != MEMCACHED_SUCCESS)
	{
		m_last_error += memcached_strerror(m_memc, _rc5);
		m_last_error += " | ";
		ret = true;
	}
	if (_rc6 != MEMCACHED_SUCCESS)
	{
		m_last_error += memcached_strerror(m_memc, _rc6);
		m_last_error += " | ";
		ret = true;
	}
	if (_rc7 != MEMCACHED_SUCCESS)
	{
		m_last_error += memcached_strerror(m_memc, _rc7);
		m_last_error += " | ";
		ret = true;
	}
	if (_rc8 != MEMCACHED_SUCCESS)
	{
		m_last_error += memcached_strerror(m_memc, _rc8);
		m_last_error += " | ";
		ret = true;
	}
	if (_rc9 != MEMCACHED_SUCCESS)
	{
		m_last_error += memcached_strerror(m_memc, _rc9);
		m_last_error += " | ";
		ret = true;
	}

	return ret;
}

bool McReader::McGetPc2(McPointCloud2 &_out_data)
{
	m_last_error = "";

	size_t val_len;
	uint32_t flags;

	memcached_return rc_seq, rc_stamp, rc_frame_id, rc_width, rc_height, rc_bigendian, rc_point_step, rc_row_step, rc_dense;

	// header
	char *seq_id = memcached_get(m_memc, key_seq_id, strlen(key_seq_id), &val_len, &flags, &rc_seq);
	char *stamp = memcached_get(m_memc, key_stamp, strlen(key_stamp), &val_len, &flags, &rc_stamp);
	char *frame_id = memcached_get(m_memc, key_frame_id, strlen(key_frame_id), &val_len, &flags, &rc_frame_id);
	// info
	char *width = memcached_get(m_memc, key_width, strlen(key_width), &val_len, &flags, &rc_width);
	char *height = memcached_get(m_memc, key_height, strlen(key_height), &val_len, &flags, &rc_height);
	char *is_bigendian = memcached_get(m_memc, key_is_bigendian, strlen(key_is_bigendian), &val_len, &flags, &rc_bigendian);
	char *point_step = memcached_get(m_memc, key_point_step, strlen(key_point_step), &val_len, &flags, &rc_point_step);
	char *row_step = memcached_get(m_memc, key_row_step, strlen(key_row_step), &val_len, &flags, &rc_row_step);
	char *is_dense = memcached_get(m_memc, key_is_dense, strlen(key_is_dense), &val_len, &flags, &rc_dense);

	if (CheckLastError_Step1(rc_seq, rc_stamp, rc_frame_id, rc_width, rc_height, rc_bigendian, rc_point_step, rc_row_step, rc_dense))
	{
		free(seq_id);
		free(stamp);
		free(frame_id);
		free(width);
		free(height);
		free(is_bigendian);
		free(point_step);
		free(row_step);
		free(is_dense);

		std::cerr << "Reading Point Cloud Step 1 Error, Please Call McReader::LastError() for information." << std::endl;
		return false;
	}

#ifdef DEBUG
	std::cout << seq_id << "," << stamp << "," << frame_id << "," << width << "," << height << "," << is_bigendian << "," << point_step
			  << "," << row_step << "," << is_dense << std::endl;
#endif
	_out_data.m_seq_id = atoi(seq_id);
	_out_data.m_stamp_in_sec = atof(stamp);
	_out_data.m_frame_id = frame_id;
	_out_data.m_width = atoi(width);
	_out_data.m_height = atoi(height);
	_out_data.m_b_big_endian = atoi(is_bigendian);
	_out_data.m_point_step = atoi(point_step);
	_out_data.m_row_step = atoi(row_step);
	_out_data.m_b_dense = atoi(is_dense);

	free(seq_id);
	free(stamp);
	free(frame_id);
	free(width);
	free(height);
	free(is_bigendian);
	free(point_step);
	free(row_step);
	free(is_dense);

	// data
	_out_data.m_data.clear();
	for (uint32_t i = 0; i < 2; i++)
	{
		std::ostringstream ss_key;
		ss_key << "data" << i;
		std::string key = ss_key.str();

		char *rslt = memcached_get(m_memc, key.c_str(), key.length(), &val_len, &flags, &m_rc);

		if (m_rc == MEMCACHED_SUCCESS)
		{
			uint8_t *arr = new uint8_t[val_len];
			memcpy((uint8_t *)arr, (const uint8_t *)rslt, val_len);
			_out_data.m_data.insert(_out_data.m_data.end(), arr, arr + val_len);

			delete[] arr;
		}
		else
		{
			m_last_error += memcached_strerror(m_memc, m_rc);
			std::cerr << "Reading Point Cloud Step 2 Error, Please Call McReader::LastError() for information." << std::endl;
			free(rslt);

			return false;
		}

		free(rslt);
	}

#ifdef DEBUG
	size_t total_len = _out_data.m_data.size();
	std::cout << (int)(_out_data.m_data[0]) << " " << (int)(_out_data.m_data[total_len/2]) << " " << (int)(_out_data.m_data[total_len-1]) << std::endl;
#endif

	return true;
}

bool McReader::ReadPointCloud_AsRosMsg(sensor_msgs::PointCloud2& _out_msg)
{
	McPointCloud2 obj;
	bool ret = McGetPc2(obj);
	if (ret)
	{
		_out_msg = obj.ToRosMsg();
		return true;
	}

	return false;
}

void McReader::TestCase()
{
#ifdef DEBUG
	sensor_msgs::PointCloud2 msg;

	if (!ReadPointCloud_AsRosMsg(msg))
	{
		std::cout << "Test Case Fail.\n";
		return;
	}
	else
	{
		m_publisher.publish(msg);
	}
#endif
}