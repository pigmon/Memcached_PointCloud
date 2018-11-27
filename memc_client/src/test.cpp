#include <string>
#include <vector>
#include <sstream>

#include <libmemcached/memcached.h>

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

const unsigned int BLOCK_COUNT = 20;

struct Info
{
	uint32_t m_seq;
	int64_t m_stamp;
	std::string m_frame_id;
	uint8_t m_point_type;
	uint32_t m_width;
	uint32_t m_height;
	uint32_t m_point_step;
	uint32_t m_row_step;
	uint8_t m_is_bigendian;
	uint8_t m_is_dense;

	std::vector<::pcl::PCLPointField> m_fields;
	std::vector<uint8_t> m_data;

	void Print()
	{
		std::cout << "Info:\n";
		std::cout << "Seq: " << m_seq << "; Stamp: " << m_stamp << "; frame_id: " << m_frame_id << std::endl;
		std::cout << "Size: <" << m_width << ", " << m_height << ", " << m_point_step << ", " << m_row_step << ">\n";
		std::cout << "Bools: " << (int)m_is_bigendian << ", " << (int)m_is_dense << std::endl;

		std::cout << "Point Fields: \n";
		for (size_t i = 0; i < m_fields.size(); i++)
		{
			std::cout << m_fields[i].name << "," << m_fields[i].offset << "," << (int)(m_fields[i].datatype) << "," << m_fields[i].count << std::endl;
		}

		std::cout << "\nData Len in Bytes: " << m_data.size() << std::endl;
	}

	void ToRosMsg(sensor_msgs::PointCloud2 &msg)
	{
		msg.header.seq = m_seq;
		//msg.header.stamp.fromNSec(m_stamp * 1000ull);
		msg.header.stamp.fromNSec(m_stamp);
		msg.header.frame_id = m_frame_id;
		msg.width = m_width;
		msg.height = m_height;
		msg.point_step = m_point_step;
		msg.row_step = m_row_step;
		msg.is_bigendian = m_is_bigendian;
		msg.is_dense = m_is_dense;

		pcl_conversions::fromPCL(m_fields, msg.fields);
		msg.data = m_data;
	}
};

bool ReadInfo(const char *_raw, Info &_out_info);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Test_Memcached_Reader");

	// test
	ros::NodeHandle nh;
	ros::Publisher Pubber = nh.advertise<sensor_msgs::PointCloud2>("mc_testcase", 10);

	time_t expiration = (time_t)0;
	uint32_t flags;
	memcached_return rc;

	memcached_st *memc = memcached_create(NULL);
	memcached_server_st *server = memcached_server_list_append(NULL, "localhost", 11211, &rc);
	rc = memcached_server_push(memc, server);
	memcached_server_list_free(server);

	size_t val_len = 0;

	ros::Time::init();
	ros::Rate r(10);
	const char *info_key = "pcinfo";

	while (ros::ok())
	{
		double start_time = ros::Time::now().toSec();
		Info info;

		// header & info
		char *raw_info = memcached_get(memc, info_key, strlen(info_key), &val_len, &flags, &rc);
		if (rc == MEMCACHED_SUCCESS)
		{
			std::cout << raw_info << std::endl;
			ReadInfo(raw_info, info);
		}
		else
			std::cout << "Getting Error: " << memcached_strerror(memc, rc) << std::endl;

		free(raw_info);

		// data
		info.m_data.clear();
		for (uint32_t i = 0; i < BLOCK_COUNT; i++)
		{
			std::ostringstream ss_key;
			ss_key << "data_" << i;
			std::string key = ss_key.str();

			char *rslt = memcached_get(memc, key.c_str(), key.length(), &val_len, &flags, &rc);

			if (rc == MEMCACHED_SUCCESS)
			{
				uint8_t *arr = new uint8_t[val_len];
				memcpy((uint8_t *)arr, (const uint8_t *)rslt, val_len);
				info.m_data.insert(info.m_data.end(), arr, arr + val_len);

				delete[] arr;
			}
			else
			{
				std::string error = memcached_strerror(memc, rc);
				std::cerr << key << " " << error << std::endl;
				free(rslt);
			}

			free(rslt);
		}
		double dur = ros::Time::now().toSec() - start_time;
		ROS_INFO("Reading Cost: %f ms", dur * 1000);

		info.Print();

		sensor_msgs::PointCloud2 msg;
		info.ToRosMsg(msg);
		Pubber.publish(msg);

		r.sleep();
	}

	return 0;
}

bool ReadInfo(const char *_raw, Info &_out_info)
{
	std::string str_raw = _raw;
	assert(str_raw.front() == '#' && str_raw.back() == '#');

	std::string trim(str_raw, 1, str_raw.length() - 2);
	std::istringstream ss(trim);
	std::vector<std::string> buff;

	std::string tmp;
	while (std::getline(ss, tmp, '_'))
	{
		buff.push_back(tmp);
	}

	assert(buff.size() == 10);

	_out_info.m_seq = atoi(buff[0].c_str());
	_out_info.m_stamp = atol(buff[1].c_str());
	_out_info.m_frame_id = buff[2];
	_out_info.m_point_type = atoi(buff[3].c_str());
	_out_info.m_width = atoi(buff[4].c_str());
	_out_info.m_height = atoi(buff[5].c_str());
	_out_info.m_point_step = atoi(buff[6].c_str());
	_out_info.m_row_step = atoi(buff[7].c_str());
	_out_info.m_is_bigendian = atoi(buff[8].c_str());
	_out_info.m_is_dense = atoi(buff[9].c_str());

	typedef pcl::PointXYZI PointType;
	//if (_out_info.m_point_type != 0)
	//	typedef pcl::PointXYZI PointType;

	// fields
	_out_info.m_fields.clear();
	pcl::for_each_type<typename pcl::traits::fieldList<PointType>::type>(pcl::detail::FieldAdder<PointType>(_out_info.m_fields));
}