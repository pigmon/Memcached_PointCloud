#include <iostream>
#include <vector>
#include "ros/ros.h"

#include "mc.h"

//#define DEBUG

MemCached_Agnt *MemCached_Agnt::m_instance = 0;

MemCached_Agnt *MemCached_Agnt::GetInstance()
{
	if (!m_instance)
	{
		m_instance = new MemCached_Agnt();
	}
	return m_instance;
}

MemCached_Agnt::MemCached_Agnt()
{
	m_memc = memcached_create(NULL);
	m_server = memcached_server_list_append(NULL, "localhost", 11211, &m_rc);
	m_rc = memcached_server_push(m_memc, m_server);
	memcached_server_list_free(m_server);

	m_pc2 = new McPc2();
}

void MemCached_Agnt::SavePc2()
{
	uint32_t flags;

	// header
	m_rc = memcached_set(
		m_memc, m_pc2->key_seq_id.c_str(), m_pc2->keylen_seq_id, 
		m_pc2->val_seq_id.c_str(), m_pc2->vallen_seq_id, (time_t)0, flags);
	m_rc = memcached_set(
		m_memc, m_pc2->key_stamp.c_str(), m_pc2->keylen_stamp, 
		m_pc2->val_stamp.c_str(), m_pc2->vallen_stamp, (time_t)0, flags);
	m_rc = memcached_set(
		m_memc, m_pc2->key_frame_id.c_str(), m_pc2->keylen_frame_id, 
		m_pc2->val_frame_id.c_str(), m_pc2->vallen_frame_id, (time_t)0, flags);

	// info
	m_rc = memcached_set(
		m_memc, m_pc2->key_width.c_str(), m_pc2->keylen_width, 
		m_pc2->val_width.c_str(), m_pc2->vallen_width, (time_t)0, flags);
	m_rc = memcached_set(
		m_memc, m_pc2->key_height.c_str(), m_pc2->keylen_height, 
		m_pc2->val_height.c_str(), m_pc2->vallen_height, (time_t)0, flags);
	m_rc = memcached_set(
		m_memc, m_pc2->key_is_bigendian.c_str(), m_pc2->keylen_is_bigendian, 
		m_pc2->val_is_bigendian.c_str(), m_pc2->vallen_is_bigendian, (time_t)0, flags);
	m_rc = memcached_set(
		m_memc, m_pc2->key_point_step.c_str(), m_pc2->keylen_point_step, 
		m_pc2->val_point_step.c_str(), m_pc2->vallen_point_step, (time_t)0, flags);
	m_rc = memcached_set(
		m_memc, m_pc2->key_row_step.c_str(), m_pc2->keylen_row_step, 
		m_pc2->val_row_step.c_str(), m_pc2->vallen_row_step, (time_t)0, flags);
	m_rc = memcached_set(
		m_memc, m_pc2->key_is_dense.c_str(), m_pc2->keylen_is_dense, 
		m_pc2->val_is_dense.c_str(), m_pc2->vallen_is_dense, (time_t)0, flags);

	//ROS_INFO("Info CacheTime: %f", ros::Time::now().toSec());

	// data
	uint32_t block_count = m_pc2->data_block_cnt;

	//std::cout << "Total " << block_count << " Blocks.\n";
	for (uint32_t i = 0; i < block_count; i++)
	{
		std::string key = m_pc2->key_data[i];
		size_t key_len = m_pc2->keylen_data[i];
		uint8_t* val = m_pc2->val_data[i].data();
		size_t val_len = m_pc2->vallen_data[i];

		double before = ros::Time::now().toSec();
		m_rc = memcached_set(m_memc, key.c_str(), key_len, (char*)val, val_len, (time_t)0, flags);
		double dur = ros::Time::now().toSec() - before;
		ROS_INFO("Only Data Cache Cost: %f ms", dur * 1000);

		//if (m_rc == MEMCACHED_SUCCESS)
		//	std::cout << "Setting Succeed!" << std::endl;
		//else
		//	std::cout << "Setting Error: " << memcached_strerror(m_memc, m_rc) << std::endl;
	}
	//ROS_INFO("CacheTime: %f", ros::Time::now().toSec());

#ifdef DEBUG
	Test();
#endif
}

void MemCached_Agnt::Test()
{
	size_t val_len;
	uint32_t flags;

	// header
	char* seq_id = memcached_get(m_memc, m_pc2->key_seq_id.c_str(), m_pc2->keylen_seq_id, &val_len, &flags, &m_rc);
	char* stamp = memcached_get(m_memc, m_pc2->key_stamp.c_str(), m_pc2->keylen_stamp, &val_len, &flags, &m_rc);
	char* frame_id = memcached_get(m_memc, m_pc2->key_frame_id.c_str(), m_pc2->keylen_frame_id, &val_len, &flags, &m_rc);
	// info
	char* width = memcached_get(m_memc, m_pc2->key_width.c_str(), m_pc2->keylen_width, &val_len, &flags, &m_rc);
	char* height = memcached_get(m_memc, m_pc2->key_height.c_str(), m_pc2->keylen_height, &val_len, &flags, &m_rc);
	char* is_bigendian = memcached_get(m_memc, m_pc2->key_is_bigendian.c_str(), m_pc2->keylen_is_bigendian, &val_len, &flags, &m_rc);
	char* point_step = memcached_get(m_memc, m_pc2->key_point_step.c_str(), m_pc2->keylen_point_step, &val_len, &flags, &m_rc);
	char* row_step = memcached_get(m_memc, m_pc2->key_row_step.c_str(), m_pc2->keylen_row_step, &val_len, &flags, &m_rc);
	char* is_dense = memcached_get(m_memc, m_pc2->key_is_dense.c_str(), m_pc2->keylen_is_dense, &val_len, &flags, &m_rc);

	std::cout << seq_id << "," << stamp << "," << frame_id << "," << 
		width << "," << height << "," << is_bigendian  << "," << point_step 
		<< "," << row_step << "," << is_dense << std::endl;

	free(seq_id);
	free(stamp);
	free(frame_id);
	free(width);
	free(height);
	free(is_bigendian);
	free(point_step);
	free(row_step);
	free(is_dense);

	double timenow = ros::Time::now().toSec();
	double dstamp = atof(stamp);
	double delay = timenow - dstamp;
	//ROS_INFO("Delay: %f", delay);
	ROS_INFO("ReadTime: %f", timenow);
	
	// data
	std::vector<uint8_t> all_data;
	for (uint32_t i = 0; i < 2; i++)
	{
		std::ostringstream ss_key;
		ss_key << "data" << i;
		std::string key = ss_key.str();

		char* rslt = memcached_get(m_memc, key.c_str(), key.length(), &val_len, &flags, &m_rc);

		if (m_rc == MEMCACHED_SUCCESS)
		{
			
			std::cout << "Reading Succeed. " << val_len << std::endl;

			uint8_t* arr = new uint8_t[val_len];
			memcpy((uint8_t*)arr, (const uint8_t*)rslt, val_len);
			all_data.insert(all_data.end(), arr, arr + val_len);

			delete [] arr;
			
		}
		else 
		{
			std::cout << "Test Error: " << memcached_strerror(m_memc, m_rc) << std::endl;
		}

		free(rslt);
	}

	size_t total_len = all_data.size();
	std::cout << (int)(all_data[0]) << " " << (int)(all_data[total_len/2]) << " " << (int)(all_data[total_len-1]) << std::endl;
	all_data.clear();
	
}