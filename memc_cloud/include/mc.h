#pragma once

#include <libmemcached/memcached.h>
#include "mc_pointcloud2.h"

class MemCached_Agnt
{
private:
	memcached_st* m_memc;
	memcached_server_st *m_server;
	memcached_return m_rc;

	McPc2* m_pc2;

	static MemCached_Agnt *m_instance;

	MemCached_Agnt();

public:
	~MemCached_Agnt() { memcached_free(m_memc); }

	static MemCached_Agnt *GetInstance();

	void SetFrame(const sensor_msgs::PointCloud2ConstPtr &pc2_msg) { m_pc2->SetFrame(pc2_msg); SavePc2(); }
	void SavePc2();
	void DelPc2() {}

	void Test();
};