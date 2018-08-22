#pragma once

#define NOMINMAX
#include "LMS400.h"

#include <cmath>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>


#include <boost\timer.hpp>

#include <vector>
#include <queue>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<pcl::PointNormal> PointCloudN;

class LaserScan2PCL {
public:
	LaserScan2PCL(std::string fileName);
	~LaserScan2PCL();
	
	void SaveClud();

	void setTransform(double *pose);

	void spin();

	PointCloudN::Ptr m_cloud_with_normals;

	LMS400 *m_lms;

private:
	std::string m_FileName;

	Eigen::Isometry3d m_transform; //����任����
	Eigen::Isometry3d m_trans_fixed; //���̶�����ϵ����
	
	boost::shared_ptr<pcl::visualization::CloudViewer> m_viewer;// ���ӻ�����

	int m_startPointIndex; // ��ʼ���±�
	int m_endPointIndex;   // �������±�

	double m_max_distance; //����������
	double m_min_distance; //��С��������

	bool Inver; //�����Ƿ�����

	PointCloudN::Ptr m_new_cloud;
	PointCloudT::Ptr m_show_cloud;

	// ��������ת��Ϊ����
	PointCloudN::Ptr laserScan2PointCloud(LaserScan& laserScan);
	void joinPointCloud(PointCloudN::Ptr original, PointCloudT::Ptr pointShow, PointCloudN::Ptr newCloud, Eigen::Isometry3d transform);

	//���̻ص�����
	void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* nothing);
};

