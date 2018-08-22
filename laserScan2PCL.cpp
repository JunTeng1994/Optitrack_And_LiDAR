#include "laserScan2PCL.h"


LaserScan2PCL::LaserScan2PCL(std::string fileName) :
m_startPointIndex(70),
m_endPointIndex(130),
m_min_distance(1.2),
m_max_distance(1.4),
Inver(true)
{
	m_FileName.assign(fileName);

	m_lms = new LMS400();

	m_transform = Eigen::Isometry3d::Identity();
	m_trans_fixed.matrix() << 0, -1, 0, 0,
		-1, 0, 0, 0,
		0, 0, -1, -0.1085,
		0, 0, 0, 1;
	m_new_cloud = PointCloudN::Ptr(new PointCloudN);
	m_cloud_with_normals = PointCloudN::Ptr(new PointCloudN);
	m_show_cloud = PointCloudT::Ptr(new PointCloudT);
	m_viewer.reset(new pcl::visualization::CloudViewer("PCL Windows"));
	m_viewer->registerKeyboardCallback(&LaserScan2PCL::keyboardEventOccurred, *this);
}

LaserScan2PCL::~LaserScan2PCL(){
	delete m_lms;
}

PointCloudN::Ptr LaserScan2PCL::laserScan2PointCloud(LaserScan& laserScan)
{
	PointCloudN::Ptr cloud(new PointCloudN());
	pcl::PointNormal p;
	for (int i = m_startPointIndex; i < m_endPointIndex; i++){
		if (laserScan[i].getY() > m_min_distance && laserScan[i].getY() < m_max_distance){
			p.z = 0.0;
			p.x = laserScan[i].getX();
			p.y = laserScan[i].getY();
			//std::cout << p.x << "      " << p.y << std::endl;
			p.normal_z = 0.0;
			p.normal_x = -laserScan[i].getX();
			p.normal_y = -laserScan[i].getY();
			cloud->push_back(p);
		}
	}
	return cloud;
}


void LaserScan2PCL::joinPointCloud(PointCloudN::Ptr original, PointCloudT::Ptr pointShow, PointCloudN::Ptr newCloud, Eigen::Isometry3d transform)
{
	//std::cout << "size of cloud_original:" << original->size() << std::endl;
	//std::cout << "size of cloud_newcloud:" << newCloud->size() << std::endl;
	pcl::transformPointCloudWithNormals(*newCloud, *newCloud, transform.matrix());
	*original += *newCloud;

	PointT p;
	for (int i = 0; i < newCloud->size(); i++){
		p.x = newCloud->points[i].x;
		p.y = newCloud->points[i].y;
		p.z = newCloud->points[i].z;
		pointShow->push_back(p);
	}

	/*static pcl::VoxelGrid<PointT> voxel;
	voxel.setLeafSize(0.02, 0.02, 0.02);
	voxel.setInputCloud(output);
	PointCloud::Ptr tmp(new PointCloud());
	voxel.filter(*tmp);*/
}

void LaserScan2PCL::SaveClud()
{
	pcl::io::savePCDFile(m_FileName, *m_cloud_with_normals);
}

void LaserScan2PCL::setTransform(double *pose)
{
	Eigen::Quaterniond Q(pose[6], pose[3], pose[4], pose[5]);
	Eigen::Matrix3d R = Q.normalized().toRotationMatrix();
	Eigen::Vector3d t(pose[0], pose[1], pose[2]);

	if (Inver){
		m_transform.matrix().block(0, 0, 3, 3) = R.transpose();
		m_transform.matrix().block(0, 3, 3, 1) = -R.transpose()*t;
	}
	else{
		m_transform.matrix().block(0, 0, 3, 3) = R;
		m_transform.matrix().block(0, 3, 3, 1) = t;
	}
}

void LaserScan2PCL::spin()
{
	LaserScan laserScan;
	laserScan = m_lms->getLaserScan();
	m_new_cloud = laserScan2PointCloud(laserScan);
	Eigen::Isometry3d trans_temp = Eigen::Isometry3d::Identity();
	trans_temp.matrix() = m_transform.matrix()*m_trans_fixed.matrix();
	joinPointCloud(m_cloud_with_normals, m_show_cloud, m_new_cloud, trans_temp);
	m_viewer->showCloud(m_show_cloud);
}

void LaserScan2PCL::keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* nothing){
	
	if (event.getKeySym() == "s" && event.keyDown()){
		std::cout << "Save Cloud with normal information" << std::endl;
		pcl::io::savePCDFile(m_FileName, *m_cloud_with_normals);
	}
}
