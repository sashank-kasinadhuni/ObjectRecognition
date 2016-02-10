#ifndef KEYPOINTS_H
#define KEYPOINTS_H


#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/keypoints/iss_3d.h>
#include <iostream>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/keypoints/harris_3d.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<pcl::PointNormal> PointNormalCloudT;

class HarrisKeypoints{
private:
	PointCloudT::Ptr model;
	pcl::PointCloud<pcl::Normal>::Ptr normals;
	double har_radius;
	double har_threshold;
	int har_threads;
	bool har_do_refine;
	
	pcl::HarrisKeypoint3D<PointT,PointT,pcl::Normal> hk3d;

public:
	void setInputCloud(PointCloudT::Ptr inputCloud);
	void setRadius(double radius); //radius for normal estimation and non maxima suppression
	void setThreshold (double threshold); //threshold for detecting corners
	void setRefine(bool do_refine); // should the keypoints be refined
	void setNormals(pcl::PointCloud<pcl::Normal>::Ptr normalCloud);// input normals cloud
	void setNumberOfThreads(int nr_threads);
	void setResponseMethod(std::string resMethod);
	void computeKeypoints();
	PointCloudT::Ptr keypoints;
	HarrisKeypoints():har_threads(4),normals(new pcl::PointCloud<pcl::Normal>),model(new PointCloudT),har_radius(0.01),har_threshold(0.0),har_do_refine(false),keypoints(new PointCloudT){}

};
#endif