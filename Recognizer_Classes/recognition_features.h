#ifndef RECOGNITION_FEATURES_H
#define RECOGNITION_FEATURES_H

#include "planar_filtering.h"

#include <pcl/point_cloud.h>
#include <pcl/features/vfh.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <vector>
#include <pcl/filters/voxel_grid.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<pcl::PointNormal> PointNormalCloudT;
class Recognizer{
	private:
	pcl::PointCloud<PointT>::Ptr _cloud;
    pcl::PointCloud<pcl::PointNormal>::Ptr _cloud_with_normals;
    pcl::PointCloud<pcl::Normal>::Ptr _normals;
	pcl::VFHEstimation<pcl::PointXYZ,pcl::Normal, pcl::VFHSignature308> _vfh_estimator;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr _vfh_tree ;
	pcl::VoxelGrid<pcl::PointNormal> _vgn;

	public:
	void setInputCloud(PointNormalCloudT::Ptr cloud);
	void setInputCloud(PointNormalCloudT::Ptr cloud,double leaf_size);
	pcl::PointCloud<pcl::VFHSignature308>::Ptr _vfh_features;
	void Estimate_VFH_Features();
	Recognizer():_cloud(new PointCloudT),_normals(new pcl::PointCloud<pcl::Normal>),_vfh_tree(new pcl::search::KdTree<pcl::PointXYZ> ()),_vfh_features (new pcl::PointCloud<pcl::VFHSignature308> ()){}


};


#endif