#ifndef RECOGNITION_FEATURES_H
#define RECOGNITION_FEATURES_H

#include "planar_filtering.h"
#include <math.h>
#include <pcl/point_cloud.h>
#include <pcl/features/vfh.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <vector>
#include <pcl/filters/voxel_grid.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<pcl::PointNormal> PointNormalCloudT;
class Recognizer{
	private:
	pcl::PointCloud<PointT>::Ptr _cloud;
    pcl::PointCloud<pcl::PointNormal>::Ptr _cloud_with_normals;
    pcl::PointCloud<pcl::Normal>::Ptr _normals;
	pcl::VFHEstimation<PointT,pcl::Normal, pcl::VFHSignature308> _vfh_estimator;
	pcl::search::KdTree<PointT>::Ptr _vfh_tree ;
	pcl::VoxelGrid<pcl::PointNormal> _vgn;

	public:
	void setInputCloud(PointNormalCloudT::Ptr cloud);
	void setInputCloud(PointNormalCloudT::Ptr cloud,double leaf_size);
	PointCloudT::Ptr getCloud(){return _cloud;}
	PointNormalCloudT::Ptr getPointNormalCloud() {return _cloud_with_normals;}
	pcl::PointCloud<pcl::Normal>::Ptr getNormalCloud(){return _normals;}
	pcl::PointCloud<pcl::VFHSignature308>::Ptr _vfh_features;
	double* _rv_features_181;
	pcl::Normal rv_resultant;
	PointT rv_centroid;
	void Estimate_VFH_Features();
	void Estimate_RV_Features();
	Recognizer():_cloud(new PointCloudT),_normals(new pcl::PointCloud<pcl::Normal>),_cloud_with_normals(new PointNormalCloudT),_vfh_tree(new pcl::search::KdTree<PointT> ()),_vfh_features (new pcl::PointCloud<pcl::VFHSignature308> ()){_rv_features_181 = new double[181];}
	~Recognizer(){delete _rv_features_181;}
};


#endif