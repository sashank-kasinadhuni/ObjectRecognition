#include "recognition_features.h"
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<pcl::PointNormal> PointNormalCloudT;

void Recognizer::setInputCloud(PointNormalCloudT::Ptr cloud){
	pcl::copyPointCloud(*cloud,*_cloud);
	pcl::copyPointCloud(*cloud,*_normals);

}
void Recognizer::setInputCloud(PointNormalCloudT::Ptr cloud,double leaf_size){
	  _vgn.setInputCloud (cloud);
      _vgn.setLeafSize (leaf_size,leaf_size,leaf_size);
      PointNormalCloudT::Ptr filtered_cloud(new PointNormalCloudT);
      _vgn.filter (*filtered_cloud);
      *cloud = *filtered_cloud;
	pcl::copyPointCloud(*cloud,*_cloud);
	pcl::copyPointCloud(*cloud,*_normals);	
}
void Recognizer::Estimate_VFH_Features(){
	_vfh_estimator.setInputCloud (_cloud);
  	_vfh_estimator.setInputNormals (_normals);
  	_vfh_estimator.setSearchMethod (_vfh_tree);
  	_vfh_estimator.compute(*_vfh_features);
}