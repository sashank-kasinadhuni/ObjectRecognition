#include "recognition_features.h"
#define M_PI           3.14159265358979323846  /* pi */
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<pcl::PointNormal> PointNormalCloudT;

void Recognizer::setInputCloud(PointNormalCloudT::Ptr cloud){
	pcl::copyPointCloud(*cloud,*_cloud);
	pcl::copyPointCloud(*cloud,*_normals);
  pcl::copyPointCloud(*cloud,*_cloud_with_normals);

}
void Recognizer::setInputCloud(PointNormalCloudT::Ptr cloud,double leaf_size){
	  _vgn.setInputCloud (cloud);
      _vgn.setLeafSize (leaf_size,leaf_size,leaf_size);
      PointNormalCloudT::Ptr filtered_cloud(new PointNormalCloudT);
      _vgn.filter (*filtered_cloud);
      *cloud = *filtered_cloud;
	pcl::copyPointCloud(*cloud,*_cloud);
	pcl::copyPointCloud(*cloud,*_normals);
  pcl::copyPointCloud(*cloud,*_cloud_with_normals);	
}
void Recognizer::Estimate_RV_Features(){
    rv_centroid.x = 0;
    rv_centroid.y = 0;
    rv_centroid.z = 0;
    rv_resultant.normal_x = 0;
    rv_resultant.normal_y = 0;
    rv_resultant.normal_z = 0;
    int valid_points =0;
    int valid_normals = 0;
    for(int i =0;i<_cloud_with_normals->points.size();i++){

        if(!isnan(_cloud_with_normals->points[i].x)){
          rv_centroid.x += _cloud_with_normals->points[i].x;
          rv_centroid.y += _cloud_with_normals->points[i].y;
          rv_centroid.z += _cloud_with_normals->points[i].z;
          valid_points+=1;
        }
        if(!isnan(_cloud_with_normals->points[i].normal_x)){
          rv_resultant.normal_x += _cloud_with_normals->points[i].normal_x;
          rv_resultant.normal_y += _cloud_with_normals->points[i].normal_y;
          rv_resultant.normal_z += _cloud_with_normals->points[i].normal_z;
          valid_normals+=1;
        }
    }
    rv_centroid.x /= valid_points;
    rv_centroid.y /= valid_points;
    rv_centroid.z /= valid_points;
    double res_mod = sqrt(pow(rv_resultant.normal_x,2)+pow(rv_resultant.normal_y,2)+pow(rv_resultant.normal_z,2));
    rv_resultant.normal_x /= res_mod;
    rv_resultant.normal_y /= res_mod;
    rv_resultant.normal_z /= res_mod;
    for(int i = 0;i<_cloud_with_normals->points.size();i++){
        if(!isnan(_cloud_with_normals->points[i].normal_x)){
            double dot_product = (rv_resultant.normal_x) * (_cloud_with_normals->points[i].normal_x) +
                                 (rv_resultant.normal_y) * (_cloud_with_normals->points[i].normal_y) +
                                 (rv_resultant.normal_z) * (_cloud_with_normals->points[i].normal_z);
            double a_mag = sqrt(pow(rv_resultant.normal_x,2)+pow(rv_resultant.normal_y,2)+pow(rv_resultant.normal_z,2));
            double b_mag = sqrt(pow(_cloud_with_normals->points[i].normal_x,2)+pow(_cloud_with_normals->points[i].normal_y,2)+pow(_cloud_with_normals->points[i].normal_z,2));
            int angle = acos(dot_product/(a_mag*b_mag))*(180/M_PI);
            _rv_features_181[angle] += 1;
        }
    }
}

void Recognizer::Estimate_VFH_Features(){
	_vfh_estimator.setInputCloud (_cloud);
  	_vfh_estimator.setInputNormals (_normals);
  	_vfh_estimator.setSearchMethod (_vfh_tree);
  	_vfh_estimator.compute(*_vfh_features);
}