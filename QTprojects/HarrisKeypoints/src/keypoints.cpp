#include "keypoints.h"
#include <pcl/visualization/pcl_visualizer.h>

//typedef pcl::PointXYZRGBA PointT;
//typedef pcl::PointCloud<PointT> PointCloudT;
//typedef pcl::PointCloud<pcl::PointNormal> PointNormalCloudT;

void HarrisKeypoints::setInputCloud(PointCloudT::Ptr inputCloud){
  pcl::copyPointCloud(*inputCloud,*model);
}

void HarrisKeypoints::setThreshold(double threshold){
  har_threshold = threshold;
}

void HarrisKeypoints::setRadius(double radius){
  har_radius = radius;
}

void HarrisKeypoints::setRefine(bool do_refine){
  har_do_refine = do_refine;
}

void HarrisKeypoints::setNormals(pcl::PointCloud<pcl::Normal>::Ptr normalCloud){
  pcl::copyPointCloud(*normalCloud,*normals);
}

void HarrisKeypoints::setNumberOfThreads(int nr_threads){
  har_threads = nr_threads;
}

void HarrisKeypoints::setResponseMethod(std::string resMethod){
  if(resMethod == "HARRIS"){
    hk3d.setMethod(pcl::HarrisKeypoint3d::HARRIS);
  }else if (resMethod == "TOMASI"){
    hk3d.setMethod(pcl::HarrisKeypoint3d::TOMASI);
  }else if (resMethod == "LOWE"){
    hk3d.setMethod(pcl::HarrisKeypoint3d::LOWE);
  }else if (resMethod == "CURVATURE"){
    hk3d.setMethod(pcl::HarrisKeypoint3d::CURVATURE);
  }else if (resMethod == "NOBLE"){
    hk3d.setMethod(pcl::HarrisKeypoint3d::NOBLE);
  }else{
    std::cout<<"Unrecognized response method : "<<resMethod<<" given"<<std::endl;
  }

}
void HarrisKeypoints::computeKeypoints(){
  hk3d.setRadius(har_radius);
  hk3d.setThreshold(har_threshold);
  hk3d.setRefine(har_do_refine);
  if(normals->points.size()!= 0){
    hk3d.setNormals(normals);
  }
  hk3d.setInputCloud(model);
  hk3d.setNumberOfThreads(har_threads);
  hk3d.compute(*keypoints);
}

