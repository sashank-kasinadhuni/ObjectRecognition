#include "keypoints.h"
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<pcl::PointNormal> PointNormalCloudT;

double ISSKeypoints::computeCloudResolution (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
{
  double res = 0.0;
  int n_points = 0;
  int nres;
  std::vector<int> indices (2);
  std::vector<float> sqr_distances (2);
  pcl::search::KdTree<pcl::PointXYZRGBA> tree;
  tree.setInputCloud (cloud);

  for (size_t i = 0; i < cloud->size (); ++i)
  {
    if (! pcl_isfinite ((*cloud)[i].x))
    {
      continue;
    }
    //Considering the second neighbor since the first is the point itself.
    nres = tree.nearestKSearch (i, 2, indices, sqr_distances);
    if (nres == 2)
    {
      res += sqrt (sqr_distances[1]);
      ++n_points;
    }
  }
  if (n_points != 0)
  {
    res /= n_points;
  }
  return res;
}

void ISSKeypoints::setInputCloud(PointCloudT::Ptr InputCloud){
    pcl::copyPointCloud(*InputCloud,*model);
}
void ISSKeypoints::setInputCloud(PointNormalCloudT::Ptr InputCloud){
    pcl::copyPointCloud(*InputCloud,*model);
}
void ISSKeypoints::setNonMaxMultiplier(int mult){
  this->non_max_multiplier = mult;
}
void ISSKeypoints::setSalRadMultiplier(int mult){
  this->salient_radius_multiplier= mult;
}

void ISSKeypoints::ComputeISSKeypoints(){
    double model_resolution= static_cast<double>(computeCloudResolution(model));
      iss_salient_radius_ = salient_radius_multiplier * model_resolution;
      iss_non_max_radius_ =  non_max_multiplier* model_resolution;
    iss_detector.setSalientRadius (iss_salient_radius_);  
    iss_detector.setNonMaxRadius (iss_non_max_radius_);

    iss_detector.setSearchMethod (tree);
    iss_detector.setThreshold21 (iss_gamma_21_);
    iss_detector.setThreshold32 (iss_gamma_32_);
    
    iss_detector.setMinNeighbors (iss_min_neighbors_);
    iss_detector.setNumberOfThreads (iss_threads_);
    iss_detector.setInputCloud (model);
    iss_detector.compute (*keypoints);

}
void ISSKeypoints::setNeighbors(double _min_neighbors){
    iss_min_neighbors_ = _min_neighbors; 
    iss_detector.setMinNeighbors( iss_min_neighbors_);
}
void ISSKeypoints::setNonMaxRadius(double _nonmax_radius){
  iss_non_max_radius_= _nonmax_radius;
  iss_detector.setNonMaxRadius(iss_non_max_radius_);
}

void ISSKeypoints::setNormals(PointNormalCloudT::Ptr InputCloud){
  pcl::copyPointCloud(*InputCloud,*normals);
  iss_detector.setNormals(normals);
}



////////////////////////////////////////////////////////
//  Uniform Sampling keypoints Class Definitions below//
////////////////////////////////////////////////////////


void USKeypoints::ComputeUSKeypoints(){
  double resolution = static_cast<double> (computeCloudResolution (model));
  std::cout<<"model ss before is : "<<us_model_ss_<<std::endl;
  /*if (resolution != 0.0f)
  {
    us_model_ss_   *= resolution;
  }*/
  std::cout<<"the model ss is : "<<us_model_ss_<<"with the cloud resolution being :"<<resolution<<std::endl;
  pcl::PointCloud<int> sampled_indices;
  uniform_sampling.setInputCloud (model);
  uniform_sampling.setRadiusSearch (us_model_ss_);
  uniform_sampling.compute (sampled_indices);
  pcl::copyPointCloud (*model, sampled_indices.points, *keypoints);
} 
double USKeypoints::computeCloudResolution(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud){
  double res = 0.0;
  int n_points = 0;
  int nres;
  std::vector<int> indices (2);
  std::vector<float> sqr_distances (2);
  pcl::search::KdTree<pcl::PointXYZRGBA> tree;
  tree.setInputCloud (cloud);

  for (size_t i = 0; i < cloud->size (); ++i)
  {
    if (! pcl_isfinite ((*cloud)[i].x))
    {
      continue;
    }
    //Considering the second neighbor since the first is the point itself.
    nres = tree.nearestKSearch (i, 2, indices, sqr_distances);
    if (nres == 2)
    {
      res += sqrt (sqr_distances[1]);
      ++n_points;
    }
  }
  if (n_points != 0)
  {
    res /= n_points;
  }
  return res;
}

void USKeypoints::setInputCloud(PointNormalCloudT::Ptr InputCloud){
  pcl::copyPointCloud(*InputCloud,*model);
  pcl::copyPointCloud(*InputCloud,*normals);
}

