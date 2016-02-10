#include "keypoints.h"
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<pcl::PointNormal> PointNormalCloudT;
typedef pcl::PointCloud<pcl::PointXYZRGBNormal> PointColorNormalCloudT;
double Keypoints::computeCloudResolution (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
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

void Keypoints::setInputCloud(PointCloudT::Ptr InputCloud){
    pcl::copyPointCloud(*InputCloud,*model);
}
void Keypoints::setInputCloud(PointNormalCloudT::Ptr InputCloud){
    pcl::copyPointCloud(*InputCloud,*model);
}
void Keypoints::setInputCloud(PointColorNormalCloudT::Ptr InputCloud){
  pcl::copyPointCloud(*InputCloud,*model); 
}

void Keypoints::setNormals(pcl::PointCloud<pcl::Normal>::Ptr inputNormals){
  iss_detector.setNormals(inputNormals);
}
void Keypoints::ComputeISSKeypoints(){
    double model_resolution= static_cast<double>(computeCloudResolution(model));
    iss_salient_radius_ = 6 * model_resolution;
    iss_non_max_radius_ = 4 * model_resolution;
    iss_normal_radius_ = 4 * model_resolution;
    iss_border_radius_ = 1 * model_resolution;
    iss_detector.setSearchMethod (tree);
    iss_detector.setSalientRadius (iss_salient_radius_);
    iss_detector.setNonMaxRadius (iss_non_max_radius_);
    
    iss_detector.setNormalRadius (iss_normal_radius_);
    iss_detector.setBorderRadius (iss_border_radius_);

    iss_detector.setThreshold21 (iss_gamma_21_);
    iss_detector.setThreshold32 (iss_gamma_32_);
    
    iss_detector.setMinNeighbors (iss_min_neighbors_);
    iss_detector.setNumberOfThreads (iss_threads_);
    iss_detector.setInputCloud (model);
    iss_detector.compute (*keypoints);

}

void Keypoints::ComputeUSKeypoints(){
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
/*void ISSKeypoints::DisplayKeypoints(pcl::visualization::PCLVisualizer::Ptr viewer){
    //viewer->addPointCloud (model, "model_cloud");
    //  viewer.addPointCloud (model, "off_scene_model");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> model_keypoints_color_handler (model_keypoints, 0, 255, 0);
    viewer->addPointCloud (keypoints, keypoints_color_handler, "model_keypoints");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "model_keypoints");
}*/
