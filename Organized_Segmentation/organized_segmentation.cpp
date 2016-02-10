#include <pcl/common/angles.h>
#include <pcl/console/parse.h>
#include "organized_segmentation_demo.h"
#include <pcl/filters/passthrough.h>
//QT4
//#include <QApplication>
// #include <QMutexLocker>
// #include <QEvent>
// #include <QObject>

#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/surface/convex_hull.h>

// #include <vtkRenderWindow.h>
pcl::Grabber* interface;
std::queue<pcl::PointCloud<pcl::PointXYZRGBA> > Q;
OrganizedSegmentationDemo seg_demo;
boost::condition_variable cond;
boost::mutex cloud_mutex;

int normalsratio(200);
bool  disp_norms_ = false;
bool  disp_curv_ = false;
bool  disp_dist_map_ = false;

bool  use_planar_ref_ = true;
bool  use_clust_ = false;
double Z;
void
displayPlanarRegions (std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > &regions, 
                      boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
  char name[1024];
  unsigned char red [6] = {255,   0,   0, 255, 255,   0};
  unsigned char grn [6] = {  0, 255,   0, 255,   0, 255};
  unsigned char blu [6] = {  0,   0, 255,   0, 255, 255};

  pcl::PointCloud<PointT>::Ptr contour (new pcl::PointCloud<PointT>);

  for (size_t i = 0; i < regions.size (); i++)
  {
    Eigen::Vector3f centroid = regions[i].getCentroid ();
    Eigen::Vector4f model = regions[i].getCoefficients ();
    pcl::PointXYZ pt1 = pcl::PointXYZ (centroid[0], centroid[1], centroid[2]);
    pcl::PointXYZ pt2 = pcl::PointXYZ (centroid[0] + (0.5f * model[0]),
                                       centroid[1] + (0.5f * model[1]),
                                       centroid[2] + (0.5f * model[2]));
    sprintf (name, "normal_%d", unsigned (i));
    viewer->addArrow (pt2, pt1, 1.0, 0, 0, false, name);
    
    contour->points = regions[i].getContour ();
    sprintf (name, "plane_%02d", int (i));
    pcl::visualization::PointCloudColorHandlerCustom <PointT> color (contour, red[i%6], grn[i%6], blu[i%6]);
    if(!viewer->updatePointCloud(contour, color, name))
      viewer->addPointCloud (contour, color, name);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, name);
  }
}

void
displayEuclideanClusters (const pcl::PointCloud<PointT>::CloudVectorType &clusters, 
                          boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
  char name[1024];
  unsigned char red [6] = {255,   0,   0, 255, 255,   0};
  unsigned char grn [6] = {  0, 255,   0, 255,   0, 255};
  unsigned char blu [6] = {  0,   0, 255,   0, 255, 255};
  //std::cout<<"found : "<<clusters.size()<<" Clusters"<<std::endl;
  for (size_t i = 0; i < clusters.size (); i++)
  {
    sprintf (name, "cluster_%d" , int (i));
    pcl::visualization::PointCloudColorHandlerCustom<PointT> color0(boost::make_shared<pcl::PointCloud<PointT> >(clusters[i]),red[i%6],grn[i%6],blu[i%6]);
    if (!viewer->updatePointCloud (boost::make_shared<pcl::PointCloud<PointT> >(clusters[i]),color0,name))
      viewer->addPointCloud (boost::make_shared<pcl::PointCloud<PointT> >(clusters[i]),color0,name);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, name);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, name);
  }
}

void
displayCurvature (pcl::PointCloud<PointT>& cloud, pcl::PointCloud<pcl::Normal>& normals, boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
  pcl::PointCloud<pcl::PointXYZRGBA> curvature_cloud = cloud;
  for (size_t i  = 0; i < cloud.points.size (); i++)
  {
    if (normals.points[i].curvature < 0.04)
    {
      curvature_cloud.points[i].r = 0;
      curvature_cloud.points[i].g = 255;
      curvature_cloud.points[i].b = 0;
    }
    else
    {
      curvature_cloud.points[i].r = 255;
      curvature_cloud.points[i].g = 0;
      curvature_cloud.points[i].b = 0;
    }
  }
  
  if (!viewer->updatePointCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA> >(curvature_cloud), "curvature"))
    viewer->addPointCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA> >(curvature_cloud), "curvature");
  
}

void
displayDistanceMap (pcl::PointCloud<PointT>& cloud, float* distance_map, boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
  pcl::PointCloud<pcl::PointXYZRGBA> distance_map_cloud = cloud;
  for (size_t i  = 0; i < cloud.points.size (); i++)
  {
    if (distance_map[i] < 5.0)
    {
      distance_map_cloud.points[i].r = 255;
      distance_map_cloud.points[i].g = 0;
      distance_map_cloud.points[i].b = 0;
    }
    else
    {
      distance_map_cloud.points[i].r = 0;
      distance_map_cloud.points[i].g = 255;
      distance_map_cloud.points[i].b = 0;
    }
  }
  
  if (!viewer->updatePointCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA> >(distance_map_cloud), "distance_map"))
    viewer->addPointCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA> >(distance_map_cloud), "distance_map");
}

void
removePreviousDataFromScreen (size_t prev_models_size, size_t prev_clusters_size, boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
  char name[1024];
  for (size_t i = 0; i < prev_models_size; i++)
  {
    sprintf (name, "normal_%d", unsigned (i));
    viewer->removeShape (name);
    
    sprintf (name, "plane_%02d", int (i));
    viewer->removePointCloud (name);
  }
  
  for (size_t i = 0; i < prev_clusters_size; i++)
  {
    sprintf (name, "cluster_%d", int (i));
    viewer->removePointCloud (name);
  }
}

bool
compareClusterToRegion (pcl::PlanarRegion<PointT>& region, pcl::PointCloud<PointT>& cluster)
{
  Eigen::Vector4f model = region.getCoefficients ();
  pcl::PointCloud<PointT> poly;
  poly.points = region.getContour ();
  
  for (size_t i = 0; i < cluster.points.size (); i++)
  {
    double ptp_dist = fabs (model[0] * cluster.points[i].x +
                            model[1] * cluster.points[i].y +
                            model[2] * cluster.points[i].z +
                            model[3]);
    bool in_poly = pcl::isPointIn2DPolygon<PointT> (cluster.points[i], poly);
    if (in_poly && ptp_dist < 0.02)
      return true;
  }
  return false;
}

bool
comparePointToRegion (PointT& pt, pcl::ModelCoefficients& model, pcl::PointCloud<PointT>& poly)
{
  //bool dist_ok;
  
  double ptp_dist = fabs (model.values[0] * pt.x +
                          model.values[1] * pt.y +
                          model.values[2] * pt.z +
                          model.values[3]);
  if (ptp_dist >= 0.1)
    return (false);
//  else
//    dist_ok = true;

  //project the point onto the plane
  Eigen::Vector3f mc (model.values[0], model.values[1], model.values[2]);
  Eigen::Vector3f pt_vec;
  pt_vec[0] = pt.x;
  pt_vec[1] = pt.y;
  pt_vec[2] = pt.z;
  Eigen::Vector3f projected (pt_vec - mc * float (ptp_dist));
  PointT projected_pt;
  projected_pt.x = projected[0];
  projected_pt.y = projected[1];
  projected_pt.z = projected[2];  

  PCL_INFO ("pt: %lf %lf %lf\n", projected_pt.x, projected_pt.y, projected_pt.z);

  if (pcl::isPointIn2DPolygon (projected_pt, poly))
  {
    PCL_INFO ("inside!\n");
    return true;
  }
  else
  {
    PCL_INFO ("not inside!\n");
    return false;
  }
  
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
OrganizedSegmentationDemo::OrganizedSegmentationDemo ()
{
  previous_data_size_ = 0;
  previous_clusters_size_ = 0;
  data_modified_ = true;


  display_normals_ = disp_norms_;
  display_curvature_ = disp_curv_;
  display_distance_map_ = disp_dist_map_;

  use_planar_refinement_ = use_planar_ref_;
  use_clustering_ = use_clust_;

  // Set up Normal Estimation
  //ne.setNormalEstimationMethod (ne.SIMPLE_3D_GRADIENT);
  ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
  ne.setMaxDepthChangeFactor (0.02f);
  ne.setNormalSmoothingSize (20.0f);

  plane_comparator_.reset (new pcl::PlaneCoefficientComparator<PointT, pcl::Normal> ());
  euclidean_comparator_.reset (new pcl::EuclideanPlaneCoefficientComparator<PointT, pcl::Normal> ());
  rgb_comparator_.reset (new pcl::RGBPlaneCoefficientComparator<PointT, pcl::Normal> ());
  edge_aware_comparator_.reset (new pcl::EdgeAwarePlaneComparator<PointT, pcl::Normal> ());
  euclidean_cluster_comparator_ = pcl::EuclideanClusterComparator<PointT, pcl::Normal, pcl::Label>::Ptr (new pcl::EuclideanClusterComparator<PointT, pcl::Normal, pcl::Label> ());

  // Set up Organized Multi Plane Segmentation
  mps.setMinInliers (10000);
  mps.setAngularThreshold (pcl::deg2rad (3.0)); //3 degrees
  mps.setDistanceThreshold (0.02); //2cm
  

  PCL_INFO ("starting grabber\n");
}

void cloud_cb (const CloudConstPtr& cloud)
{  
  if(Q.empty() && cloud_mutex.try_lock()){
   // locker.lock();
    FPS_CALC ("computation");
    
    //Downsampling
    /*pcl::VoxelGrid<PointT> _vg;
    _vg.setInputCloud (pcloud);
      _vg.setLeafSize (0.01f,0.01f,0.01f);
      pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
      _vg.filter (*cloud);
      cloud->width = 640;
      cloud->height = 480;
      //*_cloud = *filtered_cloud;
    

    //PassThrough filter
    /*pcl::PassThrough<PointT> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (-Z,Z);
        //pass.setFilterLimitsNegative (true);
    pass.filter(*cloud);
*/
    // Estimate Normals
    pcl::PointCloud<pcl::Normal>::Ptr normal_cloud (new pcl::PointCloud<pcl::Normal>);
    seg_demo.ne.setInputCloud (cloud);
    seg_demo.ne.compute (*normal_cloud);
    float* distance_map = seg_demo.ne.getDistanceMap ();
    boost::shared_ptr<pcl::EdgeAwarePlaneComparator<PointT,pcl::Normal> > eapc = boost::dynamic_pointer_cast<pcl::EdgeAwarePlaneComparator<PointT,pcl::Normal> >(seg_demo.edge_aware_comparator_);
    eapc->setDistanceMap (distance_map);
    eapc->setDistanceThreshold (0.01f, false);

    // Segment Planes
    double mps_start = pcl::getTime ();
    std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > regions;
    std::vector<pcl::ModelCoefficients> model_coefficients;
    std::vector<pcl::PointIndices> inlier_indices;  
    pcl::PointCloud<pcl::Label>::Ptr labels (new pcl::PointCloud<pcl::Label>);
    std::vector<pcl::PointIndices> label_indices;
    std::vector<pcl::PointIndices> boundary_indices;
    seg_demo.mps.setInputNormals (normal_cloud);
    seg_demo.mps.setInputCloud (cloud);
    if (seg_demo.use_planar_refinement_)
    {
      seg_demo.mps.segmentAndRefine (regions, model_coefficients, inlier_indices, labels, label_indices, boundary_indices);
    }
    else
    {
      seg_demo.mps.segment (regions);//, model_coefficients, inlier_indices, labels, label_indices, boundary_indices);
    }
    double mps_end = pcl::getTime ();
    std::cout << "MPS+Refine took: " << double(mps_end - mps_start) << std::endl;

    //Segment Objects
    pcl::PointCloud<PointT>::CloudVectorType clusters;

    if (use_clust_ && regions.size () > 0)
    {
      std::vector<bool> plane_labels;
      plane_labels.resize (label_indices.size (), false);
      for (size_t i = 0; i < label_indices.size (); i++)
      {
        if (label_indices[i].indices.size () > 10000)
        {
          plane_labels[i] = true;
        }
      }  
      
      seg_demo.euclidean_cluster_comparator_->setInputCloud (cloud);
      seg_demo.euclidean_cluster_comparator_->setLabels (labels);
      seg_demo.euclidean_cluster_comparator_->setExcludeLabels (plane_labels);
      seg_demo.euclidean_cluster_comparator_->setDistanceThreshold (0.01f, false);

      pcl::PointCloud<pcl::Label> euclidean_labels;
      std::vector<pcl::PointIndices> euclidean_label_indices;
      pcl::OrganizedConnectedComponentSegmentation<PointT,pcl::Label> euclidean_segmentation (seg_demo.euclidean_cluster_comparator_);
      euclidean_segmentation.setInputCloud (cloud);
      euclidean_segmentation.segment (euclidean_labels, euclidean_label_indices);
      
      for (size_t i = 0; i < euclidean_label_indices.size (); i++)
      {
        if (euclidean_label_indices[i].indices.size () > 1000)
        {
          pcl::PointCloud<PointT> cluster;
          pcl::copyPointCloud (*cloud,euclidean_label_indices[i].indices,cluster);
          clusters.push_back (cluster);
        }    
      }
      
      std::cout<<"Got "<< clusters.size ()<<"euclidean clusters!\n"<<std::endl;
    }          
   
      //QMutexLocker vis_locker (&vis_mtx_);
      seg_demo.prev_cloud_ = *cloud;
      seg_demo.prev_normals_ = *normal_cloud;
      seg_demo.prev_regions_ = regions;
      seg_demo.prev_distance_map_ = distance_map;
      seg_demo.prev_clusters_ = clusters;
      seg_demo.data_modified_ = true;
      Q.push(*cloud);
      cond.notify_one();
      cloud_mutex.unlock();
  }
}

void visualize(){
  std::cout<<"Child thread started execution"<<std::endl;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> vis_(new pcl::visualization::PCLVisualizer);
    //vis_.reset (new pcl::visualization::PCLVisualizer ("", false));
    interface = new pcl::OpenNIGrabber();
   boost::function<void (const CloudConstPtr&)> f = boost::bind (&cloud_cb,_1);
   //boost::signals2::connection c = grabber_.registerCallback(f);
    interface->registerCallback (f);
    //OrganizedSegmentationDemo seg_demo (argc,argv);
    interface->start();
    while (!vis_->wasStopped()){
      vis_->spinOnce();
      boost::unique_lock<boost::mutex> locker(cloud_mutex);
      while(Q.empty())cond.wait(locker);
      pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
      pcl::copyPointCloud(Q.front(),*cloud);
      Q.pop();
      vis_->removeAllPointClouds();
      pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(cloud);
      vis_->addPointCloud<pcl::PointXYZRGBA> (cloud, rgb, "sample cloud");
      displayEuclideanClusters(seg_demo.prev_clusters_,vis_);
      locker.unlock();
    }    
  interface->stop();
}
int
main (int argc, char ** argv)
{ 
  if (pcl::console::find_switch (argc, argv, "-dn"))
    disp_norms_ = true;
  if (pcl::console::find_switch (argc, argv, "-dc"))
    disp_curv_ = true;
  if (pcl::console::find_switch (argc, argv, "-ddm"))
    disp_dist_map_ = true;
  if (pcl::console::find_switch(argc, argv, "-upr"))
    use_planar_ref_ = true;
  if (pcl::console::find_switch(argc, argv, "-uc"))
    use_clust_ = true;
  boost::thread worker_thread(visualize);
  /*if (pcl::console::find_switch (argc, argv, "--Z"))
    pcl::console::parse_argument (argc, argv, "--Z", Z);
  else{
    std::cout<<"no Z threshold found"<<std::endl;
  }*/
  worker_thread.join();
}