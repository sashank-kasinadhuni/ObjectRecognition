
void
showHelp (char *filename)
{
  std::cout << std::endl;
  std::cout << "***************************************************************************" << std::endl;
  std::cout << "*                                                                         *" << std::endl;
  std::cout << "*                   Correspondence Grouping - version 2                   *" << std::endl;
  std::cout << "*                                                                         *" << std::endl;
  std::cout << "***************************************************************************" << std::endl << std::endl;
  std::cout << "Usage: " << filename << " model_filename.pcd scene_filename.pcd [Options]" << std::endl << std::endl;
  std::cout << "Options:" << std::endl;
  std::cout << "     -h:                     Show this help." << std::endl;
  std::cout << "     -k:                     Show used keypoints." << std::endl;
  std::cout << "     -c:                     Show used correspondences." << std::endl;
  std::cout << "     -r:                     Compute the model cloud resolution and multiply" << std::endl;
  std::cout << "                             each radius given by that value." << std::endl;
  std::cout << "     --algorithm (Hough|GC): Clustering algorithm used (default Hough)." << std::endl;
  std::cout << "     --model_ss val:         Model uniform sampling radius (default 0.01)" << std::endl;
  std::cout << "     --scene_ss val:         Scene uniform sampling radius (default 0.03)" << std::endl;
  std::cout << "     --rf_rad val:           Reference frame radius (default 0.015)" << std::endl;
  std::cout << "     --descr_rad val:        Descriptor radius (default 0.02)" << std::endl;
  std::cout << "     --cg_size val:          Cluster size (default 0.01)" << std::endl;
  std::cout << "     --cg_thresh val:        Clustering threshold (default 5)" << std::endl;
  std::cout << "this program requires that the model and scene clouds be of the type PointXYZRGBNormal"<<std::cout;
}

double CorrespondenceGrouping::computeCloudResolution (const pcl::PointCloud<PointType>::ConstPtr &cloud)
{
  double res = 0.0;
  int n_points = 0;
  int nres;
  std::vector<int> indices (2);
  std::vector<float> sqr_distances (2);
  pcl::search::KdTree<PointType> tree;
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

void CorrespondenceGrouping::setModelNonMaxSuppMult(double value)
{
    isskModel.setNonMaxSuppMultiplier(value);
}
void CorrespondenceGrouping::setModelSalRadMult(double value)
{
  isskModel.setSalRadMultiplier(value);
}

void CorrespondenceGrouping::setModelNormRadMult(double value){
  isskModel.setNormRadMultiplier(value);
}
void CorrespondenceGrouping::setModelBordRadMult(double value){
  isskModel.setBordRadMultiplier(value);
}
void CorrespondenceGrouping::setModelGamma21(double value){
  isskModel.setGamma21(value);
}
void CorrespondenceGrouping::setModelGamma32(double value){
  isskModel.setGamma32(value);
}
void CorrespondenceGrouping::setModelMinNeighbors(int value){
  issk.setNeighbors(value);
}
void CorrespondenceGrouping::setModelDownsampleLeafSize(double value){
  modelDownsamplingLeafSize = value;
  modelDownsamplingFlag = true;
  vox.setInputCloud(model);
  vox.setLeafSize(value,value,value);
  vox.filter(*modelDownsampled);
}
void CorrespondenceGrouping::setModel(PointCloudT::Ptr modelCloud){
  pcl::copyPointCloud(*modelCloud,*model);
}
void CorrespondenceGrouping::setScene(PointCloudT::Ptr sceneCloud){
  pcl::copyPointCloud(*sceneCloud,*scene);
}

void CorrespondenceGrouping::setSceneNonMaxSuppMult(double value){
  isskScene.setNonMaxSuppMultiplier(value);
}
void CorrespondenceGrouping::setSceneSalRadMult(double value){
  isskScene.setSalRadMultiplier(value);
}
void CorrespondenceGrouping::setSceneNormRadMult(double value){
  isskScene.setNormRadMultiplier(value);
}
void CorrespondenceGrouping::setSceneBordRadMult(double value){
  isskScene.setBordRadMultiplier(value);
}
void CorrespondenceGrouping::setSceneGamma21(double value){
  isskScene.setGamma21(value);
}
void CorrespondenceGrouping::setSceneGamma32(double value){
  isskScene.setGamma32(value);
}
void CorrespondenceGrouping::setSceneMinNeighbors(int value){
  isskScene.setNeighbors(value);
}
void CorrespondenceGrouping::setSceneDownsampleLeafSize(double value){
  sceneDownsamplingLeafSize = value;
  sceneDownsamplingFlag = true;
  vox.setInputCloud(scene);
  vox.setLeafSize(value,value,value);
  vox.filter(*sceneDownsampled); 
}



int
main (int argc, char *argv[])
{
  parseCommandLine (argc, argv);

  //
  //  Load clouds
  //
  if (pcl::io::loadPCDFile (model_filename_, *temp_model) < 0)
  {
    std::cout << "Error loading model cloud." << std::endl;
    showHelp (argv[0]);
    return (-1);
  }
  if (pcl::io::loadPCDFile (scene_filename_, *temp_scene) < 0)
  {
    std::cout << "Error loading scene cloud." << std::endl;
    showHelp (argv[0]);
    return (-1);
  }
  pcl::PassThrough<pcl::PointXYZRGBNormal> pass;
  pass.setInputCloud (temp_scene);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (-Z, Z);
  pass.filter(*temp_scene);

  pcl::copyPointCloud(*temp_scene,*scene);
  pcl::copyPointCloud(*temp_model,*model);
  
  pcl::copyPointCloud(*temp_scene,*scene_normals);
  pcl::copyPointCloud(*temp_model,*model_normals);
  if(scene_normals->points.size() == 0 || model_normals->points.size() == 0){
    //pcl::PointCloud<pcl::Normal> sceneNormalsCalculated (new pcl::PointCloud<pcl::Normal>);
    //pcl::PointCloud<pcl::Normal> modelNormalsCalculated (new pcl::PointCloud<pcl::Normal>);
  pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
  norm_est.setKSearch (10);
  norm_est.setInputCloud (model);
  norm_est.compute (*model_normals);

  norm_est.setInputCloud (scene);
  norm_est.compute (*scene_normals);
    std::cout<<"normals calculation successful"<<std::endl;
    }
  //
  //  Set up resolution invariance
  //
  if (use_cloud_resolution_)
  {
    float resolution = static_cast<float> (computeCloudResolution (model));
    if (resolution != 0.0f)
    {
      model_ss_   *= resolution;
      scene_ss_   *= resolution;
      rf_rad_     *= resolution;
      descr_rad_  *= resolution;
      cg_size_    *= resolution;
    }

    std::cout << "Model resolution:       " << resolution << std::endl;
    std::cout << "Model sampling size:    " << model_ss_ << std::endl;
    std::cout << "Scene sampling size:    " << scene_ss_ << std::endl;
    std::cout << "LRF support radius:     " << rf_rad_ << std::endl;
    std::cout << "SHOT descriptor radius: " << descr_rad_ << std::endl;
    std::cout << "Clustering bin size:    " << cg_size_ << std::endl << std::endl;
  }


  /// Calculate keypoints
  if(keypoints == "iss"){
  issk.setInputCloud(model);
  issk.setNormals(model_normals);
  issk.ComputeISSKeypoints();
  pcl::copyPointCloud(*issk.keypoints,*model_keypoints);

  issk.setInputCloud(scene);
  issk.setNormals(scene_normals);
  issk.ComputeISSKeypoints();
  pcl::copyPointCloud(*issk.keypoints,*scene_keypoints);
}
else if(keypoints == "us"){
  issk.setInputCloud(model);
  issk.setNormals(model_normals);
  issk.ComputeUSKeypoints();
  pcl::copyPointCloud(*issk.keypoints,*model_keypoints);

  issk.setInputCloud(scene);
  issk.setNormals(scene_normals);
  issk.ComputeUSKeypoints();
  pcl::copyPointCloud(*issk.keypoints,*scene_keypoints); 
}

  //
  //  Compute Descriptor for keypoints
  //
  pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
  descr_est.setRadiusSearch (descr_rad_);

  descr_est.setInputCloud (model_keypoints);
  descr_est.setInputNormals (model_normals);
  descr_est.setSearchSurface (model);
  descr_est.compute (*model_descriptors);

  descr_est.setInputCloud (scene_keypoints);
  descr_est.setInputNormals (scene_normals);
  descr_est.setSearchSurface (scene);
  descr_est.compute (*scene_descriptors);

  //
  //  Find Model-Scene Correspondences with KdTree
  //
  pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());

  pcl::KdTreeFLANN<DescriptorType> match_search;
  match_search.setInputCloud (model_descriptors);

  //  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
  for (size_t i = 0; i < scene_descriptors->size (); ++i)
  {
    std::vector<int> neigh_indices (1);
    std::vector<float> neigh_sqr_dists (1);
    if (!pcl_isfinite (scene_descriptors->at (i).descriptor[0])) //skipping NaNs
    {
      continue;
    }
    int found_neighs = match_search.nearestKSearch (scene_descriptors->at (i), 1, neigh_indices, neigh_sqr_dists);
    if(found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
    {
      pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
      model_scene_corrs->push_back (corr);
    }
  }
  std::cout << "Correspondences found: " << model_scene_corrs->size () << std::endl;

  //
  //  Actual Clustering
  //
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
  std::vector<pcl::Correspondences> clustered_corrs;

  //  Using Hough3D
  if (use_hough_)
  {
    //
    //  Compute (Keypoints) Reference Frames only for Hough
    //
    pcl::PointCloud<RFType>::Ptr model_rf (new pcl::PointCloud<RFType> ());
    pcl::PointCloud<RFType>::Ptr scene_rf (new pcl::PointCloud<RFType> ());

    pcl::BOARDLocalReferenceFrameEstimation<PointType, NormalType, RFType> rf_est;
    rf_est.setFindHoles (true);
    rf_est.setRadiusSearch (rf_rad_);

    rf_est.setInputCloud (model_keypoints);
    rf_est.setInputNormals (model_normals);
    rf_est.setSearchSurface (model);
    rf_est.compute (*model_rf);

    rf_est.setInputCloud (scene_keypoints);
    rf_est.setInputNormals (scene_normals);
    rf_est.setSearchSurface (scene);
    rf_est.compute (*scene_rf);

    //  Clustering
    pcl::Hough3DGrouping<PointType, PointType, RFType, RFType> clusterer;
    clusterer.setHoughBinSize (cg_size_);
    clusterer.setHoughThreshold (cg_thresh_);
    clusterer.setUseInterpolation (true);
    clusterer.setUseDistanceWeight (false);

    clusterer.setInputCloud (model_keypoints);
    clusterer.setInputRf (model_rf);
    clusterer.setSceneCloud (scene_keypoints);
    clusterer.setSceneRf (scene_rf);
    clusterer.setModelSceneCorrespondences (model_scene_corrs);

    //clusterer.cluster (clustered_corrs);
    clusterer.recognize (rototranslations, clustered_corrs);
  }
  else // Using GeometricConsistency
  {
    pcl::GeometricConsistencyGrouping<PointType, PointType> gc_clusterer;
    gc_clusterer.setGCSize (cg_size_);
    gc_clusterer.setGCThreshold (cg_thresh_);

    gc_clusterer.setInputCloud (model_keypoints);
    gc_clusterer.setSceneCloud (scene_keypoints);
    gc_clusterer.setModelSceneCorrespondences (model_scene_corrs);

    //gc_clusterer.cluster (clustered_corrs);
    gc_clusterer.recognize (rototranslations, clustered_corrs);
  }

  //
  //  Output results
  //
  std::cout << "Model instances found: " << rototranslations.size () << std::endl;
  for (size_t i = 0; i < rototranslations.size (); ++i)
  {
    std::cout << "\n    Instance " << i + 1 << ":" << std::endl;
    std::cout << "        Correspondences belonging to this instance: " << clustered_corrs[i].size () << std::endl;

    // Print the rotation matrix and translation vector
    Eigen::Matrix3f rotation = rototranslations[i].block<3,3>(0, 0);
    Eigen::Vector3f translation = rototranslations[i].block<3,1>(0, 3);

    printf ("\n");
    printf ("            | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
    printf ("        R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
    printf ("            | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
    printf ("\n");
    printf ("        t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));
  }

  //
  //  Visualization
  //
  pcl::visualization::PCLVisualizer viewer ("Correspondence Grouping");
  viewer.addPointCloud (scene, "scene_cloud");

  pcl::PointCloud<PointType>::Ptr off_scene_model (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<PointType>::Ptr off_scene_model_keypoints (new pcl::PointCloud<PointType> ());

  if (show_correspondences_ || show_keypoints_)
  {
    //  We are translating the model so that it doesn't end in the middle of the scene representation
    pcl::transformPointCloud (*model, *off_scene_model, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));
    pcl::transformPointCloud (*model_keypoints, *off_scene_model_keypoints, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));

    pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_color_handler (off_scene_model, 255, 255, 128);
    viewer.addPointCloud (off_scene_model, off_scene_model_color_handler, "off_scene_model");
  }

  if (show_keypoints_)
  {
    pcl::visualization::PointCloudColorHandlerCustom<PointType> scene_keypoints_color_handler (scene_keypoints, 0, 0, 255);
    viewer.addPointCloud (scene_keypoints, scene_keypoints_color_handler, "scene_keypoints");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_keypoints");

    pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_keypoints_color_handler (off_scene_model_keypoints, 0, 0, 255);
    viewer.addPointCloud (off_scene_model_keypoints, off_scene_model_keypoints_color_handler, "off_scene_model_keypoints");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "off_scene_model_keypoints");
  }

  for (size_t i = 0; i < rototranslations.size (); ++i)
  {
    pcl::PointCloud<PointType>::Ptr rotated_model (new pcl::PointCloud<PointType> ());
    pcl::transformPointCloud (*model, *rotated_model, rototranslations[i]);

    std::stringstream ss_cloud;
    ss_cloud << "instance" << i;

    pcl::visualization::PointCloudColorHandlerCustom<PointType> rotated_model_color_handler (rotated_model, 255, 0, 0);
    viewer.addPointCloud (rotated_model, rotated_model_color_handler, ss_cloud.str ());

    if (show_correspondences_)
    {
      for (size_t j = 0; j < clustered_corrs[i].size (); ++j)
      {
        std::stringstream ss_line;
        ss_line << "correspondence_line" << i << "_" << j;
        PointType& model_point = off_scene_model_keypoints->at (clustered_corrs[i][j].index_query);
        PointType& scene_point = scene_keypoints->at (clustered_corrs[i][j].index_match);

        //  We are drawing a line for each pair of clustered correspondences found between the model and the scene
        viewer.addLine<PointType, PointType> (model_point, scene_point, 0, 255, 0, ss_line.str ());
      }
    }
  }

  while (!viewer.wasStopped ())
  {
    viewer.spinOnce ();
  }

  return (0);
}

