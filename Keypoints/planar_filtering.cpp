#include "planar_filtering.h"
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<pcl::PointNormal> PointNormalCloudT;

  void Planar_filter::setLeafSize(float leaf_size){this->_leaf_size = leaf_size;}
    void Planar_filter::setInputCloud(PointCloudT::Ptr cloud)
    {
        copyPointCloud(*cloud,*_cloud);
        //this->Downsample_cloud();
        
          Downsample_cloud();
    }
    void Planar_filter::setInputCloud(PointNormalCloudT::Ptr cloud,bool _Downsample){
       //std::cout<<"trying to copyPointCloud"<<std::endl;
       pcl::copyPointCloud(*cloud,*_cloud_with_normals);
       if(_Downsample)
       Downsample_cloud("Cloud_With_Normals");
    }
  // Create the filtering object: downsample the dataset using a leaf size of 1cm
    void Planar_filter::Downsample_cloud()
    {
      _vg.setInputCloud (_cloud);
      _vg.setLeafSize (0.01f,0.01f,0.01f);
      PointCloudT::Ptr filtered_cloud(new PointCloudT);
      _vg.filter (*filtered_cloud);
      *_cloud = *filtered_cloud;
      //std::cout << "PointCloud after filtering has: " << _cloud->points.size ()  << " data points." << std::endl; //*
    }
    void Planar_filter::Downsample_cloud(std::string input)
    {
        _vgn.setInputCloud (_cloud_with_normals);
      _vgn.setLeafSize (_leaf_size,_leaf_size,_leaf_size);
      PointNormalCloudT::Ptr filtered_cloud(new PointNormalCloudT);
      _vgn.filter (*filtered_cloud);
      *_cloud_with_normals = *filtered_cloud;
      //std::cout<<"cloud with normals has been downsampled " <<std::endl;
    }
  // Create the segmentation object for the planar model and set all the parameters
    void Planar_filter::Segment_cloud_with_normals(){
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        PointNormalCloudT::Ptr cloud_plane (new PointNormalCloudT ());
        segN.setOptimizeCoefficients (true);
        segN.setModelType (pcl::SACMODEL_PLANE);
        segN.setMethodType (pcl::SAC_RANSAC);
        segN.setMaxIterations (100);
        segN.setDistanceThreshold (0.02);
        std::vector<pcl::PointIndices> cluster_indices;
        segN.setInputCloud (_cloud_with_normals);
        segN.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
          std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
        }
        

        pcl::ExtractIndices<pcl::PointNormal> extract;
        extract.setInputCloud (_cloud_with_normals);
        extract.setIndices (inliers);
        extract.setNegative (false);

        // Get the points associated with the planar surface
        PointNormalCloudT::Ptr cloud_f(new PointNormalCloudT);
        extract.filter (*cloud_plane);
        //std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;
        extract.setNegative (true);
        extract.filter (*cloud_f);
        *_cloud_with_normals = *cloud_f;
        
         //std::cout<<"Cloud with normals now has : "<< _cloud_with_normals->points.size()<<std::endl;
         pcl::search::KdTree<pcl::PointNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointNormal>);
         tree->setInputCloud (_cloud_with_normals);
        
        pcl::EuclideanClusterExtraction<pcl::PointNormal> ec;
        ec.setClusterTolerance (0.02); // 2cm
        ec.setMinClusterSize (100);
        ec.setMaxClusterSize (25000);
        ec.setSearchMethod (tree);
        ec.setInputCloud (_cloud_with_normals);
        ec.extract (cluster_indices);
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
        {
          PointNormalCloudT::Ptr cloud_cluster (new PointNormalCloudT);
          for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            cloud_cluster->points.push_back (_cloud_with_normals->points[*pit]); //*
          cloud_cluster->width = cloud_cluster->points.size();
          cloud_cluster->height = 1;
          cloud_cluster->is_dense = true;
          _Segmented_clouds_with_normals.push_back(cloud_cluster);
        }
    }
    void Planar_filter::Segment_cloud()
    {
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
      PointCloudT::Ptr cloud_plane (new PointCloudT ());
      seg.setOptimizeCoefficients (true);
      seg.setModelType (pcl::SACMODEL_PLANE);
      seg.setMethodType (pcl::SAC_RANSAC);
      seg.setMaxIterations (100);
      seg.setDistanceThreshold (0.02);
      std::vector<pcl::PointIndices> cluster_indices;
        seg.setInputCloud (_cloud);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
          std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
        }

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud (_cloud);
        extract.setIndices (inliers);
        extract.setNegative (false);

        // Get the points associated with the planar surface
        PointCloudT::Ptr cloud_f(new PointCloudT);
        extract.filter (*cloud_plane);
        //std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;
        extract.setNegative (true);
        extract.filter (*cloud_f);
        *_cloud = *cloud_f;

      // Creating the KdTree object for the search method of the extraction
      pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
      tree->setInputCloud (_cloud);

      pcl::EuclideanClusterExtraction<PointT> ec;
      ec.setClusterTolerance (0.02); // 2cm
      ec.setMinClusterSize (100);
      ec.setMaxClusterSize (25000);
      ec.setSearchMethod (tree);
      ec.setInputCloud (_cloud);
      ec.extract (cluster_indices);
      for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
        {
          PointCloudT::Ptr cloud_cluster (new PointCloudT);
          for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            cloud_cluster->points.push_back (_cloud->points[*pit]); //*
          cloud_cluster->width = cloud_cluster->points.size();
          cloud_cluster->height = 1;
          cloud_cluster->is_dense = true;
          _Segmented_clouds.push_back(cloud_cluster);
        }
    }  
    
    void Planar_filter::Passthrough_filter()
    {
        pass.setInputCloud (_cloud);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (-_z_threshold, _z_threshold);
        //pass.setFilterLimitsNegative (true);
        pass.filter(*_cloud);
        //std::cout<<"Points remaining after Passthrough_filter: "<<_cloud->points.size()<<std::endl;
    }
    void Planar_filter::Passthrough_filter(std::string input)
    {
        passN.setInputCloud (_cloud_with_normals);
        passN.setFilterFieldName ("z");
        passN.setFilterLimits (-_z_threshold, _z_threshold);
        //pass.setFilterLimitsNegative (true);
        passN.filter(*_cloud_with_normals);
        //std::cout<<"Points remaining after Passthrough_filter: "<<_cloud->points.size()<<std::endl;
    }

