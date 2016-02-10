#include "planar_filtering.h"
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

    void Planar_filter::setInputCloud(PointCloudT::Ptr cloud)
    {
        copyPointCloud(*cloud,*_cloud);
        this->Downsample_cloud();
    }
  // Create the filtering object: downsample the dataset using a leaf size of 1cm
    void Planar_filter::Downsample_cloud()
    {
      _vg.setInputCloud (_cloud);
      _vg.setLeafSize (_leaf_size,_leaf_size,_leaf_size);
      _vg.filter (*_cloud);
      std::cout << "PointCloud after filtering has: " << _cloud->points.size ()  << " data points." << std::endl; //*
    }
  // Create the segmentation object for the planar model and set all the parameters
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
      int i=0, nr_points = (int) _cloud->points.size ();
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
        std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;
        extract.setNegative (true);
        extract.filter (*cloud_f);
        *_cloud = *cloud_f;

      // Creating the KdTree object for the search method of the extraction
      pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
      tree->setInputCloud (_cloud);

      pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
      ec.setClusterTolerance (0.02); // 2cm
      ec.setMinClusterSize (100);
      ec.setMaxClusterSize (25000);
      ec.setSearchMethod (tree);
      ec.setInputCloud (_cloud);
      ec.extract (cluster_indices);
      for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
        {
          boost::shared_ptr<PointCloudT> cloud_cluster (new PointCloudT);
          for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            cloud_cluster->points.push_back (_cloud->points[*pit]); //*
          cloud_cluster->width = cloud_cluster->points.size ();
          cloud_cluster->height = 1;
          cloud_cluster->is_dense = true;
          _Segmented_clouds.push_back(cloud_cluster);
        }
    }  
    
    void Planar_filter::Passthrough_filter()
    {
        pass.setInputCloud (_cloud);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (-Z_threshold, Z_threshold);
        //pass.setFilterLimitsNegative (true);
        pass.filter(*_cloud);
    }

  void Planar_filter::setLeafSize(float leaf_size){_leaf_size = leaf_size;}