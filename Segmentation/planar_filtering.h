#ifndef PLANAR_FILTERING_H
#define PLANAR_FILTERING_H

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>
#include <iostream>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
class Planar_filter{
	private:  
    	pcl::VoxelGrid<PointT> _vg;
    	pcl::PointCloud<PointT>::Ptr _cloud;
    	pcl::SACSegmentation<PointT> seg;
    	
    	pcl::PassThrough<PointT> pass;
        float _leaf_size,_z_threshold;
    public:
    	void setInputCloud(pcl::PointCloud<PointT>::Ptr cloud);
        std::vector<boost::shared_ptr<PointCloudT> > _Segmented_clouds;
    	void Downsample_cloud();	
    	void Segment_cloud();
    	std::vector<boost::shared_ptr<PointCloudT> >  getSegmentedClouds(){return _Segmented_clouds;}
    	void setLeafSize(float leaf_size = 0.01f);
        Planar_filter(float Z_theshold):_cloud(new PointCloudT),_leaf_size(0.01f),_z_threshold(Z_theshold){}
        void Passthrough_filter();
};

#endif