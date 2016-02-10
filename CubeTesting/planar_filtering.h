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
#include <pcl/features/integral_image_normal.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<pcl::PointNormal> PointNormalCloudT;
class Planar_filter{
	private:  
    	pcl::VoxelGrid<PointT> _vg;
        pcl::VoxelGrid<pcl::PointNormal> _vgn;
    	pcl::PointCloud<PointT>::Ptr _cloud;
        pcl::PointCloud<pcl::PointNormal>::Ptr _cloud_with_normals;
    	pcl::SACSegmentation<PointT> seg;
        pcl::SACSegmentation<pcl::PointNormal> segN;
        
    	
    	pcl::PassThrough<PointT> pass;
        pcl::PassThrough<pcl::PointNormal> passN;
        float _leaf_size,_z_threshold;
    public:
    	void setInputCloud(pcl::PointCloud<PointT>::Ptr cloud);
        void setInputCloud(PointNormalCloudT::Ptr cloud,bool _Downsample = true);
        void Downsample_cloud(std::string input);
    	void Downsample_cloud();
        void Passthrough_filter(std::string input);	
    	void Segment_cloud();
        void Segment_cloud_with_normals();
        std::vector<PointCloudT::Ptr > _Segmented_clouds;
        std::vector<PointNormalCloudT::Ptr> _Segmented_clouds_with_normals;
    	std::vector<PointCloudT::Ptr >  getSegmentedClouds(){return _Segmented_clouds;}
    	void setLeafSize(float leaf_size);
        Planar_filter(float Z_theshold):_cloud(new PointCloudT),_cloud_with_normals(new PointNormalCloudT),_leaf_size(0.01f),_z_threshold(Z_theshold){}
        void Passthrough_filter();
};

#endif