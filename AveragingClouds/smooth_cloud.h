#ifndef AVERAGE_CLOUD_H
#define AVERAGE_CLOUD_H

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/features/normal_3d.h>
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<pcl::PointNormal> PointNormalCloudT;
typedef pcl::PointCloud<pcl::PointXYZRGBNormal> PointColorNormalCloudT;

class SmoothCloud{
	private:
		int acc_size;
		PointColorNormalCloudT::Ptr res_cloud;
		PointCloudT::Ptr res_color_cloud;
		std::vector<PointColorNormalCloudT::Ptr> accumulator;
	public:
		int GetAccumulatorSize(){return accumulator.size();}
		void setInputCloud(PointColorNormalCloudT::Ptr inputCloud);
		void CloudSizes();
		void ProcessClouds();
		PointColorNormalCloudT::Ptr GetCloud(){return res_cloud;}
		bool ready;
		SmoothCloud(int accmulator_size):acc_size(accmulator_size),res_cloud(new PointColorNormalCloudT),ready(false){
			std::cout<<"the accumulator size is set to"<<acc_size<<std::endl;
		}
};

#endif