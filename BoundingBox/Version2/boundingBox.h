#ifndef BOUNDINGBOX_H
#define BOUNDINGBOX_H

#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class BoundingBox{
	private:
		PointCloudT::Ptr cloud;
		pcl::MomentOfInertiaEstimation <PointT> feature_extractor;
		std::vector <float> moment_of_inertia;
  		std::vector <float> eccentricity;
  		PointT min_point_AABB;
  		PointT max_point_AABB;
  		PointT min_point_OBB;
  		PointT max_point_OBB;
  		PointT position_OBB;
  		Eigen::Matrix3f rotational_matrix_OBB;
  		float major_value, middle_value, minor_value;
  		Eigen::Vector3f major_vector, middle_vector, minor_vector;
  		Eigen::Vector3f mass_center;
	public:
		BoundingBox();
		void setInputCloud(PointCloudT::Ptr input_cloud)
		{
			pcl::copyPointCloud(*input_cloud,*cloud);
			initializeExtractor();
		}
		void calculate_OBB();
		void calculate_AABB();
		void show_OBB(pcl::visualization::PCLVisualizer * viewer);
		void show_AABB(pcl::visualization::PCLVisualizer * viewer);
		std::vector<float> GetDimensions();
		void initializeExtractor();
};

#endif