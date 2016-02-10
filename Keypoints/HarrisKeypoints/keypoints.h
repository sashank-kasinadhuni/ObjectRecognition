#ifndef KEYPOINTS_H
#define KEYPOINTS_H


#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/keypoints/iss_3d.h>
#include <iostream>
#include <pcl/keypoints/uniform_sampling.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<pcl::PointNormal> PointNormalCloudT;

class Keypoints{
	private:
		double iss_salient_radius_;
		double iss_non_max_radius_;
		double iss_gamma_21_ ;
		double iss_gamma_32_ ;
		double iss_min_neighbors_ ;
		double iss_normal_radius_ ;
    	double iss_border_radius_ ;
		double us_model_ss_ ;
		int iss_threads_ ;
		PointCloudT::Ptr model ;
		pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree ;
		pcl::ISSKeypoint3D<PointT, PointT> iss_detector;
		pcl::UniformSampling<PointT> uniform_sampling;
	public:
		double computeCloudResolution (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud);
		void setInputCloud(PointCloudT::Ptr inputCloud);
		void setInputCloud(PointNormalCloudT::Ptr InputCloud);
		PointCloudT::Ptr keypoints ;
		void ComputeISSKeypoints();
		void ComputeUSKeypoints();
		//void DisplayKeypoints(pcl::visualization::PCLVisualizer::Ptr viewer);
		Keypoints () : iss_gamma_21_(0.90),iss_gamma_32_(0.90),iss_min_neighbors_(10),iss_threads_(4),us_model_ss_(0.02),model (new PointCloudT),keypoints (new PointCloudT),tree (new pcl::search::KdTree<PointT> ()){}
};

#endif