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

class ISSKeypoints{
	private:
		double iss_salient_radius_;
		double iss_non_max_radius_;
		double iss_gamma_21_ ;
		double iss_gamma_32_ ;
		double iss_min_neighbors_ ;
		double iss_normal_radius_ ;
    	double iss_border_radius_ ;
    	int non_max_multiplier;
    	int salient_radius_multiplier;
		int iss_threads_ ;
		PointCloudT::Ptr model ;
		pcl::PointCloud<pcl::Normal>::Ptr normals;
		pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree ;
		pcl::ISSKeypoint3D<PointT, PointT> iss_detector;

	public:
		double computeCloudResolution (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud);
		void setInputCloud(PointCloudT::Ptr inputCloud);
		void setInputCloud(PointNormalCloudT::Ptr InputCloud);
		void setNeighbors(double _min_neighbors);
		void setNonMaxRadius(double _nonmax_radius);
		void setNonMaxMultiplier(int mult);
		void setSalRadMultiplier(int mult);
		PointCloudT::Ptr keypoints ;
		void setNormals(PointNormalCloudT::Ptr Inputcloud);
		void ComputeISSKeypoints();
		
		//void DisplayKeypoints(pcl::visualization::PCLVisualizer::Ptr viewer);
		ISSKeypoints () : iss_gamma_21_(0.975),iss_gamma_32_(0.975),non_max_multiplier(4),salient_radius_multiplier(6),iss_min_neighbors_(30),iss_threads_(4),normals(new pcl::PointCloud<pcl::Normal>),model (new PointCloudT),keypoints (new PointCloudT),tree (new pcl::search::KdTree<PointT> ()){}
};

class USKeypoints{
private:
	double us_model_ss_ ;
	pcl::UniformSampling<PointT> uniform_sampling;
	PointCloudT::Ptr model ;
	pcl::PointCloud<pcl::Normal>::Ptr normals;
	pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree ;
public:
	double computeCloudResolution (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud);
	void setInputCloud(PointCloudT::Ptr inputCloud);
	void ComputeUSKeypoints();
	void setInputCloud(PointNormalCloudT::Ptr InputCloud);
	PointCloudT::Ptr keypoints ;
	USKeypoints () : us_model_ss_(0.02),normals(new pcl::PointCloud<pcl::Normal>),model (new PointCloudT),keypoints (new PointCloudT),tree (new pcl::search::KdTree<PointT> ()){}
};
#endif