#ifndef KEYPOINTS_H
#define KEYPOINTS_H


#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/keypoints/iss_3d.h>
#include <iostream>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/features/normal_3d.h>
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<pcl::PointNormal> PointNormalCloudT;

class ISSKeypoints{
	private:
		bool use_multipliers;
		double iss_salient_radius_;
		double iss_non_max_radius_;
		double iss_gamma_21_ ;
		double iss_gamma_32_ ;
		double iss_min_neighbors_ ;
		double iss_normal_radius_ ;
    	double iss_border_radius_ ;
    	
    	double non_max_multiplier;
   		double salient_radius_multiplier;
   		double bord_rad_multiplier;
   		double norm_rad_multiplier;

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
		void setBordRadius(double bordRadius);
		void setNormRadius(double normRadius);
		void setSalRadius(double salRadius);
		
		void setNonMaxMultiplier(double mult);
		void setSalRadMultiplier(double mult);
		void setNormRadMultiplier(double mult);
		void setBordRadMultiplier(double mult);
		
		void setGamma21(float gamma);
		void setGamma32(float gamma);


		PointCloudT::Ptr keypoints ;
		void setNormals(PointNormalCloudT::Ptr Inputcloud);
		void setNormals(pcl::PointCloud<pcl::Normal>::Ptr InputCloud);
		void CalcNormals();
		void ComputeISSKeypoints();
		
		//void DisplayKeypoints(pcl::visualization::PCLVisualizer::Ptr viewer);
		ISSKeypoints () : iss_gamma_21_(0.975),iss_gamma_32_(0.975),non_max_multiplier(4),salient_radius_multiplier(6),
						  bord_rad_multiplier(1),norm_rad_multiplier(4),iss_min_neighbors_(30),iss_threads_(4),normals(new pcl::PointCloud<pcl::Normal>),
						  model (new PointCloudT),keypoints (new PointCloudT),tree (new pcl::search::KdTree<PointT> ()),use_multipliers(true),normals_set(false){}
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