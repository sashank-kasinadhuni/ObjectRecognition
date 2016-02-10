#ifndef KEYPOINTS_H
#define KEYPOINTS_H


#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/keypoints/iss_3d.h>
#include <iostream>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<pcl::PointNormal> PointNormalCloudT;

class ISSKeypoints{
	private:
		bool iss_use_multipliers;
		bool normals_set;
		double iss_salient_radius_;
		double iss_non_max_radius_;
		double iss_gamma_21_ ;
		double iss_gamma_32_ ;
		double iss_min_neighbors_ ;
		double iss_normal_radius_ ;
    	double iss_border_radius_ ;
    	
    	double iss_non_max_multiplier_;
   		double iss_sal_rad_multiplier_;
   		double iss_bord_rad_multiplier_;
   		double iss_norm_rad_multiplier_;

   		double iss_cloud_resolution_;
		int iss_threads_ ;
		PointCloudT::Ptr model ;
		pcl::PointCloud<pcl::Normal>::Ptr normals;
		pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree ;
		pcl::ISSKeypoint3D<PointT, PointT> iss_detector;

	public:
		double computeCloudResolution (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud);
		void setInputCloud(PointCloudT::Ptr inputCloud);
		void setInputCloud(PointNormalCloudT::Ptr InputCloud);
		
		void ShowParameters();
		void setNeighbors(double _min_neighbors);

		void setNonMaxRadius(double _nonmax_radius);
		void setBordRadius(double bordRadius);
		void setNormRadius(double normRadius);
		void setSalRadius(double salRadius);
		
		void setNonMaxMultiplier(double mult);
		void setSalRadMultiplier(double mult);
		void setNormRadMultiplier(double mult);
		void setBordRadMultiplier(double mult);
		
		void setGamma21(double gamma);
		void setGamma32(double gamma);

		double GetModelResolution();

		void setUseMultipliers(bool value);
		PointCloudT::Ptr keypoints ;
		void setNormals(PointNormalCloudT::Ptr Inputcloud);
		void setNormals(pcl::PointCloud<pcl::Normal>::Ptr InputCloud);
		void CalcNormals();
		void ComputeISSKeypoints();
		
		//void DisplayKeypoints(pcl::visualization::PCLVisualizer::Ptr viewer);
		ISSKeypoints () : iss_gamma_21_(0.975),iss_gamma_32_(0.975),iss_non_max_multiplier_(4),iss_sal_rad_multiplier_(6),
						  iss_bord_rad_multiplier_(1),iss_norm_rad_multiplier_(4),iss_min_neighbors_(30),iss_threads_(4),normals(new pcl::PointCloud<pcl::Normal>),
						  model (new PointCloudT),keypoints (new PointCloudT),tree (new pcl::search::KdTree<PointT> ()),iss_use_multipliers(true),normals_set(false){
						  	std::cout<<"initializing keypoints class"<<std::endl;
						  	ShowParameters();
						  }
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