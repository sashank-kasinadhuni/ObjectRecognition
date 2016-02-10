#ifndef CORRESPONDENCE_GROUPING_H
#define CORRESPONDENCE_GROUPING_H

#include "keypoints.h"
#include "planar_filtering.h"
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::Normal NormalType;
typedef pcl::ReferenceFrame RFT;
typedef pcl::SHOT352 DescriptorT;

class CorrespondenceGrouping{
	private:
		//std::string modelFilename;
		//std::string sceneFilename;
		bool showKeypoints;
		bool showCorrespondences;
		bool modelDownsamplingFlag;
		bool sceneDownsamplingFlag;


		PointCloudT::Ptr modelDownsampled;
		PointCloutT::Ptr sceneDownsampled;
		PointCloudT::Ptr model;
		PointCloudT::Ptr scene;
		PointCloudT::Ptr modelKeypoints;
		PointCloudT::Ptr sceneKeypoints;
		pcl::PointCloud<pcl::Normal>::Ptr modelNormals;
		pcl::PointCloudT<pcl::Normal>::Ptr sceneNormals;
		pcl::PointCloud<DescriptorT>::Ptr modelDescriptors;
		pcl::PointCloud<DescriptorT>::Ptr sceneDescriptors;

		double modelNonMaxSuppMult;
		double modelSalRadMult;
		double modelNormRadMult;
		double modelBordRadMult;
		double modelGamma21;
		double modelGamma32;
		int modelMinNeighbors;
		double modelDownsampleLeafSize;

		double sceneNonMaxSuppMult;
		double sceneSalRadMult;
		double sceneNormRadMult;
		double sceneBordRadMult;
		double sceneGamma21;
		double sceneGamma32;
		int sceneMinNeighbors;
		double sceneDownsampleLeafSize;
		double sceneZThreshold;

		double referenceFrameRadius;
		double SHOTDescriptorRadius;
		double houghBinSize;
		double houghThreshold;
		pcl::VoxelGrid<PointCloudT> vox;
		//double sceneReferenceFrameRadius;
		//double sceneSHOTDescriptorRadius;


	public:
		void calculateSceneNormals();
		void calculateModelNormals();
		void downsampleModel();
		void downsampelScene();

		void setModelNonMaxSuppMult(double value);
		void setModelSalRadMult(double value);
		void setModelNormRadMult(double value);
		void setModelBordRadMult(double value);
		void setModelGamma21(double value);
		void setModelGamma32(double value);
		void setModelMinNeighbors(int value);
		void setModelDownsampleLeafSize(double value);
		void setModelDownsamplingFlag(bool value);
		void setModel(PointCloudT::Ptr model);
		void setScene(PointCloudT::Ptr scene);

		void setSceneNonMaxSuppMult(double value);
		void setSceneSalRadMult(double value);
		void setSceneNormRadMult(double value);
		void setSceneBordRadMult(double value);
		void setSceneGamma21(double value);
		void setSceneGamma32(double value);
		void setSceneMinNeighbors(int value);
		void setSceneDownsamplingFlag(bool value);
		void setSceneDownsampleLeafSize(double value);

		double computeCloudResolution (const pcl::PointCloud<PointType>::ConstPtr &cloud);

		ISSKeypoints isskmodel;
		ISSKeypoints isskscene;
		CorrespondenceGrouping():showKeypoints(false),showCorrespondences(false),referenceFrameRadius(0.015),
								 SHOTDescriptorRadius(0.02),houghBinSize(0.01),houghThreshold(5.0),
								 modelNonMaxSuppMult(4),modelSalRadMult(6),modelBordRadMult(1),modelNormRadMult(4),sceneFilename(""),modelDownsamplingFlag(false),
								 modelGamma32(0.975),modelGamma21(0.975),modelMinNeighbors(10),downsampleModel(false),modelDownsampleLeafSize(0.005),
								 sceneNonMaxSuppMult(4),sceneSalRadMult(6),sceneBordRadMult(1),sceneNormRadMult(4),modelFilename(""),sceneDownsamplingFlag(false),
								 sceneGamma32(0.975),sceneGamma21(0.975),sceneMinNeighbors(10),downsamplescene(false),sceneDownsampleLeafSize(0.005),sceneZThreshold(2),
								 model(new PointCloudT),scene(new PointCloudT),modelNormals(new pcl::PointCloud<pcl::Normal>),sceneNormals(new pcl::PointCloud<pcl::Normal>),
								 modelDownsampled(new PointCloudT),sceneDownsampled(new PointCloudT),modelKeypoints(new PointCloudT),sceneKeypoints(new PointCloudT),
								 modelDescriptors(new PointCloudT<DescriptorT>),sceneDescriptors(new PointCloud<DescriptorT>){}


double Z(3.0);
};


#endif