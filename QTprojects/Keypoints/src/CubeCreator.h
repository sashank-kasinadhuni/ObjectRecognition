#ifndef CUBECREATOR_H
#define CUBECREATOR_H

#include <iostream>
#include <string>
#include <sstream>
#include <cmath>

#include <pcl/io/pcd_io.h>
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

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class CubeCreator{
private:
	float edge_points ;
	float edge_size ;
	PointCloudT::Ptr cubecloud;
public:
	//CubeCreator(float edgesize = 0.4,float edgepoints = 10):edge_size(edgesize),edge_points(edgepoints),cubecloud(new PointCloudT){}
	CubeCreator():edge_size(0.4),edge_points(10),cubecloud(new PointCloudT){}
	void setEdgeLength(double length);
	void setEdgePoints(int Points);
	void setEdgePoints(double Points);
	void GenerateCube();
	void resetCube(); 	
	PointCloudT::Ptr GetCube(){return cubecloud;}
};
#endif