
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <pcl/console/parse.h>
#include <string>
#include <sstream>
#include <cmath>
#include "CubeCreator.h"
#include "keypoints.h"
#include "planar_filtering.h"

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
float edge_points = 10;
float edge_size = 0.4;
int nonmaxradmult(6);
int salradmult(4);
int neighbors(5); 
int numNormals(1);
int normNeighbors(20);
pcl::visualization::PCLVisualizer viewer("CubeMaker");
 PointCloudT::Ptr cubecloud(new PointCloudT);
  PointCloudT::Ptr preNormalKeypoints(new PointCloudT);
  PointCloudT::Ptr postNormalKeypoints(new PointCloudT);
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  ISSKeypoints issk;
  ISSKeypoints issk2;
bool showNormals(false);

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event,
                          void* nothing)
{
        if (event.getKeySym() == "n" && event.keyDown()){
            showNormals = showNormals?false:true ;
            pcl::visualization::PointCloudColorHandlerCustom<PointT> color_handler (cubecloud, 255, 255, 255);
  			pcl::visualization::PointCloudColorHandlerCustom<PointT> keypoint_color_handler (cubecloud, 255, 0, 0);
              	if(!showNormals)
  				{
			  		viewer.removeAllPointClouds();
				    viewer.addPointCloud(cubecloud,color_handler,"cubecloud");
				    viewer.addPointCloud(preNormalKeypoints,keypoint_color_handler,"keypointcloud");
				    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5,"keypointcloud");    
				}	
				else{
					viewer.removeAllPointClouds();
					viewer.addPointCloud(cubecloud,color_handler,"cubecloud");
					viewer.addPointCloud(postNormalKeypoints,keypoint_color_handler,"keypointcloud");
				    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5,"keypointcloud");
				    viewer.addPointCloudNormals<PointT,pcl::Normal>(cubecloud,normals,numNormals,0.05,"normalsCloud");
				}
	viewer.addCoordinateSystem();
        }

}

int main(int argc,char ** argv)
{
  pcl::console::parse_argument (argc, argv,"--edgepoints",edge_points);
  pcl::console::parse_argument (argc, argv,"--edgesize",edge_size);
  pcl::console::parse_argument (argc, argv,"--minneighbors",neighbors);
  pcl::console::parse_argument (argc, argv,"--salradmult",salradmult);
  pcl::console::parse_argument (argc, argv,"--nonmaxradmult",nonmaxradmult);
  pcl::console::parse_argument (argc, argv,"--numNormals",numNormals);
  pcl::console::parse_argument (argc, argv,"--normNeighbors",normNeighbors);
  std::cout<<"generating a cube with each edge having "<<edge_points<<" points and an edge of length "<<edge_size  <<std::endl;

  viewer.registerKeyboardCallback(keyboardEventOccurred);
  CubeCreator cube(edge_size,edge_points);  
  cube.GenerateCube();
  pcl::copyPointCloud(*(cube.GetCube()),*cubecloud);
  std::cout<<"cube generation with : "<<cubecloud->points.size() <<" points complete"<<std::endl;

  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  ne.setInputCloud (cubecloud);
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
  ne.setSearchMethod (tree);
  //ne.setRadiusSearch (0.03);
  ne.setKSearch(normNeighbors);
  ne.compute (*normals);
    
  pcl::visualization::PointCloudColorHandlerCustom<PointT> color_handler (cubecloud, 255, 255, 255);
  pcl::visualization::PointCloudColorHandlerCustom<PointT> keypoint_color_handler (cubecloud, 255, 0, 0);
  
  
  issk.setInputCloud(cubecloud);
  issk.setNeighbors(neighbors);
  issk.setNonMaxMultiplier(nonmaxradmult);
  issk.setSalRadMultiplier(salradmult);
  //issk.setNormals(normals)
  issk.ComputeISSKeypoints();

  issk2.setInputCloud(cubecloud);
  issk2.setNeighbors(neighbors);
  issk2.setNonMaxMultiplier(nonmaxradmult);
  issk2.setSalRadMultiplier(salradmult);
  issk2.setNormals(normals);
  issk2.ComputeISSKeypoints();
  
  pcl::copyPointCloud(*(issk.keypoints),*preNormalKeypoints);
  pcl::copyPointCloud(*(issk2.keypoints),*postNormalKeypoints);
	viewer.removeAllPointClouds();
	viewer.addPointCloud(cubecloud,color_handler,"cubecloud");
	viewer.addPointCloud(preNormalKeypoints,keypoint_color_handler,"keypointcloud");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5,"keypointcloud");
  
  while(!viewer.wasStopped()){
    viewer.spinOnce();
  }
}
