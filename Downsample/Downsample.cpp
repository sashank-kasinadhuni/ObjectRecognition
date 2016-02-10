#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <sstream>
#include <string>
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

int
main (int argc, char** argv)
{
  PointCloudT::Ptr cloud(new PointCloudT);
  PointCloudT::Ptr cloud_filtered(new PointCloudT);
    if(argc<3){
        std::cerr<<"Usage : input.pcd output.pcd [leafsize]"<<std::endl;
    }
    double val = 0.001f;
    if(argc == 4){
      val = atof(argv[3]);
    }

  pcl::io::loadPCDFile (argv[1], *cloud);
  std::cout<<"downsampling with a leaf size of : "<<val<<std::endl;
  std::cout<<"Pointcloud before filtering has : " <<cloud->points.size()<<" points"<<std::endl;

  // Create the filtering object
  pcl::VoxelGrid<PointT> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (val, val, val);
  sor.filter (*cloud_filtered);

  std::cout<<"saving point cloud with : "<< cloud_filtered->points.size()<< " points to file : "<<argv[2] <<std::endl;
  pcl::io::savePCDFileASCII(argv[2],*cloud_filtered);

  return (0);
}
