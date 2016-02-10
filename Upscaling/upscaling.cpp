#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <sstream>
#include <string>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
void Scale_cloud(PointCloudT::Ptr cloud,PointCloudT::Ptr cloud_final , double Scale_factor)
{
	for (size_t i = 0; i < cloud->points.size (); ++i)
	{
            PointT temp;
            temp.x=(cloud->points[i].x)*Scale_factor;
            temp.y=(cloud->points[i].y)*Scale_factor;
            temp.z=(cloud->points[i].z)*Scale_factor;
            //std::cout<<temp.x<<","<<temp.y<<","<<temp.z<<std::endl;
            temp.rgb=cloud->points[i].rgb;
            (*cloud_final).points.push_back(temp);
    }
     (*cloud_final).width = (int) (*cloud_final).points.size ();  (*cloud_final).height = 1;
}

int main(int argc, char** argv){
	if (argc<2){
		std::cerr << "Usage : <input.pcd> Scale_factor"<<std::endl;
		return -1;
	}
	PointCloudT::Ptr cloud(new PointCloudT);
	if (pcl::io::loadPCDFile<pcl::PointXYZRGBA> (argv[1], *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file \n");
    return (-1);
  }
	double scale = atof(argv[2]);
	pcl::visualization::PCLVisualizer viewer("PCL Viewer");
	PointCloudT::Ptr cloud_final(new PointCloudT);
	Scale_cloud(cloud, cloud_final, scale);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final_copy(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*cloud_final,*cloud_final_copy);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(cloud);
    viewer.addPointCloud<pcl::PointXYZRGBA> (cloud, rgb, "base cloud");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud_final_copy, 0,255,0);
    viewer.addPointCloud<pcl::PointXYZ> (cloud_final_copy, single_color, "transformed cloud");
    if (argc>3){
    	std::stringstream filename;
    	filename << argv[3];
    	std::cout<<"saving transformed pointcloud to file : "<<filename.str()<<std::endl;
    	pcl::io::savePCDFileASCII(filename.str(),*cloud_final);
    }
    while(!viewer.wasStopped()){
    	viewer.spinOnce();
    }
    
}