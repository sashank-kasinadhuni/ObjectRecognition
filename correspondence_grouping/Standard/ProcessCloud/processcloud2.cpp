#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <string>
int
main (int argc, char** argv)
{
    if(argc<7){
        std::cerr<<"Please enter input.pcd output.pcd Z-Threshold X-Threshold-Left X-Threshold-Right Y-Threshold-bottom"<<std::endl;
    }
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
   pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_final(new pcl::PointCloud<pcl::PointXYZRGBA>);

  if (pcl::io::loadPCDFile<pcl::PointXYZRGBA> (argv[1], *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file \n");
    return (-1);
  }
    float z_threshold=std::atof(argv[3]);
    float x_threshold_left=std::atof(argv[4]);
    float x_threshold_right=std::atof(argv[5]);
    float y_threshold_bottom=std::atof(argv[6]);
    std::cout<<"setting z-threshold to "<<z_threshold<<std::endl;
    std::cout<<"setting x-threshold_left to "<<x_threshold_left<<std::endl;
    std::cout<<"setting x-threshold_right to "<<x_threshold_right<<std::endl;
    std::cout<<"setting y-threshold_bottom to "<<y_threshold_bottom<<std::endl;

     for (size_t i = 0; i < cloud->points.size (); ++i){
        if((cloud->points[i].z)<z_threshold && (cloud->points[i].x)>x_threshold_left &&  (cloud->points[i].x)<x_threshold_right && (cloud->points[i].y)<y_threshold_bottom)
        {
            pcl::PointXYZRGBA temp;
            temp.x=cloud->points[i].x;
            temp.y=cloud->points[i].y;
            temp.z=cloud->points[i].z;
            //std::cout<<temp.x<<","<<temp.y<<","<<temp.z<<std::endl;
            temp.rgb=cloud->points[i].rgb;
            (*cloud_final).points.push_back(temp);
        }
     }
     (*cloud_final).width = (int) (*cloud_final).points.size ();  (*cloud_final).height = 1;
     std::cout<<"Saving "<<cloud_final->points.size()<<" points to "<<argv[2]<<std::endl;
     pcl::io::savePCDFileASCII (argv[2], *cloud_final);

  }
