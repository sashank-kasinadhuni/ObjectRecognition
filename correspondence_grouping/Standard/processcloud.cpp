#include <pcl/io/openni_grabber.h>
 #include <pcl/visualization/cloud_viewer.h>
 #include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <string>
int
main (int argc, char** argv)
{
    if(argc<4){
        std::cerr<<"Please enter input.pcd output.pcd Z-Threshold"<<std::endl;
    }
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
   pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_final(new pcl::PointCloud<pcl::PointXYZRGBA>);

  if (pcl::io::loadPCDFile<pcl::PointXYZRGBA> (argv[1], *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }
    float z_threshold=std::atof(argv[3]);
    std::cout<<"setting z-threshold to "<<z_threshold<<std::endl;
     for (size_t i = 0; i < cloud->points.size (); ++i){
        if((cloud->points[i].z)<z_threshold){
            pcl::PointXYZRGBA temp;
            temp.x=cloud->points[i].x;
            temp.y=(-1)*cloud->points[i].y;
            temp.z=cloud->points[i].z;
            std::cout<<temp.x<<","<<temp.y<<","<<temp.z<<std::endl;
            temp.rgb=cloud->points[i].rgb;
            (*cloud_final).points.push_back(temp);
        }
     }
     (*cloud_final).width = (int) (*cloud_final).points.size ();  (*cloud_final).height = 1;
     std::cout<<"Saving "<<cloud_final->points.size()<<" points to "<<argv[2]<<std::endl;
     pcl::io::savePCDFileASCII (argv[2], *cloud_final);

  }
