#include "planar_filtering.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <pcl/filters/passthrough.h>
#include <string>
#include <cstdlib>
#include <ctime>
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
int main(int argc,char** argv){
	if (argc<2)
	{
		std::cerr <<"Input.pcd Z-threshold required"<<std::endl;
		return -1;
	}
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
	const std::string filename(argv[1]);
	const float Z = atof(argv[2]);
	try{
	pcl::io::loadPCDFile (filename, *cloud);
	}
	catch(...)
	{
		std::cerr<<"couldnt load the file"<<std::endl;
	}
	std::cout<<"loaded pcd file"<<std::endl;
	pcl::visualization::PCLVisualizer viewer("PCL Viewer");
	pcl::PassThrough<PointT> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (-Z, Z);
    //pass.setFilterLimitsNegative (true);
    pass.filter(*cloud);
    pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);
    //viewer.addPointCloud(cloud,rgb,"cloud");
    Planar_filter Myfilter;
    Myfilter.setInputCloud(cloud);
    Myfilter.Segment_cloud();
    std::vector<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA> > > Myclouds = Myfilter.getSegmentedClouds();
    srand(time(0));
    for(int i =0 ; i < Myclouds.size();i++){
    	const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGBA> > next_cloud (Myclouds[i]);
    	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> single_color(next_cloud, rand()%255, rand()%255, rand()%255);
    	std::stringstream ss;
    	ss<<"cloud"<<i;
    	viewer.addPointCloud(Myclouds[i],ss.str());
    }




    while (!viewer.wasStopped())
    {
    	viewer.spinOnce();

 	}
}