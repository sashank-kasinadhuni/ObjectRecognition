#include "boundingBox.h"
#include <boost/thread/thread.hpp>
#include <pcl/common/time.h>
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
int main (int argc, char** argv)
{
  if (argc != 2)
    return (0);

  PointCloudT::Ptr cloud (new PointCloudT());
  if (pcl::io::loadPCDFile (argv[1], *cloud) == -1)
    return (-1);

	double Scale_factor= 2;	
	if(argc ==3 ){
		Scale_factor = atof(argv[2]);
	}
	double tot_time_start = pcl::getTime();
	BoundingBox box;
	BoundingBox box2;
	std::cout<<"initialized clouds"<<std::endl;
	PointCloudT::Ptr final_cloud(new PointCloudT());
	Scale_cloud(cloud,final_cloud,Scale_factor);
	double box_start = pcl::getTime();
	box.setInputCloud(cloud);
	box.initializeExtractor();
	box.calculate_OBB();
	double box_end = pcl::getTime();
	std::cout<<"calculating bounding box for one cloud took: "<<double(box_end - box_start)<<std::endl;
	box2.setInputCloud(final_cloud);
	box2.initializeExtractor();
	box2.calculate_OBB();
	std::vector<float> base = box.GetDimensions();
	std::vector<float> scaled = box2.GetDimensions();
	std::cout<<"Properties of Base cloud "<<std::endl;
	for(int i = 0; i < base.size(); i++){
		std::cout <<base[i]<<std::endl;
	}
	std::cout<<"Properties of Scaled cloud "<<std::endl;
	for(int i = 0; i < scaled.size(); i++){
		std::cout <<scaled[i]<<std::endl;
	}
	box.calculate_AABB();
	double tot_time_end= pcl::getTime();
	std::cout<<"Total Time for 2 clouds for OBB and ABB: "<<double(tot_time_end - tot_time_start)<<std::endl;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
	viewer->addCoordinateSystem (1.0);
	viewer->initCameraParameters ();
	viewer->addPointCloud<PointT> (cloud, "sample cloud");

	box.show_OBB(&*viewer);
	box.show_AABB(&*viewer);
	while(!viewer->wasStopped()){
		viewer->spinOnce();
	}


}
