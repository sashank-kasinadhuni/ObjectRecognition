#include <pcl/io/openni_grabber.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
float depthfactor(0.02f);
float smoothingfactor(0.5f);
int normalsratio(10);

int main(int argc,char** argv){
	pcl::console::parse_argument (argc, argv, "--depth_factor", depthfactor);
  	pcl::console::parse_argument (argc, argv, "--smoothing_factor", smoothingfactor);
	pcl::console::parse_argument (argc, argv, "--normals_ratio", normalsratio);
	if (argc<2){
		std::cout<<"Please provide the following files: <cloud.pcd> "<<std::endl;
	}
	PointCloudT::Ptr cloud(new PointCloudT);
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
	pcl::io::loadPCDFile (argv[1], *cloud);
	if(cloud->height == 1){
		cloud->width = cloud->points.size() * 0.002083333;
		cloud->height = cloud->points.size() * 0.0015625;
	}
	pcl::visualization::PCLVisualizer viewer("PCL Viewer");
	std::cout<<"loaded file with: "<<std::endl;
	std::cout<<cloud->points.size()<<" points" <<std::endl;
	std::cout<<"cloud dimensions: "<<cloud->width<<"x"<<cloud->height<<std::endl;
	pcl::IntegralImageNormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne;
	ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
	ne.setMaxDepthChangeFactor(depthfactor);
	ne.setNormalSmoothingSize(smoothingfactor);
	ne.setInputCloud(cloud);
	ne.compute(*normals);
	std::cout<<"cloud has :"<<normals->points.size()<<"normals"<<std::endl;
	viewer.removeAllPointClouds();
	viewer.addPointCloudNormals<pcl::PointXYZRGBA,pcl::Normal>(cloud, normals,normalsratio, 0.05, "normals");
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(cloud);
	viewer.addPointCloud<pcl::PointXYZRGBA> (cloud, rgb, "sample cloud");
	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}
}
