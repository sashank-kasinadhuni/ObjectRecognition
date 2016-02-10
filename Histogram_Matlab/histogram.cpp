#include "recognition_features.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <iostream>
#include <fstream>
#include <pcl/visualization/cloud_viewer.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<pcl::PointNormal> PointNormalCloudT;

double _leaf_size(0.01);
bool _downsample(false);
std::string save_file("_hist.txt");
std::string model_name("");
int normalsratio(20);
bool vfh(false);
bool rv(false);
bool _show_axis(false);
bool show_normals(false);
PointNormalCloudT::Ptr model(new PointNormalCloudT);
pcl::visualization::PCLVisualizer viewer("PCL Viewer");
Recognizer rec;

int main(int argc,char** argv){
	if (argc < 2){
		std::cout<<"Please enter <input.pcd> file"<<std::endl;
		return 0;
	}
	if (pcl::io::loadPCDFile (argv[1], *model) < 0)
	{
    	std::cout << "Error loading model cloud." << std::endl;
    	return (-1);
  	}
  	model_name = argv[1];
  	model_name = model_name.substr(0,model_name.size()-4);
  	if(pcl::console::find_switch(argc,argv,"-vfh")){
  		vfh = true;
  	}
  	else if(pcl::console::find_switch(argc,argv,"-rv")){
  		std::cout<<"performing Resultant vector feature calculation"<<std::endl;
  		rv = true;
  	}else{
  		std::cout<<"no algorithm specified using default algorithm vfh"<<std::endl;
  		vfh = true;
  	}
  	if (pcl::console::find_switch (argc, argv, "-ds"))
  	{
    	_downsample = true;
    	std::cout<<"performing downsampling on the input file"<<std::endl;
  	}
  	if (pcl::console::find_switch (argc, argv, "-sn"))
  	{
    	show_normals = true;
    	std::cout<<"showing calclated normals"<<std::endl;
  	}
  	if(_downsample){
  		rec.setInputCloud(model,_leaf_size);
  		std::cout<<"saving downsampled file to model_down.pcd"<<std::endl;
  		pcl::io::savePCDFileASCII ("model_down.pcd", *(rec.getCloud()));
  	}
  	else{
  		rec.setInputCloud(model);
  		std::cout<<"setting input model without further downsampling"<<std::endl;
  	}
  	if(pcl::console::find_switch(argc,argv,"--showaxis")){
  		_show_axis = true;
  	}
	if(vfh){
		std::cout<<"estimating VFH features"<<std::endl;
		rec.Estimate_VFH_Features();
	}
	else if(rv){
		std::cout<<"estimating features using the resultant vector"<<std::endl;
		rec.Estimate_RV_Features();
		PointNormalCloudT::Ptr cloud (new PointNormalCloudT);
		pcl::PointCloud<pcl::Normal>::Ptr norm_cloud (new pcl::PointCloud<pcl::Normal>);
		cloud = rec.getPointNormalCloud();
		norm_cloud = rec.getNormalCloud();
		pcl::PointCloud<pcl::PointXYZ>::Ptr plaincloud (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::copyPointCloud(*cloud,*plaincloud);
		//pcl::PointXYZ mass_center(rec.rv_centroid.x,rec.rv_centroid.y,rec.rv_centroid.z);
		pcl::PointXYZ mass_center(0,0,0);
		pcl::PointXYZ kinectZ(0,0,-1);
		pcl::PointXYZ res_vec (rec.rv_resultant.normal_x,rec.rv_resultant.normal_y,rec.rv_resultant.normal_z);
		//pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZ> rgb(plaincloud);
        //viewer.addPointCloud<pcl::PointXYZ> (plaincloud, rgb, "model_cloud");
        
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> single_color(cloud, double(0), double(255), double(0));
        viewer.addPointCloud(cloud,single_color,"sample cloud");
        if(_show_axis){
        	viewer.addLine(mass_center,res_vec,1.0f,0.0f,0.0f,"resultantvector");
        	viewer.addLine(mass_center,kinectZ,1.0f,1.0f,0.0f,"KinectZ");
        }
        std::cout<<"resultant vector :"<<res_vec.x<<" i"<<" + "<<res_vec.y<<" j"<<" + "<<res_vec.z<<" k"<<std::endl;
        if(show_normals){
        	std::cout<<"showing 1 in "<<normalsratio<<" normals"<<std::endl;
        	viewer.addPointCloudNormals<pcl::PointNormal,pcl::Normal>(cloud, norm_cloud,normalsratio, 0.05, "normalscloud");
        }
        while(!viewer.wasStopped())
        	viewer.spinOnce();
	}
	std::cout<<"feature calculation complete"<<std::endl;
	
	ofstream myfile;
	
	if (vfh){
		std::stringstream ss;
		ss<<"vfh_"<<model_name<<".txt";
		myfile.open(ss.str().c_str());
		for(size_t k =0 ;k<308;k++){
			if(k != 307)
				myfile << rec._vfh_features->points[0].histogram[k]<<",";
			else 
				myfile << rec._vfh_features->points[0].histogram[k];
		}
		std::cout<<"wrote the histogram to file :" <<ss.str()<<std::endl;
		myfile.close();
	}else if(rv){
		std::stringstream ss;
		ss<<"rv_"<<model_name<<".txt";
		myfile.open(ss.str().c_str());
		for(int k = 0;k<181;k++){
			if(k != 180)
				myfile << rec._rv_features_181[k]<<",";
			else 
				myfile << rec._rv_features_181[k];
		}
		std::cout<<"wrote the histogram to file: "<< ss.str()<<std::endl;
		//myfile.open(ss.c_str());
	}
    
}
