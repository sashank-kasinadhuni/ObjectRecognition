#include "recognition_features.h"
#include "planar_filtering.h"
#include <pcl/io/openni_grabber.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <queue>
#include <cstdlib>
#include <ctime>
#include <string>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <pcl/common/time.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/correspondence.h>
#include <pcl/features/board.h>

boost::mutex cloud_mutex;
boost::condition_variable cond;
pcl::Grabber* interface;
std::queue<pcl::PointCloud<pcl::PointXYZRGBA> > Q;
//pcl::KdTreeFLANN<pcl::VFHSignature308> vfh_Kdtree;
//pcl::PointCloud<pcl::PointNormal>::Ptr input_cloud(new pcl::PointCloud<pcl::PointNormal>);
std::vector<pcl::PointCloud<pcl::VFHSignature308>::Ptr> _model_features;
pcl::PointCloud<pcl::VFHSignature308>::Ptr _scene_features (new pcl::PointCloud<pcl::VFHSignature308> ());
Recognizer myRec;

std::string _downsample("false");
std::string model_filename("");
std::string data_filename("data.txt");
float dist_threshold(0.25);
float _leaf_size(0.01f);
double Z(3.0);
bool _show_candidates("false");
//float match_threshold(0.8);

void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &callback_cloud)
{
  // for not overwriting the point cloud from another thread
  if (Q.empty() && cloud_mutex.try_lock ())
  {
      Q.push(*callback_cloud);
      cloud_mutex.unlock ();
      cond.notify_one();
      //boost::this_thread::sleep(boost::posix_time::milliseconds(100));
  }
}
void get_model_features(PointNormalCloudT::Ptr input_cloud){
	if(_downsample == "false")
		myRec.setInputCloud(input_cloud);
	else
		myRec.setInputCloud(input_cloud,_leaf_size);
	myRec.Estimate_VFH_Features();
	pcl::PointCloud<pcl::VFHSignature308>::Ptr modfeat(new pcl::PointCloud<pcl::VFHSignature308>);
	pcl::copyPointCloud(*(myRec._vfh_features),*modfeat);
	_model_features.push_back(modfeat);
	//std::cout<<"model feature estimation complete"<<std::endl;
	//vfh_Kdtree.setInputCloud(_model_features);
	//std::cout<<"KDtree ready"<<std::endl;
}
void process_data(){
	ifstream myfile (data_filename.c_str());
	std::vector<PointNormalCloudT::Ptr> models;
	int start_Idx,end_Idx;
	std::string line;
	if(myfile.is_open()){
		getline(myfile,line);
		while(line!=""){
			using namespace boost::algorithm;
			std::vector<std::string> tokens;
			split(tokens,line,is_any_of(" "));
			if(tokens[0] == "--model"){
				model_filename = tokens[1];
			}else if (tokens[0] == "--start"){
				start_Idx = atoi(tokens[1].c_str());
			}else if (tokens[0] == "--end"){
				end_Idx = atoi(tokens[1].c_str());
			}else if(tokens[0] == "--downsample" ){
				_downsample = (tokens[1] == "true")?"true":"false";
			}else if(tokens[0] == "--leafsize "){
				_leaf_size = atof(tokens[1].c_str());
			}
			getline(myfile,line);
		}
		for(int i = start_Idx;i<=end_Idx;i++){
			PointNormalCloudT::Ptr cloud(new PointNormalCloudT);
			std::stringstream fname;
			fname<<model_filename<<i<<".pcd";
			if (pcl::io::loadPCDFile (fname.str(), *cloud) < 0)
  				{
    				std::cout << "Error loading model: "<<fname.str()<< std::endl;
    				return ;
  				}
  			get_model_features(cloud);
  			//models.push_back(cloud);	
		}
	}else{
		std::cout<<"cannot open file :" <<data_filename<<std::endl;
		return;
	}
	std::cout<<"model data Processing complete"<<std::endl;

}


//void showCandidateClouds(vector)
void visualize()
{
    std::cout<<"Child thread started execution"<<std::endl;
    process_data();
    //get_model_features();
    //std::cout<<"setting Search Tree to look for : "<<num_neighbours<<" Closest matches"<<std::endl;
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    interface = new pcl::OpenNIGrabber();
    boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f = boost::bind (&cloud_cb_, _1);
    interface->registerCallback (f);
    interface->start ();
    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
        boost::unique_lock<boost::mutex> locker(cloud_mutex);
        while(Q.empty())cond.wait(locker);
        PointNormalCloudT::Ptr cloud_norm (new PointNormalCloudT);
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ColorCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::copyPointCloud(Q.front(),*cloud_norm);
        pcl::copyPointCloud(Q.front(),*ColorCloud);
        Q.pop();
        double ne_start = pcl::getTime();
        pcl::IntegralImageNormalEstimation<pcl::PointNormal, pcl::PointNormal> ne;
        ne.setInputCloud (cloud_norm);
        ne.compute (*cloud_norm);
        double ne_end = pcl::getTime();
        //std::cout<<"Normal Estimation took : "<< double(ne_end - ne_start)<<std::endl;

        Planar_filter Myfilter(Z);
        pcl::PassThrough<pcl::PointXYZRGBA> pass;
        pass.setInputCloud (ColorCloud);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (-Z, Z);
        pass.filter(*ColorCloud);        
        if (_downsample == "true"){
          Myfilter.setInputCloud(cloud_norm,true);
          Myfilter.setLeafSize(_leaf_size);
        }
        else 
          Myfilter.setInputCloud(cloud_norm,false);
        Myfilter.Passthrough_filter("normals_cloud");
        Myfilter.Segment_cloud_with_normals();
        
        viewer.removeAllPointClouds();
        srand(time(0));
        
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(ColorCloud);
        viewer.addPointCloud<pcl::PointXYZRGBA> (ColorCloud, rgb, "sample cloud");
        float best_diff = 324234;
        int candidate_idx = 55;
        PointNormalCloudT::Ptr best_Match (new PointNormalCloudT);
        // std::cout<<"Found: "<<Myfilter._Segmented_clouds_with_normals.size()<<" euclidian clusters "<<std::endl;
      for(int i =0 ; i < Myfilter._Segmented_clouds_with_normals.size();i++)
      {
        
        const boost::shared_ptr<const PointNormalCloudT > next_cloud (Myfilter._Segmented_clouds_with_normals[i]);
        myRec.setInputCloud(Myfilter._Segmented_clouds_with_normals[i]);
        myRec.Estimate_VFH_Features();
        pcl::copyPointCloud(*(myRec._vfh_features),*_scene_features);
        //std::cout<<"Estimated features for cluster: "<< i<<std::endl;
        pcl::PointCloud<pcl::Normal>::Ptr norm_cloud (new pcl::PointCloud<pcl::Normal>);
        pcl::copyPointCloud(*Myfilter._Segmented_clouds_with_normals[i],*norm_cloud);
          //int match_num = 0;
          //int valid_points = _scene_features->size();
          // std::cout<<"the point cloud has : "<<_model_features->points.size()<<" points"<<std::endl;
          double model_diff = 2222222;
          //std::cout<<"beginning feature difference calculation "<<std::endl;
          for(int mod_num = 0;mod_num<_model_features.size();mod_num++){
          	//std::cout<<"calculating feature differences for model :"<<mod_num<<std::endl;
          		float diff_val = 0.0;
          		for(size_t k =0 ;k<308;k++){
              		diff_val+= pow((_model_features[mod_num]->points[0].histogram[k] - _scene_features->points[0].histogram[k]),2);
            	}
            	model_diff = model_diff<diff_val?model_diff:diff_val;
            }
            //std::cout<<"calculated the difference for all models "<<std::endl;
            //std::cout<<"distance: "<<diff<<std::endl;
	          //double best_val = *std::min_element(diff.begin(),diff.end()); 
	          if (model_diff < best_diff)
	  			  {
             		best_diff = model_diff;
             		candidate_idx = i;
             		pcl::copyPointCloud(*Myfilter._Segmented_clouds_with_normals[i],*best_Match); 
  		    	}

  		    }
  		    //std::cout<<"found best match among the "<<_model_features.size()<<" models"<<std::endl;
  		    if(_show_candidates){
  		    	//std::cout<<"showing candidate clouds"<<std::endl;
  		    		for(int i =0 ; i < Myfilter._Segmented_clouds_with_normals.size();i++)
     			 {

  		    		if(i!=candidate_idx){

  		    		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> single_color(Myfilter._Segmented_clouds_with_normals[i], double(0), double(255), double(0));
    				std::stringstream ss,ssn;
    				ss<<"Candidate_Cloud"<<i;
   				 	viewer.addPointCloud(Myfilter._Segmented_clouds_with_normals[i],single_color,ss.str());
    				viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, ss.str());
    			}
  		    	}
  		    	//std::cout<<"candidate clouds display complete"<<std::endl;
  		    }
          //std::cout<<"matches: "<<match_num<<" valid points :"<<valid_points<<std::endl
    //std::cout<<"found instance at idx: "<<i<<std::endl
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> single_color(best_Match, double(255), double(0), double(0));
    std::stringstream ss,ssn;
    ss<<"best_Match";
    viewer.addPointCloud(best_Match,single_color,ss.str());
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, ss.str());
        //cout<<"displaying point cloud of size: "<<cloud->points.size()<<endl;
        locker.unlock ();
    }
    interface->stop();
}
void showHelp(){
	std::cout<<"******************************************************************************"<<std::endl;
	std::cout<<"*****************************Object Recognition*******************************"<<std::endl;
	std::cout<<"******************************************************************************"<<std::endl;
	std::cout<<"         This a program used for Recognizing Objects using global features" <<std::endl;
	std::cout<<"         Usage : --model <input.pcd> [options]"<<std::endl;
	std::cout<<"         Options :"<<std::endl;
	std::cout<<"         --downsample true/false  : if true the scene will be downsampled"<<std::endl;
	std::cout<<"         --leafsize 			  : Set Leaf Size for downsampling       "<<std::endl;
	std::cout<<"		 -sc 					  : Show all Segmented Candidate Clouds	 "<<std::endl;
	std::cout<<"******************************************************************************"<<std::endl;

}

int main(int argc,char** argv){
	std::cout<<"setting Z threshold to : "<<Z<<std::endl;
    //pcl::console::parse_argument (argc, argv, "--normalsratio", normalsratio);
    if(pcl::console::find_switch(argc,argv,"--data")){
    	pcl::console::parse_argument (argc, argv, "--data", data_filename);
    	process_data();	
    }else{
    	std::cout<<"This program requires a data filename containing the following things:"<<std::endl;
    	std::cout<<"--model   <ModelName>"<<std::endl;
    	std::cout<<"--start   <ModelStart> "<<std::endl;
    	std::cout<<"--downsample <'true'/'false'> "<<std::endl;
    	std::cout<<"--leafsize <leafsize>"<<std::endl;

    }
    if (pcl::console::find_switch(argc,argv,"-help")){
  		showHelp();
  		return 0;
  	}
    if(pcl::console::find_switch(argc,argv,"-sc")){
    	_show_candidates = true;
    }
    //if (pcl::console::find_switch (argc, argv, "-un")){
      boost::thread work(visualize);
      work.join();
    //}
    return 0;
}