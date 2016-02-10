#include "planar_filtering.h"
#include <pcl/io/openni_grabber.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <queue>
#include <cstdlib>
#include <ctime>
#include <pcl/common/time.h>
#include <pcl/console/parse.h>

std::string model_name("Model");

void getTokens(std::string& line,std::vector<float> & tokens){
  int tokencount = 0;
  std::stringstream res;
  for(int i = 0;i<line.length();i++){
      if(tokencount<3){
        if(line[i] != ' ')
        {
          res<<line[i];
        }else{
          std::string result = res.str();
          tokens.push_back(atof(result.c_str()));
          tokencount++;
          res.str(std::string());
        }
      }else
        return;
  }
}

int main(int argc,char** argv){	
	if(argc<2){
		std::cout<<"provide a valid txt file containing the xyz data to generate a pcd file"<<std::endl;
		return 0;
	}
	if(argc == 3){
		model_name = argv[2];
	}
	std::ifstream myfile(argv[1]);
	std::string line;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if(myfile.is_open()){
		while ( getline (myfile,line) )
    	{
    		std::vector<float> tokens;
    		getTokens(line,tokens);
    		pcl::PointXYZ point;
    		point.x = tokens[0]*10;
    		point.y = tokens[1]*10;
    		point.z = tokens[2]*50;
    		cloud->points.push_back(point);

    	}
    	cloud->width = 1;
    	cloud->height = cloud->points.size();
		std::stringstream cloud_saver;
        cloud_saver<<model_name<<".pcd";
        pcl::io::savePCDFileASCII(cloud_saver.str(), *cloud);
        std::cout<<"successfully wrote to the file :"<<cloud_saver.str()<<std::endl;
	}
	myfile.close();

}