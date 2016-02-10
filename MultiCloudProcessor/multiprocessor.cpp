#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <string>
#include <fstream>
#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
//typedef pcl::PointXYZRGBA PointT;
//typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloud;
using namespace pcl;
using namespace std;
int Counter = 0;
struct PCD
{
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;
  std::string f_name;

  PCD() : cloud (new pcl::PointCloud<pcl::PointXYZRGBA>) {};
};
void loadData (int argc, char **argv, std::vector<PCD, Eigen::aligned_allocator<PCD> > &models)
{
      ifstream myfile ("Thresholds.txt");
      string line,outfileName;
      int lines_read = 0;
       int start_Idx,end_Idx;
      if(myfile.is_open())
        {
              getline(myfile,line);
              stringstream ss(line);
              ss>>start_Idx;
              getline(myfile,line);
              stringstream kk(line);
              kk>>end_Idx;
        }
        else
            {
                std::cerr<<"cannot open Thresholds.txt"<<std::endl;
            }
       myfile.close();
      // Load the cloud and saves it into the global list of models
      for(int i =start_Idx;i<=end_Idx;i++)
      {
        PCD m;
          stringstream filename;
          filename << argv[1]<<i<<".pcd";
          m.f_name = filename.str();
          pcl::io::loadPCDFile (filename.str(), *m.cloud);
          //remove NAN points from the cloud
          std::vector<int> indices;
          pcl::removeNaNFromPointCloud(*m.cloud,*m.cloud, indices);

          models.push_back (m);
      }
    }
void printUsage(const char* programName)
    {
        cout    << "Usage: " << programName << "<source_filename>"
                <<endl
                << "[*] - multiple files can be added , all files will be thresholded based on the same parameters"
                <<endl
                <<"this program requires a \"Thresholds.txt\" file with the thresholds in the following order"
                <<endl
                <<"file_start_Index file_end_Index Z-Threshold-Far Z-Threshold-Near X-Threshold-Left X-Threshold-Right Y-Threshold-bottom Y-Threshold-Upper <output_filename> on separate lines"
                <<endl;
    }

void Cloud_Processor(std::vector<PCD, Eigen::aligned_allocator<PCD> >& data){
   std::ifstream myfile ("Thresholds.txt");
   vector<double> thresholds;
   string line,outfileName;
    double temp;
   int lines_read = 2;
   if(myfile.is_open()){
        getline(myfile,line);
        getline(myfile,line);
        while(lines_read<8){
            getline(myfile,line);
            stringstream conv(line);
            conv>>temp;
            //std::cout<<"read the value : "<<temp<<" ,original string :"<<conv.str()<<std::endl;
            thresholds.push_back(temp);
            lines_read++;
        }
        getline(myfile,line);
        outfileName = line;
        std::cout<<"filename is :"<<outfileName<<std::endl;
   }
   else
    {
        cout << "Unable to open file"<<endl;
        return;
    }
    myfile.close();
    float z_threshold_near=thresholds[0];
    float z_threshold_far=thresholds[1];
    float x_threshold_left=thresholds[2];
    float x_threshold_right=thresholds[3];
    float y_threshold_bottom=thresholds[4];
    float y_threshold_upper=thresholds[5];
    std::cout<<"setting z-threshold-near to "<<z_threshold_near<<std::endl;
    std::cout<<"setting z-threshold-far to "<<z_threshold_far<<std::endl;
    std::cout<<"setting x-threshold_left to "<<x_threshold_left<<std::endl;
    std::cout<<"setting x-threshold_right to "<<x_threshold_right<<std::endl;
    std::cout<<"setting y-threshold_bottom to "<<y_threshold_bottom<<std::endl;
    std::cout<<"setting y-threshold_upper to "<<y_threshold_upper<<std::endl;
    for(int k = 0;k<data.size();k++)
    {

        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_final(new pcl::PointCloud<pcl::PointXYZRGBA>);
        cloud = (data[k]).cloud;
        std::cout<<"Processing cloud # "<<k<<"with "<<cloud->points.size ()<<" points"<<std::endl;
        for (size_t i = 0; i < cloud->points.size (); ++i)
        {
            if((cloud->points[i].z)>z_threshold_near
               && (cloud->points[i].z)<z_threshold_far
               && (cloud->points[i].x)>x_threshold_left
               &&  (cloud->points[i].x)<x_threshold_right
               && (cloud->points[i].y)>y_threshold_bottom
               && (cloud->points[i].y)<y_threshold_upper)
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
        std::stringstream ss;
        ss << outfileName << k <<".pcd";
        std::cout<<"Saving "<<cloud_final->points.size()<<" points to "<<ss.str()<<std::endl;
        pcl::io::savePCDFileASCII (ss.str(), *cloud_final);
    }
}

int main(int argc,char** argv)
{
    if(console::find_argument(argc,argv,"-h") >= 0)
        printUsage(argv[0]);
    std::vector<PCD, Eigen::aligned_allocator<PCD> > data;
    loadData (argc, argv, data);
    if(data.size() == 0){
        std::cerr<<"Failed to load data , check program usage"<<std::endl;
        printUsage(argv[0]);
        return -1;
    }
    PCL_INFO ("Loaded %d datasets.", (int)data.size ());
    std::cout<<std::endl;
Cloud_Processor(data);
}
