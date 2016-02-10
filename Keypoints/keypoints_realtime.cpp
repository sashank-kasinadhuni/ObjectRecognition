#include "keypoints.h"
#include "planar_filtering.h"
#include <pcl/console/parse.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <ctime>
#include <string>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <pcl/common/time.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/correspondence.h>
#include <pcl/features/board.h>
#include <iostream>
#include <queue>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
boost::mutex cloud_mutex;
pcl::Grabber* interface;
std::queue<pcl::PointCloud<pcl::PointXYZRGBA> > Q;
boost::condition_variable cond;
pcl::VoxelGrid<pcl::PointNormal> vgn;
double Z(2.0);
std::string _downsample("false");
bool calc_iss_keypoints(false);
bool calc_us_keypoints(false);
bool _segment(false);
bool show_clouds(false);
float _leaf_size(0.007f);
int min_Neighbors(20);
int salradmult(6);
int nonmaxmult(4);
ISSKeypoints issk;
USKeypoints usk;
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

void visualize()
{
    std::cout<<"Child thread started execution"<<std::endl;
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
        Planar_filter Myfilter(Z);
        pcl::PassThrough<pcl::PointXYZRGBA> pass;
        pass.setInputCloud (ColorCloud);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (-Z, Z);
        pass.filter(*ColorCloud);
        if(_segment)
        {
            if (_downsample == "true"){
              Myfilter.setLeafSize(_leaf_size);
              Myfilter.setInputCloud(cloud_norm,true);
            }
            else 
              Myfilter.setInputCloud(cloud_norm,false);
            Myfilter.Passthrough_filter("normals_cloud");
            Myfilter.Segment_cloud_with_normals();
        }

        viewer.removeAllPointClouds();
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(ColorCloud);
        viewer.addPointCloud<pcl::PointXYZRGBA> (ColorCloud, rgb, "sample cloud");
        if(_segment){
          //std::cout<<"found : "<<Myfilter._Segmented_clouds_with_normals.size()<<"euclidian clusters"<<std::endl;
            for(int i =0 ; i < Myfilter._Segmented_clouds_with_normals.size();i++)
              {
                  const boost::shared_ptr<const PointNormalCloudT > next_cloud (Myfilter._Segmented_clouds_with_normals[i]);
                  PointCloudT::Ptr keypoints(new PointCloudT);
                  PointCloudT::Ptr segmented_cloud(new PointCloudT);
                  if(calc_iss_keypoints == true && (Myfilter._Segmented_clouds_with_normals[i])->points.size()>400){
                    //std::cout<<"input segmented cloud has : "<< (Myfilter._Segmented_clouds_with_normals[i])->points.size()<<std::endl;
                      issk.setInputCloud(Myfilter._Segmented_clouds_with_normals[i]);
                      issk.setNormals(Myfilter._Segmented_clouds_with_normals[i]);
                      issk.setNeighbors(min_Neighbors);
                      issk.setSalRadMultiplier(salradmult);
                      issk.setNonMaxMultiplier(nonmaxmult);
                      issk.ComputeISSKeypoints();
                      pcl::copyPointCloud(*(issk.keypoints),*keypoints);
                      pcl::copyPointCloud(*Myfilter._Segmented_clouds_with_normals[i],*segmented_cloud);
                      //std::cout<<"the keypoints cloud has: "<<keypoints->points.size()<<std::endl;
                    }
                   if(calc_us_keypoints == true){
                      usk.setInputCloud(Myfilter._Segmented_clouds_with_normals[i]);
                      usk.ComputeUSKeypoints();
                      pcl::copyPointCloud(*usk.keypoints,*keypoints);
                   } 
                  pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color(keypoints, double(255), double(0), double(0));
                  std::stringstream ss,cloud_stream;
                  ss<<"KeypointCloud"<<i;
                  viewer.addPointCloud(keypoints,single_color,ss.str());
                  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, ss.str());
                  if(show_clouds){
                    pcl::visualization::PointCloudColorHandlerCustom<PointT>single_color2(segmented_cloud,double(0),double(255),double(0));
                    cloud_stream<<"segcloud"<<i;
                    viewer.addPointCloud(segmented_cloud,single_color2,cloud_stream.str());
                  }

              }
        }
        else
        {
            if (_downsample == "true"){
              vgn.setInputCloud (cloud_norm);
              vgn.setLeafSize (0.01f,0.01f,0.01f);
              PointNormalCloudT::Ptr filtered_cloud(new PointNormalCloudT);
              vgn.filter (*filtered_cloud);
              *cloud_norm = *filtered_cloud;
            }
            PointCloudT::Ptr keypoints(new PointCloudT);
            if(calc_iss_keypoints){
            issk.setInputCloud(cloud_norm);
            issk.ComputeISSKeypoints();
            pcl::copyPointCloud(*issk.keypoints,*keypoints);
            }else if(calc_us_keypoints){
              usk.setInputCloud(cloud_norm);
              usk.ComputeUSKeypoints();
              pcl::copyPointCloud(*usk.keypoints,*keypoints);
            }

            pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color(keypoints, double(0), double(255), double(0));
            std::stringstream ss;
            ss<<"KeypointCloud";
            viewer.addPointCloud(keypoints,single_color,ss.str());
            viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, ss.str());
        }
            locker.unlock ();
    }      
    interface->stop();
}

int main(int argc,char ** argv)
{   
    std::cout<<"setting Z threshold to : "<<Z<<std::endl;
    if(pcl::console::find_switch(argc,argv,"-ds")){
      _downsample = "true";
      std::cout<<"downsampling input cloud"<<std::endl;
    }
    if(pcl::console::find_switch(argc,argv,"-us")){
      calc_us_keypoints = true;
      std::cout<<"calculating US keypoints"<<std::endl;
    }else if (pcl::console::find_switch(argc,argv,"-iss")){
      calc_iss_keypoints = true;
      std::cout<<"calculating isskeypoits"<<std::endl;
    }
    if (pcl::console::find_switch(argc,argv,"-segment")){
      _segment = true;
      std::cout<<"segmenting clouds "<<std::endl;
    }
    if(pcl::console::find_switch(argc,argv,"-showclouds")){
      show_clouds = true;
    }
    pcl::console::parse_argument(argc,argv,"--leaf_size",_leaf_size);
    if(pcl::console::find_switch(argc,argv,"--minnb")){
      pcl::console::parse_argument (argc, argv,"--minnb",min_Neighbors);
    }
    pcl::console::parse_argument(argc,argv,"--nonmax",nonmaxmult);
    pcl::console::parse_argument(argc,argv,"--salrad" ,salradmult);
    boost::thread  work(visualize);
    work.join();
}
