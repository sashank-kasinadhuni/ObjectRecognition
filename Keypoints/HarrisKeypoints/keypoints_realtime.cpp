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
#include <pcl/keypoints/harris_3d.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
boost::mutex cloud_mutex;
pcl::Grabber* interface;
std::queue<pcl::PointCloud<pcl::PointXYZRGBA> > Q;
boost::condition_variable cond;
double Z(3.0);
double harris_radius(0.01f);
std::string _downsample("false");
float _leaf_size(0.01f);

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
        if (_downsample == "true"){
          Myfilter.setInputCloud(cloud_norm,true);
          Myfilter.setLeafSize(_leaf_size);
        }
        else 
          Myfilter.setInputCloud(cloud_norm,false);
        Myfilter.Passthrough_filter("normals_cloud");
        Myfilter.Segment_cloud_with_normals();
        
        viewer.removeAllPointClouds();
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(ColorCloud);
        viewer.addPointCloud<pcl::PointXYZRGBA> (ColorCloud, rgb, "sample cloud");
        
        for(int i =0 ; i < Myfilter._Segmented_clouds_with_normals.size();i++)
          {
              const boost::shared_ptr<const PointNormalCloudT > next_cloud (Myfilter._Segmented_clouds_with_normals[i]);
              pcl::PointCloud<pcl::Normal>::Ptr norm_cloud(new pcl::PointCloud<pcl::Normal>);
              pcl::copyPointCloud(*(Myfilter._Segmented_clouds_with_normals[i]),*norm_cloud);
              pcl::HarrisKeypoint3D< PointT, PointT, pcl::Normal > harris;
              
              harris.setInputCloud(Myfilter._Segmented_clouds_with_normals[i]);
              harris.setNormals(norm_cloud);
              
              harris.setNumberOfThreads(4);
              
              harris.setRadius(0.01f);

              pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color(issk.keypoints, double(0), double(255), double(0));
              std::stringstream ss;
              ss<<"KeypointCloud"<<i;
              viewer.addPointCloud(issk.keypoints,single_color,ss.str());
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
    }
    pcl::console::parse_argument (argc, argv,"--radius",harris_radius);
    boost::thread  work(visualize);
    work.join();
}
