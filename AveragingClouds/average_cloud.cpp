#include "smooth_cloud.h"
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <queue>
#include <vector>
#include <pcl/console/parse.h>
#include <memory>
#include <boost/aligned_storage.hpp>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<pcl::PointNormal> PointNormalCloudT;
typedef pcl::PointCloud<pcl::PointXYZRGBNormal> PointColorNormalCloudT;
boost::mutex cloud_mutex;
pcl::Grabber* interface;
std::queue<pcl::PointCloud<pcl::PointXYZRGBA> > Q;
boost::condition_variable cond;
std::vector<PointColorNormalCloudT::Ptr> accumulator;
PointColorNormalCloudT::Ptr res_cloud(new PointColorNormalCloudT);
int accumulator_size(5);
float depthfactor(0.02f);
float smoothingfactor(10.0f);
int normalsratio(10);
bool show_normals(false);
//SmoothCloud SC(accumulator_size);
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
PointColorNormalCloudT::Ptr calc_normals(PointCloudT::Ptr cloud){
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>); 
    pcl::IntegralImageNormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne;
    ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
    ne.setMaxDepthChangeFactor(depthfactor);
    ne.setNormalSmoothingSize(smoothingfactor);
    ne.setInputCloud(cloud);
    ne.compute(*normals);
    PointColorNormalCloudT::Ptr res_cloud (new PointColorNormalCloudT);
    pcl::copyPointCloud(*cloud,*res_cloud);
    pcl::copyPointCloud(*normals,*res_cloud);
    return res_cloud;
}
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event,void* nothing)
{
    if (event.getKeySym() == "n" && event.keyDown())
    {
      show_normals = !(show_normals);
    }
}
void ProcessClouds(){
  for(int i = 0;i<307200;i++)
  {
    int valid_points = 0;
    int valid_normals = 0;
    pcl::PointXYZRGBNormal avg_point;
    avg_point.x = std::numeric_limits<float>::quiet_NaN();
    avg_point.y = std::numeric_limits<float>::quiet_NaN();
    avg_point.z = std::numeric_limits<float>::quiet_NaN();
    avg_point.rgb = std::numeric_limits<float>::quiet_NaN();
    avg_point.normal_x = std::numeric_limits<float>::quiet_NaN();
    avg_point.normal_y = std::numeric_limits<float>::quiet_NaN();
    avg_point.normal_z = std::numeric_limits<float>::quiet_NaN();
    bool rgbIsSet = false;
    for(int j = 0;j<accumulator.size();j++)
    {
      if(!isnan((accumulator[j])->points[i].x))
      {
        if(!isnan(avg_point.x))
        {
          avg_point.x += (accumulator[j])->points[i].x;
          avg_point.y += (accumulator[j])->points[i].y;
          avg_point.z += (accumulator[j])->points[i].z;
          if(!rgbIsSet){
            avg_point.rgb = (accumulator[j])->points[i].rgb;
            rgbIsSet = true;
          }
          valid_points += 1;
        }
        else
        {
          avg_point.x = (accumulator[j])->points[i].x;
          avg_point.y = (accumulator[j])->points[i].y;
          avg_point.z = (accumulator[j])->points[i].z;
          valid_points += 1;
        }
      }
      if(!isnan((accumulator[j])->points[i].normal_x))
      {
        if(!isnan(avg_point.normal_x))
        {
          avg_point.normal_x += (accumulator[j])->points[i].normal_x;
          avg_point.normal_y += (accumulator[j])->points[i].normal_y;
          avg_point.normal_z += (accumulator[j])->points[i].normal_z;
          valid_normals += 1;
        }
        else
        {
          avg_point.normal_x = (accumulator[j])->points[i].normal_x;
          avg_point.normal_y = (accumulator[j])->points[i].normal_y;
          avg_point.normal_z = (accumulator[j])->points[i].normal_z;
          valid_normals += 1;
        }
      }
    }
    if(valid_points != 0)
    {
      avg_point.x = avg_point.x / valid_points;
      avg_point.y = avg_point.y / valid_points;
      avg_point.z = avg_point.z / valid_points;
      if(valid_normals != 0){
        avg_point.normal_x = avg_point.normal_x / valid_normals;
        avg_point.normal_y = avg_point.normal_y / valid_normals;
        avg_point.normal_z = avg_point.normal_z / valid_normals;
      }
    }
    res_cloud->points.push_back(avg_point);
  }
  res_cloud->height = 480;
  res_cloud->width = 640;
  //res_cloud->points.resize(res_cloud->height*res_cloud->width);
}

void visualize()
{
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.registerKeyboardCallback(keyboardEventOccurred);
    interface = new pcl::OpenNIGrabber();
    boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f = boost::bind (&cloud_cb_, _1);
    interface->registerCallback (f);
    interface->start ();
    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
        boost::unique_lock<boost::mutex> locker(cloud_mutex);
        while(Q.empty())cond.wait(locker);
        PointCloudT::Ptr colorcloud( new PointCloudT);
        PointColorNormalCloudT::Ptr cloud_normals_color (new PointColorNormalCloudT);
        pcl::copyPointCloud(Q.front(),*colorcloud);
        //pcl::copyPointCloud(Q.front(),*cloud_with_normals);
        pcl::copyPointCloud(*(calc_normals(colorcloud)),*cloud_normals_color);
        
        Q.pop();
        if(accumulator.size()<accumulator_size){
          
          accumulator.push_back(cloud_normals_color);
          cloud_normals_color.reset();
        }else{
          if(res_cloud->points.size()>0){
            res_cloud.reset(new PointColorNormalCloudT);
          }
          ProcessClouds();
          accumulator.clear();
          
        }
        viewer.removeAllPointClouds();
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal> rgb(res_cloud);
        pcl::PointCloud<pcl::Normal>::Ptr normals_cloud (new pcl::PointCloud<pcl::Normal> );
        pcl::copyPointCloud(*res_cloud,*normals_cloud);
        viewer.addPointCloud<pcl::PointXYZRGBNormal> (res_cloud, rgb, "sample cloud");
        if(show_normals)
          viewer.addPointCloudNormals<pcl::PointXYZRGBNormal,pcl::Normal>(res_cloud,normals_cloud,normalsratio, 0.05, "normals");
        normals_cloud.reset();
        colorcloud.reset();
        locker.unlock ();
    }
    interface->stop();
}
void showHelp(){
  std::cout<<"**********************************************"<<std::endl;
  std::cout<<"*********** Averaging PointClouds*************"<<std::endl;
  std::cout<<"**********************************************"<<std::endl;
  std::cout<<"Usage : --accsize <Number of frames to average together>"<<std::endl;
}

int main(int argc,char ** argv)
{
    pcl::console::parse_argument (argc, argv, "--accsize", accumulator_size);
    if(pcl::console::find_switch(argc,argv,"--help"))
    {
      showHelp();
      return -1;
    }
    std::cout<<"using accumulator of size: "<<accumulator_size<<std::endl;
    boost::thread  work(visualize);
    work.join();
}
