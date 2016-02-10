#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
pcl::visualization::PCLVisualizer viewer("PCL Viewer");
boost::mutex cloud_mutex;
pcl::Grabber* interface;
bool new_cloud_available_flag = false;
PointCloudT::Ptr cloud (new PointCloudT);

void cloud_cb_ (const PointCloudT::ConstPtr &callback_cloud)
{
  cloud_mutex.lock ();    // for not overwriting the point cloud from another thread
  *cloud = *callback_cloud;
  new_cloud_available_flag = true;
  cloud_mutex.unlock ();
}


int main()
{
    interface = new pcl::OpenNIGrabber();
    boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f = boost::bind (&cloud_cb_, _1);
    interface->registerCallback (f);
    interface->start ();

    while (!viewer.wasStopped())
  {
    viewer.spinOnce();
    if (new_cloud_available_flag && cloud_mutex.try_lock ())    // if a new cloud is available
    {
      new_cloud_available_flag = false;
      if(!viewer.updatePointCloud(cloud, "sample cloud")){
        pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);
        viewer.addPointCloud(cloud, rgb, "sample cloud");
      }
      cout<<"displaying point cloud of size: "<<cloud->points.size()<<endl;
      cloud_mutex.unlock ();
    }
    }
    interface->stop();
}
