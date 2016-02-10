#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <queue>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
boost::mutex cloud_mutex;
pcl::Grabber* interface;
std::queue<pcl::PointCloud<pcl::PointXYZRGBA> > Q;
boost::condition_variable cond;

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
        PointCloudT::Ptr cloud (new PointCloudT);
        pcl::copyPointCloud(Q.front(),*cloud);
        Q.pop();
        viewer.removeAllPointClouds();
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(cloud);
        viewer.addPointCloud<pcl::PointXYZRGBA> (cloud, rgb, "sample cloud");
        locker.unlock ();
    }
    interface->stop();
}

int main(int argc,char ** argv)
{
    boost::thread  work(visualize);
    work.join();
}
