#include <pcl/io/openni_grabber.h>
 #include <pcl/visualization/cloud_viewer.h>
 #include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <string>
#include <sstream>
 class SimpleOpenNIViewer
 {
private:
    int counter;
    int version;
   public:

     SimpleOpenNIViewer () : viewer ("PCL OpenNI Viewer") {counter=0;version=0;}
     void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
     {

       if (!viewer.wasStopped()){
         viewer.showCloud (cloud);
       counter++;
           if(counter==200){
           std::stringstream filename;
           filename<<"test"<<version<<".pcd";
           pcl::io::savePCDFileASCII (filename.str(), *cloud);
           counter = 0;
           version++;
           }
       }
     }

     void run ()
     {
       pcl::Grabber* interface = new pcl::OpenNIGrabber();

       boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
         boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);

       interface->registerCallback (f);

       interface->start ();

       while (!viewer.wasStopped())
       {
         boost::this_thread::sleep (boost::posix_time::seconds (1));
       }

       interface->stop ();
     }

     pcl::visualization::CloudViewer viewer;
 };

 int main ()
 {
   SimpleOpenNIViewer v;
   v.run ();
   return 0;
 }
