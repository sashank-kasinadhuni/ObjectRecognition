#include <pcl/io/openni_grabber.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>     
#include <pcl/features/integral_image_normal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
bool saveCloud(false);
boost::mutex cloud_mutex;
pcl::Grabber* interface;
std::queue<pcl::PointCloud<pcl::PointXYZRGBA> > Q;
double Z;
float depthfactor(0.02f);
float smoothingfactor(10.0f);
int normalsratio(200);
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
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event,void* nothing)
{
    if (event.getKeySym() == "space" && event.keyDown())
        saveCloud = true;
}
void visualize()
{
    std::cout<<"Child thread started execution"<<std::endl;
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
        PointCloudT::Ptr cloud (new PointCloudT);
        pcl::copyPointCloud(Q.front(),*cloud);
        cloud->width = 640; // Image-like organized structure, with 640 rows and 480 columns,
		cloud->height = 480; // thus 640*480=307200 points total in the dataset
        Q.pop();
        //Planar_filter Myfilter(Z);
        //pcl::PassThrough<pcl::PointXYZRGBA> pass;
        //pass.setInputCloud (cloud);
        //pass.setFilterFieldName ("z");
        //pass.setFilterLimits (-Z, Z);
        //pass.filter(*cloud);
       	

        pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>); 
		pcl::IntegralImageNormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne;
		ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
		ne.setMaxDepthChangeFactor(depthfactor);
		ne.setNormalSmoothingSize(smoothingfactor);
		ne.setInputCloud(cloud);
		ne.compute(*normals);
	    viewer.removeAllPointClouds();
        viewer.addPointCloudNormals<pcl::PointXYZRGBA,pcl::Normal>(cloud, normals,normalsratio, 0.05, "normals");
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(cloud);
        viewer.addPointCloud<pcl::PointXYZRGBA> (cloud, rgb, "sample cloud");
        if (saveCloud)
        {
            PointCloudT::Ptr cloud_save (new PointCloudT);
            pcl::copyPointCloud(*cloud, *cloud_save);
            //pcl::copyPointCloud(*normals, *cloud_with_normals); 
            pcl::io::savePCDFileASCII("cloud.pcd", *cloud_save); 
            locker.unlock();
            interface->stop();
            return;
        }
		locker.unlock ();
    }
    interface->stop();
}



int main (int argc,char** argv)
{
	//assert (argc>1);
    //Z = atof(argv[1]);
    //std::cout<<"setting Z threshold to : "<<Z<<std::endl;
    //std::string depth_factor,smoothing_factor;
	pcl::console::parse_argument (argc, argv, "--depth_factor", depthfactor);
  	pcl::console::parse_argument (argc, argv, "--smoothing_factor", smoothingfactor);
    pcl::console::parse_argument (argc, argv, "--normals_ratio", normalsratio);
    boost::thread  work(visualize);

    work.join();
}