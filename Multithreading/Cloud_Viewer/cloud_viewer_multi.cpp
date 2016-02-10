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
//#include <thread>


//typedef pcl::PointXYZ PointT;
//typedef pcl::PointCloud<PointT> PointCloudT;
boost::mutex cloud_mutex;
pcl::Grabber* interface;
std::queue<pcl::PointCloud<pcl::PointXYZRGBA> > Q;
double Z(3.0);
int normalsratio(200);
int model_number(0);
bool saveCloud(false);
bool exit_prg(false);
std::string model_name("Model");
std::string _downsample("true");

//bool new_cloud_available_flag = false;
//PointCloudT::Ptr cloud (new PointCloudT);
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
    if (event.getKeySym() == "q" && event.keyDown())
        exit_prg = true;  
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
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ColorCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::copyPointCloud(Q.front(),*cloud);
        pcl::copyPointCloud(Q.front(),*ColorCloud);
        Q.pop();
        Planar_filter Myfilter(Z);
        pcl::PassThrough<pcl::PointXYZRGBA> pass;
        pass.setInputCloud (ColorCloud);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (-Z, Z);
        //pass.setFilterLimitsNegative (true);
        pass.filter(*ColorCloud);
        Myfilter.setInputCloud(cloud);
        Myfilter.Passthrough_filter();
        Myfilter.Segment_cloud();
        viewer.removeAllPointClouds();
        srand(time(0));
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(ColorCloud);
        viewer.addPointCloud<pcl::PointXYZRGBA> (ColorCloud, rgb, "sample cloud");
      for(int i =0 ; i < Myfilter._Segmented_clouds.size();i++)
      {
          const boost::shared_ptr<const PointCloudT > next_cloud (Myfilter._Segmented_clouds[i]);
          pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color(next_cloud, double(rand()%255), double(rand()%255), double(rand()%255));
          std::stringstream ss;
          ss<<"cloud"<<i;
          viewer.addPointCloud(next_cloud,single_color,ss.str());
          viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, ss.str());
      }
        //cout<<"displaying point cloud of size: "<<cloud->points.size()<<endl;
        locker.unlock ();
    }
    interface->stop();
}

void visualize_norms(){
    std::cout<<"Child thread started execution"<<std::endl;
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.registerKeyboardCallback(keyboardEventOccurred);
    //cloud_saver<<"Model_with_keypoints";
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
        if (_downsample == "true")
          Myfilter.setInputCloud(cloud_norm,true);
        else 
          Myfilter.setInputCloud(cloud_norm,false);
        Myfilter.Passthrough_filter("Normals only");
        Myfilter.Segment_cloud_with_normals();
        
        viewer.removeAllPointClouds();
        srand(time(0));
        
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(ColorCloud);
        viewer.addPointCloud<pcl::PointXYZRGBA> (ColorCloud, rgb, "sample cloud");
        //std::cout<<"found "<< Myfilter._Segmented_clouds_with_normals.size()<<" euclidian clusters" <<std::endl;
        //std::cout<<"found "<<Myfilter._Segmented_clouds_with_normals.size()<< " clusters " <<std::endl;
      for(int i =0 ; i < Myfilter._Segmented_clouds_with_normals.size();i++)
      {
        const boost::shared_ptr<const PointNormalCloudT > next_cloud (Myfilter._Segmented_clouds_with_normals[i]);
        pcl::PointCloud<pcl::Normal>::Ptr norm_cloud (new pcl::PointCloud<pcl::Normal>);
        pcl::copyPointCloud(*Myfilter._Segmented_clouds_with_normals[i],*norm_cloud);
          pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> single_color(next_cloud, double(rand()%255), double(rand()%255), double(rand()%255));
          std::stringstream ss,ssn;
          ss<<"cloud"<<i;
          ssn<<"normals"<<i;
          //std::cout<<"cluster has : "<<next_cloud->width << "x"<<next_cloud->height <<" normals have : "<<norm_cloud->width<<"x"<<norm_cloud->height<<std::endl;
          //std::cout<<"this is cluster number: "<< i <<"among "<<Myfilter._Segmented_clouds_with_normals.size()<<" clusters "<<std::endl;
          viewer.addPointCloud(next_cloud,single_color,ss.str());
          viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, ss.str());
          viewer.addPointCloudNormals<pcl::PointNormal,pcl::Normal>(next_cloud, norm_cloud,normalsratio, 0.05, ssn.str());
        if (saveCloud)
        {
            std::stringstream cloud_saver;
            cloud_saver<<model_name<<i<<".pcd";
            PointNormalCloudT::Ptr cloud_save (new PointNormalCloudT);
            pcl::copyPointCloud(*next_cloud, *cloud_save);
            //pcl::copyPointCloud(*normals, *cloud_with_normals); 
            pcl::io::savePCDFileASCII(cloud_saver.str(), *cloud_save); 
        }   
      }
      if(saveCloud){
        std::cout<<"saved "<<Myfilter._Segmented_clouds_with_normals.size()<<" clusters with Names "<<model_name<<0<<"->"<<model_name<<Myfilter._Segmented_clouds_with_normals.size()-1<<std::endl;
        saveCloud = false;
      }
      if (exit_prg){
        std::cout<<"Exiting the program"<<std::endl;
        locker.unlock();
        interface->stop();
        return;
      }

        //cout<<"displaying point cloud of size: "<<cloud->points.size()<<endl;
        locker.unlock ();
    }
    interface->stop();
}
void visualize_color_norms(){
	std::cout<<"Child thread started execution"<<std::endl;
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.registerKeyboardCallback(keyboardEventOccurred);
    //cloud_saver<<"Model_with_keypoints";
    interface = new pcl::OpenNIGrabber();
    boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f = boost::bind (&cloud_cb_, _1);
    interface->registerCallback (f);
    interface->start ();
    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
        boost::unique_lock<boost::mutex> locker(cloud_mutex);
        while(Q.empty())cond.wait(locker);
        PointColorNormalCloudT::Ptr cloud_color_norm (new PointColorNormalCloudT);
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ColorCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::copyPointCloud(Q.front(),*cloud_color_norm);
        pcl::copyPointCloud(Q.front(),*ColorCloud);
        Q.pop();
        double ne_start = pcl::getTime();
        pcl::IntegralImageNormalEstimation<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> ne;
        ne.setInputCloud (cloud_color_norm);
        ne.compute (*cloud_color_norm);
        double ne_end = pcl::getTime();
        //std::cout<<"Normal Estimation took : "<< double(ne_end - ne_start)<<std::endl;

        Planar_filter Myfilter(Z);
        ///applying a passthrough for the color cloud only
        
        pcl::PassThrough<pcl::PointXYZRGBA> pass;
        pass.setInputCloud (ColorCloud);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (-Z, Z);
        pass.filter(*ColorCloud);        

        ///setting the input cloud for applying passthrough and segmenting it

        if (_downsample == "true")
          Myfilter.setInputCloud(cloud_color_norm,true);
        else 
          Myfilter.setInputCloud(cloud_color_norm,false);
        Myfilter.Passthrough_filter("Color normals");
        Myfilter.Segment_cloud_color_normals();
        
        viewer.removeAllPointClouds();
        srand(time(0));
        
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(ColorCloud);
        viewer.addPointCloud<pcl::PointXYZRGBA> (ColorCloud, rgb, "sample cloud");
        //std::cout<<"found "<< Myfilter._Segmented_clouds_with_normals.size()<<" euclidian clusters" <<std::endl;
        //std::cout<<"found "<<Myfilter._Segmented_clouds_with_normals.size()<< " clusters " <<std::endl;
      for(int i =0 ; i < Myfilter._Segmented_clouds_color_normals.size();i++)
      {
        const boost::shared_ptr<const PointColorNormalCloudT > next_cloud (Myfilter._Segmented_clouds_color_normals[i]);
        pcl::PointCloud<pcl::Normal>::Ptr norm_cloud (new pcl::PointCloud<pcl::Normal>);
        pcl::copyPointCloud(*Myfilter._Segmented_clouds_color_normals[i],*norm_cloud);
          pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormal> single_color(next_cloud, double(rand()%255), double(rand()%255), double(rand()%255));
          std::stringstream ss,ssn;
          ss<<"cloud"<<i;
          ssn<<"normals"<<i;
          //std::cout<<"cluster has : "<<next_cloud->width << "x"<<next_cloud->height <<" normals have : "<<norm_cloud->width<<"x"<<norm_cloud->height<<std::endl;
          //std::cout<<"this is cluster number: "<< i <<"among "<<Myfilter._Segmented_clouds_with_normals.size()<<" clusters "<<std::endl;
          viewer.addPointCloud(next_cloud,single_color,ss.str());
          viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, ss.str());
          viewer.addPointCloudNormals<pcl::PointXYZRGBNormal,pcl::Normal>(next_cloud, norm_cloud,normalsratio, 0.05, ssn.str());
        if (saveCloud)
        {
            std::stringstream cloud_saver;
            cloud_saver<<model_name<<i<<".pcd";
            PointColorNormalCloudT::Ptr cloud_save (new PointColorNormalCloudT);
            pcl::copyPointCloud(*next_cloud, *cloud_save);
            //pcl::copyPointCloud(*normals, *cloud_with_normals); 
            pcl::io::savePCDFileASCII(cloud_saver.str(), *cloud_save); 
        }   
      }
      if(saveCloud){
      	std::stringstream ss;
      	ss<<model_name<<"_source.pcd";
      	PointColorNormalCloudT::Ptr source(new PointColorNormalCloudT);
      	pcl::copyPointCloud(*Myfilter.GetSourceCloud(),*source);
      	pcl::io::savePCDFileASCII(ss.str(),*source);
      	std::cout<<"saved the source cloud to "<<ss.str()<<std::endl;
        std::cout<<"saved "<<Myfilter._Segmented_clouds_color_normals.size()<<" clusters with Names "<<model_name<<0<<"->"<<model_name<<Myfilter._Segmented_clouds_color_normals.size()-1<<std::endl;
        saveCloud = false;
      }
      if (exit_prg){
        std::cout<<"Exiting the program"<<std::endl;
        locker.unlock();
        interface->stop();
        return;
      }

        //cout<<"displaying point cloud of size: "<<cloud->points.size()<<endl;
        locker.unlock ();
    }
    interface->stop();

}
int main(int argc,char ** argv)
{
    
    std::cout<<"setting Z threshold to : "<<Z<<std::endl;
    pcl::console::parse_argument (argc, argv, "--normalsratio", normalsratio);
    pcl::console::parse_argument (argc, argv, "--downsample", _downsample);
    if (pcl::console::find_switch (argc, argv, "-un")){
      boost::thread work(visualize_norms);
      work.join();
    }
    else if ((pcl::console::find_switch (argc, argv, "-ucn"))){
    	boost::thread work(visualize_color_norms);
    	work.join();
    }else{	
      boost::thread  work(visualize);  
      work.join();
    }
}
