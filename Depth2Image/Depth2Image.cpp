#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <queue>
#include <pcl/console/parse.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
boost::mutex cloud_mutex;
pcl::Grabber* interface;
std::queue<pcl::PointCloud<pcl::PointXYZRGBA> > Q;
boost::condition_variable cond;
bool saveCloud(false);
std::string saveFileName("Image");
int filecounter(0);
bool usept(false);
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

void savePointCloud(PointCloudT::Ptr input){
  ofstream myfile;
  std::stringstream ss;
  ss<<saveFileName<<filecounter<<".txt";
  myfile.open(ss.str().c_str());
  std::cout<<"the pointcloud resolution: "<<input->height<<"x"<<input->width<<std::endl;
  for(int i = 0;i<480;i++){
      for(int j = 0;j<640;j++){
          if(j!= 639){
            if(!isnan(input->points[i*480+j].z))
              myfile << input->points[i*480+j].z<<",";
            else
              myfile << 6 <<",";
          }else{
            if(!isnan(input->points[i*480+j].z))
              myfile << input->points[i*480+j].z;
            else
              myfile <<6;
            myfile << std::endl;
          }
      }

  }

  
  saveCloud = false;
  std::cout<<"saved Cloud to : "<<ss.str()<<std::endl;
  std::stringstream cloudsaver;
  cloudsaver<<saveFileName<<filecounter<<".pcd";
  pcl::io::savePCDFileASCII(cloudsaver.str(), *input);
  std::cout<<"saved PointCloud to : "<<cloudsaver.str()<<std::endl;
  myfile.close();
  filecounter++;
}
  void savePTcloud(PointCloudT::Ptr input){
    ofstream myfile;
    std::stringstream ss;
    ss<<saveFileName<<filecounter<<".txt";
    myfile.open(ss.str().c_str());
    for(int i = 0;i<480;i++){
      for(int j= 0;j<640;j++){
        if(j!=639)
            myfile << input->points[i*480+j].z<<",";
        else{
          myfile<<input->points[i*480+j].z;
          myfile<<std::endl;    
          }
        }
        }
      saveCloud = false;
    std::cout<<"saved Cloud to : "<<ss.str()<<std::endl;
    std::stringstream cloudsaver;
    cloudsaver<<saveFileName<<filecounter<<".pcd";
    pcl::io::savePCDFileASCII(cloudsaver.str(), *input);
    std::cout<<"saved PointCloud to : "<<cloudsaver.str()<<std::endl;
    filecounter++;
    myfile.close();
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
        Q.pop();
        viewer.removeAllPointClouds();
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(cloud);
        viewer.addPointCloud<pcl::PointXYZRGBA> (cloud, rgb, "sample cloud");
        if(saveCloud){
          if(!usept)
            savePointCloud(cloud);
          else
            savePTcloud(cloud);
        }
        locker.unlock ();
    }
    interface->stop();
}

int main(int argc,char ** argv)
{
    if(pcl::console::find_switch(argc,argv,"-usept")){
      usept = true;
    }
    boost::thread  work(visualize);
    work.join();
}
