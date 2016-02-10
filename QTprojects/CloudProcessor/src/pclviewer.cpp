#include "pclviewer.h"
#include "../build/ui_pclviewer.h"

PCLViewer::PCLViewer (QWidget *parent) :
  QMainWindow (parent),
  ui (new Ui::PCLViewer)
{
  ui->setupUi (this);
  this->setWindowTitle ("PCL viewer");

  std::ifstream myfile("data.txt");
  std::string line,outfileName;
  if(myfile.is_open())
    {
        getline(myfile,line);
    }
    else{
      std::cout<<"cannot open data.txt"<<std::endl;
    }
    myfile.close();
  // Setup the cloud pointer
  cloud.reset (new PointCloudT);
  rotated_cloud.reset(new PointCloudT);
  pcl::io::loadPCDFile (line, *cloud);
  pcl::io::loadPCDFile (line, *rotated_cloud);

  x_rot = 0;
  y_rot = 0;
  z_rot = 0;

  z_near = -4.0;
  z_far = 5.0;
  x_left = -4.0;
  x_right = 4.0;
  y_top = 4.0;
  y_bottom = -4.0;

  // Set up the QVTK window
  viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
  ui->qvtkWidget->SetRenderWindow (viewer->getRenderWindow ());
  viewer->setupInteractor (ui->qvtkWidget->GetInteractor (), ui->qvtkWidget->GetRenderWindow ());
  ui->qvtkWidget->update ();

 //connecting buttons
  connect (ui->randomcolorbutton,  SIGNAL (clicked ()), this, SLOT (RandomColorButtonPressed()));
  connect (ui->saveButton,  SIGNAL (clicked ()), this, SLOT (SaveButtonPressed()));

  // Connect x,y,z rotation sliders and their functions
  connect (ui->x_rotation, SIGNAL (valueChanged (double)), this, SLOT (xAngleSliderValueChanged (double)));
  connect (ui->y_rotation, SIGNAL (valueChanged (double)), this, SLOT (yAngleSliderValueChanged (double)));
  connect (ui->z_rotation, SIGNAL (valueChanged (double)), this, SLOT (zAngleSliderValueChanged (double)));

  //connecting the thresholding boxes
  connect (ui->x_threshold_left,SIGNAL(valueChanged(double)),this, SLOT(x_left_changed (double)));
  connect (ui->x_threshold_right,SIGNAL(valueChanged(double)),this, SLOT(x_right_changed (double)));
  connect (ui->y_threshold_top,SIGNAL(valueChanged(double)),this, SLOT(y_top_changed (double)));
  connect (ui->y_threshold_bottom,SIGNAL(valueChanged(double)),this, SLOT(y_bottom_changed (double)));
  connect (ui->z_threshold_near,SIGNAL(valueChanged(double)),this, SLOT(z_near_changed (double)));
  connect (ui->z_threshold_far,SIGNAL(valueChanged(double)),this, SLOT(z_far_changed (double)));

 
  pcl::visualization::PointCloudColorHandlerCustom<PointT> color_handler (rotated_cloud, 255, 255, 255);
  viewer->addPointCloud (rotated_cloud,color_handler, "cloud");
  viewer->addCoordinateSystem();

  //XYZsliderChanged();
  viewer->resetCamera ();
  ui->qvtkWidget->update ();
}

void PCLViewer::RandomColorButtonPressed(){
  viewer->removeAllPointClouds();
  pcl::visualization::PointCloudColorHandlerCustom<PointT> color_handler (rotated_cloud, 255 *(1024 * rand () / (RAND_MAX + 1.0f)), 255 *(1024 * rand () / (RAND_MAX + 1.0f)), 255 *(1024 * rand () / (RAND_MAX + 1.0f)));
  viewer->addPointCloud (rotated_cloud,color_handler, "cloud");
  viewer->addCoordinateSystem();
  ui->qvtkWidget->update ();
  std::cout<<"setting random colors to the pointcloud"<<std::endl;
}
void PCLViewer::SaveButtonPressed(){
  std::ifstream myfile("data.txt");
  std::string line,outfileName;
  if(myfile.is_open())
    {
        getline(myfile,line);
        getline(myfile,line);
    }
    else{
      std::cout<<"cannot open data.txt"<<std::endl;
    }
    std::cout<<"saving data to :"<<line<<std::endl;
    myfile.close();
    PointCloudT::Ptr saveCloud(new PointCloudT);
    pcl::copyPointCloud(*rotated_cloud,*saveCloud);
    saveCloud->width = rotated_cloud->points.size();
    saveCloud->height = 1;
    saveCloud->resize(saveCloud->width * saveCloud->height);
    std::cout<<"saveCloud has "<<saveCloud->points.size()<<" points and a width of "<<saveCloud->width<<" and a height of "<<saveCloud->height<<std::endl;
    pcl::io::savePCDFileASCII (line, *saveCloud);
}

void
PCLViewer::XYZSliderChanged ()
{
  rotated_cloud.reset(new PointCloudT);
  

  Eigen::Affine3f transform_x_rotate = Eigen::Affine3f::Identity();
  Eigen::Affine3f transform_y_rotate = Eigen::Affine3f::Identity();
  Eigen::Affine3f transform_z_rotate = Eigen::Affine3f::Identity();
  double x_angle = (M_PI / 180) * x_rot;
  double y_angle = (M_PI / 180) * y_rot;
  double z_angle = (M_PI / 180) * z_rot;
  
  transform_x_rotate.rotate(Eigen::AngleAxisf (x_angle, Eigen::Vector3f::UnitX()));
  transform_y_rotate.rotate(Eigen::AngleAxisf (y_angle, Eigen::Vector3f::UnitY()));
  transform_z_rotate.rotate(Eigen::AngleAxisf (z_angle, Eigen::Vector3f::UnitZ()));
  //PointCloudT::Ptr transformed_cloud (new PointCloudT);
  pcl::transformPointCloud(*cloud,*rotated_cloud,transform_x_rotate);
  pcl::transformPointCloud(*rotated_cloud,*rotated_cloud,transform_y_rotate);
  pcl::transformPointCloud(*rotated_cloud,*rotated_cloud,transform_z_rotate);

  PointCloudT::Ptr tempcloud(new PointCloudT);
  for(int i = 0;i<rotated_cloud->points.size();i++){
            if((rotated_cloud->points[i].z)>z_near
               && (rotated_cloud->points[i].z)<z_far
               && (rotated_cloud->points[i].x)>x_left
               &&  (rotated_cloud->points[i].x)<x_right
               && (rotated_cloud->points[i].y)>y_bottom
               && (rotated_cloud->points[i].y)<y_top)
            {
                pcl::PointXYZRGBA temp;
                temp.x=rotated_cloud->points[i].x;
                temp.y=rotated_cloud->points[i].y;
                temp.z=rotated_cloud->points[i].z;
                //std::cout<<temp.x<<","<<temp.y<<","<<temp.z<<std::endl;
                temp.rgb=rotated_cloud->points[i].rgb;
                (*tempcloud).points.push_back(temp);
            }
  }
  pcl::copyPointCloud(*tempcloud,*rotated_cloud);
  std::cout<<"pointcloud has: "<<rotated_cloud->points.size()<<" points"<<std::endl;
  viewer->removeAllPointClouds();
  pcl::visualization::PointCloudColorHandlerCustom<PointT> color_handler (rotated_cloud, 255, 255, 255);
  viewer->addPointCloud (rotated_cloud,color_handler, "cloud");
  viewer->addCoordinateSystem();
  ui->qvtkWidget->update ();
}


void
PCLViewer::xAngleSliderValueChanged (double value)
{
  x_rot = value;
  XYZSliderChanged();
}

void
PCLViewer::yAngleSliderValueChanged (double value)
{
  y_rot = value;
  XYZSliderChanged();
}

void
PCLViewer::zAngleSliderValueChanged (double value)
{
  z_rot= value;
  XYZSliderChanged();
}

void
PCLViewer::x_left_changed (double value){
  x_left = value;
  XYZSliderChanged();
  std::cout<<"xleft changed to :"<<value<<std::endl;
}

void PCLViewer::x_right_changed(double value){
  x_right = value;
  XYZSliderChanged();
  std::cout<<"xright changed to :"<<value<<std::endl;
}


void PCLViewer::y_top_changed(double value)
{
  y_top = value;
  XYZSliderChanged();
  std::cout<<"ytop changed to :"<<value<<std::endl;
}

void PCLViewer::y_bottom_changed(double value){
  y_bottom = value;
  XYZSliderChanged();
  std::cout<<"ybottom changed to :"<<value<<std::endl;
}

void PCLViewer::z_near_changed(double value){
  z_near = value;
  XYZSliderChanged();
  std::cout<<"znear changed to :"<<value<<std::endl;
}

void PCLViewer::z_far_changed(double value){
  z_far = value;
  XYZSliderChanged();
  std::cout<<"zfar changed to :"<<value<<std::endl;
}

PCLViewer::~PCLViewer ()
{
  delete ui;
}
