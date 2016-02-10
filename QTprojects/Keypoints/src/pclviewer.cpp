#include "pclviewer.h"
#include "../build/ui_pclviewer.h"

PCLViewer::PCLViewer (QWidget *parent) :
  QMainWindow (parent),
  ui (new Ui::PCLViewer)
{
  ui->setupUi (this);
  this->setWindowTitle ("PCL viewer");


  // Setup the cloud pointer
  cloud.reset (new PointCloudT);
  filtered_cloud.reset(new PointCloudT);

  // Set up the QVTK window
  viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
  ui->qvtkWidget->SetRenderWindow (viewer->getRenderWindow ());
  viewer->setupInteractor (ui->qvtkWidget->GetInteractor (), ui->qvtkWidget->GetRenderWindow ());
  ui->qvtkWidget->update ();
  useMultipliers = true;
  edgePoints = 10;
  edgeLength = 0.4;
  minNeighbors = 30;
  nonMaxSuppMult = 4;
  normalRadiusMult=4;
  borderRadiusMult =1;
  salientRadiusMult = 6;
  usePcdInputFile = false;
  pcdFileLoaded=false;
  cloudFilteringComplete = false;
  downSampleModel = false;
 //connecting buttons
  connect (ui->CalculateKeypoints,  SIGNAL (clicked ()), this, SLOT (CalculateKeypointsButtonPressed()));
  connect (ui->defaultValues,  SIGNAL (clicked ()), this, SLOT (DefaultValuesButtonPressed()));

  //connectingTickBoxes

  connect (ui->MultiplyResolution, SIGNAL(toggled(bool)),this,SLOT(MultiplyResolutionButtonPressed()));

  // Connect all the parameter controls and their functions
  connect (ui->nonMaxSuppBox,SIGNAL (valueChanged(double)) ,this , SLOT(nonMaxSuppValueChanged (double)));
  connect (ui->salientRadiusBox,SIGNAL(valueChanged(double)) ,this , SLOT(salientRadiusValueChanged(double)));
  connect (ui->normalRadiusBox, SIGNAL(valueChanged(double)), this , SLOT(normalRadiusValueChanged(double)));
  connect (ui->borderRadiusBox, SIGNAL(valueChanged(double)), this, SLOT(borderRadiusValueChanged(double)));

  connect (ui->gamma21Box,SIGNAL(valueChanged(double)), this,SLOT(gamma21ValueChanged(double)));
  connect (ui->gamma32Box,SIGNAL(valueChanged(double)), this,SLOT(gamma32ValueChanged(double)));
  
  connect (ui->CubeEdgeLengthBox,SIGNAL(valueChanged(double)),this, SLOT(CubeEdgeLengthChanged(double)));
  connect (ui->CubeEdgePointsBox,SIGNAL(valueChanged(double)),this, SLOT(CubeEdgePointsChanged(double)));
  
  connect (ui->nonMaxSuppBox_2,SIGNAL (valueChanged(double)) ,this , SLOT(nonMaxSuppMultChanged (double)));
  connect (ui->salientRadiusBox_2,SIGNAL(valueChanged(double)) ,this , SLOT(salientRadiusMultChanged(double)));
  connect (ui->normalRadiusBox_2, SIGNAL(valueChanged(double)), this , SLOT(normalRadiusMultChanged(double)));
  connect (ui->borderRadiusBox_2, SIGNAL(valueChanged(double)), this, SLOT(borderRadiusValueChanged(double)));
  connect (ui->inputModelBox,SIGNAL(textChanged(QString)),this , SLOT(modelFileNameChanged(QString)));
  connect (ui->downSampleLeafSizeBox,SIGNAL(valueChanged(double)),this,SLOT(leafSizeValueChanged(double)));

  cube.setEdgePoints(10);
  cube.setEdgeLength(0.4);

  cube.GenerateCube();
  pcl::copyPointCloud(*cube.GetCube(),*cloud);
  issk.setInputCloud(cloud);
 
  pcl::visualization::PointCloudColorHandlerCustom<PointT> color_handler (cloud, 255, 255, 255);
  viewer->addPointCloud (cloud,color_handler, "cloud");
 // viewer->addCoordinateSystem();

  //XYZsliderChanged();
  viewer->resetCamera ();
  ui->qvtkWidget->update ();
}

void PCLViewer::DownSampleModel(){
  vox.setInputCloud(cloud);
  vox.setLeafSize(downSampleLeafSize,downSampleLeafSize,downSampleLeafSize);
  vox.filter(*filtered_cloud);
  cloudFilteringComplete = true;
}
void PCLViewer::modelFileNameChanged(QString mfilename){
  std::cout<<"model filename changed to "<<mfilename.toStdString()<<std::endl;
  modelFileName = mfilename.toStdString();
  pcdFileLoaded = false;
}
void PCLViewer::ParametersChanged(){
  edgeLength = ui->CubeEdgeLengthBox->value();
  edgePoints = static_cast<int>(ui->CubeEdgePointsBox->value());
  
  nonMaxSuppMult = ui->nonMaxSuppBox->value();
  normalRadiusMult = ui->normalRadiusBox->value();
  borderRadiusMult = ui->borderRadiusBox->value();
  salientRadiusMult = ui->salientRadiusBox ->value();

  nonMaxSuppRadius = ui->nonMaxSuppBox_2->value();
  normalRadius = ui->normalRadiusBox_2->value();
  borderRadius = ui->borderRadiusBox_2->value();
  salientRadius = ui->salientRadiusBox_2->value();

  gamma32 = ui->gamma32Box->value();
  gamma21 = ui->gamma21Box->value();
  downSampleLeafSize = ui->downSampleLeafSizeBox->value();

  useMultipliers = ui->MultiplyResolution->isChecked();
  usePcdInputFile = ui->UsePcdInputBox->isChecked();
  downSampleModel = ui->DownSampleBox->isChecked();
  modelFileName = (ui->inputModelBox->text()).toStdString();
}
//value change handlers
void PCLViewer::leafSizeValueChanged(double value){
  downSampleLeafSize = value;
  cloudFilteringComplete = false;

}
void PCLViewer::CubeEdgeLengthChanged(double value){
  edgeLength = value;
}
void PCLViewer::CubeEdgePointsChanged(double value){
  edgePoints = static_cast<int>(value);
}
void PCLViewer::gamma21ValueChanged(double value){
  gamma21 = value;
}
void PCLViewer::gamma32ValueChanged(double value){
  gamma32 = value;
}
void PCLViewer::salientRadiusValueChanged(double value){
  salientRadius = value;
}
void PCLViewer::normalRadiusValueChanged(double value){
  normalRadius = value;
}
void PCLViewer::borderRadiusValueChanged(double value){
  borderRadius = value;
}
void PCLViewer::nonMaxSuppValueChanged(double value){
  nonMaxSuppRadius = value;
}
void PCLViewer::minNeighborsValueChanged(double value){
  minNeighbors = static_cast<int>(value);
}
//multiplier handlers
void PCLViewer::borderRadiusMultChanged(double value){
  borderRadiusMult = value;
}
void PCLViewer::normalRadiusMultChanged(double value){
  normalRadiusMult = value;
}
void PCLViewer::salientRadiusMultChanged(double value){
  salientRadiusMult = value;
}
void PCLViewer::nonMaxSuppMultChanged(double value){
  nonMaxSuppMult = value;
}

//button press handlers
void PCLViewer::MultiplyResolutionButtonPressed(){
  useMultipliers = true;
}
void PCLViewer::NonMultiplyButtonPressed(){
  useMultipliers = false;
}
void PCLViewer::CalculateKeypointsButtonPressed(){
  ParametersChanged();
  if(!usePcdInputFile || modelFileName == ""){
    std::cout<<"UseInputBox not checked or model filename is empty"<<std::endl;
    std::cout<<"current model name: "<<modelFileName<<std::endl;
    cube.setEdgePoints(edgePoints);
    cube.setEdgeLength(edgeLength);
    cube.GenerateCube();
    pcl::copyPointCloud(*cube.GetCube(),*cloud);
  }else{
    if(!pcdFileLoaded){
      pcl::io::loadPCDFile (modelFileName, *cloud);
      pcdFileLoaded = true;
    }
  }
  if(downSampleModel){
    if(!cloudFilteringComplete){
      DownSampleModel();
      cloudFilteringComplete = true;
    }
    issk.setInputCloud(filtered_cloud);
  }else{
    issk.setInputCloud(cloud);
  }
  issk.CalcNormals();
  double reso = issk.computeCloudResolution(cloud);
  std::cout<<"input cloud has "<<cloud->points.size()<<" points and a resolution of "<<reso<<std::endl;
  if(useMultipliers){
    issk.setUseMultipliers(true);
    issk.setNormRadMultiplier(normalRadiusMult);
    issk.setBordRadMultiplier(borderRadiusMult);
    issk.setSalRadMultiplier(salientRadiusMult);
    issk.setNonMaxMultiplier(nonMaxSuppMult);

    ui->borderRadiusBox_2->setValue(borderRadiusMult*reso);
    ui->normalRadiusBox_2->setValue(normalRadiusMult*reso);
    ui->salientRadiusBox_2->setValue(salientRadiusMult*reso);
    ui->nonMaxSuppBox_2->setValue(nonMaxSuppMult*reso);
  }else{
    issk.setUseMultipliers(false);
    issk.setNormRadius(normalRadius);
    issk.setNonMaxRadius(nonMaxSuppRadius);
    issk.setBordRadius(borderRadius);
    issk.setSalRadius(salientRadius);

    ui->borderRadiusBox->setValue(borderRadius/reso);
    ui->normalRadiusBox->setValue(normalRadius/reso);
    ui->nonMaxSuppBox->setValue(nonMaxSuppRadius/reso);
    ui->salientRadiusBox->setValue(salientRadius/reso);
  }
  ui->ModelResolutionDisp->display(reso);
  issk.setGamma32(gamma32);
  issk.setGamma21(gamma21);
  issk.setNeighbors(minNeighbors);
  issk.ComputeISSKeypoints();
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::copyPointCloud(*(issk.keypoints),*keypoints);
  std::cout<<"obtained keypoints cloud with : "<<keypoints->points.size()<<std::endl;

  viewer->removeAllPointClouds();
  pcl::visualization::PointCloudColorHandlerCustom<PointT> keypoint_color_handler (keypoints, 255, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<PointT> cube_color_handler(cloud,255,255,255);
  if(!downSampleModel)
    viewer->addPointCloud(cloud,cube_color_handler,"cubeCloud");
  else 
    viewer->addPointCloud(filtered_cloud,cube_color_handler,"cubeCloud");
  viewer->addPointCloud(keypoints,keypoint_color_handler,"keypointcloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5,"keypointcloud");
  ui->qvtkWidget->update();
}
void PCLViewer::DefaultValuesButtonPressed(){
  std::cout<<"placeholder for the default values button no actual functionality yet"<<std::endl;
  
}




PCLViewer::~PCLViewer ()
{
  delete ui;
}
