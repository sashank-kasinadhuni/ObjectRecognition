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
  tree.reset(new pcl::search::KdTree<PointT>);


  // Set up the QVTK window
  viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
  ui->qvtkWidget->SetRenderWindow (viewer->getRenderWindow ());
  viewer->setupInteractor (ui->qvtkWidget->GetInteractor (), ui->qvtkWidget->GetRenderWindow ());
  ui->qvtkWidget->update ();
  edgePoints = 10;
  edgeLength = 0.4;
  usePcdInputFile = false;
  pcdFileLoaded=false;
  cloudFilteringComplete = false;
  downSampleModel = false;
  useMLS = false;
  hk_radius = 0.01;
  hk_threshold = 0;
  mlsSearchRadius = 0.03;

 //connecting buttons
  connect (ui->CalculateKeypoints,  SIGNAL (clicked ()), this, SLOT (CalculateKeypointsButtonPressed()));
  connect (ui->defaultValues,  SIGNAL (clicked ()), this, SLOT (DefaultValuesButtonPressed()));
  connect (ui->inputModelBox,SIGNAL(textChanged(QString)),this,SLOT(modelFileNameChanged(QString)));
  connect (ui->mlsSearchRadiusBox,SIGNAL(valueChanged(double)),this,SLOT(mlsSearchRadiusValueChanged(double)));
  connect(ui->downSampleLeafSizeBox,SIGNAL(valueChanged(double)),this,SLOT(leafSizeValueChanged(double)));
  cube.setEdgePoints(10);
  cube.setEdgeLength(0.4);

  cube.GenerateCube();
  pcl::copyPointCloud(*cube.GetCube(),*cloud);
  hKeypoints.setInputCloud(cloud);
 
  pcl::visualization::PointCloudColorHandlerCustom<PointT> color_handler (cloud, 255, 255, 255);
  viewer->addPointCloud (cloud,color_handler, "cloud");
  viewer->addCoordinateSystem();

  //XYZsliderChanged();
  viewer->resetCamera ();
  ui->qvtkWidget->update ();
}

void PCLViewer::DownSampleModel(){
  if(!useMLS)
    vox.setInputCloud(cloud);
  else{
    if(!mlsComplete){
      pcl::PointCloud<pcl::PointNormal>::Ptr mls_points(new pcl::PointCloud<pcl::PointNormal>);
      pcl::MovingLeastSquares<PointT, pcl::PointNormal> mls;
      mls.setComputeNormals (true);
      mls.setInputCloud (cloud);
      mls.setPolynomialFit (true);
      mls.setSearchMethod (tree);
      mls.setSearchRadius (mlsSearchRadius);
      mls.process(*mls_points);
      pcl::copyPointCloud(*mls_points,*postMLS_cloud);
      mlsComplete = true;
    }
    vox.setInputCloud(postMLS_cloud);
  }
  vox.setLeafSize(downSampleLeafSize,downSampleLeafSize,downSampleLeafSize);
  vox.filter(*filtered_cloud);
  cloudFilteringComplete = true;
}
void PCLViewer::modelFileNameChanged(QString mfilename){
  //std::cout<<"model filename changed to "<<mfilename.toStdString()<<std::endl;
  modelFileName = mfilename.toStdString();
  pcdFileLoaded = false;
}
void PCLViewer::ParametersChanged(){
  usePcdInputFile = ui->UsePcdInputBox->isChecked();
  modelFileName = (ui->inputModelBox->text()).toStdString();
  
  edgeLength = ui->CubeEdgeLengthBox->value();
  edgePoints = static_cast<int>(ui->CubeEdgePointsBox->value());

  downSampleModel = ui->DownSampleBox->isChecked();
  downSampleLeafSize = ui->downSampleLeafSizeBox->value();
  
  hk_do_refine = ui->refineKeypointsBox->isChecked();
  useMLS =  ui->useMLSBox->isChecked();
  mlsSearchRadius = ui->mlsSearchRadiusBox->value();

  hk_radius = ui->nonMaxNormalRadiusBox->value();
  hk_threshold = ui->thresholdBox->value();
  hk_resMethod = (ui->responseMethodBox->currentText()).toStdString();

}
//value change handlers
void PCLViewer::leafSizeValueChanged(double value){
  downSampleLeafSize = value;
  cloudFilteringComplete = false;
}
void PCLViewer::mlsSearchRadiusValueChanged(double radius){
  mlsSearchRadius = radius;
  mlsComplete = false;
}

//button press handlers
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
    }
    hKeypoints.setInputCloud(filtered_cloud);
  }else{
    hKeypoints.setInputCloud(cloud);
  }
  std::cout<<"input cloud has "<<cloud->points.size()<<" points"<<std::endl;
  
  hKeypoints.setRadius(hk_radius);
  hKeypoints.setThreshold(hk_threshold);
  hKeypoints.setRefine(hk_do_refine);
  hKeypoints.setNumberOfThreads(4);
  hKeypoints.setResponseMethod(hk_resMethod);

  
  PointCloudT::Ptr keypoints(new PointCloudT);
  pcl::copyPointCloud(*(hKeypoints.keypoints),*keypoints);
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
