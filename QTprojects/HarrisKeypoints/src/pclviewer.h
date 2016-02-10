#ifndef PCLVIEWER_H
#define PCLVIEWER_H

#include "keypoints.h"
#include "CubeCreator.h"

#include <iostream>
#include <sstream>
#include <string>
#include <fstream>

// Qt
#include <QMainWindow>

// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/surface/mls.h>
// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>

//typedef pcl::PointXYZRGBA PointT;
//typedef pcl::PointCloud<PointT> PointCloudT;

namespace Ui
{
  class PCLViewer;
}

class PCLViewer : public QMainWindow
{
  Q_OBJECT

public:
  explicit PCLViewer (QWidget *parent = 0);
  ~PCLViewer ();

public slots:

  void CalculateKeypointsButtonPressed();
  void DefaultValuesButtonPressed();
  void DownSampleModel();
  void modelFileNameChanged(QString mfilename);
  void leafSizeValueChanged(double leafsize);
  void ParametersChanged();
  void mlsSearchRadiusValueChanged(double radius);


protected:
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  PointCloudT::Ptr cloud;
  PointCloudT::Ptr filtered_cloud;
  PointCloudT::Ptr postMLS_cloud;

  HarrisKeypoints hKeypoints;
  CubeCreator cube;
  pcl::search::KdTree<PointT>::Ptr tree;

  int edgePoints;
  double edgeLength;
  std::string hk_resMethod;
  double hk_radius;
  double hk_threshold;
  double mlsSearchRadius;
  double downSampleLeafSize;
  bool hk_do_refine;
  bool usePcdInputFile;
  bool pcdFileLoaded;
  bool downSampleModel;
  bool cloudFilteringComplete;
  bool mlsComplete;
  bool useMLS;
  pcl::VoxelGrid<PointT> vox;
  std::string modelFileName;

private:
  Ui::PCLViewer *ui;

};

#endif // PCLVIEWER_H
