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
// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

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
  void MultiplyResolutionButtonPressed();
  void NonMultiplyButtonPressed();
  void DefaultValuesButtonPressed();

  void CubeEdgeLengthChanged(double value);
  void CubeEdgePointsChanged(double value);

  void gamma21ValueChanged(double value);
  void gamma32ValueChanged(double value);
  void nonMaxSuppMultChanged(double value);
  void salientRadiusMultChanged(double value);
  void normalRadiusMultChanged(double value);
  void borderRadiusMultChanged(double value);
  void salientRadiusValueChanged(double value);
  void normalRadiusValueChanged(double value);
  void borderRadiusValueChanged(double value);
  void nonMaxSuppValueChanged(double value);
  void minNeighborsValueChanged(double value);
  void ParametersChanged();
  void DownSampleModel();
  void modelFileNameChanged(QString mfilename);
  void leafSizeValueChanged(double value);
protected:
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  PointCloudT::Ptr cloud;
  PointCloudT::Ptr filtered_cloud;
  ISSKeypoints issk;
  CubeCreator cube;

  int edgePoints;
  int minNeighbors;
  double downSampleLeafSize;
  double edgeLength;
  double nonMaxSuppMult;
  double salientRadiusMult;
  double normalRadiusMult;
  double borderRadiusMult;
  double gamma21;
  double gamma32;
  double nonMaxSuppRadius;
  double salientRadius;
  double borderRadius;
  double normalRadius;
  bool useMultipliers;
  bool usePcdInputFile;
  bool pcdFileLoaded;
  bool downSampleModel;
  bool cloudFilteringComplete;
  pcl::VoxelGrid<PointT> vox;
  std::string modelFileName;

private:
  Ui::PCLViewer *ui;

};

#endif // PCLVIEWER_H
