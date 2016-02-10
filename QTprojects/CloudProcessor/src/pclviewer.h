#ifndef PCLVIEWER_H
#define PCLVIEWER_H

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

  void RandomColorButtonPressed();
  void SaveButtonPressed();

  void XYZSliderChanged();
  
  void xAngleSliderValueChanged (double value);
  void yAngleSliderValueChanged(double value);
  void zAngleSliderValueChanged (double value);

  void x_right_changed(double value);
  void x_left_changed(double value);
  void y_top_changed(double value);
  void y_bottom_changed(double value);
  void z_near_changed(double value);
  void z_far_changed(double value);

protected:
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  PointCloudT::Ptr cloud;
  PointCloudT::Ptr rotated_cloud;
  unsigned int red;
  unsigned int green;
  unsigned int blue;
  double x_rot;
  double y_rot;
  double z_rot;
  double x_left;
  double x_right;
  double y_top;
  double y_bottom;
  double z_near;
  double z_far;
private:
  Ui::PCLViewer *ui;

};

#endif // PCLVIEWER_H
