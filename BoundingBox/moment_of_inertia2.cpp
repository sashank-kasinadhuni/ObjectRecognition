#include "boundingBox.h"
#include <boost/thread/thread.hpp>

int main (int argc, char** argv)
{
  if (argc != 2)
    return (0);

  PointCloudT::Ptr cloud (new PointCloudT());
  if (pcl::io::loadPCDFile (argv[1], *cloud) == -1)
    return (-1);
	
	BoundingBox box(cloud);
	box.calculate_OBB();
	box.calculate_AABB();
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
	viewer->addCoordinateSystem (1.0);
	viewer->initCameraParameters ();
	viewer->addPointCloud<PointT> (cloud, "sample cloud");

	box.show_OBB(biewer);
	box.show_AABB(viewer);
	while(!viewer->wasStopped()){
		viewer->spinOnce();
	}
}
