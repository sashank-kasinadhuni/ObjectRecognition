#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;


int
main (int argc, char** argv)
{
  bool binary = true;

  if(argc < 3) {
    std::cerr << "Usage:" << std::endl;
    std::cerr << argv[0] << " [-a] input.ply output.pcd" << std::endl;
    std::cerr << "\t-a\tASCII output" << std::endl;
    return (1);
  }

  if(argc == 4) {
    if(strncmp(argv[1],"-a",2) != 0) {
      std::cerr << "Error: unknown option!" << std::endl;
      return (1);
    }
    else {
      binary = false;
      argv += 1;
    }
  }



  pcl::PolygonMesh mesh;

  pcl::io::loadPolygonFilePLY(argv[1], mesh);

  std::cerr << "Read cloud: " << std::endl;
  pcl::io::saveVTKFile ("temp.vtk", mesh);

  // then use pcl_vtk2pcd

  vtkSmartPointer<vtkPolyDataReader> reader = vtkSmartPointer<vtkPolyDataReader>::New ();
  reader->SetFileName ("temp.vtk");
  reader->Update ();
  vtkSmartPointer<vtkPolyData> polydata = reader->GetOutput ();

  pcl::PointCloud<pcl::PointXYZ> cloud;
  vtkPolyDataToPointCloud (polydata, cloud);

  pcl::PCDWriter pcdwriter;
  pcdwriter.write<pcl::PointXYZ> (argv[2], cloud);

  return (0);
}
