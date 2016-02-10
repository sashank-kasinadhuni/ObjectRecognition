#include "CubeCreator.h"
void CubeCreator::setEdgeLength(double length){
	edge_size = length;
}
void CubeCreator::resetCube(){
  this->cubecloud.reset(new PointCloudT);
}
void CubeCreator::setEdgePoints(int Points){
	edge_points = Points;
}
void CubeCreator::setEdgePoints(double Points){
	edge_points = static_cast<int>(Points);
}
void CubeCreator::GenerateCube(){
  float step = (edge_size*2)/edge_points;
  resetCube();
  for(float x = -edge_size;x<=edge_size+step;x+=step)
  {
    for(float y = -edge_size;y<=edge_size+step;y+=step)
      {
        for(float z = -edge_size;z<=edge_size+step;z+=step)
          {
              if(x == -edge_size ||x >= edge_size|| y == -edge_size || y>=edge_size || z==-edge_size || z>=edge_size) {
                pcl::PointXYZRGBA temp;
                temp.x=x;
                temp.y=y;
                temp.z=z;
                (*cubecloud).points.push_back(temp);
              }
          }         
      }  
  }
}