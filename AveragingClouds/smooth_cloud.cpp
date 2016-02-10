#include "smooth_cloud.h"
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<pcl::PointNormal> PointNormalCloudT;
typedef pcl::PointCloud<pcl::PointXYZRGBNormal> PointColorNormalCloud;
void SmoothCloud::setInputCloud(PointColorNormalCloudT::Ptr inputCloud){
	if(accumulator.size() < acc_size){
		accumulator.push_back(inputCloud);
		ready = false;
	}
	else{
		res_cloud->points.clear();
		ProcessClouds();
		ready = true;
	}
}
void SmoothCloud::CloudSizes(){
	for(int i = 0;i<accumulator.size();i++){
		std::cout<<"cloud #"<<i<<" size: "<<accumulator[i]->points.size()<<std::endl;
	}
}

void SmoothCloud::ProcessClouds(){
	for(int i = 0;i<307200;i++)
	{
		int valid_points = 0;
		int valid_normals = 0;
		pcl::PointXYZRGBNormal avg_point;
		avg_point.x = std::numeric_limits<float>::quiet_NaN();
		avg_point.y = std::numeric_limits<float>::quiet_NaN();
		avg_point.z = std::numeric_limits<float>::quiet_NaN();
		avg_point.rgb = std::numeric_limits<float>::quiet_NaN();
		avg_point.normal_x = std::numeric_limits<float>::quiet_NaN();
		avg_point.normal_y = std::numeric_limits<float>::quiet_NaN();
		avg_point.normal_z = std::numeric_limits<float>::quiet_NaN();
		bool rgbIsSet = false;
		for(int j = 0;j<accumulator.size();j++)
		{
			if(!isnan((accumulator[j])->points[i].x))
			{
				if(!isnan(avg_point.x))
				{
					avg_point.x += (accumulator[j])->points[i].x;
					avg_point.y += (accumulator[j])->points[i].y;
					avg_point.z += (accumulator[j])->points[i].z;
					if(!rgbIsSet){
						avg_point.rgb = (accumulator[j])->points[i].rgb;
						rgbIsSet = true;
					}
					valid_points += 1;
				}
				else
				{
					avg_point.x = (accumulator[j])->points[i].x;
					avg_point.y = (accumulator[j])->points[i].y;
					avg_point.z = (accumulator[j])->points[i].z;
					valid_points += 1;
				}
			}
			if(!isnan((accumulator[j])->points[i].normal_x))
			{
				if(!isnan(avg_point.normal_x))
				{
					avg_point.normal_x += (accumulator[j])->points[i].normal_x;
					avg_point.normal_y += (accumulator[j])->points[i].normal_y;
					avg_point.normal_z += (accumulator[j])->points[i].normal_z;
					valid_normals += 1;
				}
				else
				{
					avg_point.normal_x = (accumulator[j])->points[i].normal_x;
					avg_point.normal_y = (accumulator[j])->points[i].normal_y;
					avg_point.normal_z = (accumulator[j])->points[i].normal_z;
					valid_normals += 1;
				}
			}
		}
		if(valid_points != 0)
		{
			avg_point.x = avg_point.x / valid_points;
			avg_point.y = avg_point.y / valid_points;
			avg_point.z = avg_point.z / valid_points;
			if(valid_normals != 0){
				avg_point.normal_x = avg_point.normal_x / valid_normals;
				avg_point.normal_y = avg_point.normal_y / valid_normals;
				avg_point.normal_z = avg_point.normal_z / valid_normals;
			}
		}
		res_cloud->points.push_back(avg_point);
	}
	res_cloud->height = 480;
	res_cloud->width = 640;
	res_cloud->points.resize(res_cloud->height*res_cloud->width);
}