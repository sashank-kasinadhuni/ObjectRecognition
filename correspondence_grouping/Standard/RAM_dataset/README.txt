The scene from this dataset were captured using a Kinect sensor by Aitor Aldoma, Christian Potthast 
and Bernhard Zeisl at Willow Garage. The files are in PCD format (http://pointclouds.org/documentation/tutorials/pcd_file_format.php)

If you use this dataset please cite the following paper:
TODO: Add citation after paper is published.

They are organized as follows:
	- pcd_files/no_occlusions/
		Objects can be segmented using a dominant plane assumption and euclidean clustering and the objects do not occlude each other.
	- pcd_files/occlusions/
		Objects might not be able to be segmented with the previous approach and objects might occlude each other.
	- pcd_files/special/
		Only two scenes with lots of objects close to each other and high levels of occlusions.

The CAD models under cad_models are in PLY format and form a subset of the household_objects_database from Willow Garage:
http://www.ros.org/wiki/household_objects_database
NOTE: The model's coordinates are in milimeters while the rest (PCD files, transforms and distances in meters)

The ground truth data containing object identifier and 6D0F pose can be found under gt_files and follow the same no_occlusions, occlusions and special
subdirectory structure. The ground truth file is as follows:

*********************************************************
pcd_filename [The filename of the PCD file]
num_objects [Number of objects in the scene]
max_z_distance [From this distance on, the data can be ignored (see pcl::PassThrough filter)]
[Repeat for num_objects]
model_id - Identifier of the model
transform - 4x4 matrix aligning the PLY model to the scene (row order)
*********************************************************

Example: gt_files/no_occlusions/frame_20111220T115445.303284.txt

*********************************************************
occlusions/frame_20111220T115445.303284.pcd
4
1.5
300.151.23.ply
-0.000882239 0.000468459 -4.73472e-05 0.0203809 
-0.000375554 -0.000639463 0.000670867 -0.205065 
0.000283991 0.000609636 0.000740082 0.75608 
0 0 0 1 

800.919.49.dec.ply
0.000730478 -0.000681744 -4.06187e-05 -0.0518446 
0.000542305 0.00054286 0.00064128 -0.123598 
-0.000415133 -0.000490459 0.000766251 0.733512 
0 0 0 1 

201.327.78.ply
-0.000878438 0.000471301 -7.9122e-05 0.0827787 
-0.000404899 -0.000646044 0.000647078 -0.254666 
0.000253843 0.000600436 0.000758325 0.833869 
0 0 0 1 

963.111.00.dec.ply
0.000824408 0.000564035 -4.69931e-05 -0.114332 
-0.000403223 0.000643564 0.000650574 -0.106565 
0.000397187 -0.000517386 0.000757996 0.610158 
0 0 0 1 
*********************************************************

