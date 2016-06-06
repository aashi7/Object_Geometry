#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/ply_io.h>
#include <math.h> 
#include <pcl/features/normal_3d.h>

#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <iterator>

#include <pcl/octree/octree.h>

float angle_between_vectors (float *nu, float *nv)
{
float l1, l2, angle, param ;
l1 = sqrt(nu[0]*nu[0] + nu[1]*nu[1] + nu[2]*nu[2]) ;
l2 = sqrt(nv[0]*nv[0] + nv[1]*nv[1] + nv[2]*nv[2]) ;
float dot ;
dot = nu[0]*nv[0] + nu[1]*nv[1] + nu[2]*nv[2] ;
param = dot/(l1*l2) ;
//if (param < 0)
//param = -(param) ;
angle = std::acos(param) ;
angle = floor(angle*100 + 0.5)/100 ;  // round off to two decimal places
return angle ;
}

/*struct descriptor_vector {
float f2 ;
float f3 ;
float f4 ;
};*/

int
main (int argc, char** argv)
{
	
  float p[3];  //point coordinates
  float n[3];  //normal vector
  float u[6];  //oriented point

  /*pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
  pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());

  // Fill in the cloud data
  pcl::PLYReader reader;
  // Replace the path below with the path where you saved your file
  reader.read ("poisson_mesh.ply", *cloud); // Remember to download the file first!

  std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
       << " data points (" << pcl::getFieldsList (*cloud) << ")." << std::endl;

  // Create the filtering object
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.filter (*cloud_filtered);

  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")." << std::endl;

  pcl::PCDWriter writer;
  writer.write ("poisson_mesh_downsampled.pcd", *cloud_filtered, 
         Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);
 */

  std::ofstream output_file("output_angles_jun6.csv");

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr descriptor (new pcl::PointCloud<pcl::PointXYZ>);

  descriptor->width = 5000 ;
  descriptor->height = 1 ;
  descriptor->points.resize (descriptor->width * descriptor->height) ;
 std::cerr << descriptor->points.size() << std::endl ;

  pcl::PCDReader reader;
  // Replace the path below with the path where you saved your file
  reader.read ("poisson_mesh_downsampled.pcd", *cloud); // Remember to download the file first!

  
  //descriptor_vector * descriptor = new descriptor_vector[460000] ;
 	
   // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (cloud);

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);

  // Output datasets
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

  // Use all neighbors in a sphere of radius 3cm
  ne.setRadiusSearch (0.03);

  // Compute the features
  ne.compute (*cloud_normals);
  //std::cerr << "size of the normals " << cloud_normals->points.size() << std::endl ; 

/*  for ( int i = 0; i < cloud_normals->points.size() ; i++)
  {
  std::cerr << "Coordinates of point are " <<  " x:" << cloud->points[i].x << " y:" << cloud->points[i].y << " z:" << cloud->points[i].z << std::endl ;
 // std::cerr << "coordinates of the normal vector of " << i << " point " << "curvature: " << cloud_normals->points[i].curvature << " x:" << cloud_normals->points[i].normal_x << " y:"  << cloud_normals->points[i].normal_y << " z:" << cloud_normals->points[i].normal_z <<  std::endl ;
  u[0] = cloud->points[i].x ;
  u[1] = cloud->points[i].y ; 
  u[2] = cloud->points[i].z ;
  u[3] = cloud_normals->points[i].normal_x ;
  u[4] = cloud_normals->points[i].normal_y ;
  u[5] = cloud_normals->points[i].normal_z ;
 //std::cerr << ((cloud_normals->points[i].normal_x)*(cloud_normals->points[i].normal_x)) + ((cloud_normals->points[i].normal_y)*(cloud_normals->points[i].normal_y)) + ((cloud_normals->points[i].normal_z)*(cloud_normals->points[i].normal_z)) << std::endl ;
  }*/
	int k=0 ;
 	float dist ;
        float square_of_dist ;
	float x1,y1,z1,x2,y2,z2 ;
	float nu[3], nv[3], pv_pu[3], pu_pv[3] ;

  for ( int i = 0; i < cloud_normals->points.size() ; i++)
   {
 	for (int j = i+1 ; (j < cloud_normals->points.size()) ; j++)     
           {
			x1 = cloud->points[i].x ;
			y1 = cloud->points[i].y ;
			z1 = cloud->points[i].z ;
			nu[0] = cloud_normals->points[i].normal_x ;
                        nu[1] = cloud_normals->points[i].normal_y ;
                        nu[2] = cloud_normals->points[i].normal_z ;
			x2 = cloud->points[j].x ;
			y2 = cloud->points[j].y ;
			z2 = cloud->points[j].z ;
			nv[0] = cloud_normals->points[j].normal_x ;
                        nv[1] = cloud_normals->points[j].normal_y ;
                        nv[2] = cloud_normals->points[j].normal_z ;
			square_of_dist = ((x2-x1)*(x2-x1)) + ((y2-y1)*(y2-y1)) + ((z2-z1)*(z2-z1)) ;
			dist = sqrt(square_of_dist) ;
			//std::cerr << dist ;
			pv_pu[0] = x2-x1 ;
			pv_pu[1] = y2-y1 ;
			pv_pu[2] = z2-z1 ;
			pu_pv[0] = x1-x2 ;
			pu_pv[1] = y1-y2 ;
			pu_pv[2] = z1-z2 ;
			if ((dist > 0.029) && (dist < 0.031))
			{
		              descriptor->points[k].x = angle_between_vectors (nu, nv) ;
			      descriptor->points[k].y = angle_between_vectors (nu, pv_pu) ;
			      descriptor->points[k].z = angle_between_vectors (nv, pu_pv) ;
			      output_file << descriptor->points[k].x << "\t" << descriptor->points[k].y << "\t" << descriptor->points[k].z  ;
                              output_file << "\n";	
			      k=k+1 ;
			}
                              
           }
   }

         descriptor->width = k ;
         descriptor->height = 1 ;
         descriptor->points.resize (descriptor->width * descriptor->height) ;
         std::cerr << descriptor->points.size() << std::endl ;
	float voxelSize = 0.01f ;  // how to find appropriate voxel resolution
	pcl::octree::OctreePointCloud<pcl::PointXYZ> octree (voxelSize);
	octree.setInputCloud(descriptor) ;
        octree.defineBoundingBox(0.0,0.0,0.0,3.14,3.14,3.14) ;   //octree.defineBoundingBox (minX, minY, minZ, maxX, maxY, maxZ)
	octree.addPointsFromInputCloud ();
 
        //Check if voxel at given point coordinates exist
	double X,Y,Z ;
	bool occupied ;
	X = 0.02 ; Y = 1.55 ; Z = 1.58 ;
	occupied = octree.isVoxelOccupiedAtPoint (X,Y,Z) ;    

	

	std::cerr << occupied << std::endl ;	

	//std::cerr << descriptor->points[k-1].x << "\t" << descriptor->points[k-1].y << "\t" << descriptor->points[k-1].z  << std::endl ;

 return (0);
}
