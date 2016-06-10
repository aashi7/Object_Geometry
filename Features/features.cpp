#include <iostream>
#include <pcl/common/impl/common.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/ply_io.h>
#include <math.h> 
#include <pcl/features/normal_3d.h>
#include <pcl/console/parse.h>
#include <pcl/octree/octree.h>

#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <iterator>

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

void
showHelp(char * program_name)
{
  std::cout << std::endl;
  std::cout << "Usage: " << program_name << " cloud_filename.[pcd|ply]" << std::endl;
  std::cout << "-h:  Show this help." << std::endl;
}


float calculateAreaPolygon(const pcl::PointCloud<pcl::PointXYZ> &polygon )
{
        float area=0.0;
        int num_points = polygon.size();
  int j = 0;
        Eigen::Vector3f va,vb,res;
        res(0) = res(1) = res(2) = 0.0f;
  for (int i = 0; i < num_points; ++i)
  {
      j = (i + 1) % num_points;
                        va = polygon[i].getVector3fMap();
                        vb = polygon[j].getVector3fMap();
      res += va.cross(vb);
  }
  area=res.norm();
        return area*0.5;
} 

int main (int argc, char** argv)
{

	// Fill in the cloud data
	if (pcl::console::find_switch (argc, argv, "-h") || pcl::console::find_switch (argc, argv, "--help")) {
		showHelp (argv[0]);
		return 0;
	}

	// Fetch point cloud filename in arguments | Works with PCD and PLY files
	std::vector<int> filenames;
	bool file_is_pcd = false;

	filenames = pcl::console::parse_file_extension_argument (argc, argv, ".ply");

	if (filenames.size () != 1)  {
		filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");

		if (filenames.size () != 1) {
		  showHelp (argv[0]);
		  return -1;
		} else {
		file_is_pcd = true;
		}
	}

	std::ofstream output_file("properties.txt");
	std::ofstream curvature("curvature.txt");
	std::ofstream normals("normals.txt");

	pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
	pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());


	pcl::io::loadPCDFile (argv[filenames[0]], *cloud); 
	
	/*pcl::PLYReader reader;
	// Replace the path below with the path where you saved your file
	reader.read ("poisson_mesh.ply", *cloud); // Remember to download the file first!
	*/
	std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
		<< " data points (" << pcl::getFieldsList (*cloud) << ")." << std::endl;

	// Create the filtering object
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud (cloud);
	sor.setLeafSize (0.01f, 0.01f, 0.01f);
	sor.filter (*cloud_filtered);

	output_file << "Number of filtered points" << std::endl;
	output_file << cloud_filtered->width * cloud_filtered->height<<std::endl;

	std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
		<< " data points (" << pcl::getFieldsList (*cloud_filtered) << ")." << std::endl;

	/*pcl::PCDWriter writer;
	writer.write ("poisson_mesh_downsampled.pcd", *cloud_filtered, 
			Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);
	*/

	pcl::PointCloud<pcl::PointXYZ>::Ptr descriptor (new pcl::PointCloud<pcl::PointXYZ>);

	descriptor->width = 5000 ;
	descriptor->height = 1 ;
	descriptor->points.resize (descriptor->width * descriptor->height) ;
	std::cerr << descriptor->points.size() << std::endl ;

	 pcl::PCDWriter writer;
  	 writer.write ("out.pcd", *cloud_filtered, Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_converted (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ> test;

	pcl::PCDReader reader_ball;
	reader_ball.read ("out.pcd", *cloud_filtered_converted);
	reader_ball.read ("out.pcd", *test);	
	std::cout<<data.size()<<std::endl;	
	std::vector<pcl::PointXYZ> data = test.points;
	// Create the normal estimation class, and pass the input dataset to it
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;	
	
	ne.setInputCloud (cloud_filtered_converted);

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

	output_file << "size of first file points " << cloud_filtered_converted->points.size() << std::endl ;

	output_file << "size of the normals " << cloud_normals->points.size() << std::endl ; 	

	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);

	int k=0 ;
	float dist ;
	float square_of_dist ;
	float x1,y1,z1,x2,y2,z2 ;
	float nu[3], nv[3], pv_pu[3], pu_pv[3] ;
	float highest = cloud_filtered_converted->points[0].z;
	float lowest = cloud_filtered_converted->points[0].z;


	for ( int i = 0; i < cloud_normals->points.size() ; i++)
	{
		if(cloud_filtered_converted->points[i].z > highest)
		{
			highest = cloud_filtered_converted->points[i].z;
		}
		if(cloud_filtered_converted->points[i].z < lowest)
		{
                        lowest = cloud_filtered_converted->points[i].z;
                }
		normals <<i <<": "<<" x-normal-> "<<cloud_normals->points[i].normal_x<<" y-normal-> "<<cloud_normals->points[i].normal_y<<" z-normal-> "<<cloud_normals->points[i].normal_z<<std::endl;
		curvature <<i <<": curvature: "<<cloud_normals->points[i].curvature<<std::endl;
		
		/*
		pcl::PointXYZRGB point;
		point.x = cloud_filtered_converted->points[i].x;
		point.y = cloud_filtered_converted->points[i].y;
		point.z = cloud_filtered_converted->points[i].z;
		point.r = 0;
		point.g = 100;
		point.b = 200;
		point_cloud_ptr->points.push_back (point);
 		*/
	}
	output_file << "highest point: "<< highest<<std::endl;
	output_file << "lowest point: "<< lowest<<std::endl;	
	//pcl::PointCloud<pcl::PointXYZ>::Ptr test (new pcl::PointCloud<pcl::PointXYZ>);
	//float surface_area = calculateAreaPolygon(test);
        //std::cout<< surface_area<<std::endl;

	/*

	descriptor->width = k ;
	descriptor->height = 1 ;
	descriptor->points.resize (descriptor->width * descriptor->height) ;
	std::cerr << descriptor->points.size() << std::endl ;
	float voxelSize = 0.01f ;  // how to find appropriate voxel resolution
	pcl::octree::OctreePointCloud<pcl::PointXYZ> octree (voxelSize);
	octree.setInputCloud(descriptor) ;
	ctree.defineBoundingBox(0.0,0.0,0.0,3.14,3.14,3.14) ;   //octree.defineBoundingBox (minX, minY, minZ, maxX, maxY, maxZ)
	octree.addPointsFromInputCloud ();   // octree created for block

	int k_ball=0 ;
	float dist_ball ;
	float square_of_dist_ball ;
	double X,Y,Z ;
	bool occupied ;
	highest = cloud_ball->points[0].z;

	for ( int i = 0; i < cloud_normals_ball->points.size() ; i++)
	{
		if(cloud->points[i].z > highest){
			highest = cloud_ball->points[i].z;
		}
		for (int j = i+1 ; (j < cloud_normals_ball->points.size()) ; j++)     
		  {
			  x1 = cloud_ball->points[i].x ;
			  y1 = cloud_ball->points[i].y ;
			  z1 = cloud_ball->points[i].z ;
			  nu[0] = cloud_normals_ball->points[i].normal_x ;
			  nu[1] = cloud_normals_ball->points[i].normal_y ;
			  nu[2] = cloud_normals_ball->points[i].normal_z ;
			  x2 = cloud_ball->points[j].x ;
			  y2 = cloud_ball->points[j].y ;
			  z2 = cloud_ball->points[j].z ;
			  nv[0] = cloud_normals_ball->points[j].normal_x ;
			  nv[1] = cloud_normals_ball->points[j].normal_y ;
			  nv[2] = cloud_normals_ball->points[j].normal_z ;
			  square_of_dist = ((x2-x1)*(x2-x1)) + ((y2-y1)*(y2-y1)) + ((z2-z1)*(z2-z1)) ;
			  dist = sqrt(square_of_dist) ;
			//std::cerr << dist ;
			pv_pu[0] = x2-x1 ;
			pv_pu[1] = y2-y1 ;
			pv_pu[2] = z2-z1 ;
			pu_pv[0] = x1-x2 ;
			pu_pv[1] = y1-y2 ;
			pu_pv[2] = z1-z2 ;
			if ((dist > 0.0099) && (dist < 0.0101))
			{
				X = angle_between_vectors (nu, nv) ;
				Y  = angle_between_vectors (nu, pv_pu) ;
				Z = angle_between_vectors (nv, pu_pv) ;
				// output_file << descriptor->points[k].x << "\t" << descriptor->points[k].y << "\t" << descriptor->points[k].z  ;
				// output_file << "\n";	
				//k_ball = k_ball + 1 ;
				occupied = octree.isVoxelOccupiedAtPoint (X,Y,Z) ;
				if (occupied == 1)
				{
				//k_ball = k_ball + 1 ;
				std::cerr << "Objects Matched" << "\t" << k_ball << std::endl ;
				return(0); 
				}

			}

		}

	}	

	*/
	
	/*

	points.open("secondItemPoints.txt");
	myfile<<"Second point \n";
	myfile<<"second highest "<<highest;
	for(int i = 0; i < cloud_normals->points.size(); i++){
		points<<cloud->points[i].x<<", "<<cloud->points[i].y<<", "<<cloud->points[i].z<<"\n";
		if(cloud->points[i].z >= highest - (highest/100)){
			myfile<<cloud->points[i].x<<", "<<cloud->points[i].y<<", "<<cloud->points[i].z<<"\n";
		}
	}
	points.close();
	myfile.close();

	std::cerr << "Objects Not Matched" << "\t" << k_ball << std::endl ;
	
	*/
	
	//output_file <<"Volume: "<<volume <<std::endl;
	//output_file <<"Surface Area: "<<surface_area <<std::endl;

	return (0);
}
