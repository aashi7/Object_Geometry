#include <pcl/common/impl/common.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/ply_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/console/parse.h>
#include <pcl/octree/octree.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/gp3.h>
#include <pcl_conversions/pcl_conversions.h>

#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <iterator>
#include <math.h>

/* FUNCTION DECLARATIONS *************************************************************************************************************/

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

void showHelp(char * program_name)
{
	std::cout << std::endl;
	std::cout << "Usage: " << program_name << " cloud_filename.[pcd|ply]" << std::endl;
	std::cout << "-h:  Show this help." << std::endl;
}


float calculateAreaPolygon(pcl::PolygonMesh &polygon, const pcl::PointCloud<pcl::PointXYZ> &cloud)
{
	std::cout<<"in calculateAreaPolgyon"<<std::endl;
	float area=0.0;
	int num_triangles = polygon.polygons.capacity();
	std::cout<<num_triangles<<" triangles"<<std::endl;
	int j, k, ax, ay, az, bx, by, bz, cx, cy, cz, acx, acy, acz, abx, aby, abz, x, y, z;
	for (int i = 0; i < 8; i++)
	{
		std::cout<<"1"<<std::endl;
		j = (i + 1) % num_triangles;
		std::cout<<"2"<<std::endl;
		k = (i + 2) % num_triangles;		
		std::cout<<polygon.polygons[i].vertices[1]<<std::endl;
		ax = cloud.points[polygon.polygons[i].vertices[0]].x;
		std::cout<<"2.5"<<std::endl;
		ay = cloud.points[polygon.polygons[i].vertices[0]].y;
		az = cloud.points[polygon.polygons[i].vertices[0]].z;
		bx = cloud.points[polygon.polygons[i].vertices[1]].x;
		by = cloud.points[polygon.polygons[i].vertices[1]].y;
		bz = cloud.points[polygon.polygons[i].vertices[1]].z;
		cx = cloud.points[polygon.polygons[i].vertices[2]].x;
		cy = cloud.points[polygon.polygons[i].vertices[2]].y;
		cz = cloud.points[polygon.polygons[i].vertices[2]].z;
		std::cout<<"3"<<std::endl;
		acx = cx-ax;
		acy = cy-ay;
		acz = cz-az;
        	abx = bx-ax;
		aby = by-ay;
        	abz = bz-az;
		x = acx*abz-abx*acz;
		y = acx*aby-abx*acz;
		z = acx*aby-abx*acy;
		area += sqrt(pow(x, 2.0) + pow(y, 2.0) + pow(z, 2.0));		
	}
	std::cout<<"end calculateAreaPolgyon"<<std::endl;
	return area*0.5;
} 


/*************************************************************************************************************************************/

int main (int argc, char** argv)
{

	/* DOWNSAMPLING ********************************************************************************************************************/
	std::ofstream output_file("properties.txt");
	std::ofstream curvature("curvature.txt");
	std::ofstream normals_text("normals.txt");

	/*

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


	pcl::PointCloud<pcl::PointXYZ>::Ptr descriptor (new pcl::PointCloud<pcl::PointXYZ>);

	descriptor->width = 5000 ;
	descriptor->height = 1 ;
	descriptor->points.resize (descriptor->width * descriptor->height) ;
	std::cerr << descriptor->points.size() << std::endl ;

	pcl::PCDWriter writer;
	writer.write ("out.pcd", *cloud_filtered, Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);

	*/
		
	/***********************************************************************************************************************************/

	/* CALCULATING VOLUME AND SURFACE AREA ********************************************************************************************************************/		

	/*pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
	  pcl::ConcaveHull<pcl::PointXYZ> chull;
	  chull.setInputCloud(test);

	  chull.reconstruct(*cloud_hull);*/

	/*for (size_t i=0; i < test->points.size (); i++)
	  {
	  std::cout<< test->points[i].x <<std::endl;
	  std::cout<< test->points[i].y <<std::endl;
	  std::cout<< test->points[i].z <<std::endl;
	  }*/
	//std::cout<<data.size()<<std::endl;

	// Load input file into a PointCloud<T> with an appropriate type
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCLPointCloud2 cloud_blob;
	pcl::io::loadPCDFile ("mini_soccer_ball_downsampled.pcd", cloud_blob);
	pcl::fromPCLPointCloud2 (cloud_blob, *cloud);		
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2 (cloud_blob, *cloud1);
		//* the data should be available in cloud

	// Normal estimation*
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud (cloud);
	n.setInputCloud (cloud);
	n.setSearchMethod (tree);
	n.setKSearch (20);
	n.compute (*normals);
	//* normals should not contain the point normals + surface curvatures

	// Concatenate the XYZ and normal fields*
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields (*cloud, *normals,*cloud_with_normals);
	//* cloud_with_normals = cloud + normals

	// Create search tree*
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud (cloud_with_normals);

	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	pcl::PolygonMesh triangles;

	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius (0.025);

	// Set typical values for the parameters
	gp3.setMu (2.5);
	gp3.setMaximumNearestNeighbors (100);
	gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
	gp3.setMinimumAngle(M_PI/18); // 10 degrees
	gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
	gp3.setNormalConsistency(false);


	// Get result
	gp3.setInputCloud (cloud_with_normals);
	gp3.setSearchMethod (tree2);
	gp3.reconstruct (triangles);

	// Additional vertex information
	std::vector<int> parts = gp3.getPartIDs();
	std::vector<int> states = gp3.getPointStates();	

	pcl::PolygonMesh::Ptr mesh(&triangles);
	pcl::PointCloud<pcl::PointXYZ>::Ptr triangle_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2(mesh->cloud, *triangle_cloud);	
/*	for(int i = 0; i < 2; i++){
		std::cout<<triangles.polygons[i]<<std::endl;
	}
*/	std::cout<<"first vertice "<<triangles.polygons[0].vertices[0] << std::endl; 	
	
	//std::cout<<"Prolly not gonna work "<<triangle_cloud->points[triangles.polygons[0].vertices[0]] << std::endl; 	


	//pcl::fromPCLPointCloud2(triangles.cloud, triangle_cloud); 
	std::cout << "size of points " << triangle_cloud->points.size() << std::endl ;
	
	std::cout<<triangle_cloud->points[0]<<std::endl;
	for(unsigned i = 0; i < triangle_cloud->points.size(); i++){
		std::cout << triangles.polgyons[i].getVector3fMap() <<"test"<< std::endl;
	} 	
	//std::cout<<"surface: "<<calculateAreaPolygon(triangles, triangle_cloud)<<std::endl;

	/******************************************************************************************************************************************/	

	/* CALCULATING CURVATURE AND NORMALS ********************************************************************************************************************/

	// Create the normal estimation class, and pass the input dataset to it
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;	

	ne.setInputCloud (cloud);

	// Create an empty kdtree representation, and pass it to the normal estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree3 (new pcl::search::KdTree<pcl::PointXYZ> ());

	ne.setSearchMethod (tree3);

	// Output datasets
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

	// Use all neighbors in a sphere of radius 3cm
	ne.setRadiusSearch (0.03);

	// Compute the features
	ne.compute (*cloud_normals);

	output_file << "size of points " << cloud->points.size() << std::endl ;

	output_file << "size of the normals " << cloud_normals->points.size() << std::endl ; 	

	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);

	/************************************************************************************************************************************/

	/* PARSING DATA ********************************************************************************************************************/

	int k=0 ;
	float dist ;
	float square_of_dist ;
	float x1,y1,z1,x2,y2,z2 ;
	float nu[3], nv[3], pv_pu[3], pu_pv[3] ;
	float highest = triangle_cloud->points[0].z;
	float lowest = triangle_cloud->points[0].z;


	for ( int i = 0; i < cloud_normals->points.size() ; i++)
	{
		output_file<<i<<": triangulated "<<triangle_cloud->points[i].x<<", "<<triangle_cloud->points[i].y<<", "<<triangle_cloud->points[i].z<<std::endl;
		output_file<<i<<": normal"<<cloud1->points[i].x<<", "<<cloud1->points[i].y<<", "<<cloud1->points[i].z<<std::endl;
		if(triangle_cloud->points[i].z > highest)
		{
			highest = triangle_cloud->points[i].z;
		}
		if(triangle_cloud->points[i].z < lowest)
		{
			lowest = triangle_cloud->points[i].z;
		}
		normals_text <<i+1 <<": "<<" x-normal-> "<<cloud_normals->points[i].normal_x<<" y-normal-> "<<cloud_normals->points[i].normal_y<<" z-normal-> "<<cloud_normals->points[i].normal_z<<std::endl;
		curvature <<i+1 <<": curvature: "<<cloud_normals->points[i].curvature<<std::endl;
		
		float x, y, z, dist, nx, ny, nz, ndist;
		
		/*

		if(i != cloud_normals->points.size()-1){
			x = cloud->points[i+1].x - cloud->points[i].x;
			y = cloud->points[i+1].y - cloud->points[i].y;
			z = cloud->points[i+1].z - cloud->points[i].z;
			dist = sqrt(pow(x, 2)+pow(y, 2) + pow(z, 2));
			output_file << i+1 <<" -> "<< i+2 << " distance normal: " << dist <<std::endl;
			
			nx = triangle_cloud[i+1].indices[0] - triangle_cloud.points[i].x;
			ny = triangle_cloud.points[i+1].y - triangle_cloud.points[i].y;
			nz = triangle_cloud.points[i+1].z - triangle_cloud.points[i].z;
			ndist = sqrt(pow(nx, 2)+pow(ny, 2) + pow(nz, 2));
			output_file << i+1 <<" -> "<< i+2 << " distance triangulated: " << ndist <<std::endl;
		}

		*/	
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

	/***********************************************************************************************************************************/


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
