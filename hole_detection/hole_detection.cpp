#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/console/parse.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>

#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <ctime>
#include <math.h>

void showHelp()
{
	std::cout << std::endl;
	std::cout << "Commands:" << std::endl;
	std::cout << "\n_help:  Show this help." << std::endl;
	std::cout << "_tree:  Runs kd tree algorithm with specified K number of closest points and/or with a specified radius." << std::endl;
	std::cout << "_quit:  Exits program." << std::endl;	
	std::cout << "_points:  Prints points contained in point cloud." << std::endl;
	std::cout << "_normals:  Prints normals of points in cloud." << std::endl;
	std::cout << "_holes:  Calculates hole (returns set of points on boundary and visualizes image with boundary points red).\n" << std::endl;


}

void points(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud){
	for(unsigned i = 0; i < cloud->points.size(); i++){
		std::cout<<cloud->points[i]<<std::endl;
	}
}

void normals(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud){
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
	ne.setRadiusSearch (5);

	// Compute the features
	ne.compute (*cloud_normals);

	std::cout << "size of the normals " << cloud_normals->points.size() << std::endl ; 		
	for(unsigned i = 0; i < cloud_normals->points.size(); i++){
		std::cout<< cloud_normals->points[i] <<std::endl;
	}
}

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

void kd_tree(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud){
	bool r = false;
	bool k = false;
	std::cout<<"Would you like a radius(r) search, a K(k) search or both(b): "<<std::endl;
	char input;
	std::cin >> input;	
	while(true){
		if(input == 'k'){
			k = true;
			break;
		}
		else if(input == 'r'){
			r = true;
			break;
		}
		else if(input == 'b'){
			k = true;
			r = true;
			break;
		}	
		else{
			std::cout<<"option '"<<input<<"' doesn't exist"<<std::endl;
		}
	}

	srand (time (NULL));

	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

	kdtree.setInputCloud (cloud);

	pcl::PointXYZ searchPoint;

	searchPoint.x = 1;
	searchPoint.y = 1;
	searchPoint.z = 1;

	if(k){
		std::cout<< "Enter a K value: " <<std::endl;
		int K = 0;
		std::cin >> K;

		// K nearest neighbor search
		std::vector<int> pointIdxNKNSearch(K);
		std::vector<float> pointNKNSquaredDistance(K);

		std::cout << "K nearest neighbor search at (" << searchPoint.x 
			<< ", " << searchPoint.y 
			<< ", " << searchPoint.z
			<< ") with K=" << K << std::endl;

		if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
		{
			for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
				std::cout << "    "  <<   cloud->points[ pointIdxNKNSearch[i] ].x 
					<< ", " << cloud->points[ pointIdxNKNSearch[i] ].y 
					<< ", " << cloud->points[ pointIdxNKNSearch[i] ].z 
					<< " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
		}
	}

	if(r){
		std::cout<< "Enter a radius: " <<std::endl;
		float radius = 0;
		std::cin >> radius;

		// Neighbors within radius search
		std::vector<int> pointIdxRadiusSearch;
		std::vector<float> pointRadiusSquaredDistance;

		std::cout << "Neighbors within radius search at (" << searchPoint.x 
			<< ", " << searchPoint.y 
			<< ", " << searchPoint.z
			<< ") with radius=" << radius << std::endl;


		if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
		{
			for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
				std::cout << "    "  <<   cloud->points[ pointIdxRadiusSearch[i] ].x 
					<< ", " << cloud->points[ pointIdxRadiusSearch[i] ].y 
					<< ", " << cloud->points[ pointIdxRadiusSearch[i] ].z 
					<< " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
		}
	}
}

void calculate_hole(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud){
	
/****************K POINTS & RADIUS POINTS*****************************/
	srand (time (NULL));

	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

	kdtree.setInputCloud (cloud);

	pcl::PointXYZ searchPoint;

	std::string coor[3] = {"","",""};	

	std::cout<< "Enter a search point xyz value separated by spaces: " <<std::endl;
	std::string line;
	std::getline(std::cin, line);
	std::stringstream stream(line);
	for(int i  = 0; i < 3; i++) {
		stream >> coor[i];
		if(!stream)
			break;
		std::cout << "Found integer: " << coor[i] << "\n";
	}
	std::cout<< coor[0] <<" "<< coor[1] <<" "<< coor[2] <<std::endl;

	std::cout<< "Enter a K value: " <<std::endl;
	int K = 0;
	std::cin >> K;

	// K nearest neighbor search
	std::vector<int> pointIdxNKNSearch(K);
	std::vector<float> pointNKNSquaredDistance(K);
	kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);

	std::cout<< "Enter a radius: " <<std::endl;
	int radius = 0;
	std::cin >> radius;

	// Neighbors within radius search
	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;
	kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);	
/**********************************************************************/

/************** NORMALS ***********************************************/

float p1[3] = {1,1,1}; //said point 1
float p2[3] = {2,2,2}; //said point 2 

//angle_between_vector(p1,p2);

float dist = sqrt(pow(p2[0]-p1[0],2.0) + pow(p2[1]-p1[1],2.0) + pow(p2[2]-p1[2],2.0))
/**********************************************************************/

}
// This is the main function
int main (int argc, char** argv)
{
	// Fetch point cloud filename in arguments | Works with PCD and PLY files
	std::vector<int> filenames;
	bool file_is_pcd = false;

	filenames = pcl::console::parse_file_extension_argument (argc, argv, ".ply");

	if (filenames.size () != 1)  {
		filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");

		if (filenames.size () != 1) {
			std::cout << std::endl;
			std::cout << "Usage: " << argv[0] << "cloud_filename.[pcd][ply]" << std::endl;
			return -1;
		} else {
			file_is_pcd = true;
		}
	}

	// Load file | Works with PCD and PLY files
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());

	if (file_is_pcd) {
		if (pcl::io::loadPCDFile (argv[filenames[0]], *cloud) < 0)  {
			std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl << std::endl;
			std::cout << std::endl;
			std::cout << "Usage: " << argv[0] << "cloud_filename.[pcd][ply]" << std::endl;
			return -1;
		}
	} else {
		if (pcl::io::loadPLYFile (argv[filenames[0]], *cloud) < 0)  {
			std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl << std::endl;
			std::cout << std::endl;
			std::cout << "Usage: " << argv[0] << "cloud_filename.[pcd][ply]" << std::endl;	
			return -1;
		}
	}
	showHelp();
	while(true){
		std::cout<<">> ";
		std::string line;
		getline(std::cin, line);
		// Show help
		if (line == "_quit"){
			break;
		}
		else if (line == "_help") {
			showHelp ();
		}
		else if (line == "_tree") {
			kd_tree (cloud);
			std::cin.ignore(INT_MAX, '\n');
		}
		else if (line == "_points") {
			points (cloud);
		}
		else if (line == "_normals") {
			normals (cloud);
		}
		else if (line == "_holes") {
			calculate_hole (cloud);
			std::cin.ignore(INT_MAX, '\n');
		}
		else{
			std::cout <<"problem -> "<<line<< " *Error: Command not recognized enter '_help' to view help menu" << std::endl;
		}
	}
	return 0;
}
