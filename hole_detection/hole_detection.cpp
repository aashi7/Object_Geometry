#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/console/parse.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>

#include <iostream>
#include <vector>
#include <ctime>

void showHelp()
{
	std::cout << std::endl;
	std::cout << "Commands:" << std::endl;
	std::cout << "\n_help:  Show this help." << std::endl;
	std::cout << "_tree:  Runs kd tree algorithm with specified K number of closest points and/or with a specified radius." << std::endl;
	std::cout << "_quit:  Exits program.\n" << std::endl;	
	std::cout << "_points:  Prints points contained in point cloud.\n" << std::endl;
	//std::cout << "_quit:  Exits program.\n" << std::endl;
	//std::cout << "_quit:  Exits program.\n" << std::endl;


}

void points(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud){
	
	std::cout << std::endl;
	std::cout << "Commands:" << std::endl;
	std::cout << "\n_help:  Show this help." << std::endl;
	std::cout << "_tree:  Runs kd tree algorithm with specified K number of closest points and/or with a specified radius." << std::endl;
	std::cout << "_quit:  Exits program.\n" << std::endl;	
	std::cout << "_points:  Prints points contained in point cloud.\n" << std::endl;
	//std::cout << "_quit:  Exits program.\n" << std::endl;
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
			<< " " << searchPoint.y 
			<< " " << searchPoint.z
			<< ") with K=" << K << std::endl;

		if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
		{
			for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
				std::cout << "    "  <<   cloud->points[ pointIdxNKNSearch[i] ].x 
					<< " " << cloud->points[ pointIdxNKNSearch[i] ].y 
					<< " " << cloud->points[ pointIdxNKNSearch[i] ].z 
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
			<< " " << searchPoint.y 
			<< " " << searchPoint.z
			<< ") with radius=" << radius << std::endl;


		if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
		{
			for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
				std::cout << "    "  <<   cloud->points[ pointIdxRadiusSearch[i] ].x 
					<< " " << cloud->points[ pointIdxRadiusSearch[i] ].y 
					<< " " << cloud->points[ pointIdxRadiusSearch[i] ].z 
					<< " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
		}
	}
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
			continue;
		}
		else if (line == "_tree") {
			kd_tree (cloud);
		}
		else if (line == "_points") {
			points (cloud);
		}
		/*else if (pcl::console::find_switch (argc, argv, "-h") || pcl::console::find_switch (argc, argv, "--help")) {
			showHelp (argv[0]);
		}
		else if (pcl::console::find_switch (argc, argv, "-h") || pcl::console::find_switch (argc, argv, "--help")) {
			showHelp (argv[0]);
		}*/
		else{
			std::cout <<"problem -> "<<line<< "*Error: Command not recognized enter '_help' to view help menu" << std::endl;
			continue;
		}
		std::cin.ignore(INT_MAX, '\n');
	}
	return 0;
}
