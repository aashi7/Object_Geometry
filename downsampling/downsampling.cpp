#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <stdlib.h> 


int main (int argc, char** argv)
{

	pcl::PointCloud<pcl::PointXYZ> cloud;

	// Fill in the cloud data
	cloud.width    = 8;
	cloud.height   = 1;
	cloud.is_dense = false;
	cloud.points.resize (cloud.width * cloud.height);
	
	for(int i = 0; i < 8; i++){
		cloud.points[i].x = rand() %10;	
		cloud.points[i].y = rand() %10;
		cloud.points[i].z = rand() %10;
		std::cout<<cloud.points[i]<<std::endl;
	}

	/*
	for (size_t i = 0; i < 2; i++){
		for(size_t k = 0; k < 2; k++){
			for(size_t p = 0;p < 2; p++){	
				cloud.points[i+2*k+4*p].x = i;
				cloud.points[i+2*k+4*p].y = k;
				cloud.points[i+2*k+4*p].z = p;
				std::cout<<cloud.points[i+2*k+4*p]<<std::endl;
			}
		}
	}
	*/
	pcl::io::savePCDFileASCII ("cube.pcd", cloud);
	std::cerr << "Saved " << cloud.points.size () << " data points to cube.pcd." << std::endl;	
	
	return (0);
}
