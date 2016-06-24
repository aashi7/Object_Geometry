#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <math.h> 
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/filters/voxel_grid.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/extract_clusters.h>


int
 main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);


  pcl::PCDReader reader;
  reader.read ("01_downsampled.pcd", *cloud); 
  

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);

  seg.setInputCloud (cloud);
  seg.segment (*inliers, *coefficients);

  if (inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    return (-1);
  }

  std::cerr << "Model coefficients: " << coefficients->values[0] << " "        // a in normal to the plane (a,b,c)
                                      << coefficients->values[1] << " "        // b in normal to the plane (a,b,c)
                                      << coefficients->values[2] << " "        // c in normal to the plane (a,b,c)
                                      << coefficients->values[3] << std::endl;

  std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud (cloud);
  extract.setIndices (inliers);
  extract.setNegative(false);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ground (new pcl::PointCloud<pcl::PointXYZ>);

  extract.filter (*cloud_ground);//get the ground plane

  std::cerr << "Ground cloud after filtering: " << std::endl;
  std::cerr << *cloud_ground << std::endl;                    // point cloud of the ground plane

  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ> ("ground_new.pcd", *cloud_ground, false);

  float tx, ty, tz ;
  float p,q,r,d,c,s ;

  p = coefficients->values[0] ;
  q = coefficients->values[1] ;
  r = coefficients->values[2] ;
    d = coefficients->values[3] ;
  float kx,ky,kz ;

  kx = -r ;
  ky = 0 ;
  kz = p ;

  Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity() ;

 // tx = -coefficients->values[3]*coefficients->values[0] ;
 // ty = -coefficients->values[3]*coefficients->values[1] ;
 // tz = -coefficients->values[3]*coefficients->values[2] ;

 float theta ;
 theta = acos(q/ (p*p + q*q + r*r)); 
 c = cos(theta);
 s = sin(theta);

  transform_1 (0,0) = kx*kx*(1-c) + c ;
  transform_1 (0,1) = kx*ky*(1-c) - kz*s ;
  transform_1 (0,2) = kx*kz*(1-c) + ky*s ;
  transform_1 (1,0) = ky*kx*(1-c) + kz*s ;
  transform_1 (1,1) = ky*ky*(1-c) + c ;
  transform_1 (1,2) = ky*kz*(1-c) - kx*s ;
  transform_1 (2,0) = kz*kx*(1-c) - ky*s ; 
  transform_1 (2,1) = kz*ky*(1-c) + kx*s ;
  transform_1 (2,2) = kz*kz*(1-c) + c ;

  //    (row, column)
  transform_1 (0,3) = 0.0 ;
  transform_1 (1,3) = 0.0 ;
  transform_1 (2,3) = 0.0 ;

  // Print the transformation
  printf ("Method #1: using a Matrix4f\n");
  std::cout << transform_1 << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile ("01_downsampled.pcd", *source_cloud) ;
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ>);
   pcl::transformPointCloud (*source_cloud, *transformed_cloud, transform_1);

    pcl::ModelCoefficients::Ptr coefficients_t (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_t (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg_t;
  // Optional
  seg_t.setOptimizeCoefficients (true);
  // Mandatory
  seg_t.setModelType (pcl::SACMODEL_PLANE);
  seg_t.setMethodType (pcl::SAC_RANSAC);
  seg_t.setDistanceThreshold (0.01);

  seg_t.setInputCloud (transformed_cloud);   
  seg_t.segment (*inliers_t, *coefficients_t);

  if (inliers_t->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    return (-1);
  }

  std::cerr << "Model coefficients: " << coefficients_t->values[0] << " "        // a in normal to the plane (a,b,c)
                                      << coefficients_t->values[1] << " "        // b in normal to the plane (a,b,c)
                                      << coefficients_t->values[2] << " "        // c in normal to the plane (a,b,c)
                                      << coefficients_t->values[3] << std::endl;


 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
 

  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZ> seg_s;
  pcl::PointIndices::Ptr inliers_s (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients_s (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());

  seg_s.setOptimizeCoefficients (true);
  seg_s.setModelType (pcl::SACMODEL_PLANE);
  seg_s.setMethodType (pcl::SAC_RANSAC);
  seg_s.setMaxIterations (100);
  seg_s.setDistanceThreshold (0.02);

  int i=0, nr_points = (int) transformed_cloud->points.size ();
  while (transformed_cloud->points.size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg_s.setInputCloud (transformed_cloud);
    seg_s.segment (*inliers_s, *coefficients_s);
    if (inliers_s->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (transformed_cloud);
    extract.setIndices (inliers_s);
    extract.setNegative (false);

    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);
    std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *transformed_cloud = *cloud_f;
  }

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (transformed_cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (transformed_cloud);
  ec.extract (cluster_indices);

  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->points.push_back (transformed_cloud->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    std::stringstream ss;
    ss << "cloud_cluster_" << j << ".pcd";
    writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
    j++;
  }



  return (0);
}
