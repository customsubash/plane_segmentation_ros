#include <math.h>
#include <chrono>
#include <eigen3/Eigen/Core>

#include <pcl/filters/filter_indices.h> // for pcl::removeNaNFromPointCloud
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/segmentation/sac_segmentation.h>

#include "pointcloud.h"
#include "timestamp.h"
#include <mina-pcl/config.h>


pcl::IndicesPtr get_normals_index(pcl::PointCloud <pcl::Normal>::ConstPtr normals, const Eigen::Vector3f orientation_vector, double tolerance){
  // pcl::PointIndices::Ptr eligible_indices (new pcl::PointIndices);
  pcl::IndicesPtr eligible_indices (new std::vector <int>);
  Eigen::Vector3f buffer(0.0, 0.0, 0.0);
  Eigen::Vector3f diff(0.0, 0.0, 0.0);
  for( int index = 0; index < normals->points.size(); index++){
    buffer[0] = normals->points[index].normal_x;
    buffer[1] = normals->points[index].normal_y;
    buffer[2] = normals->points[index].normal_z;
    diff = orientation_vector - buffer;
    // double mag = diff[0]*diff[0] + diff[1] * diff[1] + diff[2]* diff[2];
    // if (mag < tolerance*tolerance){
    if((abs(diff[0]) < tolerance) & (abs(diff[1]) < tolerance) & (abs(diff[2]) < tolerance)){ // improves performance by 10% than magnitude
      eligible_indices->push_back(index);
    }
  }
  return eligible_indices;
}


void cluster_plane_with_normals(PointCloudXYZRGB::ConstPtr input_cloud, PointCloudXYZRGB::Ptr output_cloud, Eigen::Vector3f orientation = {0.0, -1.0, 0.0}){
  CheckPoint cp; // DIsabled at default
  #ifdef DEBUG
  cp.enable();
  cp.begin();
  #endif

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;

  PointCloudXYZ::Ptr cloud (new PointCloudXYZ);
  pcl::copyPointCloud(*input_cloud, *cloud);

  pcl::search::Search<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normal_estimator;
  // pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod (tree);
  normal_estimator.setInputCloud (cloud);
  normal_estimator.setNumberOfThreads(0);
  normal_estimator.setKSearch (16);

  // normal_estimator.setRadiusSearch(0.4);
  cp.create("Surface Normal preparation: ");
  normal_estimator.compute (*normals);
  cp.create("Only Normal's Calculation timespan: ");

  /************** find points normal aligning to vector **************/
  pcl::IndicesPtr aligned_indices (new std::vector <int>);
  // Eigen::Vector3f orientation = {0.0, -1.0, 0.0};
  cp.create("Identification of normal timespan: ");

  /************* Rearrange the axis to match correctly **********/
  // x ->  y
  // y -> -z
  // z ->  x
  // One can realize these transformation using debug mode from visualization windows
  Eigen::Vector3f V;
  V[0] = orientation[1];
  V[1] = -orientation[2];
  V[2] = orientation[0];

  // std::cout<< "After: " <<"{"<<V<<"}" << std::endl;

  aligned_indices = get_normals_index(normals, V, 0.1);
  cp.create("Finding aligned indices timespan: ");
  
  /************ remove misaligned indices ************/
  extract.setInputCloud(input_cloud);
  extract.setIndices (aligned_indices);
  extract.setNegative (true);
  extract.filter (*output_cloud);
}