#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

pcl::IndicesPtr pass_filter(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,float distance=-1.0, float height=-1.0){
    pcl::IndicesPtr eligible_indices (new std::vector <int>);
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (cloud);
    pass.filter (*eligible_indices);
    if(distance > 0.0){
        /************** Distance filter of cloud ***************/
        // pass.setInputCloud (cloud);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (0.0, distance);
        pass.filter (*eligible_indices);
    }
    if(height > 0.0){
        /************** Height filter of cloud ***************/
        // pass.setInputCloud (cloud);
        if(eligible_indices->size() > 0){
            pass.setIndices(eligible_indices);
        }
        pass.setFilterFieldName ("y");
        pass.setFilterLimits (-height, 10);
        pass.filter (*eligible_indices);
    }

    return eligible_indices;
}

// pcl::PCLPointCloud2::Ptr 
void voxel_filter(
    pcl::PCLPointCloud2::ConstPtr cloud,  pcl::PCLPointCloud2::Ptr output_cloud, float x_dim=0.05, float y_dim=0.05, float z_dim=0.05)
{
    // pcl::PCLPointCloud2::Ptr output_cloud (new pcl::PCLPointCloud2 ());
    // Create the filtering object
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (x_dim, y_dim, z_dim);
    sor.filter (*output_cloud);

    // return (output_cloud);
}

// pcl::PointCloud<pcl::PointXYZ>::Ptr  
void voxel_filter(
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud, float x_dim=0.05, float y_dim=0.05, float z_dim=0.05, float distance=-1.0, float height=-1.0)
{
    pcl::IndicesPtr eligible_indices (new std::vector <int>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    if(distance > 0.0){
        /************** Distance filter of cloud ***************/
        pass.setInputCloud (cloud);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (0.0, distance);
        pass.filter (*eligible_indices);
    }
    if(height > 0.0){
        /************** Height filter of cloud ***************/
        pass.setInputCloud (cloud);
        if(eligible_indices->size() > 0){
            pass.setIndices(eligible_indices);
        }
        pass.setFilterFieldName ("y");
        pass.setFilterLimits (-height, 10);
        pass.filter (*eligible_indices);
    }
    
    // pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    // Create the filtering object
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud);

    if(eligible_indices->size()){
        sor.setIndices(eligible_indices);
    }

    sor.setLeafSize (x_dim, y_dim, z_dim);
    sor.filter (*output_cloud);

    // return (output_cloud);
}


// pcl::PointCloud<pcl::PointXYZRGB>::Ptr  
void voxel_filter(
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud, float x_dim=0.05, float y_dim=0.05, float z_dim=0.05, float distance=-1.0, float height=-1.0)
{   
    // pcl::IndicesPtr eligible_indices (new std::vector <int>);
    // pcl::PassThrough<pcl::PointXYZRGB> pass;
    // if(distance > 0.0){
    //     /************** Distance filter of cloud ***************/
    //     pass.setInputCloud (cloud);
    //     pass.setFilterFieldName ("z");
    //     pass.setFilterLimits (0.0, distance);
    //     pass.filter (*eligible_indices);
    // }
    // if(height > 0.0){
    //     /************** Height filter of cloud ***************/
    //     pass.setInputCloud (cloud);
    //     if(eligible_indices->size() > 0){
    //         pass.setIndices(eligible_indices);
    //     }
    //     pass.setFilterFieldName ("y");
    //     pass.setFilterLimits (-height, 10);
    //     pass.filter (*eligible_indices);
    // }
    auto eligible_indices = pass_filter(cloud, distance, height);
    
    // Create the filtering object
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud (cloud);

    if(eligible_indices->size()){
        sor.setIndices(eligible_indices);
    }

    sor.setLeafSize (x_dim, y_dim, z_dim);
    sor.filter (*output_cloud);
}