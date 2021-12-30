#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <mina-pcl/pointcloud.h>
#include <eigen3/Eigen/Core>
#include <tf/tf.h>

pcl::IndicesPtr pass_filter(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,float distance=-1.0, float height=-1.0, tf::Quaternion orientation={0,0,0,1}){
    pcl::IndicesPtr eligible_indices (new std::vector <int>);
    /** Subash Ghimire, Need to work here to implement dynamics filtereing when sensor orientation is different
    currently it works when tilted in only one direction**/
    /******* transformation of point cloud to align pointcloud to true horizontal *******/
    // tf::Quaternion rotation_depth_frame = transform_depth_frame.getRotation();
    // tf::Quaternion rotation_camera_frame = transform_camera_frame.getRotation();

    // tf::Quaternion final = rotation_depth_frame * rotation_camera_frame;
    ///*
    tf::Matrix3x3 m(orientation);
    double roll_, pitch_, yaw_;
    m.getRPY(roll_, pitch_, yaw_);
    // std::cout << roll_ << " " << pitch_ << " " << yaw_ << std::endl;
    double roll, pitch, yaw;

    /* Changing the axis of rotations, since sensor has different rotation scheme for depth */
    roll = -pitch_;
    pitch = -yaw_;
    yaw = roll_;

    /* Apply transform for different correction of effects of sensor orientation */
    PointCloudXYZRGB::Ptr cloud_ (new PointCloudXYZRGB);
    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
    transform_2.rotate (Eigen::AngleAxisf (roll, Eigen::Vector3f::UnitX()));
    transform_2.rotate (Eigen::AngleAxisf (pitch, Eigen::Vector3f::UnitY()));
    transform_2.rotate (Eigen::AngleAxisf (yaw, Eigen::Vector3f::UnitZ()));
    // std::cout << transform_2.matrix() << std::endl;
    pcl::transformPointCloud (*cloud, *cloud_, transform_2);

    pcl::PassThrough<pcl::PointXYZRGB> pass;
    // pass.setInputCloud (cloud_);
    pass.setInputCloud (cloud_);
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
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud, float x_dim=0.05, float y_dim=0.05, float z_dim=0.05, float distance=-1.0, float height=-1.0, tf::Quaternion orientation={0,0,0,1})
{   
    auto eligible_indices = pass_filter(cloud, distance, height, orientation);
    
    // Create the filtering object
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud (cloud);

    if(eligible_indices->size()){
        sor.setIndices(eligible_indices);
    }

    sor.setLeafSize (x_dim, y_dim, z_dim);
    sor.filter (*output_cloud);
}