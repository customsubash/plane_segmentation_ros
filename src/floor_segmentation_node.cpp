#include <ros/ros.h>
// #include <pcl_ros/point_cloud.h>
// #include <pcl/point_types.h>
// #include <boost/foreach.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/io/pcd_io.h>
#include <mina-pcl/filters.h>
#include <mina-pcl/segmentation.h>
#include <mina-pcl/pointcloud.h>
#include <mina-pcl/timestamp.h>

#include <sensor_msgs/PointCloud2.h>
#include <thread>


PointCloudXYZRGB::Ptr cloud_visualization(new PointCloudXYZRGB);
PointCloudXYZRGB::Ptr cloud_filtered(new PointCloudXYZRGB);

ros::Publisher pub;
tf::StampedTransform transform_camera_frame;

double height_ = -1.0;
double distance_ = -1.0;
double grid_size_ = -1.0;

bool grid_uniform_ = true;
double grid_height_ = 0.01;
double grid_width_ = 0.01;
double grid_depth_ = 0.01;

void cloud_cb(const boost::shared_ptr<const sensor_msgs::PointCloud2>& msg){
    CheckPoint checkpoint;
    printf("Start of callback\n");
    printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud_rgb);
    pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud);

    /************************* pcl algorithms implementations *********************/
    checkpoint.begin();
    if (grid_size_ >= 0.01){
      if (grid_uniform_){
        voxel_filter(temp_cloud_rgb, cloud_filtered, grid_size_, grid_size_, grid_size_, distance_, height_, transform_camera_frame.getRotation());
        // ROS_WARN("uniform");
        // ROS_WARN("%f", grid_size_);
      }
      else{
        voxel_filter(temp_cloud_rgb, cloud_filtered, grid_width_, grid_height_, grid_depth_, distance_, height_, transform_camera_frame.getRotation());
        // ROS_WARN("Non-uniform");
        // ROS_WARN("%f", grid_width_);
        // ROS_WARN("%f", grid_height_);
        // ROS_WARN("%f", grid_depth_);
      }
      
      std::cout << "PointCloud after voxel filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;
    }
    else{
      /************ filter incides ************/
      auto indices = pass_filter(temp_cloud_rgb, distance_, height_);
      pcl::ExtractIndices<pcl::PointXYZRGB> extract;
      extract.setInputCloud(temp_cloud_rgb);
      extract.setIndices (indices);
      extract.setNegative (false);
      extract.filter (*cloud_filtered);
    }
    // std::cerr << grid_size_ << " " << distance_ << " " << height_ << std::endl; 
    checkpoint.create("Voxel filter timespan: ");

    /************ Subtract the indices that are normal ************/
    cluster_plane_with_normals(cloud_filtered, cloud_visualization);
    checkpoint.create("Normal clustering timespan: ");

    /************ Publish the PointCloud2 message to ROS ************/
    sensor_msgs::PointCloud2::Ptr tempROSMsg(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*cloud_visualization, *tempROSMsg);
    pub.publish(tempROSMsg);
    checkpoint.create("Update cloud: ");
  }


int main(int argc, char** argv)
{
  ros::init(argc, argv, "floor_segmentation");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/color/points", 1, cloud_cb);
  pub = nh.advertise<sensor_msgs::PointCloud2>("/segmented_cloud", 1);
  
  nh_private.param("height_above_max", height_, 0.5);
  nh_private.param("distance_front_max", distance_, 5.0);

  nh_private.param("grid_size", grid_size_, 0.05);
  nh_private.param("grid_uniform", grid_uniform_, true);

  nh_private.param("grid_height", grid_height_, 0.01);
  nh_private.param("grid_width", grid_width_, 0.01);
  nh_private.param("grid_depth", grid_depth_, 0.01);
  
  tf::TransformListener listener;
  while(ros::ok()){
    try{
      // listener.lookupTransform("base_link", "camera_depth_optical_frame", ros::Time(0), transform_depth_frame);
      listener.lookupTransform("base_link", "camera_link", ros::Time(0), transform_camera_frame);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      // ros::Duration(1.0).sleep();
    }
    ros::spinOnce();
  }
}