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


PointCloudXYZRGB::Ptr cloud_visualization(new PointCloudXYZRGB);
PointCloudXYZRGB::Ptr cloud_filtered(new PointCloudXYZRGB);

ros::Publisher pub;

double height_ = -1.0;
double distance_ = -1.0;
double grid_size_ = -1.0;

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
    if (grid_size_ >= 0.1){
      // cloud_filtered = voxel_filter(temp_cloud_rgb, cloud_filtered, grid_size_, grid_size_, grid_size_, distance_, height_);
      voxel_filter(temp_cloud_rgb, cloud_filtered, grid_size_, grid_size_, grid_size_, distance_, height_);
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

    cluster_plane_with_normals(cloud_filtered, cloud_visualization);
    checkpoint.create("Normal clustering timespan: ");

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
  
  nh_private.param("height_above_max", height_, -1.0);
  nh_private.param("distance_front_max", distance_, -1.0);
  nh_private.param("grid_size", grid_size_, -1.0);

  // if(grid_size_ < 0.1){
  //     grid_size_ = 0.1;
  // }
  
  while(ros::ok()){
    ros::spinOnce();
  }
}