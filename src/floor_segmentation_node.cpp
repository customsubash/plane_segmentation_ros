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

/* Already import with filters */
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <eigen3/Eigen/Core>

/*** Custom Headers ***/
#include <mina-pcl/config.h>

PointCloudXYZRGB::Ptr cloud_visualization(new PointCloudXYZRGB);
PointCloudXYZRGB::Ptr cloud_filtered(new PointCloudXYZRGB);

PointCloudXYZRGB::Ptr temp_cloud_rgb(new PointCloudXYZRGB);
PointCloudXYZ::Ptr temp_cloud(new PointCloudXYZ);

ros::Publisher pub;
tf::Quaternion orientation;

double height_ = -1.0;
double distance_ = -1.0;
double grid_size_ = -1.0;

bool grid_uniform_ = true;
double grid_height_ = 0.01;
double grid_width_ = 0.01;
double grid_depth_ = 0.01;

int rate_ = 3;

std::string parent_frame_id_ = "base_link";
std::string frame_id_ = "camera_link";

double is_recieved = false;
Eigen::Vector3f V;

#ifdef DEBUG
#include <pcl/visualization/cloud_viewer.h>
pcl::visualization::PCLVisualizer::Ptr rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

auto viewer = rgbVis(cloud_visualization);
#endif

// void cloud_cb(const boost::shared_ptr<const sensor_msgs::PointCloud2>& msg){
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& msg){
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud_rgb);
    pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud);
    is_recieved = true;
}

void process_publish(){
    /************************* pcl algorithms implementations *********************/
    CheckPoint checkpoint;
    #ifdef DEBUG
    checkpoint.enable();
    checkpoint.begin();
    printf("Start of callback\n");
    // printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
    #endif
    if (grid_size_ >= 0.01){
      if (grid_uniform_){
        voxel_filter(temp_cloud_rgb, cloud_filtered, grid_size_, grid_size_, grid_size_, distance_, height_, orientation);
        // ROS_WARN("uniform");
        // ROS_WARN("%f", grid_size_);
      }
      else{
        voxel_filter(temp_cloud_rgb, cloud_filtered, grid_width_, grid_height_, grid_depth_, distance_, height_, orientation);
        // ROS_WARN("Non-uniform");
        // ROS_WARN("%f", grid_width_);
        // ROS_WARN("%f", grid_height_);
        // ROS_WARN("%f", grid_depth_);
      }
      #ifdef DEBUG
      std::cout << "PointCloud after voxel filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;
      #endif
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
    cluster_plane_with_normals(cloud_filtered, cloud_visualization, V);
    // cluster_plane_with_normals(cloud_filtered, cloud_visualization);
    checkpoint.create("Normal clustering timespan: ");

    /******* visualization ******/
    #ifdef DEBUG
    viewer->updatePointCloud(cloud_filtered, "sample cloud");
    viewer->spinOnce(50);
    #endif

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
  
  nh_private.param("height_above_max", height_, 2.0);
  nh_private.param("distance_front_max", distance_, 8.0);

  nh_private.param("grid_size", grid_size_, 0.05);
  nh_private.param("grid_uniform", grid_uniform_, true);

  nh_private.param("grid_height", grid_height_, 0.01);
  nh_private.param("grid_width", grid_width_, 0.01);
  nh_private.param("grid_depth", grid_depth_, 0.01);
  nh_private.param("rate", rate_, 3);

  nh_private.param("parent_frame_id", parent_frame_id_, std::string("base_link"));
  nh_private.param("frame_id", frame_id_, std::string("camera_link"));

  tf::TransformListener listener;
  tf::StampedTransform transform;

  ros::Rate rate(rate_);
  
  while(ros::ok()){
    if (is_recieved){
      process_publish();
      is_recieved = false;
    }

    try{
      listener.lookupTransform(parent_frame_id_, frame_id_,  
                               ros::Time(0), transform);

      float x = transform.getRotation().x();
      float y = transform.getRotation().y();
      float z = transform.getRotation().z();
      float w = transform.getRotation().w();
      orientation = transform.getRotation();

      V[0] = 2 * (x * z - w * y);
      V[1] = 2 * (y * z + w * x);
      V[2] = 1 - 2 * (x * x + y * y);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }
    rate.sleep();
    ros::spinOnce();
  }
}