#include "ros/ros.h"
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <map>
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <Eigen/Dense>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/PointIndices.h>

#include <ndt_omp.h>
#include <read_transformations.h>

#include <stdio.h>
#include <math.h> 

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <map>
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <Eigen/Dense>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/PointIndices.h>
#include <boost/thread/thread.hpp>
#include <time.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_estimation_normal_shooting.h>
#include <pcl/registration/correspondence_estimation_backprojection.h>
#include <pcl/registration/correspondence_rejection.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/correspondence_rejection_median_distance.h>
#include <pcl/features/normal_3d.h>

#include <ndt_omp.h>
#include <read_transformations.h>

#define PI 3.14159265


using namespace std;
using namespace pcl;

int counter = 0;
ros::Publisher pose;
ros::Publisher obstacle;
ros::Publisher aligned_point_cloud;
pcl::PointXYZ position;
pcl::PointXYZ euler_angles;
pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr combined_cloud(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr temporary_cloud(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_avoidance_cloud(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr closest_point_cloud(new pcl::PointCloud<pcl::PointXYZ>());
pcl::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ>);
std::vector<int> nn_indices (1);
std::vector<float> nn_dists (1);

pcl::PassThrough<pcl::PointXYZ> pass;

pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::Ptr ndt_omp(new pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>());

pcl::VoxelGrid<pcl::PointXYZ> voxelgrid_local;
pcl::VoxelGrid<pcl::PointXYZ> voxelgrid_global;
Eigen::Matrix4f global_transformation;
Eigen::Matrix4f local_transformation;
Eigen::Matrix4f IMU_GYRO_transformation_current= Eigen::Matrix4f::Identity();
Eigen::Matrix4f IMU_GYRO_transformation_previous= Eigen::Matrix4f::Identity();
Eigen::Matrix4f yaw_matrix;
Eigen::Matrix4f pitch_roll_matrix;
Eigen::Matrix4f rotation_matrix;
sensor_msgs::PointCloud2 transformed_current_lidar_frame;
geometry_msgs::Pose current_UAV_pose;
geometry_msgs::Point obstacle_position;

void localization(const sensor_msgs::PointCloud2 &current_lidar_frame)
{
  Eigen::Matrix4f transform;
  transform(0,0)= 1.0;
  transform(0,1)= 0.0;
  transform(0,2)= 0.0;
  transform(0,3)= 0.0;
  transform(1,0)= 0.0;
  transform(1,1)= -1.0;
  transform(1,2)= 0.0;
  transform(1,3)= 0.0;
  transform(2,0)= 0.0;
  transform(2,1)= 0.0;
  transform(2,2)= -1.0;
  transform(2,3)= 0.0;
  transform(3,0)= 0.0;
  transform(3,1)= 0.0;
  transform(3,2)= 0.0;
  transform(3,3)= 1.0;

  pcl_ros::transformPointCloud(transform,current_lidar_frame,transformed_current_lidar_frame);
  pcl::fromROSMsg(transformed_current_lidar_frame, *source_cloud);
  *temporary_cloud= *source_cloud;
  if (counter == 0){
    counter = counter + 1;
}
  else{
    if(counter == 1){
      *target_cloud= *source_cloud;
      *combined_cloud= *source_cloud;
    }
    if(counter > 100){
      voxelgrid_global.setInputCloud(combined_cloud);
      voxelgrid_global.filter(*downsampled);
      *combined_cloud = *downsampled;    
    }
    else{
      voxelgrid_local.setInputCloud(combined_cloud);
      voxelgrid_local.filter(*downsampled);
      *combined_cloud = *downsampled;
    }

    voxelgrid_local.setInputCloud(target_cloud);
    voxelgrid_local.filter(*downsampled);
    *target_cloud = *downsampled;

    voxelgrid_local.setInputCloud(source_cloud);
    voxelgrid_local.filter(*downsampled);
    *source_cloud = *downsampled;

    local_transformation= IMU_GYRO_transformation_previous.inverse()*IMU_GYRO_transformation_current;
    ndt_omp->setInputTarget(target_cloud);
    ndt_omp->setInputSource(source_cloud);
    ndt_omp->align( *aligned, local_transformation); //Eigen::Matrix4f::Identity() );
    local_transformation= ndt_omp->getFinalTransformation();

    global_transformation= global_transformation*local_transformation;
    pcl::transformPointCloud(*source_cloud,*aligned, global_transformation);

    pass.setInputCloud (combined_cloud);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (global_transformation(0,3)-7, global_transformation(0,3)+7);
    pass.filter (*combined_cloud);
    pass.setInputCloud (combined_cloud);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (global_transformation(1,3)-7, global_transformation(1,3)+7);
    pass.filter (*combined_cloud);

    ndt_omp->setInputTarget(combined_cloud);
    ndt_omp->setInputSource(aligned);
    ndt_omp->align( *aligned , Eigen::Matrix4f::Identity() );
    local_transformation= ndt_omp->getFinalTransformation();

    global_transformation= local_transformation*global_transformation;
    pcl::transformPointCloud (*source_cloud,*aligned, global_transformation);
  
    *combined_cloud += *aligned;
  
    position.x = global_transformation(0,3);
    position.y = global_transformation(1,3);
    position.z = global_transformation(2,3);
    euler_angles.x= atan2( global_transformation(2,1) , global_transformation(2,2) ) * 180 / PI;
    euler_angles.y= atan2( -global_transformation(2,0) , sqrt( pow( global_transformation(2,1) , 2.0 ) + pow( global_transformation(2,2) ,2.0 ) ) ) * 180 / PI;
    euler_angles.z= atan2( global_transformation(1,0) , global_transformation(0,0) ) * 180 / PI;
    current_UAV_pose.position.x = position.x;
    current_UAV_pose.position.y = position.y;
    current_UAV_pose.position.z = position.z;
    current_UAV_pose.orientation.x = euler_angles.x;
    current_UAV_pose.orientation.y = euler_angles.y;
    current_UAV_pose.orientation.z = euler_angles.z;
    current_UAV_pose.orientation.w = 1; 

    yaw_matrix(0,0)= cos(euler_angles.z* PI/180);
    yaw_matrix(0,1)= sin(euler_angles.z* PI/180);
    yaw_matrix(0,2)= 0.0;
    yaw_matrix(0,3)= 0.0;
    yaw_matrix(1,0)= -sin(euler_angles.z* PI/180);
    yaw_matrix(1,1)= cos(euler_angles.z* PI/180);
    yaw_matrix(1,2)= 0.0;
    yaw_matrix(1,3)= 0.0;
    yaw_matrix(2,0)= 0.0;
    yaw_matrix(2,1)= 0.0;
    yaw_matrix(2,2)= 1.0;
    yaw_matrix(2,3)= 0.0;
    yaw_matrix(3,0)= 0.0;
    yaw_matrix(3,1)= 0.0;
    yaw_matrix(3,2)= 0.0;
    yaw_matrix(3,3)= 1.0;
      
    rotation_matrix(0,0)= global_transformation(0,0);
    rotation_matrix(0,1)= global_transformation(0,1);
    rotation_matrix(0,2)= global_transformation(0,2);
    rotation_matrix(0,3)= 0.0;
    rotation_matrix(1,0)= global_transformation(1,0);
    rotation_matrix(1,1)= global_transformation(1,1);
    rotation_matrix(1,2)= global_transformation(1,2);
    rotation_matrix(1,3)= 0.0;
    rotation_matrix(2,0)= global_transformation(2,0);
    rotation_matrix(2,1)= global_transformation(2,1);
    rotation_matrix(2,2)= global_transformation(2,2);
    rotation_matrix(2,3)= 0.0;
    rotation_matrix(3,0)= 0.0;
    rotation_matrix(3,1)= 0.0;
    rotation_matrix(3,2)= 0.0;
    rotation_matrix(3,3)= 1.0;

    pitch_roll_matrix= yaw_matrix*rotation_matrix;
    pcl::transformPointCloud (*source_cloud,*obstacle_avoidance_cloud, pitch_roll_matrix);
    
     pass.setInputCloud (obstacle_avoidance_cloud);
     pass.setFilterFieldName ("x");
     pass.setFilterLimits (-10.0, 10.0);
     pass.filter (*obstacle_avoidance_cloud);

     pass.setInputCloud (obstacle_avoidance_cloud);
     pass.setFilterFieldName ("y");
     pass.setFilterLimits (-10.0, 10.0);
     pass.filter (*obstacle_avoidance_cloud);

     pass.setInputCloud (obstacle_avoidance_cloud);
     pass.setFilterFieldName ("z");
     pass.setFilterLimits (-0.4, 1.4);
     pass.filter (*obstacle_avoidance_cloud);

     if (obstacle_avoidance_cloud->size() > 0){
         pcl::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ>);
         tree->setInputCloud(obstacle_avoidance_cloud);
         std::vector<int> nn_indices (2);
         std::vector<float> nn_dists (2);
         tree->nearestKSearch(pcl::PointXYZ(0, 0, 0), 2, nn_indices, nn_dists);
         closest_point_cloud->points[0].x = obstacle_avoidance_cloud->points[nn_indices[1]].x;
         closest_point_cloud->points[0].y = obstacle_avoidance_cloud->points[nn_indices[1]].y;
         closest_point_cloud->points[0].z = obstacle_avoidance_cloud->points[nn_indices[1]].z;
         pcl::transformPointCloud (*closest_point_cloud,*closest_point_cloud, global_transformation*rotation_matrix.transpose()*yaw_matrix.transpose());
}
     else{
         pcl::PointCloud<pcl::PointXYZ>::Ptr closest_point_cloud (new pcl::PointCloud<pcl::PointXYZ>);
         closest_point_cloud->points[0].x = 0.0;
         closest_point_cloud->points[0].y = 0.0;
         closest_point_cloud->points[0].z = 0.0;
}

     obstacle_position.x = closest_point_cloud->points[0].x;
     obstacle_position.y = closest_point_cloud->points[0].y;
     obstacle_position.z = closest_point_cloud->points[0].z;
     counter = counter + 1;

     *target_cloud = *temporary_cloud;
     pcl::transformPointCloud (*temporary_cloud, *temporary_cloud, global_transformation);
     pcl::toROSMsg(*temporary_cloud, transformed_current_lidar_frame);
     pose.publish(current_UAV_pose);
     obstacle.publish(obstacle_position);
     aligned_point_cloud.publish(transformed_current_lidar_frame);
     ros::spinOnce();
  }
}

void read_IMU_GYRO_attitude_msmt(geometry_msgs::QuaternionStamped attitude_msmt){
if (counter == 1){
  IMU_GYRO_transformation_current(0,0)= 1.0 - (2*pow(attitude_msmt.quaternion.y,2)) - (2*pow(attitude_msmt.quaternion.z,2));
  IMU_GYRO_transformation_current(0,1)= (2*attitude_msmt.quaternion.x*attitude_msmt.quaternion.y) - (2*attitude_msmt.quaternion.z*attitude_msmt.quaternion.w);
  IMU_GYRO_transformation_current(0,2)= (2*attitude_msmt.quaternion.x*attitude_msmt.quaternion.z) + (2*attitude_msmt.quaternion.y*attitude_msmt.quaternion.w);
  IMU_GYRO_transformation_current(0,3)= 0.0;
  IMU_GYRO_transformation_current(1,0)= (2*attitude_msmt.quaternion.x*attitude_msmt.quaternion.y) + (2*attitude_msmt.quaternion.z*attitude_msmt.quaternion.w);
  IMU_GYRO_transformation_current(1,1)= 1.0 - (2*pow(attitude_msmt.quaternion.x,2)) - (2*pow(attitude_msmt.quaternion.z,2));
  IMU_GYRO_transformation_current(1,2)= (2*attitude_msmt.quaternion.y*attitude_msmt.quaternion.z) - (2*attitude_msmt.quaternion.x*attitude_msmt.quaternion.w);
  IMU_GYRO_transformation_current(1,3)= 0.0;
  IMU_GYRO_transformation_current(2,0)= (2*attitude_msmt.quaternion.x*attitude_msmt.quaternion.z) - (2*attitude_msmt.quaternion.y*attitude_msmt.quaternion.w);
  IMU_GYRO_transformation_current(2,1)= (2*attitude_msmt.quaternion.y*attitude_msmt.quaternion.z) + (2*attitude_msmt.quaternion.x*attitude_msmt.quaternion.w);
  IMU_GYRO_transformation_current(2,2)= 1.0 - (2*pow(attitude_msmt.quaternion.x,2)) - (2*pow(attitude_msmt.quaternion.y,2));
  IMU_GYRO_transformation_current(2,3)= 0.0;
  IMU_GYRO_transformation_current(3,0)= 0.0;
  IMU_GYRO_transformation_current(3,1)= 0.0;
  IMU_GYRO_transformation_current(3,2)= 0.0;
  IMU_GYRO_transformation_current(3,3)= 1.0;

  IMU_GYRO_transformation_previous= IMU_GYRO_transformation_current;
}
else if(counter > 1){
  IMU_GYRO_transformation_previous= IMU_GYRO_transformation_current;

  IMU_GYRO_transformation_current(0,0)= 1.0 - (2*pow(attitude_msmt.quaternion.y,2)) - (2*pow(attitude_msmt.quaternion.z,2));
  IMU_GYRO_transformation_current(0,1)= (2*attitude_msmt.quaternion.x*attitude_msmt.quaternion.y) - (2*attitude_msmt.quaternion.z*attitude_msmt.quaternion.w);
  IMU_GYRO_transformation_current(0,2)= (2*attitude_msmt.quaternion.x*attitude_msmt.quaternion.z) + (2*attitude_msmt.quaternion.y*attitude_msmt.quaternion.w);
  IMU_GYRO_transformation_current(0,3)= 0.0;
  IMU_GYRO_transformation_current(1,0)= (2*attitude_msmt.quaternion.x*attitude_msmt.quaternion.y) + (2*attitude_msmt.quaternion.z*attitude_msmt.quaternion.w);
  IMU_GYRO_transformation_current(1,1)= 1.0 - (2*pow(attitude_msmt.quaternion.x,2)) - (2*pow(attitude_msmt.quaternion.z,2));
  IMU_GYRO_transformation_current(1,2)= (2*attitude_msmt.quaternion.y*attitude_msmt.quaternion.z) - (2*attitude_msmt.quaternion.x*attitude_msmt.quaternion.w);
  IMU_GYRO_transformation_current(1,3)= 0.0;
  IMU_GYRO_transformation_current(2,0)= (2*attitude_msmt.quaternion.x*attitude_msmt.quaternion.z) - (2*attitude_msmt.quaternion.y*attitude_msmt.quaternion.w);
  IMU_GYRO_transformation_current(2,1)= (2*attitude_msmt.quaternion.y*attitude_msmt.quaternion.z) + (2*attitude_msmt.quaternion.x*attitude_msmt.quaternion.w);
  IMU_GYRO_transformation_current(2,2)= 1.0 - (2*pow(attitude_msmt.quaternion.x,2)) - (2*pow(attitude_msmt.quaternion.y,2));
  IMU_GYRO_transformation_current(2,3)= 0.0;
  IMU_GYRO_transformation_current(3,0)= 0.0;
  IMU_GYRO_transformation_current(3,1)= 0.0;
  IMU_GYRO_transformation_current(3,2)= 0.0;
  IMU_GYRO_transformation_current(3,3)= 1.0;
}

}

void initialization(){
  ndt_omp->setTransformationEpsilon (0.01);
  ndt_omp->setStepSize (0.1);
  ndt_omp->setResolution(1.0);
  ndt_omp->setNumThreads(7);
  ndt_omp->setNeighborhoodSearchMethod(pclomp::DIRECT7);
  ndt_omp->setMaximumIterations(35);
  voxelgrid_local.setLeafSize(0.3f, 0.3f, 0.3f);
  voxelgrid_global.setLeafSize(0.3f, 0.3f, 0.3f);
  global_transformation(0,0)= 1.0;
  global_transformation(0,1)= 0.0;
  global_transformation(0,2)= 0.0;
  global_transformation(0,3)= 0.0;
  global_transformation(1,0)= 0.0;
  global_transformation(1,1)= 1.0;
  global_transformation(1,2)= 0.0;
  global_transformation(1,3)= 0.0;
  global_transformation(2,0)= 0.0;
  global_transformation(2,1)= 0.0;
  global_transformation(2,2)= 1.0;
  global_transformation(2,3)= 0.0;
  global_transformation(3,0)= 0.0;
  global_transformation(3,1)= 0.0;
  global_transformation(3,2)= 0.0;
  global_transformation(3,3)= 1.0;
  closest_point_cloud->width= 1;
  closest_point_cloud->height= 1;
  closest_point_cloud->is_dense= false;
  closest_point_cloud->points.resize (closest_point_cloud->width * closest_point_cloud->height);
}

int main(int argc, char **argv)
{
  initialization();
  ros::init(argc, argv, "velodyne1");
  ros::NodeHandle n1;
  ros::Subscriber sub = n1.subscribe("/os1_node/points", 2, localization);
  pose = n1.advertise<geometry_msgs::Pose>("/UAV_pose", 2);
  obstacle = n1.advertise<geometry_msgs::Point>("/obstacle_position", 2);
  //aligned_point_cloud = n1.advertise<sensor_msgs::PointCloud2>("/aligned_point_cloud", 2);
  ros::Subscriber IMU_GYRO_attitude_msmt = n1.subscribe<geometry_msgs::QuaternionStamped>("/dji_sdk/attitude",2, read_IMU_GYRO_attitude_msmt);
  ros::spin();
  return 0;
}
