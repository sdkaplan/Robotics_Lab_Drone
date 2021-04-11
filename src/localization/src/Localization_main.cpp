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
pcl::PointXYZ position;
pcl::PointXYZ euler_angles;
pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr combined_cloud(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr temporary_cloud(new pcl::PointCloud<pcl::PointXYZ>());

pcl::PassThrough<pcl::PointXYZ> pass;

pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::Ptr ndt_omp(new pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>());

pcl::VoxelGrid<pcl::PointXYZ> voxelgrid_local;
pcl::VoxelGrid<pcl::PointXYZ> voxelgrid_global;
Eigen::Matrix4f global_transformation;
Eigen::Matrix4f local_transformation;
Eigen::Matrix4f IMU_GYRO_transformation_current= Eigen::Matrix4f::Identity();
Eigen::Matrix4f IMU_GYRO_transformation_previous= Eigen::Matrix4f::Identity();
sensor_msgs::PointCloud2 transformed_current_lidar_frame;
geometry_msgs::Pose current_UAV_pose;

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
    pass.setFilterLimits (global_transformation(0,3)-15, global_transformation(0,3)+15);
    pass.filter (*combined_cloud);
    pass.setInputCloud (combined_cloud);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (global_transformation(1,3)-15, global_transformation(1,3)+15);
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
    counter = counter + 1;

    *target_cloud = *temporary_cloud;
    pcl::transformPointCloud (*temporary_cloud, *temporary_cloud, global_transformation);
    pcl::toROSMsg(*temporary_cloud, transformed_current_lidar_frame);
    pose.publish(current_UAV_pose);
    cout << "The pose packet is sent" << endl; 
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
  ndt_omp->setNumThreads(8);
  ndt_omp->setNeighborhoodSearchMethod(pclomp::DIRECT7);
  ndt_omp->setMaximumIterations(35);
  voxelgrid_local.setLeafSize(0.5f, 0.5f, 0.5f);
  voxelgrid_global.setLeafSize(0.5f, 0.5f, 0.5f);
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
}

int main(int argc, char **argv)
{
  initialization();
  ros::init(argc, argv, "velodyne1");
  ros::NodeHandle n1;
  ros::Subscriber sub = n1.subscribe("/os1_node/points", 2, localization);
  pose = n1.advertise<geometry_msgs::Pose>("/UAV_pose", 2);
  ros::Subscriber IMU_GYRO_attitude_msmt = n1.subscribe<geometry_msgs::QuaternionStamped>("/dji_sdk/attitude",2, read_IMU_GYRO_attitude_msmt);
  ros::spin();
  return 0;
}
