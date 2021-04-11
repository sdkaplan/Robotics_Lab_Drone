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

#define PI 3.14159265


using namespace std;
using namespace pcl;

void read_pcd_file_callback(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int epoch, Eigen::Matrix4f transformation)
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

    pcl::PCDReader reader;
	string filename;
	stringstream sa;
	sa << setw(6) << setfill('0') << epoch;
	filename= sa.str();

	reader.read ("/home/robolab/Documents/2019_06_07_01/point_cloud_data/" + filename + ".pcd", *cloud);
	pcl::transformPointCloud (*cloud, *cloud, transform);
}


int main1(int argc, char *argv[]) {

	int initial_frame;
	int num_frames;
	int num_of_frames_to_skip;
	//int receding_horizon;
	istringstream (argv[1]) >> initial_frame;
	istringstream (argv[2]) >> num_frames;
	istringstream (argv[3]) >> num_of_frames_to_skip;
	//istringstream (argv[4]) >> receding_horizon;

	map<int, Eigen::Matrix4f> transformations;
	if (!read_transformations(transformations))
		cout<< "Error reading paramter files"<< endl;

	Eigen::Matrix4f transform;
    transform(0,0)= 1.0;
	transform(0,1)= 0.0;
	transform(0,2)= 0.0;
	transform(0,3)= 0.0;
	transform(1,0)= 0.0;
	transform(1,1)= 1.0;
	transform(1,2)= 0.0;
	transform(1,3)= 0.0;
	transform(2,0)= 0.0;
	transform(2,1)= 0.0;
	transform(2,2)= 1.0;
	transform(2,3)= 0.0;
	transform(3,0)= 0.0;
	transform(3,1)= 0.0;
	transform(3,2)= 0.0;
	transform(3,3)= 1.0;
	pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>());
  	pcl::PointCloud<pcl::PointXYZ>::Ptr temporary_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	pcl::PointCloud<pcl::PointXYZ>::Ptr combined_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	pcl::PointCloud<pcl::PointXYZ>::Ptr path(new pcl::PointCloud<pcl::PointXYZ>());
  	pcl::PointCloud<pcl::PointXYZ>::Ptr attitude(new pcl::PointCloud<pcl::PointXYZ>());
  	//std::vector<pcl::PointCloud<pcl::PointXYZ>> point_cloud_horizon(receding_horizon);
  	pcl::PointXYZ position;
  	pcl::PointXYZ euler_angles;
  	path->width = num_frames;
  	path->height = 1;
  	attitude->width = num_frames;
  	attitude->height = 1;

  	Eigen::Matrix4f local_transformation;
  	Eigen::Matrix4f global_transformation;

  	string filename_main;

  	pcl::PassThrough<pcl::PointXYZ> pass;

	pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::Ptr ndt_omp(new pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>());
	ndt_omp->setTransformationEpsilon (0.01);
	ndt_omp->setStepSize (0.1);
	ndt_omp->setResolution(1.0);
	ndt_omp->setNumThreads(8);
	ndt_omp->setNeighborhoodSearchMethod(pclomp::DIRECT1);
	ndt_omp->setMaximumIterations(20);
	
	pcl::visualization::PCLVisualizer vis("vis");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_handler(target_cloud, 255.0, 0.0, 0.0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_handler(source_cloud, 255.0, 255.0, 255.0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> aligned_handler(aligned, 0.0, 255.0, 0.0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> combined_handler(combined_cloud, 0.0, 0.0, 255.0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> path_handler(path, 255.0, 102.0, 0.0);

  	for (int epoch =  initial_frame ; epoch <= (initial_frame + num_frames); epoch=epoch+num_of_frames_to_skip+1){
  	  
  	  if (epoch == initial_frame){
  	  	read_pcd_file_callback(target_cloud, epoch, transformations[epoch]);
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
	    cout << epoch << endl;
	    //point_cloud_horizon[0]=*target_cloud;
	    *combined_cloud= *target_cloud;
	    filename_main= std::to_string(epoch);
	    //pcl::io::savePCDFileASCII ("/home/robolab/Documents/2019_06_05/data_with_camera_automatic/point_cloud_data_aligned/" + filename_main + ".pcd", *target_cloud);
  	  	continue;
  	  }
  	  else{
  	  	read_pcd_file_callback(source_cloud, epoch, transformations[epoch]);
  	  	*temporary_cloud= *source_cloud;
  	  }
  	  cout << epoch << endl;

	  pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
	  voxelgrid.setLeafSize(0.4f, 0.4f, 0.4f);

	  voxelgrid.setInputCloud(combined_cloud);
	  voxelgrid.filter(*downsampled);
	  *combined_cloud = *downsampled;

	  voxelgrid.setInputCloud(target_cloud);
	  voxelgrid.filter(*downsampled);
	  *target_cloud = *downsampled;

	  voxelgrid.setInputCloud(source_cloud);
	  voxelgrid.filter(*downsampled);
	  *source_cloud = *downsampled;

	  local_transformation= transformations[epoch-1].inverse()*transformations[epoch];

	  ndt_omp->setInputTarget(target_cloud);
	  ndt_omp->setInputSource(source_cloud);
	  ndt_omp->align(*aligned, local_transformation);
	  local_transformation= ndt_omp->getFinalTransformation();

	  global_transformation= global_transformation*local_transformation;
	  pcl::transformPointCloud (*source_cloud,*aligned, global_transformation);

	  pass.setInputCloud (combined_cloud);
	  pass.setFilterFieldName ("x");
	  pass.setFilterLimits (global_transformation(0,3)-35, global_transformation(0,3)+35);
	  pass.filter (*combined_cloud);
	  pass.setInputCloud (combined_cloud);
	  pass.setFilterFieldName ("y");
	  pass.setFilterLimits (global_transformation(1,3)-35, global_transformation(1,3)+35);
	  pass.filter (*combined_cloud);

	  ndt_omp->setInputTarget(combined_cloud);
	  ndt_omp->setInputSource(aligned);
	  ndt_omp->align(*aligned, transform);
	  local_transformation= ndt_omp->getFinalTransformation();

	  global_transformation= local_transformation*global_transformation;
	  pcl::transformPointCloud (*source_cloud,*aligned, global_transformation);
	  
	  *combined_cloud += *aligned;

	  filename_main= std::to_string(epoch);
	  /*pcl::io::savePCDFileASCII ("/home/robolab/Documents/2019_06_05/data_with_camera_automatic/point_cloud_data_aligned/" + filename_main + ".pcd", *aligned);
	  cout << global_transformation << endl;*/

  	  position.x = global_transformation(0,3);
	  position.y = global_transformation(1,3);
	  position.z = global_transformation(2,3);
	  euler_angles.x= atan2( global_transformation(2,1) , global_transformation(2,2) ) * 180 / PI;
	  euler_angles.y= atan2( -global_transformation(2,0) , sqrt( pow( global_transformation(2,1) , 2.0 ) + pow( global_transformation(2,2) ,2.0 ) ) ) * 180 / PI;
	  euler_angles.z= atan2( global_transformation(1,0) , global_transformation(0,0) ) * 180 / PI;
	  path->push_back(position);
	  attitude->push_back(euler_angles);
	  
	  cout << "X = " << position.x << " Y = " << position.y << " Z = " << position.z << " Roll = " << euler_angles.x << " Pitch = " << euler_angles.y << " Yaw = " << euler_angles.z << endl;
	  if ((epoch-initial_frame) > 2){
	  	vis.removePointCloud("combined_cloud");
	  	vis.removePointCloud("path");
	  }
	  if (((epoch-initial_frame) % 1) == 0){
	  	cout << ((epoch-initial_frame) % 25) << endl;
	  	vis.addPointCloud(combined_cloud, combined_handler, "combined_cloud");
	  	vis.addPointCloud(path, path_handler, "path");
	  	vis.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "path");
	  	vis.spinOnce();
	  }

	  /*if (epoch <= initial_frame+receding_horizon-1){
	  	point_cloud_horizon[epoch-initial_frame]= *aligned;
	  	*combined_cloud += *aligned;
	  }
	  else{
	  	*combined_cloud = *aligned;
	  	for (int i = 0; i < receding_horizon-1; i=i+1){
  	  		point_cloud_horizon[i]= point_cloud_horizon[i+1];
  	  		*combined_cloud += point_cloud_horizon[i];
  	  	}
  	  	point_cloud_horizon[receding_horizon-1]= *aligned;
	  }*/
	  
	  *target_cloud = *temporary_cloud;
	}

  return 0;
}
