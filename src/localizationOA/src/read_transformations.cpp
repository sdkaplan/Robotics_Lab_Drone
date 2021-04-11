#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <Eigen/Dense>

#include "read_transformations.h"


using namespace std;
// read the parameter's file
bool read_transformations (map<int,Eigen::Matrix4f> &transformations)
//bool read_transformations (map<int,Eigen::Quaternionf> &quat)
{
	ifstream file("../attitude_at_Ouster_frames_2019_06_07_1.txt");
	//ifstream file("../attitude_at_Ouster_frames_2019_05_15.txt");
	//ifstream file("../attitude_at_Ouster_frames_2019_04_06.txt");
	//ifstream file("../attitude_at_Ouster_frames_2018_11_14.txt");

	if (! file.is_open())
	{
		cout<< "Could not open parameters file"<< endl;
		return false;
	}

	int index=0;
	Eigen::Quaternionf q;

	while (!file.eof())
	{
		file >> q.x();
		file >> q.y();
		file >> q.z();
		file >> q.w();

		Eigen::Matrix4f transform;
	    transform(0,0)= 1.0 - (2*pow(q.y(),2)) - (2*pow(q.z(),2));
	    transform(0,1)= (2*q.x()*q.y()) - (2*q.z()*q.w());
	    transform(0,2)= (2*q.x()*q.z()) + (2*q.y()*q.w());
	    transform(0,3)= 0.0;
	    transform(1,0)= (2*q.x()*q.y()) + (2*q.z()*q.w());
	    transform(1,1)= 1.0 - (2*pow(q.x(),2)) - (2*pow(q.z(),2));
	    transform(1,2)= (2*q.y()*q.z()) - (2*q.x()*q.w());
	    transform(1,3)= 0.0;
	    transform(2,0)= (2*q.x()*q.z()) - (2*q.y()*q.w());
	    transform(2,1)= (2*q.y()*q.z()) + (2*q.x()*q.w());
	    transform(2,2)= 1.0 - (2*pow(q.x(),2)) - (2*pow(q.y(),2));
	    transform(2,3)= 0.0;
	    transform(3,0)= 0.0;
	    transform(3,1)= 0.0;
	    transform(3,2)= 0.0;
	    transform(3,3)= 1.0;

		transformations[index]=transform;
		index= index + 1;
	}

	cout<< "Finish reading parameters file"<< endl;
	return true;

}



