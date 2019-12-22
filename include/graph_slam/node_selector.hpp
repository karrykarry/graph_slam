#ifndef _NODE_SELECTOR_HPP_ 
#define _NODE_SELECTOR_HPP_ 

#include<iostream>
#include<vector>
#include<sstream>
#include<iomanip>
#include<fstream>

#include<ros/ros.h>
#include<ros/package.h>
#include<sensor_msgs/PointCloud2.h>

#include <tf/transform_broadcaster.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

#include"File.h"


template<typename T_p>
class Node_Selector{
	public:
		Node_Selector<T_p>(ros::NodeHandle n, ros::NodeHandle private_nh_);

		ros::Publisher node_pub;
		ros::Publisher pc_publisher;

		bool calc_dist(tf::Transform cur_tf, tf::Transform pre_tf);
		void pc_pub(const int);
		void run();
		
		double dist_threshold;
		std::vector<int> aft_id;
		const std::string csv_path = "/data/csv/";
		const std::string pcd_path = "/data/pcd/";

	private:
		GRAPH_SLAM::File* file;
        std::string package_path;
		std::vector<ID> transforms;
		
		tf::TransformBroadcaster br;
	
};

#endif
