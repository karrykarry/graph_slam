#include"node_selector.hpp"

template<typename T_p>
Node_Selector<T_p>::Node_Selector(ros::NodeHandle n, ros::NodeHandle private_nh_){
	
    package_path = ros::package::getPath("graph_slam");
	file = new GRAPH_SLAM::File();


	node_pub = n.advertise<sensor_msgs::PointCloud2>("/node", 1, true);
	pc_publisher = n.advertise<sensor_msgs::PointCloud2>("/velodyne_points/slam", 1);

}

template<typename T_p>
bool
Node_Selector<T_p>::calc_dist(tf::Transform cur_tf, tf::Transform pre_tf){
	bool flag;

	flag = (sqrt(pow(cur_tf.getOrigin().x()-pre_tf.getOrigin().x(),2)+
			pow(cur_tf.getOrigin().y()-pre_tf.getOrigin().y(),2)) > dist_threshold ? true : false);

	return flag;
}



template<typename T_p>
void
Node_Selector<T_p>::run(){
	std::cout<<"Please serect csv file:"<<std::endl;
	std::string file_name;
	std::cin >> file_name;

	std::string file_csv = package_path + csv_path + file_name;
	
	std::cout<<file_csv<<std::endl;

	file->loadTF(transforms,file_csv);
	
	tf::Transform pre_tf = transforms[0].transform;
	pcl::PointCloud<pcl::PointXYZ>::Ptr aft_node(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointXYZ temp_pc;
	temp_pc.x = pre_tf.getOrigin().x();
	temp_pc.y = pre_tf.getOrigin().y();
	temp_pc.z = 0;
	aft_node->push_back(temp_pc);
	aft_id.push_back(0);

	for(auto cur_tf : transforms){

		if(calc_dist(cur_tf.transform, pre_tf )){
			pre_tf = cur_tf.transform;
			std::cout<<"id"<<cur_tf.id<<std::endl;
			pcl::PointXYZ temp_pc;
			temp_pc.x = cur_tf.transform.getOrigin().x();
			temp_pc.y = cur_tf.transform.getOrigin().y();
			temp_pc.z = 0;
			aft_node->push_back(temp_pc);
			
			aft_id.push_back(cur_tf.id);	
		}
	}
	
	sensor_msgs::PointCloud2 pc;
	pcl::toROSMsg(*aft_node, pc);

	pc.header.frame_id  = "map";	
	pc.header.stamp  = ros::Time::now();
	node_pub.publish(pc);
}


template<typename T_p>
void
Node_Selector<T_p>::pc_pub(const int cnt_){
	std::string file_pcd = package_path + pcd_path + std::to_string(cnt_) + ".pcd";
	std::cout<<file_pcd<<std::endl;
	typename pcl::PointCloud<T_p>::Ptr output_pc(new pcl::PointCloud<T_p>);	
	typename pcl::PointCloud<T_p>::Ptr trans_output_pc(new pcl::PointCloud<T_p>);	
	file->loadCloud<T_p>(output_pc, file_pcd);
	
	tf::Transform tf = transforms[cnt_].transform; 
    pcl_ros::transformPointCloud(*output_pc, *trans_output_pc, tf);

	br.sendTransform(tf::StampedTransform(tf, ros::Time::now(), "/map", "/base_link"));

	sensor_msgs::PointCloud2 pc;
	pcl::toROSMsg(*output_pc, pc);
	pc.header.frame_id  = "/velodyne";	
	pc.header.stamp  = ros::Time(0);
	pc_publisher.publish(pc);
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "node_selector");
	ros::NodeHandle n;
	ros::NodeHandle priv_nh("~");

    ROS_INFO("\033[1;32m---->\033[0m node_selector Started.");

	Node_Selector<pcl::PointXYZINormal> node_selector(n,priv_nh);	

	std::cout<<"Please dist threshold[m]:"<<std::endl;
	std::cin >> node_selector.dist_threshold;

	node_selector.run();

	ros::Rate loop(1);
		
	std::cout<<"Number of Node:"<<node_selector.aft_id.size()<<std::endl;
	while(ros::ok()){
		static int cnt_=0;
		if(node_selector.aft_id.size()<=cnt_) break;
		node_selector.pc_pub(node_selector.aft_id[cnt_]);
		cnt_++;

		ros::spinOnce();
		loop.sleep();
	}
 
	return 0;
}

