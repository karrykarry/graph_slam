#include"odometer.hpp"

Dead_rec::Dead_rec(ros::NodeHandle n,ros::NodeHandle priv_nh):
	odom_sub(n,"/odom",1),imu_sub(n,"/imu/data",1),
	sync(SyncPolicy(10),odom_sub,imu_sub),
	x(0.0),y(0.0),z(0.0),
    roll(0.0),pitch(0.0),yaw(0.0)
{
	sync.registerCallback(boost::bind(&Dead_rec::syncMsgsCB, this, _1, _2));

	odom_pub = n.advertise<nav_msgs::Odometry>("/odom/complement",1);

	priv_nh.param("drift/droll",drift_droll,{0.0});
	priv_nh.param("drift/dpitch",drift_dpitch,{0.0});
	priv_nh.param("drift/dyaw",drift_dyaw,{0.0});
	priv_nh.param("parent_frame",parent_frame,{"/map"});
	priv_nh.param("child_frame",child_frame,{"/base_link"});
	
	lcl.header.frame_id = parent_frame;
	lcl.child_frame_id = child_frame;
}


void
Dead_rec::syncMsgsCB(const nav_msgs::OdometryConstPtr &odom, const sensor_msgs::ImuConstPtr &imu){
	double odom_vel;
	double droll,dpitch,dyaw;
	double dt;
	
    geometry_msgs::Quaternion odom_quat;

	odom_vel = odom->twist.twist.linear.x;
 	
    droll = imu->angular_velocity.x;
	droll -= drift_droll;
 	dpitch = imu->angular_velocity.y;
	dpitch -= drift_dpitch;
 	dyaw = imu->angular_velocity.z;
	dyaw -= drift_dyaw;

    dt = dt_calc(imu->header.stamp);

	double dist = odom_vel * dt; 
	
	roll += droll * dt; 
	pitch += dpitch * dt; 
	yaw += dyaw * dt; 

    complement_angle(roll);
    complement_angle(pitch);
    complement_angle(yaw);
    
	x += dist * cos(yaw);// * cos(pitch);
	y += dist * sin(yaw);// * cos(pitch);
	z += dist * sin(dpitch);
	     
    odom_quat = tf::createQuaternionMsgFromYaw(yaw);
	
	pub(imu->header.stamp,odom_quat);

}

double
Dead_rec::dt_calc(ros::Time current_time){
	
    static bool flag = true;
	if(flag){ 
		last_time = current_time;
		flag = false;		
	}

	double last_accurate,current_accurate;
	last_accurate = (double)last_time.nsec*1.0e-9 + last_time.sec;
	current_accurate = (double)current_time.nsec*1.0e-9 + current_time.sec;
	double dt = current_accurate - last_accurate;

	
	last_time = current_time;
	

	return dt;
}



void
Dead_rec::pub(ros::Time current_time,geometry_msgs::Quaternion quat){

	lcl.header.stamp = current_time;
	lcl.pose.pose.position.x = x;
	lcl.pose.pose.position.y = y;
	lcl.pose.pose.position.z = 0.0;
	lcl.pose.pose.orientation = quat;

	odom_pub.publish(lcl);

	transform.setOrigin( tf::Vector3( x, y, 0.0) );
	tf::Quaternion q;
	q.setRPY(0.0, 0.0, yaw);

	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, current_time, parent_frame, child_frame));
}
