/*
 * gyro_odometry.cpp
 *
 * calcrate gyro odometry 
 *
 * author : Yudai Sadakuni
 *
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>


using namespace std;

class odomPublisher
{
  private:
    ros::NodeHandle n;
    ros::Subscriber sub_wheel;
    ros::Subscriber sub_gyro;
    ros::Publisher pub_odom;
    ros::Publisher pub_flag_run;
    tf::TransformBroadcaster odom_broadcaster;
    tf::Quaternion quaternion;
    geometry_msgs::Quaternion quat_Msg;

    string topic_wheel;
    string topic_gyro;
    string topic_pub;
    string global_frame;
    string child_frame;
    double x, y, z;
    double roll, pitch, yaw;
    geometry_msgs::Quaternion odom_quat;
    double vel;
    double droll,dpitch,dyaw;
    double pitch_init;
    double drift_dpitch, drift_dyaw;
    double rate;
    std_msgs::Bool flag_run;
    std_msgs::Bool flag_tf;
    ros::Time current_time, last_time;

  public:
    odomPublisher();
    void wheelCallback(const nav_msgs::Odometry::ConstPtr&);
    void gyroCallback(const sensor_msgs::Imu::ConstPtr&);
    void complement();
    void publisher();
    void pubIsRun();
    void pubTF();
    void setRate(const double&);
};

odomPublisher::odomPublisher()
  : n("~"), 
    x(0.0), y(0.0), z(0.0), 
    roll(0.0), pitch(0.0), yaw(0.0), vel(0.0), 
    droll(0.0),dpitch(0.0), dyaw(0.0), rate(1.0)
{
  n.param<string>("topic_name/wheel", topic_wheel, "/tinypower/odom");
  n.param<string>("topic_name/gyro", topic_gyro, "/AMU_data");
  n.param<string>("topic_name/odom_complement", topic_pub, "/odom/complement");

  n.param<string>("complement/global_frame", global_frame, "/map");
  n.param<string>("complement/child_frame", child_frame, "/base_link");

  sub_wheel = n.subscribe<nav_msgs::Odometry>(topic_wheel, 1, &odomPublisher::wheelCallback, this);
  sub_gyro = n.subscribe<sensor_msgs::Imu>(topic_gyro, 1, &odomPublisher::gyroCallback, this);

  pub_odom = n.advertise<nav_msgs::Odometry>(topic_pub, 1);
  pub_flag_run = n.advertise<std_msgs::Bool>("/flag/is_run", 1);

  n.param("dpitch/drift", drift_dpitch, {0.287207006});
  n.param("dyaw/drift", drift_dyaw, {0.287207006});

  flag_run.data = false;

  current_time = ros::Time::now();
  last_time= ros::Time::now();
}

void odomPublisher::wheelCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  vel = msg->twist.twist.linear.x;
  vel = vel > 1.5 ? 1.5 : vel;
  flag_run.data = fabs(vel) < 0.1 ? false : true;
  vel *= rate;
}

void odomPublisher::gyroCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  /* droll = msg->angular_velocity.x; */
  /* droll *= rate; */
  /* dpitch = msg->angular_velocity.y - drift_dpitch; */
  /* dpitch *= rate; */
  dyaw = msg->angular_velocity.z - drift_dyaw;
  dyaw *= rate;
}

void odomPublisher::complement()
{
  current_time = ros::Time::now();
  double dt = (current_time - last_time).toSec();
  last_time = ros::Time::now();

  double dist = vel * dt;
  std::cout<<"--------"<<std::endl;


  roll += droll * dt;
  pitch += dpitch * dt;
  yaw += dyaw * dt;

  std::cout<<"roll:"<<roll<<std::endl;
  std::cout<<"pitch:"<<pitch<<std::endl;
  std::cout<<"yaw:"<<yaw<<std::endl;


  while(roll > M_PI) roll -= 2*M_PI;
  while(roll < -M_PI) roll += 2*M_PI;
  while(pitch > M_PI) pitch -= 2*M_PI;
  while(pitch < -M_PI) pitch += 2*M_PI;
  while(yaw > M_PI) yaw -= 2*M_PI;
  while(yaw < -M_PI) yaw += 2*M_PI;
  
  x += dist * cos(yaw);
  y += dist * sin(yaw);
  
  quaternion=tf::createQuaternionFromRPY(roll,pitch*(-1.0),yaw);
  quaternionTFToMsg(quaternion,quat_Msg);
//  odom_quat = tf::createQuaternionMsgFromYaw(yaw);
  std::cout<<"--------"<<std::endl;
}

void odomPublisher::publisher()
{
  nav_msgs::Odometry odom;
  odom.header.frame_id = global_frame;
  odom.header.stamp = ros::Time::now();

  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = quat_Msg;

  pub_odom.publish(odom);
}

void odomPublisher::pubIsRun()
{
  pub_flag_run.publish(flag_run);
}

void odomPublisher::pubTF()
{
  //first, we'll publish the transform over tf
  geometry_msgs::TransformStamped odom_trans;
  //odom_trans.header.stamp = ros::Time::now();
  odom_trans.header.stamp = ros::Time::now();
  odom_trans.header.frame_id = global_frame;
  odom_trans.child_frame_id = child_frame;

  odom_trans.transform.translation.x = x;
  odom_trans.transform.translation.y = y;
  odom_trans.transform.translation.z =  0.0;
  odom_trans.transform.rotation = quat_Msg;

  //send the transform
  odom_broadcaster.sendTransform(odom_trans);
}

void odomPublisher::setRate(const double& bag_rate)
{
  rate = bag_rate;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gyro_odometry");

  odomPublisher op;

  if(argc==2){
    op.setRate(atof(argv[1]));
  }

  ros::Rate loop_rate(100);

  while(ros::ok()){
    op.complement();
    op.publisher(); //odom/complement
    op.pubIsRun();  //flag/is_run
    op.pubTF();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
