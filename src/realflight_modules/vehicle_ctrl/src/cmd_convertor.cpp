
#include "ros/ros.h"
#include "quadrotor_msgs/PositionCommand.h"
#include <geometry_msgs/Twist.h>
#include "input.h"

ros::Subscriber position_cmd_sub;
ros::Publisher cmd_vel_push;

Odom_Data_t odom_data;
Command_Data_t cmd_data;

bool debug = false;

struct Desired_State_t
{
	Eigen::Vector3d p;
	Eigen::Vector3d v;
	Eigen::Vector3d a;
	Eigen::Vector3d j;
	Eigen::Quaterniond q;
	double yaw;
	double yaw_rate;

	Desired_State_t(){};

	Desired_State_t(Odom_Data_t &odom)
		: p(odom.p),
		  v(Eigen::Vector3d::Zero()),
		  a(Eigen::Vector3d::Zero()),
		  j(Eigen::Vector3d::Zero()),
		  q(odom.q),
		  yaw(uav_utils::get_yaw_from_quaternion(odom.q)),
		  yaw_rate(0){};
};

Desired_State_t get_cmd_des()
{
	Desired_State_t des;
	des.p = cmd_data.p;
	des.v = cmd_data.v;
	des.a = cmd_data.a;
	des.j = cmd_data.j;
	des.yaw = cmd_data.yaw;
	des.yaw_rate = cmd_data.yaw_rate;

	return des;
}

bool cmd_is_received(const ros::Time &now_time)
{
	return (now_time - cmd_data.rcv_stamp).toSec() < 1;
}

bool odom_is_received(const ros::Time &now_time)
{
	return (now_time - odom_data.rcv_stamp).toSec() < 1;
}
double fromQuaternion2yaw(Eigen::Quaterniond q)
{
  double yaw = atan2(2 * (q.x()*q.y() + q.w()*q.z()), q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z());
  return yaw;
}
/**
 * theta = yaw_cmd - yaw_odom
 * theta > 0, turn left
 * theta < 0, trun right
 * v = v_cmd * cos(theta)
 */
void calculateControl(const Desired_State_t &des,
    const Odom_Data_t &odom,
    geometry_msgs::Twist &t) 
{
  double yaw_odom = fromQuaternion2yaw(odom.q);
  double x = des.v.x();
  double y = des.v.y();
  double yaw_cmd = atan2(y ,x);
  double theta = yaw_cmd - 3.1415926 / 2 - yaw_odom;

  double v = std::sqrt(std::pow(x, 2) + std::pow(y, 2));

  double sin = std::sin(theta);
  double cos = std::cos(theta);

  t.linear.x = 1 *  v * cos;
  t.angular.z = 3 *  v * sin;

  if (debug) {
    ROS_INFO("cmd_v:(%f, %f) \t\tyaw_odom:%f \tyaw_cmd:%f \ttheta:%f",
      x, y, yaw_odom, yaw_cmd, theta);
  }
}

void process() 
{
	ros::Time now_time = ros::Time::now();
  if (debug && !odom_is_received(now_time)) {
    ROS_INFO("odom not received");
    return;
  }
  
  if (debug && !cmd_is_received(now_time)) {
    ROS_INFO("cmd not received");
    return;
  }
  geometry_msgs::Twist cmd_vel;

  Desired_State_t des = get_cmd_des();
    
  calculateControl(des, odom_data, cmd_vel);

  cmd_vel_push.publish(cmd_vel);
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "quadrotor_msg_convertor");

    ROS_INFO("cmd_convertor started");
    ros::NodeHandle nh("~");

    ros::Subscriber cmd_sub =
        nh.subscribe<quadrotor_msgs::PositionCommand>("position_cmd",
                                                      100,
                                                      boost::bind(&Command_Data_t::feed, &cmd_data, _1),
                                                      ros::VoidConstPtr(),
                                                      ros::TransportHints().tcpNoDelay());
    ros::Subscriber odom_sub =
        nh.subscribe<nav_msgs::Odometry>("odom",
                                         100,
                                         boost::bind(&Odom_Data_t::feed, &odom_data, _1),
                                         ros::VoidConstPtr(),
                                         ros::TransportHints().tcpNoDelay());

    cmd_vel_push = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    ros::Rate loop_rate(10);
    while(ros::ok()) {
        process();
        ros::spinOnce();
     
        loop_rate.sleep();
    }
}