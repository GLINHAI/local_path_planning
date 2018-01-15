#include <iostream>

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "send_position");
  ros::NodeHandle n;
  nav_msgs::Odometry global_position;
  ros::Publisher send_position = n.advertise<nav_msgs::Odometry> ("global_position", 1);
  ros::Rate loop_rate(1.0);

  double time = ros::Time::now().toSec();

  while (ros::ok()) {    
    ros::spinOnce();
    global_position.pose.pose.position.x = 12.3;
    global_position.pose.pose.position.y = 18.3;
    send_position.publish(global_position);
    ROS_INFO("I publish the data");

    loop_rate.sleep();
  }

  return 0;
}
