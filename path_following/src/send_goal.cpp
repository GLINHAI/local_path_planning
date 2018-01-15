#include <iostream>

#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"


int main(int argc, char** argv) {
  ros::init(argc, argv, "send_goal");
  ros::NodeHandle n;
  ros::Publisher send_goal = n.advertise<geometry_msgs::PoseStamped> ("goal_point", 1);
  ros::Rate loop_rate(2.0);
  while (ros::ok()) {
    geometry_msgs::PoseStamped goal_position;
    goal_position.pose.position.x = 0.0;
    goal_position.pose.position.y = 0.0;
    ROS_INFO("%f", goal_position);
    send_goal.publish(goal_position);
    goal_position.pose.position.x += 1.0;
    goal_position.pose.position.y += 2.0;
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
  

