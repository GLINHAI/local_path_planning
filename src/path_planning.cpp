#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <fstream>
#include <sstream>
#include <Eigen/Eigen>

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "custom_msg/llpath.h"





class Pose3d {
 public:
  Pose3d(const Eigen::Vector3d& position, double pitch, double yaw) :
    position_(position), pitch_(pitch), yaw_(yaw) { }
  double x() const { return position_.x(); }
  double y() const { return position_.y(); }
  double z() const { return position_.z(); }
  double pitch() const { return pitch_; }
  double yaw() const { return yaw_; }


 private:
  Eigen::Vector3d position_;
  double pitch_;
  double yaw_;
};

struct RobotState {
  Pose3d pose;
  Eigen::Vector3d linear_speed;
  double curvature;
  RobotState(const Pose3d& p,
             const Eigen::Vector3d v,
             double c) : pose(p), linear_speed(v), curvature(c) { }
};

typedef std::vector<RobotState> Path;

Path path;

bool ReadPath(Path* path, std::string filename) {
  std::fstream fins(filename, std::fstream::in);
  if (fins.is_open()) {
    std::string line;
    while (getline(fins, line)) {
      std::stringstream ss(line);
      double x, y, z, pitch, yaw, linear_speed, curvature;
      ss >> x >> y >> z >> pitch >> yaw >> linear_speed >> curvature;
      Eigen::Vector3d position(x, y, z);
      RobotState state(Pose3d(position, pitch, yaw),
                       Eigen::Vector3d(linear_speed, 0, 0),
                       curvature);
      path->emplace_back(state);
    }
  } else {
    std::cerr << "Error: Can not open file " << filename << std::endl;
    return false;
  }

  fins.close();
  return true;
}


class Send_path_segment {
 public:
  Send_path_segment(ros::NodeHandle& n);
  ~Send_path_segment(){  };
  double find_distance(size_t x1, size_t x2, const Path& path);
  std::vector<size_t> find_closest_points(double x, 
                                          double y, 
                                          const Path& path);
  std::vector<size_t> find_closest_index(const std::vector<size_t>& des,
                                         const std::vector<size_t>& cur);
  void callback_estop(const std_msgs::Bool::ConstPtr& data);
  void callback_goal_index(const geometry_msgs::PoseStamped::ConstPtr& goal);
  void callback_send_path(const nav_msgs::Odometry::ConstPtr& data);

 private:
  ros::Publisher pub_test_;
  ros::Publisher pub_plan_;
  size_t closest_goal_index_;
  size_t current_location_index_;
  double pre_time_;
  double time_step_;
  bool get_goal_;
  bool e_stop_;
  std::vector<size_t> closest_goal_list_;
  std::vector<size_t> current_location_list_;
};


Send_path_segment::Send_path_segment(ros::NodeHandle& n) {
  pub_test_ = n.advertise<custom_msg::llpath>("pathtest", 1);
  pub_plan_ = n.advertise<custom_msg::llpath>("pathtest_tmp", 1);
  closest_goal_index_ = 1;
  get_goal_ = 0;
  current_location_index_ = 1;
  pre_time_ = 0.0;
  time_step_ = 1.0;
  e_stop_ = 0;
}

std::vector<size_t> Send_path_segment::find_closest_points(double x,
                                                           double y,
                                                           const Path& path) {
  std::vector<size_t> ret;
  const double kThreshold = 0.25;
  double distance0 = 0.0;
  double distance1 = 0.0;
  ret.emplace_back(1);
  distance0 = sqrt(pow((path[1].pose.x() - x), 2) +
                   pow((path[1].pose.y() - y), 2));
  for (size_t i = 2; i < path.size(); i++) {
    distance1 = sqrt(pow((path[i].pose.x() - x), 2) +
                     pow((path[i].pose.y() - y), 2));
    if (distance0 - distance1 > kThreshold) {
      ret.clear();
      distance0 = distance1;
      ret.emplace_back(i);
    } else if (fabs(distance0 - distance1) < kThreshold) {
      ret.emplace_back(i);
    }
  }
  return ret;
}

double Send_path_segment::find_distance(size_t x1,
                                        size_t x2,
                                        const Path& path) {
  double distance_find = 0.0;
  double dist;
  double yaw_diff;
  double r;
  for (int i = x1; i < (x2 - 1); i++) {
    if (path[i].pose.yaw() == path[i+1].pose.yaw()) {
      dist = sqrt(pow((path[i].pose.x() - path[i+1].pose.x()), 2) +
                  pow((path[i].pose.y() - path[i+1].pose.y()), 2) +
                  pow((path[i].pose.z() - path[i+1].pose.z()), 2));
      distance_find += dist;
      continue;
    } else if (0 == (path[i].curvature * path[i+1].curvature)) {
      r = 1.0 / (path[i].curvature + path[i+1].curvature);
    } else {
      r = 0.5 * (1.0 / path[i].curvature + 1.0 / path[i+1].curvature);
    }
    yaw_diff = fabs(path[i].pose.yaw() - path[i+1].pose.yaw());
    dist = r * yaw_diff;
    dist = sqrt(pow(dist, 2) + pow((path[i].pose.z() - path[i+1].pose.z()), 2));
    distance_find += dist;
  }
  return distance_find;
}


std::vector<size_t> Send_path_segment::find_closest_index(const std::vector<size_t>& des,
                                                          const std::vector<size_t>& cur) {
  double dis0 = 100000;
  double dis1 = 0.0;
  int des_index;
  int cur_index;
  if (des.size() > 0 && cur.size() > 0) {
    des_index = des[0];
    cur_index = cur[0];
    for (int i = 0; i < cur.size(); i++) {
      for (int j = 0; j < des.size(); j++) {
        if (des[j] < cur[i]) continue;
        dis1 = fabs(des[j] - cur[i]);
        if (dis1 < dis0) {
          dis0 = dis1;
          des_index = j;
          cur_index = i;
        }
      }
    }
  }
  return { des[des_index], cur[cur_index] };
}


void Send_path_segment::callback_estop(const std_msgs::Bool::ConstPtr& data) {
  e_stop_ = data->data;
  if (e_stop_)
    time_step_ = 0.01;
  else
    time_step_ = 1.0;
}


void Send_path_segment::callback_goal_index(const geometry_msgs::PoseStamped::ConstPtr& goal) {
  double goal_x;
  double goal_y;
  goal_x = goal->pose.position.x;
  goal_y = goal->pose.position.y;
  get_goal_ = 1;
  closest_goal_list_ = find_closest_points(goal_x, goal_y, path);
  //ROS_INFO("I get a goal :( %f,  %f)", goal_x, goal_y);
}


void Send_path_segment::callback_send_path(const nav_msgs::Odometry::ConstPtr& data) {
  size_t index_distance;
  double location_x;
  double location_y;
  double offset_x;
  double offset_y;
  std::vector<size_t> point_index;
  if ((data->header.stamp.sec - pre_time_ > time_step_) && 1 == get_goal_) {
    pre_time_ = data->header.stamp.sec;
    location_x = data->pose.pose.position.x;
    location_y = data->pose.pose.position.y;

    current_location_list_ = find_closest_points(location_x,
                                                 location_y,
                                                 path);
    point_index = find_closest_index(closest_goal_list_,
                                     current_location_list_);
    closest_goal_index_ = point_index[0];
    current_location_index_ = point_index[1];
    index_distance = closest_goal_index_ - current_location_index_;

    offset_x = location_x - path[current_location_index_].pose.x();
    offset_y = location_y - path[current_location_index_].pose.y();

    custom_msg::llpath path_msg;
    path_msg.id = 6;
    path_msg.Forward = 1;
    std::cout << index_distance;

    if (index_distance < 200 && index_distance >= 0) {
      int i;
      for (int index = current_location_index_; index < closest_goal_index_; i++) {
        i = index - current_location_index_;
        path_msg.points[i].x = path[i].pose.x() + offset_x;
        path_msg.points[i].y = path[i].pose.y() + offset_y;
        path_msg.points[i].theta = path[i].pose.yaw();
        path_msg.points[i].r = path[i].curvature;
        if (!e_stop_) {
          path_msg.points[i].velocity = path[i].linear_speed[0];
        } else {
          path_msg.points[i].velocity = 0;
        }
        path_msg.points[i].distance = find_distance(index,
                                                    closest_goal_index_,
                                                    path);
        path_msg.amount = index_distance;
      }
      pub_test_.publish(path_msg);
      pub_plan_.publish(path_msg);
      ROS_INFO("I have %f", path_msg.points[0].x);
    } else if (index_distance >= 200) {
      int i;
      for (int index = current_location_index_;
               index < (current_location_index_ + 200); i++) {
        i = index - current_location_index_;
        path_msg.points[i].x = path[i].pose.x() + offset_x;
        path_msg.points[i].y = path[i].pose.y() + offset_y;
        path_msg.points[i].theta = path[i].pose.yaw();
        path_msg.points[i].r = path[i].curvature;
        if (!e_stop_) {
          path_msg.points[i].velocity = path[i].linear_speed[0];
        } else {
          path_msg.points[i].velocity = 0;
        }
        path_msg.points[i].distance = find_distance(index,
                                                    current_location_index_ + 200,
                                                    path);
        path_msg.amount = 200;
      }
      pub_test_.publish(path_msg);
      pub_plan_.publish(path_msg);
      ROS_INFO("%f", path_msg.points[0].x);
    } else ROS_INFO("Please choose a forward goal point!");  
  }
  //ROS_INFO("I notice");
}




int main(int argc, char** argv) {
  if (ReadPath(&path, "./graphdata.txt")) {
    for (auto state : path) { };
  } else {
    std::cerr << "Error: ReadPath Error" << std::endl;
  }
  ros::init(argc, argv, "path_planning");
  ros::NodeHandle n;
  //ROS_INFO("%f", path[3].pose.x());
  //ros::Publisher deliver = n.advertise<custom_msg::llpath> ("pathtest", 1);
  //ros::Publisher gui_send = n.advertise<custom_msg::llpath> ("planned_path", 1);
  Send_path_segment sender = Send_path_segment(n);
  ros::Subscriber sub_goal = n.subscribe("goal_point", 1, &Send_path_segment::callback_goal_index, &sender);
  ros::Subscriber sub_position = n.subscribe("global_position", 1, &Send_path_segment::callback_send_path, &sender);
  ros::Subscriber sub_stop = n.subscribe("e_stop", 1, &Send_path_segment::callback_estop, &sender);
  ros::spin();

  return 0;
}
