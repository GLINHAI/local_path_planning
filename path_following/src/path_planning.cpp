#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <Eigen/Eigen>


#include "ros/ros.h"      //how to import??
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Pose.h"
#include "custom_msg/llpath.h"   


using namespace std;

class Pose3d {
 public:
  Pose3d(const Eigen::Vector3d& position, double pitch, double yaw) :
    position_(position), pitch_(pitch), yaw_(yaw) { }
  /*string toString() {
    stringstream ss;
    ss << position_.x() << " " << position_.y() << " " << position_.z() << " ";
    ss << pitch_ << " " << yaw_;
    return ss.str();
  }*/
  double return_x() const { return position_[0]; }
  double return_y() const { return position_[1]; }
  double return_z() const { return position_[2]; }
  double return_pitch() const { return pitch_; }
  double return_yaw() const { return yaw_; }


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
  /*string toString() {
    stringstream ss;
    ss << pose.toString() << " ";
    ss << linear_speed.x() << " "
       << linear_speed.y() << " "
       << linear_speed.z() << " "
       << curvature;
    return ss.str();
  }*/
};

typedef vector<RobotState> Path;

bool ReadPath(Path* path, string filename) {
  fstream fins(filename, fstream::in);
  if (fins.is_open()) {
    string line;
    while (getline(fins, line)) {
      stringstream ss(line);
      double x, y, z, pitch, yaw, linear_speed, curvature;
      ss >> x >> y >> z >> pitch >> yaw >> linear_speed >> curvature;
      Eigen::Vector3d position(x, y, z);
      RobotState state(Pose3d(position, pitch, yaw),
                       Eigen::Vector3d(linear_speed, 0, 0),
                       curvature);
      path->push_back(state);
    }
  } else {
    cerr << "Error: Can not open file " << filename << endl;
    return false;
  }

  fins.close();
  return true;
}


class SendPathSegment
{
 public:
    SendPathSegment(ros::NodeHandle& n);
    ~SendPathSegment(){};
    vector<int> find_closest_points(double x, double y, const Path& path);
    double find_distance(int x1, int x2, const Path& path);
    vector<int> find_closest_index(const vector<int>& des, const vector<int>& cur);
    void callback_estop(const std_msgs::Bool::ConstPtr& data);
    void callback_goal_index(const geometry_msgs::PoseStamped::ConstPtr& goal);
    void callback_send_path(const nav_msgs::Odometry::ConstPtr& data);

 private:
    ros::Publisher _pub1;   //???
    ros::Publisher _pub2;
    int closest_goal_index;
    bool get_goal;
    int current_location_index;
    float pre_time;
    float time_step;
    bool e_stop;
    vector<int> closest_goal_list;
    vector<int> current_location_list;
};


SendPathSegment::SendPathSegment(ros::NodeHandle& n){
    _pub1 = n.advertise<custom_msg::llpath> ("pathtest", 1000); 
    _pub2 = n.advertise<custom_msg::llpath> ("pathtest_tmp", 1000); 
    closest_goal_index = 1;
    get_goal = 0;
    current_location_index = 1;
    pre_time = 0.0;
    time_step = 3.0;
    e_stop = 0;
    closest_goal_list = {};
    current_location_list = {};
}

vector<int> SendPathSegment::find_closest_points(double x, double y, const Path& path){
    vector<int> ret;
    float threshold = 0.25;
    double distance0 = 0.0;
    double distance1 = 0.0;
    ret.emplace_back(1);
    distance0 = sqrt(pow((path[1].pose.return_x() - x), 2)
                     + pow((path[1].pose.return_y() - y), 2));
    for (int i=2; i < path.size(); i++){
        distance1 = sqrt(pow((path[i].pose.return_x() - x), 2)
                         + pow((path[i].pose.return_y() - y), 2));
        if ((distance0 - distance1) > threshold){
            ret = {};
            distance0 = distance1;
            ret.emplace_back(i);
        }
        else if(abs(distance0 - distance1) < threshold){
            ret.emplace_back(i);
        }
    }
    return ret;
}

double SendPathSegment::find_distance(int x1, int x2, const Path& path){
    double distance_find;
    distance_find = sqrt(pow((path[x1].pose.return_x() - path[x2].pose.return_x()), 2)
                         + pow((path[x1].pose.return_y() - path[x2].pose.return_y()), 2));
    return distance_find;
}

vector<int> SendPathSegment::find_closest_index(const vector<int>& des, const vector<int>& cur){
    double dis0 = 100000;
    double dis1 = 0.0;
    int des_index;
    int cur_index;
    if (des.size() > 0 & cur.size() > 0){
        des_index = des[0];
        cur_index = cur[0];
        for (int i=0; i<cur.size(); i++){
            for (int j=0; j<des.size(); j++){
                if (des[j] < cur[i]) continue;
                dis1 = abs(des[j] - cur[i]);
                if (dis1 < dis0){
                    dis0 = dis1;
                    des_index = j;
                    cur_index = i;
                }
            }
        }
    }
    return {des[des_index],cur[cur_index]};
}

void SendPathSegment::callback_estop(const std_msgs::Bool::ConstPtr& data){
    e_stop = data.data;
    if (e_stop)
        time_step = 0.01;
    else
        time_step = 3.0;
}

//wait for change
void SendPathSegment::callback_goal_index(const geometry_msgs::PoseStamped::ConstPtr& goal){
    double goal_x;
    double goal_y;
    goal_x = goal.pose.position.x;
    goal_y = goal.pose.position.y;
    get_goal = 1;
    closest_goal_list = find_closest_points(goal_x, goal_y, path);
}

void SendPathSegment::callback_send_path(const nav_msgs::Odometry::ConstPtr& data){
    int index_distance;
    double location_x, location_y;
    double offset_x, offset_y;
    if ((data.header.stamp.secs > pre_time) && (1 == get_goal)){
        pre_time = data.header.stamp.secs;
        location_x = data.pose.pose.position.x;
        location_y = data.pose.pose.position.y;

        current_location_list = find_closest_points(location_x, location_y, path);
        closest_goal_index = find_closest_index(closest_goal_list, current_location_list)[0];
        current_location_index = find_closest_index(closest_goal_list, current_location_list)[1];
        index_distance = closest_goal_index - current_location_index;

        offset_x = location_x - path[current_location_index].pose.return_x();
        offset_y = location_y - path[current_location_index].pose.return_y();

        custom_msg::llpath path_msg;
        path_msg.id = 6;    //???
        path_msg.Forward = 1;   //????
        cout << index_distance;

        if (index_distance < 200 && index_distance >= 0 ){
            int i;
            for (int index = current_location_index; index < closest_goal_index; i++){
                i = index - current_location_index;
                path.msg.points[i].x = path[i].pose.return_x() + offset_x;
                path.msg.points[i].y = path[i].pose.return_y() + offset_y;
                path.msg.points[i].theta = path[i].pose.return_yaw();
                path.msg.points[i].r = path[i].curvature;
                if (!e_stop)
                    path.msg.points[i].velocity = path[i].linear_speed[0];
                else
                    path.msg.points[i].velocity = 0;
                path.msg.points[i].distance = find_distance(index, closest_goal_index, path);
                path.msg.amount = index_distance;
            }
            _pub1.publish(path_msg);
            _pub2.publish(path_msg);
        }
        else if(index_distance >= 200){
            int i;
            for (int index = current_location_index; index < (current_location_index + 200); i++){
                i = index - current_location_index;
                path.msg.points[i].x = path[i].pose.return_x() + offset_x;
                path.msg.points[i].y = path[i].pose.return_y() + offset_y;
                path.msg.points[i].theta = path[i].pose.return_yaw();
                path.msg.points[i].r = path[i].curvature;
                if (!e_stop)
                    path.msg.points[i].velocity = path[i].linear_speed[0];
                else
                    path.msg.points[i].velocity = 0;
                path.msg.points[i].distance = find_distance(index, (current_location_index + 200), path);
                path.msg.amount = 200;
            }
            _pub1.publish(path_msg);
            _pub2.publish(path_msg);
        }
        else ROS_INFO("Please choose a forward goal point!");//output a message to remind a goal point input..
    }
}




Path path;

int main(int argc, char** argv)
{
    if (ReadPath(&path, "test_input.txt")) {
        for (auto state : path) {};
    } else {
    cerr << "Error: ReadPath Error" << endl;
    }
    ros::init(argc, argv, "path_planning");
    ros::NodeHandle n;
    ros::Publisher deliver = n.advertise<custom_msg::llpath> ("pathtest", 1000);    //what if we have several publishers or subscribers/??
    ros::Publisher gui_send = n.advertise<custom_msg::llpath> ("planned_path", 1000);
    SendPathSegment sender = SendPathSegment(n);
    ros::Subscriber sub1 = n.subscribe("goal_point", 1000, sender.callback_goal_index);
    ros::Subscriber sub2 = n.subscribe("global_position", 1000, sender.callback_send_path);
    ros::Subscriber sub3 = n.subscribe("e_stop", 1000, sender.callback_estop);
    ros::spin();
    return 0;
}
