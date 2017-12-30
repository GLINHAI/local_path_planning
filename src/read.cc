/*
 * Copyright 2017 Yu Kunlin <yukunlin@mail.ustc.edu.cn>
 */
#include <vector>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

#include <Eigen/Eigen>

using Eigen::Vector3d;

class Pose3d {
 public:
  Pose3d(const Vector3d& position, double roll, double pitch, double yaw) :
    position_(position), roll_(roll), pitch_(pitch), yaw_(yaw) { }
  std::string toString() {
    std::stringstream ss;
    ss << position_.x() << " " << position_.y() << " " << position_.z() << " ";
    ss << roll_ << " " << pitch_ << " " << yaw_;
    return ss.str();
  }

 private:
  Vector3d position_;
  double roll_;
  double pitch_;
  double yaw_;
};

struct RobotState {
  Pose3d pose;
  Vector3d linear_speed;
  double curvature;
  RobotState(const Pose3d& p,
             const Vector3d v,
             double c) : pose(p), linear_speed(v), curvature(c) { }
  std::string toString() {
    std::stringstream ss;
    ss << pose.toString() << " ";
    ss << linear_speed.x() << " "
       << linear_speed.y() << " "
       << linear_speed.z() << " "
       << curvature;
    return ss.str();
  }
};

typedef std::vector<RobotState> Path;

bool ReadPath(Path* path, std::string filename) {
  std::fstream fins(filename, std::fstream::in);
  if (fins.is_open()) {
    std::string line;
    while (std::getline(fins, line)) {
      std::stringstream ss(line);
      double x, y, z, roll, pitch, yaw, linear_speed, curvature;
      ss >> x >> y >> z >> roll >> pitch >> yaw >> linear_speed >> curvature;
      Vector3d position(x, y, z);
      RobotState state(Pose3d(position, roll, pitch, yaw),
                       Vector3d(linear_speed, 0, 0),
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


Path path;

int main(int argc, char ** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " \"input file name\"" << std::endl;
    return 1;
  }
  if (ReadPath(&path, argv[1])) {
    for (auto state : path) {
      std::cout << state.toString() << std::endl;
    }
  } else {
    std::cerr << "Error: ReadPath Error" << std::endl;
  }

  return 0;
}
