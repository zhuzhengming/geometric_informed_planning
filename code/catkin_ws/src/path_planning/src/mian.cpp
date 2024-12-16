//
// Created by zhzhu on 08.10.24.
//

#include "mian.h"

int main(int argc, char** argv){
  ros::init(argc, argv, "mian");
  ros::NodeHandle n;

  FoV fov;
  fov.h = M_PI/2;
  fov.v = M_PI/2;
  fov.dist_range = 4.0;
  fov.drone_pos.pos = Eigen::Vector3d(0, 0, 0);
  fov.drone_pos.yaw = 1.0;

  FrontierEvaluator frontier_evaluator(fov, nh);

  ros::spin();

  return 0;

}
