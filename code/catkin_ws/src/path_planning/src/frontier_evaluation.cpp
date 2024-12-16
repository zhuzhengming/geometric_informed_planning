/* This code mainly evaluate the frontiers of segments
Input: segments： a+tb
Output: The viewpoint set with information gain

 */
#include "std_msgs/String.h"
#include <string>
#include <vector>
#include "frontier_evaluation/frontier_evaluation.h"


// global variable
double Vector3dEqual::tolerance = 1.15;
std::unordered_set<Eigen::Vector3d, Vector3dHash, Vector3dEqual> EndPoints_set;

FrontierEvaluator::FrontierEvaluator(ros::NodeHandle& nh){

  // evaluate processing thread
  evaluate_thread_ = std::thread(&FrontierEvaluator::processingThread, this);

  // set parameter
  setParam(nh);

  // init drone pose
  cur_drone_pose_.pos = Eigen::Vector3d(0,0,0);
  cur_drone_pose_.yaw = 0.0;

  terminate_signal_.data = false;
 
 endpoint_sub_ = nh.subscribe("/seg_point", 10, &FrontierEvaluator::endpointsCallback, this);
 drone_pose_sub_ = nh.subscribe("/macortex_bridge/starling/pose",
                                10, &FrontierEvaluator::dronePosCallback, this);
//  drone_pose_sub_ = nh.subscribe("/mavros/local_position/drone_pose_with_correction",
//                                 10, &FrontierEvaluator::dronePosCallback, this);


 cur_frontier_pub_ = nh.advertise<geometry_msgs::Point>("/cur_frontier", 10, this);
 frontier_in_fov_pub_ = nh.advertise<sensor_msgs::PointCloud>("/frontier_in_fov", 10, this);
 viewpoint_out_pub_ = nh.advertise<path_planning::Viewpoint>("/viewpoint_out", 10, this);  
 terminate_pub_ = nh.advertise<std_msgs::Bool>("/terminate_signal", 10, this);      
 endpoints_pub_ = nh.advertise<sensor_msgs::PointCloud>("/endpoints", 10, this);                         
}

FrontierEvaluator::~FrontierEvaluator(){
  if (evaluate_thread_.joinable()) {
        evaluate_thread_.join();
    }
}

void FrontierEvaluator::setParam(ros::NodeHandle& nh){
  // get parameters
  nh.getParam("/lambda_1", lambda_[0]);
  nh.getParam("/lambda_2", lambda_[1]);
  nh.getParam("/lambda_3", lambda_[2]);
  nh.getParam("/lambda_4", lambda_[3]);
  nh.getParam("/frontier_threshold", frontier_threshold_);
  nh.getParam("/radius", radius_);
  nh.getParam("/sample_num", sample_num_);
  nh.getParam("/collision_thre", collision_thre_);
  nh.getParam("/fov/h", cur_fov_.h);
  nh.getParam("/fov/v", cur_fov_.v);
  nh.getParam("/fov/dist", cur_fov_.dist_range);
  nh.getParam("/fov/h", fake_fov_.h);
  nh.getParam("/fov/v", fake_fov_.v);
  nh.getParam("/fov/dist", fake_fov_.dist_range);
  nh.getParam("exploration_rate", exploration_rate_);
  

  ROS_INFO("Configuration:");
  ROS_INFO("  lambda: %f, %f, %f, %f", lambda_[0], lambda_[1], lambda_[2], lambda_[3]);
  ROS_INFO("  frontier_threshold: %f", frontier_threshold_);
  ROS_INFO("  radius: %f", radius_);
  ROS_INFO("  sample_num: %d", sample_num_);
  ROS_INFO("  collision_thre: %f", collision_thre_);
  ROS_INFO("  cur fov horizon scale: %f", cur_fov_.h);
  ROS_INFO("  cur fov vertical scale: %f", cur_fov_.v);
  ROS_INFO("  cur fov dist range: %f", cur_fov_.dist_range);
  ROS_INFO("  fake fov horizon scale: %f", fake_fov_.h);
  ROS_INFO("  fake fov vertical scale: %f", fake_fov_.v);
  ROS_INFO("  fake fov dist range: %f", fake_fov_.dist_range);
  ROS_INFO("  exploration_rate: %f", exploration_rate_);

}

// Drone's current position callback function
void FrontierEvaluator::dronePosCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  {
  std::unique_lock<std::shared_timed_mutex> drone_pos_lock(cur_drone_pos_mtx_); 
  cur_drone_pose_.pos = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  cur_drone_pose_.yaw = tf::getYaw(msg->pose.orientation);
  }

 cur_fov_.pose = cur_drone_pose_;

}

// Segments callback and processing function
void FrontierEvaluator::endpointsCallback(const path_planning::Segment::ConstPtr& msg) {

  size_t num_segments = msg->seg_id.size();
  std::vector<FrontierEvaluator::Point3D> frontiers;
  std::vector<FrontierEvaluator::Segment3D> segments;
  if (msg->starts.size() != num_segments || msg->ends.size() != num_segments) {
        ROS_ERROR("Mismatch in segment data size.");
        return;
    }

  // receive all possible frontiers 
  for (size_t i = 0; i < num_segments; ++i) {
   Eigen::Vector3d start_point(msg->starts[i].x, msg->starts[i].y, msg->starts[i].z);
   Eigen::Vector3d end_point(msg->ends[i].x, msg->ends[i].y, msg->ends[i].z);
   Eigen::Vector3d dir = (end_point-start_point).normalized();

   Point3D starts, ends;
   Segment3D segment;
   starts.pos = start_point;
   ends.pos = end_point;
   starts.dir = -dir;
   ends.dir = dir;
   starts.seg_id = msg->seg_id[i];
   ends.seg_id = msg->seg_id[i];
   starts.type = FrontierEvaluator::PointType::START;
   ends.type = FrontierEvaluator::PointType::END;
   segment.start = starts;
   segment.end = ends;

  frontiers.push_back(starts);
  frontiers.push_back(ends);
  segments.push_back(segment);
  }

  {
  std::unique_lock<std::shared_timed_mutex> frontier_lock(frontier_mtx_); 
  frontiers_ = std::move(frontiers);
  segments_ = std::move(segments);

  }
}

/* Thread*/
/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------*/
void FrontierEvaluator::processingThread(){
  ros::Rate rate(50);
  double iteration_start_time, iteration_duration;

  while (ros::ok())
  {
    iteration_start_time = ros::Time::now().toNSec();

    std::vector<Point3D> cur_frontiers_set;
    geometry_msgs::Point point_pub;
    geometry_msgs::Point32 single_point;
    sensor_msgs::PointCloud frontier_set_pub;
    sensor_msgs::PointCloud endpoint_set_pub;
    path_planning::Viewpoint Viewpoint_set;
    drone_pose cur_drone_pose;
    frontier_set_pub.header.frame_id = "/mocap";
    frontier_set_pub.header.stamp = ros::Time::now();
    endpoint_set_pub.header.frame_id = "/mocap";
    endpoint_set_pub.header.stamp = ros::Time::now();

    // copy frontiers
    {
    std::shared_lock<std::shared_timed_mutex> frontier_lock(frontier_mtx_);
    if (frontiers_.empty()) continue;
    cur_frontiers_set = frontiers_;
    // ROS_INFO("cur frontier size: %d", cur_frontiers_set.size());
    }

    // copy drone pos
    {
      std::shared_lock<std::shared_timed_mutex> cur_drone_pos_lock(cur_drone_pos_mtx_);
      cur_drone_pose = cur_drone_pose_;
    }

    for (auto& cur_frontier : cur_frontiers_set) {

    // Is it a endpoints?
    if (EndPoints_set.find(cur_frontier.pos) != EndPoints_set.end()) continue;

    // Is it in the current FoV?
    if (isPointInFoV(cur_frontier.pos, cur_fov_)) {
      EndPoints_set.insert(cur_frontier.pos);
      continue;
    }

    
    // publisher the select frontoer and points in the FoV
    point_pub.x = cur_frontier.pos.x();
    point_pub.y = cur_frontier.pos.y();
    point_pub.z = cur_frontier.pos.z();
    cur_frontier_pub_.publish(point_pub);
    

  // generate viewpoints around the frontier, return the most valuable viewpoint
  FrontierEvaluator::viewpoint viewpoint = viewpointSampling(cur_frontiers_set,
   cur_frontier.pos, radius_, sample_num_);
   fake_fov_.pose = viewpoint.pose;

    viewpoint.pose.pos.z() = std::max(viewpoint.pose.pos.z(), 0.1);

  // ------------------------------------------------------------------//
  // ------------------------------------------------------------------//
  // START evaluation
  // STEP 1: in FoV or not
  frontiers_in_fov_.clear();
  frontier_set_pub.points.clear();
    
    // get the all the points inside the viewpoint
    for(const auto& point : cur_frontiers_set) {
      if(isPointInFoV(point.pos, fake_fov_)) {
          frontiers_in_fov_.push_back(point);
          
          // for ROS pubulish
          single_point.x = point.pos.x();
          single_point.y = point.pos.y();
          single_point.z = point.pos.z();
          frontier_set_pub.points.push_back(single_point);
      }
    }

    // get the all the endpoint
    for(auto it = EndPoints_set.begin(); it != EndPoints_set.end(); ++it) {
          
          // for ROS pubulish
          single_point.x = it->x();
          single_point.y = it->y();
          single_point.z = it->z();
          endpoint_set_pub.points.push_back(single_point);
    }
    
    // public
    endpoints_pub_.publish(endpoint_set_pub);
    frontier_in_fov_pub_.publish(frontier_set_pub);
    

  // Visibility gain evaluation
  double visibility_gain = evaluateVisibilityGain(frontiers_in_fov_, fake_fov_);
  // distance punishment
  double dist_punishment = evaluateDistance(cur_frontier);
  // frontier density
  int density_gain = frontiers_in_fov_.size();
  // vertical weight
  double vertical_weight = verticalPenalty(viewpoint.pose.pos, cur_drone_pose.pos);

  // update the info_gian of the viewpoint 
  viewpoint.info_gain = (lambda_[0] * visibility_gain +
                             lambda_[1] * (double)density_gain)
                            * std::exp(-lambda_[2] * dist_punishment)
                            * std::exp(-lambda_[3] * vertical_weight);

  // publish the output, all the frontiers whose info_gian is larger than a threshold
  geometry_msgs::Pose Viewpoint_pose;
    if(viewpoint.info_gain > frontier_threshold_){
      Viewpoint_pose.position.x = viewpoint.pose.pos[0];
      Viewpoint_pose.position.y = viewpoint.pose.pos[1];
      Viewpoint_pose.position.z = viewpoint.pose.pos[2];

      tf::Quaternion q;
      q.setRPY(0.0, 0.0, viewpoint.pose.yaw);
      Viewpoint_pose.orientation.x = q.x();
      Viewpoint_pose.orientation.y = q.y();
      Viewpoint_pose.orientation.z = q.z();
      Viewpoint_pose.orientation.w = q.w();
      Viewpoint_set.pose.push_back(Viewpoint_pose);
      Viewpoint_set.info_gain.push_back(viewpoint.info_gain);
    }

    }

    
    double cur_exploration_rate = (double)(cur_frontiers_set.size() - Viewpoint_set.pose.size())
                                   / cur_frontiers_set.size();
    // ROS_INFO("exploration rate : %f", cur_exploration_rate);
    if( cur_exploration_rate >= exploration_rate_) {
      terminate_signal_.data = true;
      terminate_pub_.publish(terminate_signal_);
    }

    // publish when all the frontiers in the local horizon have been evaluated 
    viewpoint_out_pub_.publish(Viewpoint_set);

    // record iteration duration
    iteration_duration = (ros::Time::now().toNSec() - iteration_start_time) / 1000000;
    if(iteration_duration > max_eval_duration_){
      max_eval_duration_ = iteration_duration;
      ROS_INFO("max evaluation time-cost: %f ms", max_eval_duration_);
    }
      
    rate.sleep();
  }
}

FrontierEvaluator::viewpoint FrontierEvaluator::viewpointSampling(
                          const std::vector<Point3D>& cur_frontiers_set,
                          const Eigen::Vector3d& target_point,
                          const double& radius,
                          const int& sample_num){
    
    std::vector<Point3D> frontiers_in_fov;
    FrontierEvaluator::viewpoint cur_viewpoint;
    FrontierEvaluator::viewpoint best_view;
    double max_info_gian = 0.0;
    double angle_increment = 2 * M_PI / sample_num;


    for (int i = 0; i < sample_num; i++)
    {
      // calculate the pose of viewpoint 
       double theta = i * angle_increment;
       cur_viewpoint.pose.pos.x() = target_point.x() + radius * std::cos(theta);
       cur_viewpoint.pose.pos.y() = target_point.y() + radius * std::sin(theta);
       cur_viewpoint.pose.pos.z() = target_point.z();

       cur_viewpoint.pose.yaw = std::atan2(target_point.y() - cur_viewpoint.pose.pos.y(),
          target_point.x() - cur_viewpoint.pose.pos.x());

        // collision check 
        if(collisionCheck(cur_viewpoint.pose.pos, collision_thre_)) continue;

        fake_fov_.pose = cur_viewpoint.pose;

        // calculate the info_gain;
        frontiers_in_fov.clear();
        for (const auto& point : cur_frontiers_set)
        { 
          // skip if frontier is not in the FOV 
          if(!isPointInFoV(point.pos, fake_fov_)) continue;
          frontiers_in_fov.push_back(point);
        }

        cur_viewpoint.info_gain = evaluateVisibilityGain(frontiers_in_fov, fake_fov_);

        if(cur_viewpoint.info_gain > max_info_gian){
          best_view = cur_viewpoint;
          max_info_gian = cur_viewpoint.info_gain;
        }
    }

    return best_view;
    
  }

bool FrontierEvaluator::collisionCheck(const Eigen::Vector3d& point, double threshold){
     
     std::vector<Segment3D> cur_segment_set;

    {
    std::shared_lock<std::shared_timed_mutex> segment_lock(segment_mtx_);
    if (segments_.empty()) return false;
    cur_segment_set = segments_;
    }

    for(const auto& segment : cur_segment_set){
      Eigen::Vector3d closest_point;
      Eigen::Vector3d v = segment.end.pos - segment.start.pos;
      Eigen::Vector3d w = point - segment.start.pos;

      float t = (w.dot(v))/(v.dot(v));

      if (t <= 0)
      {
        closest_point = segment.start.pos;
      } else if (t >= 1){
        closest_point = segment.end.pos;
      }else {
        closest_point = segment.start.pos + t * v;
      }

      if((point - closest_point).norm() < threshold ){
          return true;
        }
    }

    return false;
}

// Visibility gian
double FrontierEvaluator::evaluateVisibilityGain(const std::vector<Point3D>& frontiers_in_fov, const FoV& fov) {
    Eigen::Vector3d boundary_point;
    double visibility_gain = 0.0;
    
    // For every frontier needed to be evaluated
   for(const auto& point : frontiers_in_fov) {
    // STEP 2: get the intersections between points and hyperplane
    boundary_point = IntersectionWithinFoV(point, fov);

    // STEP 3: sum of uncovered length, bounded to z = 0
      visibility_gain += (boundary_point - point.pos).norm();
}
      return visibility_gain;
}

// Is the point in the FoV
bool FrontierEvaluator::isPointInFoV(const Eigen::Vector3d& point, const FoV& fov){
    Eigen::Vector3d fov_dir(std::cos(fov.pose.yaw), std::sin(fov.pose.yaw), 0.0);
    Eigen::Vector3d dir_to_point = point - fov.pose.pos;
    double pro_distance = dir_to_point.dot(fov_dir);

      if (pro_distance > fov.dist_range) {
          return false;
      }
    
    // angle constranint
    Eigen::Vector3d horizontal_projection = dir_to_point;
    horizontal_projection.z() = 0;

    double horizontal_angle_to_point = std::acos(fov_dir.dot(horizontal_projection.normalized()));
    double vertical_angle_to_point = std::asin(dir_to_point.z() / dir_to_point.norm());

      return (horizontal_angle_to_point < (fov.h / 2.0) ) &&
          (std::abs(vertical_angle_to_point) < (fov.v / 2.0));
}

Eigen::Vector3d FrontierEvaluator::calculatePlaneNormal(const FoV& fov,
                                          const std::string& type, bool is_positive){
  double yaw = fov.pose.yaw;
  double h_angle = fov.h;
  double v_angle = fov.v;

    Eigen::Vector3d normal;

    if (type == "horizontal") {
        double direction_sign = is_positive ? 1.0 : -1.0;
        normal << direction_sign * std::sin(h_angle / 2), std::cos(h_angle / 2), 0;

        // rotation of yaw angle
        Eigen::Matrix3d R_z;
        R_z << std::cos(yaw), -std::sin(yaw), 0,
                std::sin(yaw),  std::cos(yaw), 0,
                0,          0,                 1;

        normal = R_z * normal;

    } else if (type == "vertical") {
        double direction_sign = is_positive ? 1.0 : -1.0;
        normal << 0, std::cos(v_angle / 2), direction_sign * std::sin(v_angle / 2);
    }

    return normal;
  }


// calculate the intersection between hyperplane and line
Eigen::Vector3d FrontierEvaluator::calculateIntersection(const Point3D& point,
                                      const Eigen::Vector3d& plane_normal, const Eigen::Vector3d& plane_point) {
    Eigen::Vector3d ray_origin = point.pos;
    Eigen::Vector3d ray_direction = point.dir;

    double numerator = plane_normal.dot(plane_point - ray_origin);
    double denominator = plane_normal.dot(ray_direction);

    // parallel
    if (std::abs(denominator) < 1e-5) {
        return Eigen::Vector3d::Zero();
    }

    double t = numerator / denominator;
    if (t > 0) {
        return ray_origin + t * ray_direction;
    }

    return Eigen::Vector3d::Zero();
}

// get the all intersecitons in the FoV
Eigen::Vector3d FrontierEvaluator::IntersectionWithinFoV(const Point3D& qk, const FoV& fov) {
      std::vector<Eigen::Vector3d> intersections;

     Eigen::Vector3d origin_point_fov = fov.pose.pos;

    // left horizon
    Eigen::Vector3d left_h_normal = calculatePlaneNormal(fov, "horizontal", true);
    Eigen::Vector3d left_intersection = calculateIntersection(qk, left_h_normal, origin_point_fov);
    if (left_intersection != Eigen::Vector3d::Zero()) {
        intersections.push_back(left_intersection);
    }

    // right horizon
    Eigen::Vector3d right_h_normal = calculatePlaneNormal(fov, "horizontal", false);
    Eigen::Vector3d right_intersection = calculateIntersection(qk, right_h_normal, origin_point_fov);
    if (right_intersection != Eigen::Vector3d::Zero()) {
        intersections.push_back(right_intersection);
    }

    // up vertical
    Eigen::Vector3d up_v_normal = calculatePlaneNormal(fov, "vertical", true);
    Eigen::Vector3d up_intersection = calculateIntersection(qk, up_v_normal, origin_point_fov);
    if (up_intersection != Eigen::Vector3d::Zero()) {
        intersections.push_back(up_intersection);
    }

    // down vertical
    Eigen::Vector3d down_v_normal = calculatePlaneNormal(fov, "vertical", false);
    Eigen::Vector3d down_intersection = calculateIntersection(qk, down_v_normal, origin_point_fov);
    if (down_intersection != Eigen::Vector3d::Zero()) {
        intersections.push_back(down_intersection);
    }

    // far plane
    Eigen::Vector3d fov_dir(std::cos(fov.pose.yaw), std::sin(fov.pose.yaw), 0.0);
    Eigen::Vector3d far_normal = fov_dir;  
    Eigen::Vector3d far_point = fov.pose.pos + fov.dist_range * fov_dir;  
    Eigen::Vector3d far_intersection = calculateIntersection(qk, far_normal, far_point);
    if (far_intersection != Eigen::Vector3d::Zero()) {
        intersections.push_back(far_intersection);
    }

    // ground plane 
    Eigen::Vector3d ground_normal(0,0,1);
    Eigen::Vector3d ground_point(0,0,0);
    Eigen::Vector3d ground_intersection = calculateIntersection(qk, ground_normal, ground_point);
    if (ground_intersection != Eigen::Vector3d::Zero()) {
        intersections.push_back(ground_intersection);
    }

    // return the closest point
    Eigen::Vector3d closest_intersection = Eigen::Vector3d::Zero();
    double min_distance = std::numeric_limits<double>::max();

    for(const auto& point : intersections){
      double dist = (point - qk.pos).norm();
      if(dist < min_distance){
        min_distance = dist;
        closest_intersection = point;
      }
    }

    return closest_intersection;

}

//distance punishiment
double FrontierEvaluator::evaluateDistance(const Point3D& qk) {
  return (cur_drone_pose_.pos - qk.pos).norm();
}

// Vertical Weighted function
double FrontierEvaluator::verticalPenalty(const Eigen::Vector3d& point1, const Eigen::Vector3d& point2){
  return std::abs(point1.z() - point2.z());
}

int main(int argc, char** argv){
  ros::init(argc, argv, "frontier_evaluation");
  ros::NodeHandle nh;

  FrontierEvaluator frontier_evaluator(nh);

  ros::spin();

  return 0;

}
