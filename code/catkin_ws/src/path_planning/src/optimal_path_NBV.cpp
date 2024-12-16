//
// Created by zhzhu on 08.10.24.
// Input: The viewpoint set with information gain
// Output: The goal position and pose
//
#include "optimal_path/optimal_path_NBV.h"

// global variable
std::priority_queue<FrontierEvaluator::viewpoint,
                    std::vector<FrontierEvaluator::viewpoint>,
                    CompareViewpoint> Viewpoint_Local_Queue;

std::priority_queue<FrontierEvaluator::viewpoint,
                    std::vector<FrontierEvaluator::viewpoint>,
                    CompareViewpoint> Viewpoint_Global_Queue;

std::shared_timed_mutex Path_planner::segment_mtx_;
std::vector<Path_planner::Segment3D> Path_planner::segments_;
std::shared_timed_mutex drone_mtx_;
Path_planner::drone_pose cur_drone_pose_;
ros::Publisher central_pub_;


Path_planner::Path_planner(ros::NodeHandle& nh, const RRTStar& rrtstar):
     rrtstar_(rrtstar){
    // subscriber 
    frontier_out_sub_ = nh.subscribe("/viewpoint_out", 10, &Path_planner::ViewpointOutCallback, this);
    // cur_drone_sub_ = nh.subscribe("/mavros/local_position/drone_pose_with_correction", 10, &Path_planner::dronePosCallback, this);

    segment_sub_ = nh.subscribe("/seg_point", 10, &Path_planner::segmentsCallback, this);
    terminate_sub_ = nh.subscribe("/terminate_signal", 10, &Path_planner::shutdownCallback, this);
    
    // service 
    arming_client_ = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");

    // publisher
    goal_point_vis_pub_ = nh.advertise<geometry_msgs::Point>("/goal_point_vis", 10, this);
    target_pub_ = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10, this);
    RRT_path_pub_ = nh.advertise<nav_msgs::Path>("/RRT_path", 10, this);

    // pathplanning thread
    pathPlanning_thread = std::thread(&Path_planner::PathPlanningThread, this);

    setParam(nh);

    // Arm the drone 
    mavros_msgs::CommandBool arm_srv;
    arm_srv.request.value = true;
    exploration_over_ = false;

    ros::service::waitForService("/mavros/cmd/arming"); 
    if (arming_client_.call(arm_srv)) {
        if (arm_srv.response.success) {
            ROS_INFO("Vehicle armed successfully.");
            // start record
            start_time_ = ros::Time::now().toSec();
        } else {
            ROS_WARN("Failed to arm the vehicle.");
        }
    } else {
        ROS_ERROR("Failed to call service /mavros/cmd/arming");
    }

}

Path_planner::~Path_planner(){
    if (pathPlanning_thread.joinable()) {
        pathPlanning_thread.join();
    }
}
void Path_planner::shutdownCallback(const std_msgs::Bool::ConstPtr& msg){
    bool sent = false;

    if(msg->data){
        if(segments_.size() > 50 || exploration_over_) 
        {   
            backOrigin();
            if(!sent)
            {
                ROS_INFO("Exploration is over!!");
                sent = true;
            }
            exploration_over_ = true;
        }
    }
}

void Path_planner::backOrigin(){
    FrontierEvaluator::viewpoint original;
    original.pose.pos = Eigen::Vector3d(0.5,2.5,0);
    original.pose.yaw = 0.0;
    int status = RRTStar_planning(original, cur_drone_pose_);
}

void Path_planner::setParam(ros::NodeHandle& nh){
  // get parameters
  nh.getParam("/minGainThreshold", minGainThreshold_);
  nh.getParam("/local_horizon_scale", horizon_radius_);
  nh.getParam("/position_thre", position_thre_);
  nh.getParam("/angle_thre", angle_thre_);
  nh.getParam("/interpolate_size", interpolate_size_);
  nh.getParam("/model", model_);
  nh.getParam("/time_limitation", time_limitation_);

  ROS_INFO("Configuration:");
  ROS_INFO("  minGainThreshold: %f", minGainThreshold_);
  ROS_INFO("  local_horizon_scale: %f", horizon_radius_);
  ROS_INFO("  position_thre: %f", position_thre_);
  ROS_INFO("  angle_thre: %f", angle_thre_);
  ROS_INFO("  interpolate_size: %f", interpolate_size_);
  ROS_INFO("  model: %d", model_);
  ROS_INFO("  time_limitation: %f", time_limitation_);

}

void Path_planner::segmentsCallback(const path_planning::Segment::ConstPtr& msg) {

  size_t num_segments = msg->seg_id.size();
  std::vector<Path_planner::Segment3D> segments;
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
   starts.type = Path_planner::PointType::START;
   ends.type = Path_planner::PointType::END;
   segment.start = starts;
   segment.end = ends;

  segments.push_back(segment);
  }

  {
  std::unique_lock<std::shared_timed_mutex> frontier_lock(segment_mtx_); 
  segments_ = std::move(segments);

  }
}


void dronePosCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
 std::unique_lock<std::shared_timed_mutex> lock(drone_mtx_); 

 cur_drone_pose_.pos = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
 cur_drone_pose_.yaw = tf::getYaw(msg->pose.orientation);

}

void Path_planner::ViewpointOutCallback(const path_planning::Viewpoint::ConstPtr& msg){
    geometry_msgs::Pose goal_point;
    
    std::priority_queue<FrontierEvaluator::viewpoint,
                    std::vector<FrontierEvaluator::viewpoint>,
                    CompareViewpoint>  local_queue;

    std::priority_queue<FrontierEvaluator::viewpoint,
                    std::vector<FrontierEvaluator::viewpoint>,
                    CompareViewpoint>  global_queue;

    drone_pose cur_drone_pose;
    {
        std::shared_lock<std::shared_timed_mutex> drone_pos_lock(drone_mtx_);
        cur_drone_pose = cur_drone_pose_;
    }

    for (size_t i = 0; i < msg->pose.size(); i++)
    {
        Eigen::Vector3d pos(msg->pose[i].position.x,
         msg->pose[i].position.y,
         msg->pose[i].position.z);
        double yaw = tf::getYaw(msg->pose[i].orientation);

        // check the information gain is large enough 
        if (msg->info_gain[i] > minGainThreshold_)
        {
            // add new viewpoint
            FrontierEvaluator::viewpoint cur_viewpoint;
            cur_viewpoint.pose.pos = pos;
            cur_viewpoint.pose.yaw = yaw;
            cur_viewpoint.info_gain = msg->info_gain[i];

            if(model_ == 0){
                // solution 1: sphere 
                // push into local queue
                if((cur_viewpoint.pose.pos - cur_drone_pose.pos).norm() <= horizon_radius_){
                    local_queue.push(cur_viewpoint);
                }
            }else{

                // solution 2: horizontal band
                // push into local queue 
                if(std::abs(cur_viewpoint.pose.pos.z() - cur_drone_pose.pos.z()) <= horizon_radius_){
                    local_queue.push(cur_viewpoint);
                }
            }
            
            // push into global queue
            global_queue.push(cur_viewpoint);
        }
    }

    {
        std::unique_lock<std::shared_timed_mutex> lock(queue_mtx_);
        while (!Viewpoint_Local_Queue.empty()) Viewpoint_Local_Queue.pop();
        while (!Viewpoint_Global_Queue.empty()) Viewpoint_Global_Queue.pop();

        Viewpoint_Local_Queue = std::move(local_queue);
        Viewpoint_Global_Queue = std::move(global_queue);
        
    }


}

void Path_planner::PathPlanningThread(){
    ros::Rate rate(50);
    bool ready = false;
    double iteration_start_time, iteration_duration;

    FrontierEvaluator::viewpoint goal_viewpoint;
    drone_pose cur_drone_pose;
    int status;

    while (ros::ok())
    {
        iteration_start_time = ros::Time::now().toNSec();
        total_duration_ = ros::Time::now().toSec() - start_time_;
        if(total_duration_ >= time_limitation_) {
            backOrigin();
            ROS_INFO("Exploration timeout!");
            rate.sleep();
            continue;
        }

    {
        std::shared_lock<std::shared_timed_mutex> drone_pos_lock(drone_mtx_);
        cur_drone_pose = cur_drone_pose_;
    }

        if(!ready)
        {
        std::shared_lock<std::shared_timed_mutex> lock(queue_mtx_);
            if (!Viewpoint_Local_Queue.empty())
            {
                goal_viewpoint = Viewpoint_Local_Queue.top();
                ready = true;
             
            }else if (!Viewpoint_Global_Queue.empty())
            {   
                // ROS_INFO("local queue is empty");
                goal_viewpoint = Viewpoint_Global_Queue.top();
                ready = true;
                
            }
            else {
                // ROS_INFO("No goal point");
                rate.sleep();
                continue;
                }
        }
            // visualization 
            geometry_msgs::Pose cur_viewpoint;
            cur_viewpoint.position.x = goal_viewpoint.pose.pos[0];
            cur_viewpoint.position.y = goal_viewpoint.pose.pos[1];
            cur_viewpoint.position.z = goal_viewpoint.pose.pos[2];
            goal_point_vis_pub_.publish(cur_viewpoint.position);

            if(!exploration_over_) status = RRTStar_planning(goal_viewpoint, cur_drone_pose);

            // Arrive the goal point
            if((cur_drone_pose.pos - goal_viewpoint.pose.pos).norm() <= position_thre_
            && abs(abs(cur_drone_pose.yaw) - abs(goal_viewpoint.pose.yaw)) <= angle_thre_) ready = false;

            // Can't arrive, replan
            if(status != RRTStar::SUCCESS) ready = false;

            iteration_duration = (ros::Time::now().toNSec() - iteration_start_time) / 1000000;
            if(iteration_duration > planning_duration_){
            planning_duration_ = iteration_duration;
            ROS_INFO("max planning time-cost: %f ms", planning_duration_);
            }

        rate.sleep();
    }
}

int Path_planner::RRTStar_planning(const FrontierEvaluator::viewpoint& goal_point,
                                    const Path_planner::drone_pose& cur_pose){

    Point start(cur_pose.pos[0],cur_pose.pos[1],cur_pose.pos[2]);
    std::vector<Point> stops = {
        Point(goal_point.pose.pos[0],goal_point.pose.pos[1],goal_point.pose.pos[2])
        };
    std::vector<Point> pathCoordinates;
    std::vector<std::pair<Point, double>> interpolate_Path;
    nav_msgs::Path path_msg;
    path_msg.header.frame_id = "mocap";
    path_msg.header.stamp = ros::Time::now();
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = path_msg.header;

    int status = rrtstar_.find_path(start, stops, pathCoordinates);

    // status
    if(status == RRTStar::SUCCESS){
        
        // visulize RRT* path 
        for (const auto& point : pathCoordinates) {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header = path_msg.header;
        pose_stamped.pose.position.x = point.x;
        pose_stamped.pose.position.y = point.y;
        pose_stamped.pose.position.z = point.z;

        pose_stamped.pose.orientation.w = 1.0;

        path_msg.poses.push_back(pose_stamped);
    }

        RRT_path_pub_.publish(path_msg);

        interpolate_Path = RRTStar::interpolatePath(cur_pose.pos, pathCoordinates, cur_pose.yaw, goal_point.pose.yaw, interpolate_size_);

        ros::Rate rate(1);
        
        const auto& point = interpolate_Path[1].first;
        float yaw = interpolate_Path[1].second; 
        SetGoalPoint(point, yaw);
        rate.sleep();

        RRT_path_pub_.publish(path_msg);
    }else if(status == RRTStar::TIMEOUT){
        ROS_WARN("RRT* timeout!");
    }else{
        // ROS_ERROR("RRT* planning fail");
    }

    return status;

}

void Path_planner::SetGoalPoint(const Point& point, float yaw){
    mavros_msgs::PositionTarget target;

    target.header.stamp = ros::Time::now();
    target.header.frame_id = "mocap"; 
    target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;

    // enforce position control 
    target.type_mask = mavros_msgs::PositionTarget::IGNORE_VX |
                                mavros_msgs::PositionTarget::IGNORE_VY |
                                mavros_msgs::PositionTarget::IGNORE_VZ |
                                mavros_msgs::PositionTarget::IGNORE_AFX |
                                mavros_msgs::PositionTarget::IGNORE_AFY |
                                mavros_msgs::PositionTarget::IGNORE_AFZ |
                                mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    
    // Set goal point 
    target.position.x = point.x;
    target.position.y = point.y;
    target.position.z = point.z;
                  
    // target.yaw = goal_point.pose.yaw;
    target.yaw = yaw;

    target_pub_.publish(target);

}

// *****************************************************************
// *****************************************************************
// RRT* class
// virtual 1
    double RRTStar::getCurrentTime(){
        return ros::Time::now().toSec();
    }

    // virtual 2
    bool RRTStar::isObstacleFree(Point p){
        Eigen::Vector3d point;
        point[0] = p.x;
        point[1] = p.y;
        point[2] = p.z;
        
        std::vector<Path_planner::Segment3D> cur_segment_set;

    {
    std::shared_lock<std::shared_timed_mutex> segment_lock(Path_planner::segment_mtx_);
    if (Path_planner::segments_.empty()) return true;
    cur_segment_set = Path_planner::segments_;
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

      if((point - closest_point).norm() < threshold_distance_ ){
          return false;
        }
    }

    return true;
    }

    // virtual 3
bool RRTStar::isEdgeObstacleFree(Point a, Point b){
        Eigen::Vector3d pointA, pointB;
        pointA[0] = a.x; pointB[0] = b.x;
        pointA[1] = a.y; pointB[1] = b.y;
        pointA[2] = a.z; pointB[2] = b.z;
        
        std::vector<Path_planner::Segment3D> cur_segment_set;

    {
    std::shared_lock<std::shared_timed_mutex> segment_lock(Path_planner::segment_mtx_);
    if (Path_planner::segments_.empty()) return true;
    cur_segment_set = Path_planner::segments_;
    }

    for(const auto& segment : cur_segment_set){
        // Does line intersect with every segment?
        if(LineIntersectWithSegment(
            pointA, pointB, segment.start.pos, segment.end.pos)) return false;
    }

     return true;
    }

bool RRTStar::LineIntersectWithSegment(const Eigen::Vector3d& P0, const Eigen::Vector3d& P1,
                                       const Eigen::Vector3d& Q0, const Eigen::Vector3d& Q1) {
    gtl::DCPQuery<double, gtl::Segment<double, 3>, gtl::Segment<double, 3>> query;
    
    gtl::Vector<double, 3> p0{P0[0], P0[1], P0[2]},
                           p1{P1[0], P1[1], P1[2]},
                           q0{Q0[0], Q0[1], Q0[2]},
                           q1{Q1[0], Q1[1], Q1[2]};
    gtl::Segment<double, 3> segment0{p0, p1}, segment1{q0, q1};


    auto result =query.ComputeRobust(segment0, segment1);
    return result.distance <= threshold_distance_;

}

Eigen::Vector2d RRTStar::getCentralAxis(const Eigen::Vector3d& cur_pos){
    std::vector<Path_planner::Segment3D> cur_segment_set;
    Eigen::Vector2d weighted_sum(0.0, 0.0);
    double sum_weights = 0.0;
    double k = 1.0;

    {
        std::shared_lock<std::shared_timed_mutex> segment_lock(Path_planner::segment_mtx_);
        if (Path_planner::segments_.empty()) return Eigen::Vector2d::Zero();
        cur_segment_set = Path_planner::segments_;
    }

    std::vector<Eigen::Vector3d> point_set;

    for(const auto& segment : cur_segment_set){
        point_set.push_back(segment.start.pos);
        point_set.push_back(segment.end.pos);
    }

    for (const auto& point : point_set)
    {
        double z_dist = std::abs(point.z() - cur_pos.z());
        double weight = std::exp(-k * z_dist);

        weighted_sum += weight * point.head<2>();
        sum_weights += weight;
    }

    if (sum_weights == 0.0) {
        return Eigen::Vector2d::Zero();
    }

    Eigen::Vector2d central_axis = weighted_sum / sum_weights;

    geometry_msgs::Point central_point;
    central_point.x = central_axis[0];
    central_point.y = central_axis[1];
    central_point.z = cur_pos.z();
    central_pub_.publish(central_point);

    return central_axis;

}

std::vector<std::pair<Point, double>> RRTStar::interpolatePath(
                                    const Eigen::Vector3d& cur_pos,
                                    const std::vector<Point>& pathCoordinates,
                                    double original_yaw,
                                    double goal_yaw,
                                    double step_size) {
    std::vector<std::pair<Point, double>> newPath;
    Eigen::Vector2d centralAxis = getCentralAxis(cur_pos);

    newPath.push_back({pathCoordinates.front(), original_yaw});
    int path_size = pathCoordinates.size();

    for (size_t i = 1; i < path_size; ++i) {
        const Point& p1 = pathCoordinates[i - 1];
        const Point& p2 = pathCoordinates[i];

        double segmentLength = p1.distance(p2);
        if(path_size == 2) step_size *= 1.5; 
        int numSteps = static_cast<int>(std::floor(segmentLength / step_size)); 

        for (int j = 1; j <= numSteps-1; ++j) {
            double t = static_cast<double>(j) / numSteps;
            Point interpolatedPoint = p1 + t * (p2 - p1);

            double interpolatedYaw = std::atan2(centralAxis.y() - interpolatedPoint.y,
            centralAxis.x() - interpolatedPoint.x);
            newPath.push_back({interpolatedPoint, interpolatedYaw});
        }
    }

    newPath.push_back({pathCoordinates.back(), goal_yaw});
    newPath.push_back({pathCoordinates.back(), goal_yaw});
    return newPath;
}

// virtual 4
Eigen::Vector3d RRTStar::getCurPos(){
    Path_planner::drone_pose cur_drone_pose;
    {
        std::shared_lock<std::shared_timed_mutex> drone_pos_lock(drone_mtx_);
        cur_drone_pose = cur_drone_pose_;
    }
    return cur_drone_pose.pos;
}


int main(int argc, char** argv){
  ros::init(argc, argv, "path_planner");
  ros::NodeHandle nh;

    float goal_radius;
    float goal_sampling_prob;
    float jump_size;
    float disk_size;
    float threshold_distance;
    float limit_x_low, limit_x_high;
    float limit_y_low, limit_y_high;
    float limit_z_low, limit_z_high;
    float timeout; 

  // get parameters for RRTStar
  nh.getParam("/goal_radius", goal_radius);
  nh.getParam("/goal_sampling_prob", goal_sampling_prob);
  nh.getParam("/jump_size", jump_size);
  nh.getParam("/disk_size", disk_size);
  nh.getParam("/threshold_distance", threshold_distance);
  nh.getParam("/limit_x_low", limit_x_low);
  nh.getParam("/limit_x_high", limit_x_high);
  nh.getParam("/limit_y_low", limit_y_low);
  nh.getParam("/limit_y_high", limit_y_high);
  nh.getParam("/limit_z_low", limit_z_low);
  nh.getParam("/limit_z_high", limit_z_high);
  nh.getParam("/timeout", timeout);

  RRTStar rrtstar(nh, goal_radius, goal_sampling_prob, jump_size, disk_size, 
                      threshold_distance, limit_x_low, limit_x_high, 
                      limit_y_low, limit_y_high, limit_z_low, limit_z_high, 
                      timeout);

  Path_planner path_planner(nh, rrtstar);

  ros::Subscriber cur_drone_sub_ = nh.subscribe(
                        "/macortex_bridge/starling/pose",
                         10, &dronePosCallback);
//   ros::Subscriber cur_drone_sub_ = nh.subscribe(
//                         "/mavros/local_position/drone_pose_with_correction",
//                          10, &dronePosCallback);
  central_pub_ = nh.advertise<geometry_msgs::Point>("/central_axis", 10);

  ros::spin();

  return 0;

}