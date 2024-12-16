//
// Created by zhzhu on 08.10.24.
//
#include "optimal_path/optimal_path.h"

// parameter 
double minGainThreshold = 2.0;

// global variable
std::unordered_set<Eigen::Vector3d> visited_points;
std::priority_queue<FrontierEvaluator::viewpoint,
 std::vector<FrontierEvaluator::viewpoint>, CompareViewpoint> Viewpoint_Queue;

Path_planner::Path_planner(ros::NodeHandle& nh){
    // subscriber 
    frontier_out_sub_ = nh.subscribe("/viewpoint_out", 10, &Path_planner::FrontierOutCallback, this);

    // service 
    arming_client_ = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");

    // publisher
    goal_point_vis_pub_ = nh.advertise<geometry_msgs::Point>("/goal_point_vis", 10, this);
    target_pub_ = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10, this);

    // pathplanning thread
    pathPlanning_thread = std::thread(&Path_planner::PathPlanningThread, this);

    // Arm the drone 
    mavros_msgs::CommandBool arm_srv;
    arm_srv.request.value = true;

    ros::service::waitForService("/mavros/cmd/arming"); 
    if (arming_client_.call(arm_srv)) {
        if (arm_srv.response.success) {
            ROS_INFO("Vehicle armed successfully.");
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

void Path_planner::FrontierOutCallback(const path_planning::Viewpoint::ConstPtr& msg){
    geometry_msgs::Pose goal_point;
    
    std::unique_lock<std::mutex> lock(queue_mtx_);
    for (size_t i = 0; i < msg->pose.size(); i++)
    {
        // check visited
        Eigen::Vector3d pos(msg->pose[i].position.x,
         msg->pose[i].position.y,
         msg->pose[i].position.z);
        double yaw = tf::getYaw(msg->pose[i].orientation);

        if(visited_points.find(pos)!= visited_points.end()) continue;

        // check the information gain is large enough 
        if (msg->info_gain[i] > minGainThreshold)
        {
            // add new viewpoint
            FrontierEvaluator::viewpoint cur_viewpoint;
            cur_viewpoint.pose.pos = pos;
            cur_viewpoint.pose.yaw = yaw;
            cur_viewpoint.info_gain = msg->info_gain[i];
            Viewpoint_Queue.push(cur_viewpoint);
            visited_points.insert(pos);
        }
    }

}

void Path_planner::PathPlanningThread(){
    ros::Rate rate(1);
    while (ros::ok())
    {
        std::unique_lock<std::mutex> lock(queue_mtx_);
        if(!Viewpoint_Queue.empty()){
            const auto& goal_viewpoint = Viewpoint_Queue.top();
            Viewpoint_Queue.pop();
            geometry_msgs::Pose cur_viewpoint;

            cur_viewpoint.position.x = goal_viewpoint.pose.pos[0];
            cur_viewpoint.position.y = goal_viewpoint.pose.pos[1];
            cur_viewpoint.position.z = goal_viewpoint.pose.pos[2];
            goal_point_vis_pub_.publish(cur_viewpoint.position);

            // send goal point to Mavros
            SetGoalPoint(cur_viewpoint);
            rate.sleep();
        }
        else{
            lock.unlock(); 
            rate.sleep();
        }

    }
}

void Path_planner::SetGoalPoint(const geometry_msgs::Pose& goal_point){
    mavros_msgs::PositionTarget target;

    target.header.stamp = ros::Time::now();
    target.header.frame_id = "map"; 
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
    target.position.x = goal_point.position.x;
    target.position.y = goal_point.position.y;
    target.position.z = goal_point.position.z;
    tf::Quaternion q(goal_point.orientation.x,
                     goal_point.orientation.y,
                     goal_point.orientation.z,
                     goal_point.orientation.w);
    q.normalize();                 
    target.yaw = 0.0;

    target_pub_.publish(target);

}



int main(int argc, char** argv){
  ros::init(argc, argv, "path_planner");
  ros::NodeHandle nh;

  Path_planner Path_planner(nh);

  ros::spin();

  return 0;

}