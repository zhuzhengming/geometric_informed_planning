//
// Created by zhzhu on 08.10.24.
//

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <vector>
#include <Eigen/Dense>
#include <thread>
#include <queue>
#include <mutex>
#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <path_planning/Viewpoint.h>
#include <sensor_msgs/PointCloud.h>
#include <unordered_set>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <tf/transform_datatypes.h>
#include "frontier_evaluation/frontier_evaluation.h"

#ifndef OPTIMAL_PATH_H
#define OPTIMAL_PATH_H
class Path_planner
{
public:
    Path_planner(ros::NodeHandle& nh);

    ~Path_planner();


    void FrontierOutCallback(const path_planning::Viewpoint::ConstPtr& msg);

    void PathPlanningThread();
    
    void SetGoalPoint(const geometry_msgs::Pose& goal_point);
private:
    // service
    ros::ServiceClient arming_client_;

    // subscriber
    ros::Subscriber frontier_out_sub_;

    // Publisher
    ros::Publisher goal_point_vis_pub_;
    ros::Publisher target_pub_;

    // thread 
    std::thread pathPlanning_thread;

    //mutex
    std::mutex queue_mtx_;

};

namespace std {
    template <>
    struct hash<Eigen::Vector3d> {
        std::size_t operator()(const Eigen::Vector3d& vec) const {
            
            std::size_t hx = std::hash<double>()(vec.x());
            std::size_t hy = std::hash<double>()(vec.y());
            std::size_t hz = std::hash<double>()(vec.z());

            
            return hx ^ (hy << 1) ^ (hz << 2);
        }
    };
}

// hash compare
bool operator==(const Eigen::Vector3d& lhs, const Eigen::Vector3d& rhs) {
    double tolerance = 0.1;
    if(abs(lhs.x() - rhs.x()) < tolerance &&
       abs(lhs.y() - rhs.y()) < tolerance &&
       abs(lhs.z() - rhs.z()) < tolerance){
        return true;
       }
    else return false;
}

// priority queue compare
struct CompareViewpoint {
    bool operator()(const FrontierEvaluator::viewpoint& a, const FrontierEvaluator::viewpoint& b) {
        return a.info_gain < b.info_gain; 
    }
};


#endif //OPTIMAL_PATH_H
