//
// Created by zhzhu on 08.10.24.
//

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <vector>
#include <Eigen/Dense>
#include <queue>
#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <path_planning/Viewpoint.h>
#include <condition_variable>
#include <sensor_msgs/PointCloud.h>
#include <unordered_set>
#include <nav_msgs/Path.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/Trajectory.h>
#include <tf/transform_datatypes.h>
#include "frontier_evaluation/frontier_evaluation.h"
#include "utils/rrtstar_base.hpp"
#include "GTL/Mathematics/Distance/ND/DistSegmentSegment.h"
#include "GTL/Mathematics/Primitives/ND/Segment.h"

#ifndef OPTIMAL_PATH_H
#define OPTIMAL_PATH_H

class RRTStar : public RRTStarBase{
    public:

    double collision_thre_;

    RRTStar(ros::NodeHandle& nh, float goal_radius, float goal_sampling_prob, float jump_size, 
              float disk_size, float threshold_distance, 
              float limit_x_low, float limit_x_high, 
              float limit_y_low, float limit_y_high, 
              float limit_z_low, float limit_z_high, 
              float timeout)
        : RRTStarBase(nh, goal_radius, goal_sampling_prob, jump_size, disk_size, 
                      threshold_distance, limit_x_low, limit_x_high, 
                      limit_y_low, limit_y_high, limit_z_low, limit_z_high, timeout) {}
    

    // virtual 1
    double getCurrentTime() override;

    // virtual 2
    bool isObstacleFree(Point p) override;

    // virtual 3
    bool isEdgeObstacleFree(Point a, Point b) override;

    // virtual 4
    Eigen::Vector3d getCurPos() override;

    bool LineIntersectWithSegment(const Eigen::Vector3d& A, const Eigen::Vector3d& B,
                                                 const Eigen::Vector3d& C, const Eigen::Vector3d& D);

    static std::vector<std::pair<Point, double>> interpolatePath(
                                    const Eigen::Vector3d& cur_pos,
                                    const std::vector<Point>& pathCoordinates,
                                    double original_yaw,
                                    double goal_yaw,
                                    double step_size);

    static Eigen::Vector2d getCentralAxis(const Eigen::Vector3d& drone_pos);
};

// *****************************************************************
// *****************************************************************
class Path_planner
{
public:

enum class PointType{
        START,
        END
  };

struct Point3D {
        int seg_id;
        Eigen::Vector3d pos;
        Eigen::Vector3d dir;
        PointType type;
    };

  struct Segment3D{
    Point3D start;
    Point3D end;
  };

struct drone_pose {
        Eigen::Vector3d pos;
        double yaw;
    };

    Path_planner(ros::NodeHandle& nh, const RRTStar& rrtstar);

    ~Path_planner();
    
    void shutdownCallback(const std_msgs::Bool::ConstPtr& msg);

    void backOrigin();

    void segmentsCallback(const path_planning::Segment::ConstPtr& msg);

    void ViewpointOutCallback(const path_planning::Viewpoint::ConstPtr& msg);

    void PathPlanningThread();
    
    void SetGoalPoint(const Point& point, float yaw);
    
    int RRTStar_planning(const FrontierEvaluator::viewpoint& goal_point,
                          const Path_planner::drone_pose& cur_pose);

    
    void setParam(ros::NodeHandle& nh);

    static std::shared_timed_mutex segment_mtx_;
    static std::vector<Segment3D> segments_; 

private:

    double minGainThreshold_;
    double horizon_radius_;
    // drone_pose cur_drone_pose_;
    double position_thre_;
    double angle_thre_;
    double interpolate_size_;

    // service
    ros::ServiceClient arming_client_;

    // subscriber
    ros::Subscriber frontier_out_sub_;
    ros::Subscriber cur_drone_sub_;
    ros::Subscriber segment_sub_;
    ros::Subscriber terminate_sub_;

    // Publisher
    ros::Publisher goal_point_vis_pub_;
    ros::Publisher target_pub_;
    ros::Publisher RRT_path_pub_;
    
    // thread 
    std::thread pathPlanning_thread;

    //mutex
    std::shared_timed_mutex queue_mtx_;
    std::shared_timed_mutex drone_mtx_;


    // RRT star planner 
    RRTStar rrtstar_;

    // exploration over 
    bool exploration_over_;

    // local horizion model 
    int model_;

    // planning duration 
    double planning_duration_;
    // total_duration 
    double total_duration_;
    double start_time_;
    double time_limitation_;

};

// priority queue compare
struct CompareViewpoint {
    bool operator()(const FrontierEvaluator::viewpoint& a, const FrontierEvaluator::viewpoint& b) {
        return a.info_gain < b.info_gain; 
    }
};

#endif //OPTIMAL_PATH_H
