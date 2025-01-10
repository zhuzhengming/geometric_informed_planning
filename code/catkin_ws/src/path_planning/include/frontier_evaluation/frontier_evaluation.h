//
// Created by zhzhu on 08.10.24.
//

#ifndef FRONTIER_EVALUATION_H
#define FRONTIER_EVALUATION_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <vector>
#include <Eigen/Dense>
#include <unordered_set>
#include <thread>
#include <shared_mutex>
#include <tf/tf.h>
#include <fstream>
#include <functional>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_datatypes.h>
#include <path_planning/Segment.h>
#include <path_planning/Viewpoint.h>
#include <sensor_msgs/PointCloud.h>

class FrontierEvaluator {
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

  struct FoV {
        double h;
        double v;
        double dist_range;
        drone_pose pose;
    };

  struct viewpoint{
    drone_pose pose;
    double info_gain;
  };


    /* ------------------------------------------------------------------------------------------------- */
    /* ------------------------------------------------------------------------------------------------- */
    /* ------------------------------------------------------------------------------------------------- */
    FrontierEvaluator(ros::NodeHandle& nh);

    ~FrontierEvaluator();

    void setParam(ros::NodeHandle& nh);

    // Drone's current position callback function
    void dronePosCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

    // Segments callback function
    void endpointsCallback(const path_planning::Segment::ConstPtr& msg);

    // viewpoint sampling 
    FrontierEvaluator::viewpoint viewpointSampling(const std::vector<Point3D>& cur_frontiers_set,
        const Eigen::Vector3d& target_point, const double& radius, const int& sample_num);

    bool collisionCheck(const Eigen::Vector3d& point, double threshold);

    // visibility gain evaluation
    double evaluateVisibilityGain(const std::vector<Point3D>& frontiers, const FoV& fov);

    // in FoV or not
    bool isPointInFoV(const Eigen::Vector3d& point, const FoV& fov);

    // calculate plane normal
    Eigen::Vector3d calculatePlaneNormal(const FoV& fov,
                                 const std::string& type, bool is_positive);

    // calculate the intersection between hyperplane and line
    Eigen::Vector3d calculateIntersection(const Point3D& point,
                        const Eigen::Vector3d& plane_normal, const Eigen::Vector3d& plane_point);

    // get the all intersecitons in the FoV
    Eigen::Vector3d IntersectionWithinFoV(const Point3D& qk, const FoV& fov);

    // distance punishiment
    double evaluateDistance(const Point3D& qk);

    double verticalPenalty(const Eigen::Vector3d& point1, const Eigen::Vector3d& point2);

/*----------------------------------------------------------------------------------------
Thread*/

    void processingThread();

private:

    float lambda_[4];
    float frontier_threshold_; 
    float radius_;
    int   sample_num_;
    double collision_thre_;
    double exploration_rate_;

    // ROS subscriber
    ros::Subscriber endpoint_sub_;
    ros::Subscriber drone_pose_sub_;

    // ROS publisher
    ros::Publisher cur_frontier_pub_;
    ros::Publisher frontier_in_fov_pub_;
    ros::Publisher viewpoint_out_pub_;
    ros::Publisher terminate_pub_;
    ros::Publisher endpoints_pub_;

    std_msgs::Bool terminate_signal_;
    std::vector<Segment3D> segments_; 
    std::vector<Point3D> frontiers_;
    std::vector<Point3D> frontiers_in_fov_;
    drone_pose cur_drone_pose_;
    FoV fake_fov_;
    FoV cur_fov_;

    // thread 
    std::thread evaluate_thread_;

    // mutex 
    std::shared_timed_mutex frontier_mtx_;
    std::shared_timed_mutex cur_drone_pos_mtx_;
    std::shared_timed_mutex segment_mtx_;

    // evaluation duration 
    double max_eval_duration_;

    std::vector<double> iteration_duration_set_;

    std::string output_path_;
    int save_file_;

};

struct Vector3dEqual {
    static double tolerance;
    bool operator()(const Eigen::Vector3d& lhs, const Eigen::Vector3d& rhs) const {
        return (lhs - rhs).norm() <= tolerance;
    }
};

struct Vector3dHash {
    std::size_t operator()(const Eigen::Vector3d& vec) const {
        double tolerance = Vector3dEqual::tolerance;
        auto quantize = [tolerance](double value) {
            return static_cast<int>(std::round(value / tolerance)); 
        };

        std::size_t h1 = std::hash<int>()(quantize(vec.x()));
        std::size_t h2 = std::hash<int>()(quantize(vec.y()));
        std::size_t h3 = std::hash<int>()(quantize(vec.z()));
        
        return h1 ^ (h2 << 1) ^ (h3 << 2);
    }
};


#endif