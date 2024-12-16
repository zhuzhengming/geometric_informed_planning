//
// Created by zhzhu on 08.10.24.
//

#ifndef FRONTIER_EVALUATION_H
#define FRONTIER_EVALUATION_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <vector>
#include <Eigen/Dense>

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

// struct definition
struct drone_pos {
    Eigen::Vector3d pos;
    double yaw;
};

// FoV
struct FoV {
    double h;
    double v;
    double dist_range;
    drone_pos drone_pos;
    };

class FrontierEvaluator {
public:
    // 构造函数，传入ROS节点句柄
    FrontierEvaluator(ros::NodeHandle& nh);

    //获取机器人位姿
    void dronePosCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

    // 处理感知模块分割出的三维端点
    void endpointsCallback(const sensor_msgs::PointCloud2::ConstPtr& endpoints_msg);

    // 判断点是否在FoV中
    bool isPointInFov(const Point3D& point) const;

    //计算法平面
    Eigen::Vector3d FrontierEvaluation::calculateHorizontalNormal(double angle, const FoV& fov,
                                                                  const std::string& type, bool is_positive);

    // 判断点是否在FoV中
    bool FrontierEvaluator::isPointInFov(const Eigen::Vectors3d& point);

    // 计算信息增益的函数
    double evaluateInformationGain(double x, double y, double z);

    //评估距离惩罚
    double evaluateDistance(const drone_pos& cur_drone_pos, Point3D qk);

    //关键点的密度
    double evaluateDensity(const Point3D qk, const FoV Fov);


private:
    // 订阅感知模块分割后的端点
    ros::Subscriber endpoint_sub_;
    ros::Subscriber drone_pos_sub_;


    std::vector<Point3D> frontiers_;
    drone_pos cur_drone_pos;
    FoV fov_;

};

#endif