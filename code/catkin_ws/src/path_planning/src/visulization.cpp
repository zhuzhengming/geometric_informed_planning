#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point32.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include "frontier_evaluation/frontier_evaluation.h"

#include <string>

nav_msgs::Path robot_path_;
ros::Publisher path_pub_;
int model;

void dronePosCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    
    robot_path_.header.stamp = ros::Time::now();
    robot_path_.header.frame_id = "mocap";
    robot_path_.poses.push_back(*msg);

    path_pub_.publish(robot_path_);
}

visualization_msgs::Marker createCentralAxisMarker(const geometry_msgs::Point& point) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "mocap";  
    marker.header.stamp = ros::Time::now();
    marker.ns = "central_axis";  
    marker.id = 0;  
    marker.type = visualization_msgs::Marker::ARROW;  

    marker.action = visualization_msgs::Marker::ADD;

    
    marker.scale.x = 0.1; 
    marker.scale.y = 0.2;  
    marker.scale.z = 0.0; 

    
    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 1.0f;
    marker.color.a = 0.3f;  

    marker.pose.orientation.w = 1.0;  

    
    geometry_msgs::Point p1;
    p1.x = point.x;
    p1.y = point.y;
    p1.z = point.z;  

    geometry_msgs::Point p2;
    p2.x = point.x;
    p2.y = point.y;
    p2.z = point.z + 2.0;  

    marker.points.push_back(p1);
    marker.points.push_back(p2);

    return marker;
}

visualization_msgs::Marker createPointCloudMarker(const sensor_msgs::PointCloud& cloud,
                                                   const std::string& type = "SPHERE", 
                                                   int id = 0, 
                                                   const std::string& ns = "default" ) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "mocap";
    marker.header.stamp = ros::Time::now();
    marker.ns = ns;
    marker.id = id;  

    if (type == "SPHERE") {
        marker.type = visualization_msgs::Marker::SPHERE_LIST;
    } else if (type == "CUBE") {
        marker.type = visualization_msgs::Marker::CUBE_LIST;
    }else {
        throw std::invalid_argument("Invalid marker type specified.");
    }

    marker.action = visualization_msgs::Marker::ADD;

    // size
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    // blue green
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 0.2;

    marker.pose.orientation.w = 1.0;

    for (const auto& pt : cloud.points) {
        geometry_msgs::Point point;
        point.x = pt.x;
        point.y = pt.y;
        point.z = pt.z;
        marker.points.push_back(point);
    }

    return marker;
}

visualization_msgs::Marker createLocalHorizonMarker(const geometry_msgs::PoseStamped& cur_pose, const double& horizon_radius_) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "mocap";
    marker.header.stamp = ros::Time::now();
    marker.ns = "local_horizon";
    marker.id = 1;   
    marker.action = visualization_msgs::Marker::ADD;

    if(model == 0){
    // sphere
        marker.type = visualization_msgs::Marker::SPHERE; 
        marker.scale.x = horizon_radius_ * 2;
        marker.scale.y = horizon_radius_ * 2;
        marker.scale.z = horizon_radius_ * 2;
    }else{
        marker.type = visualization_msgs::Marker::CUBE;  
        marker.scale.x = 6;  
        marker.scale.y = 6;  
        marker.scale.z = horizon_radius_ * 2;     
    }


    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.05f;

    // pose
    marker.pose.position.x = cur_pose.pose.position.x;
    marker.pose.position.y = cur_pose.pose.position.y;
    marker.pose.position.z = cur_pose.pose.position.z;
    marker.pose.orientation.w = 1.0;

    return marker;
}

visualization_msgs::Marker createSinglePointMarker(const geometry_msgs::Point& point, 
                                                   const std::string& type = "SPHERE", 
                                                   int id = 0, 
                                                   const std::string& ns = "default") {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "mocap";
    marker.header.stamp = ros::Time::now();
    marker.ns = ns;
    marker.id = id;

    if (type == "SPHERE") {
        marker.type = visualization_msgs::Marker::SPHERE;
    } else if (type == "CUBE") {
        marker.type = visualization_msgs::Marker::CUBE;
    } else if (type == "CYLINDER") {
        marker.type = visualization_msgs::Marker::CYLINDER;
    } else if (type == "ARROW") {
        marker.type = visualization_msgs::Marker::ARROW;
    } else {
        throw std::invalid_argument("Invalid marker type specified.");
    }

    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position = point;
    marker.pose.orientation.w = 1.0;

    // size
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;

    // color: red
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    return marker;
}

void frontierInFoVCallback(const sensor_msgs::PointCloud::ConstPtr& msg, ros::Publisher marker_pub) {
    visualization_msgs::Marker marker = createPointCloudMarker(*msg, "SPHERE", 1, "frontier");
    marker_pub.publish(marker);
}

void endpointCallback(const sensor_msgs::PointCloud::ConstPtr& msg, ros::Publisher marker_pub) {
    visualization_msgs::Marker marker = createPointCloudMarker(*msg, "CUBE", 2, "endpoint");
    marker_pub.publish(marker);
}

void localHorizonCallback(const geometry_msgs::PoseStamped::ConstPtr& msg, ros::Publisher marker_pub, const double& horizon_radius){
    visualization_msgs::Marker marker = createLocalHorizonMarker(*msg, horizon_radius);
    marker_pub.publish(marker);
}

void curFrontierCallback(const geometry_msgs::Point::ConstPtr& msg, ros::Publisher marker_pub) {
    visualization_msgs::Marker marker = createSinglePointMarker(*msg, "SPHERE", 2, "cur_frontier");
    marker_pub.publish(marker);
}

void goalPointCallback(const geometry_msgs::Point::ConstPtr& msg, ros::Publisher marker_pub) {
    visualization_msgs::Marker marker = createSinglePointMarker(*msg, "CUBE", 3, "goal_point");
    marker_pub.publish(marker);
}

void centralAxisCallback(const geometry_msgs::Point::ConstPtr& msg, ros::Publisher marker_pub) {
    visualization_msgs::Marker marker = createCentralAxisMarker(*msg);
    marker_pub.publish(marker);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "visualization_node");
    ros::NodeHandle nh;
    double horizon_radius;

    nh.getParam("/local_horizon_scale", horizon_radius);
    nh.getParam("/model", model);

    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1000);
    
    path_pub_ = nh.advertise<nav_msgs::Path>("/robot_trajectory", 1000);

    ros::Subscriber cur_frontier_sub = nh.subscribe<geometry_msgs::Point>("/cur_frontier", 10, 
        boost::bind(curFrontierCallback, _1, marker_pub));
    ros::Subscriber central_point_sub = nh.subscribe<geometry_msgs::Point>("/central_axis", 10, 
        boost::bind(centralAxisCallback, _1, marker_pub));
    ros::Subscriber goal_point_sub = nh.subscribe<geometry_msgs::Point>("/goal_point_vis", 10, 
        boost::bind(goalPointCallback, _1, marker_pub));
    ros::Subscriber frontier_sub = nh.subscribe<sensor_msgs::PointCloud>("/frontier_in_fov", 10, 
        boost::bind(frontierInFoVCallback, _1, marker_pub));
    ros::Subscriber endpoint_sub = nh.subscribe<sensor_msgs::PointCloud>("/endpoints", 10, 
        boost::bind(endpointCallback, _1, marker_pub));   

         
    // ros::Subscriber local_horizon_sub = nh.subscribe<geometry_msgs::PoseStamped>(
    //     "/mavros/local_position/drone_pose_with_correction",
    //     10, boost::bind(localHorizonCallback, _1, marker_pub, horizon_radius));

    // ros::Subscriber cur_drone_sub = nh.subscribe<geometry_msgs::PoseStamped>(
    //     "/mavros/local_position/drone_pose_with_correction", 
    //     10, dronePosCallback);

    ros::Subscriber local_horizon_sub = nh.subscribe<geometry_msgs::PoseStamped>(
        "/macortex_bridge/starling/pose",
        10, boost::bind(localHorizonCallback, _1, marker_pub, horizon_radius));
    ros::Subscriber cur_drone_sub = nh.subscribe<geometry_msgs::PoseStamped>(
        "/macortex_bridge/starling/pose", 
        10, dronePosCallback);

    ros::spin(); 

    return 0;
}
