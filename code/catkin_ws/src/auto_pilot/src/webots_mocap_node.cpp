/**
 * Publish drone pose: 
 * - topic: "mavros/local_position/pose"
 * - type:  geometry_msgs::PoseStamped
 * 
 * Subscribe to motor command: 
 * - topic: "/autopilot/motor_velocities"
 * - type:  std_msgs::Float64MultiArray
 * 
 * Emulate the drone in simulation to interface the implemented packages. 
 * All actions are performed in ROS' convention (NWU). 
 * The data is converted to and from Webots only when directly interacting
 * with the simulation. 
 * 
 * The frames can be represented as follows: 
 * webots(NWU) -> home(NWU) -> drone(NWU) -> webots(NWU)
 */
// ROS 
#include "ros/ros.h"
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h> 
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/QuadWord.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <mavros_msgs/PositionTarget.h>

// Webots services 
#include <webots_ros/get_float.h>
#include <webots_ros/supervisor_get_from_def.h>
#include <webots_ros/node_get_position.h>
#include <webots_ros/node_get_orientation.h>
#include <webots_ros/set_int.h>

#define RATE 100

static ros::Publisher pose_pub; 

// Robot's pose
static geometry_msgs::Twist pose;

// Drone's name in ROS 
static std::string robotName, robotDef;

// Webots handles for the drone 
static uint64_t drone_node;

// Frame in which the pose is expressed 
static std::string frame_id = "map";

// ****************************** Utils ******************************
tf2::Vector3 quat_rot(tf2::Quaternion q, tf2::Vector3 v){
    v = tf2::quatRotate(q,v);
    return v; 
}
tf2::Quaternion quat_rot(tf2::Quaternion q1, tf2::Quaternion q2){
    q2 = q1*q2; 
    return q2; 
}

tf2::Vector3 translate(tf2::Vector3 t, tf2::Vector3 v){
    v += t; 
    return v; 
}

tf2::Vector3 get_RPY(tf2::Quaternion q){
    
    tf2::Matrix3x3 m; m.setRotation(q);

    tf2Scalar roll, pitch, yaw;
    
    m.getRPY(roll, pitch, yaw);

    tf2::Vector3 v; v.setValue(roll, pitch, yaw);

    return v; 
}

geometry_msgs::PoseStamped twist2poseStamped(geometry_msgs::Twist p){
    geometry_msgs::PoseStamped pose_stamped; 

    pose_stamped.header.frame_id = frame_id;
    pose_stamped.header.stamp = ros::Time::now(); 

    pose_stamped.pose.position.x = p.linear.x;
    pose_stamped.pose.position.y = p.linear.y;
    pose_stamped.pose.position.z = p.linear.z;

    tf2::Quaternion q; q.setRPY(p.angular.x,p.angular.y,p.angular.z);
    pose_stamped.pose.orientation.w = q.getW();
    pose_stamped.pose.orientation.x = q.getX();
    pose_stamped.pose.orientation.y = q.getY();
    pose_stamped.pose.orientation.z = q.getZ(); 

    return pose_stamped;  
}

// ******************************  Utils/ ******************************

/**
 * Use the supervisor to get the pose of the drone. 
 * Webots returns a pose in NWU frame, which can be used directly. 
 * 
 * @param nh NodeHandle reference
 * 
 * @return whether pose acquisition was successful 
 */
bool acquire_pose(ros::NodeHandle& nh){
    
    tf2::Vector3 origin;
    tf2::Quaternion rotation;

    //---------------------------------------------------------------------------------
    // Get position in NWU
    webots_ros::node_get_position srv_position; srv_position.request.node = drone_node; 
    ros::ServiceClient client = nh.serviceClient<webots_ros::node_get_position>(robotName + "/supervisor/node/get_position");
    if(!client.call(srv_position)){
        ROS_ERROR("[webots_mocap] Current position not acquired.");
        return false; 
    }
    origin.setValue(srv_position.response.position.x,
                    srv_position.response.position.y,
                    srv_position.response.position.z);

    //---------------------------------------------------------------------------------
    // Get rotation in NWU
    webots_ros::node_get_orientation srv_orientation; srv_orientation.request.node = drone_node; 
    client = nh.serviceClient<webots_ros::node_get_orientation>(robotName + "/supervisor/node/get_orientation");
    if(!client.call(srv_orientation)) {
        ROS_ERROR("[webots_mocap] Current orientation not acquired."); 
        return false; 
    }
    rotation.setW(srv_orientation.response.orientation.w);
    rotation.setX(srv_orientation.response.orientation.x);
    rotation.setY(srv_orientation.response.orientation.y);
    rotation.setZ(srv_orientation.response.orientation.z); 
    
    tf2::Vector3 rpy = get_RPY(rotation);
    rpy.setX(rpy.getX()); // roll
    rpy.setY(rpy.getY()); // pitch 
    rpy.setZ(rpy.getZ()); // Yaw 

    // Print pose and orientation 
    // ROS_INFO(" ");
    // ROS_INFO("tx:   %.2lf, ty:    %.2lf, tz:  %.2lf",get_RPY(rotation).getX(),get_RPY(rotation).getY(),get_RPY(rotation).getZ());
    // ROS_INFO("x:    %.2lf, y:     %.2lf, z:   %.2lf", origin.getX(),origin.getY(),origin.getZ());
    // ROS_INFO("roll: %.2lf, pitch: %.2lf, yaw: %.2lf", rpy.getX(),rpy.getY(),rpy.getZ());
    
    // Apply to pose 
    pose.linear.x  = origin.getX(); 
    pose.linear.y  = origin.getY(); 
    pose.linear.z  = origin.getZ();  
    pose.angular.x = rpy.getX(); 
    pose.angular.y = rpy.getY(); 
    pose.angular.z = rpy.getZ(); 

    return true; 
}

int main(int argc, char *argv[]){

    ros::init(argc, argv, "webots_mocap_node", ros::init_options::AnonymousName); 
    ros::NodeHandle nh;

    ros::Rate rate(RATE);

    ROS_INFO("Starting %s", ros::this_node::getName().c_str());

BOOT: 
    
    if(!ros::ok()) return EXIT_SUCCESS;

    sleep(1); 

    while(ros::ok() && (!nh.getParam(ros::this_node::getName()+"/robotName", robotName) 
                     || !nh.getParam(ros::this_node::getName()+"/robotDef", robotDef))){
        rate.sleep();
        ros::spinOnce(); 
    }
    if(robotName[0] != '/') robotName = std::string("/") + robotName;
    ROS_INFO_ONCE("[webots_mocap] Received robotName: '%s' and robotDef: '%s'", robotName.c_str(), robotDef.c_str()); 

    frame_id = nh.param<std::string>(ros::this_node::getName()+"/frame_id","map"); 

    // ****************************** Webots ****************************** 

    // Get the robot node
    ros::ServiceClient client; 
    client = nh.serviceClient<webots_ros::supervisor_get_from_def>(robotName + "/supervisor/get_from_def");
    webots_ros::supervisor_get_from_def srv_node; srv_node.request.name = robotDef;
    if(!client.call(srv_node)){
        ROS_WARN_ONCE("[webots_mocap] %s node not acquired, retrying...", robotDef.c_str());
        goto BOOT; 
    }
    drone_node = srv_node.response.node;
    ROS_INFO("[webots_mocap] %s node acquired", robotDef.c_str());

    // ****************************** Webots/ ******************************

    // Advertise pose 
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/local_position/pose", 1);

    geometry_msgs::PoseStamped pose_stamped;
    uint64_t prev_stamp = 0; 
    
    // Main loop 
    while(ros::ok()){
        
        if(acquire_pose(nh)){
            pose_stamped = twist2poseStamped(pose); 
            if(pose_stamped.header.stamp.toNSec() != prev_stamp){
                pose_pub.publish(pose_stamped);
            }
            prev_stamp = pose_stamped.header.stamp.toNSec();
        }
        
        ros::spinOnce();
        rate.sleep(); 
    }
    ROS_WARN("[webots_mocap] shutting down");
    ros::shutdown();
    return 0; 
}
