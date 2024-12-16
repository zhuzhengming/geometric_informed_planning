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
 * webots(NUE) -> base(NWU) -> home(NWU) -> drone(NWU) -> webots(NUE)
 */
// ROS 
#include "ros/ros.h"
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>

// Webots services 
#include <webots_ros/get_float.h>
#include <webots_ros/set_float.h>
#include <webots_ros/set_int.h>

// STL
#include <random>

#include <auto_pilot_pkg/MotorCommand.h> 
#include <auto_pilot_pkg/ComputeControl.h>

#define RATE 10 

// Basic time step in ms 
static int TIME_STEP = 8;

// Webots handles for the drone 
static uint64_t drone_node;

// Drone's name in ROS 
static std::string robotName;

static std::vector<ros::ServiceClient> velocity_clients;
static std::vector<ros::ServiceClient> position_clients;

static geometry_msgs::PoseStamped pose; 
static mavros_msgs::PositionTarget target; 

int set_rotor_speed(int rotor, double speed){
    
    webots_ros::set_float speed_srv; speed_srv.request.value = speed; 
    webots_ros::set_float position_srv; position_srv.request.value = INFINITY; 

    if(position_clients[rotor-1].call(position_srv) && position_srv.response.success &&
        velocity_clients[rotor-1].call(speed_srv) && speed_srv.response.success)
        return 0; 
    else 
        return -1; 
}

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    pose = *msg; 
}

void target_cb(const mavros_msgs::PositionTarget::ConstPtr& msg){
    target = *msg; 
}

/**
 * Apply received motor commands to the model 
 */
void motor_cmd_cb(const auto_pilot_pkg::MotorCommand::ConstPtr& msg){

    if(msg->motor_speeds.size() != 4){
        ROS_ERROR("Motor command has %ld propellers listed.",msg->motor_speeds.size());
    }else{
        set_rotor_speed(1, msg->motor_speeds.at(0));
        set_rotor_speed(2, msg->motor_speeds.at(1));
        set_rotor_speed(3, msg->motor_speeds.at(2));
        set_rotor_speed(4, msg->motor_speeds.at(3));
    }
}

int main(int argc, char *argv[]){

    ros::init(argc, argv, "drone_controller_node", ros::init_options::AnonymousName); 
    ros::NodeHandle nh;

    ros::WallRate rate(RATE);

    ROS_INFO("Starting %s", ros::this_node::getName().c_str());

BOOT:
    
    if(!ros::ok()) return EXIT_SUCCESS;

    sleep(1);

    while(ros::ok() && !nh.getParam(ros::this_node::getName()+"/robotName", robotName)){
        rate.sleep();
        ros::spinOnce(); 
    }
    if(robotName[0] != '/') robotName = std::string("/") + robotName;
    ROS_INFO_ONCE("[drone_controller] Received robotName: '%s'", robotName.c_str()); 

    bool use_service = false; 
    if(!nh.getParam(ros::this_node::getName()+"/use_service", use_service)){
        use_service = false; 
    }

    // ****************************** Webots ******************************

    // Get Webots basic time step
    ros::ServiceClient basicTimeStepClient = nh.serviceClient<webots_ros::get_float>(robotName + "/robot/get_basic_time_step"); 
    webots_ros::get_float basicTimeStepService; basicTimeStepService.request.ask = 1;
    if(basicTimeStepClient.call(basicTimeStepService)){
        TIME_STEP = basicTimeStepService.response.value;
        ROS_INFO("[drone_controller] got basic timestep %d",TIME_STEP);
    }else{
        ROS_WARN_ONCE("[drone_controller] could not get basic timestep, retrying...");
        goto BOOT; 
    }

    // Webots timestep client 
    ros::ServiceClient timeStepClient;
    webots_ros::set_int timeStepSrv;
    timeStepClient = nh.serviceClient<webots_ros::set_int>(robotName + "/robot/time_step");
    timeStepSrv.request.value = TIME_STEP;

    // Rotor clients 
    std::vector<std::string> rotor_ids = {"1","2","3","4"};
    for(int i=0; i<4; i++){
        velocity_clients.push_back(nh.serviceClient<webots_ros::set_float>(robotName + "/rotor"+rotor_ids[i]+"/set_velocity"));
        position_clients.push_back(nh.serviceClient<webots_ros::set_float>(robotName + "/rotor"+rotor_ids[i]+"/set_position"));
    }
    set_rotor_speed(1, 0.0);
    set_rotor_speed(2, 0.0);
    set_rotor_speed(3, 0.0);
    set_rotor_speed(4, 0.0);

    // ****************************** Webots/ ******************************

    // Subscribe to motor command
    ros::Subscriber motor_cmd_sub = nh.subscribe<auto_pilot_pkg::MotorCommand>("/autopilot/motor_command",1,motor_cmd_cb);

    // Subscribe to data required to use the control service 
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose",1,pose_cb);
    ros::Subscriber target_sub = nh.subscribe<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local",1,target_cb);
    ros::ServiceClient control_client = nh.serviceClient<auto_pilot_pkg::ComputeControl>("/autopilot/compute_control");

    // Listen to motor commands directly 
    if(!use_service){
        ROS_INFO("[drone_controller] using Subscribers/Publishers");
        pose_sub.shutdown(); 
        target_sub.shutdown(); 
        control_client.shutdown(); 
    }
    // Use a service to get motor commands 
    else{
        ROS_INFO("[drone_controller] using ComputeControl service");
        motor_cmd_sub.shutdown(); 
    }

    // Initialize pose data 
    pose.header.stamp = ros::Time::now(); 
    pose.pose.position.x = 0.0; 
    pose.pose.position.y = 0.0; 
    pose.pose.position.z = 0.0; 
    pose.pose.orientation.w = 1.0; 
    pose.pose.orientation.x = 0.0; 
    pose.pose.orientation.y = 0.0; 
    pose.pose.orientation.z = 0.0; 

    // Main loop 
    while(ros::ok()){
        
        if (!timeStepClient.call(timeStepSrv) || !timeStepSrv.response.success){
            ROS_ERROR("[drone_controller] Failed to call service time_step for next step.");
            goto BOOT; 
        }

        ros::spinOnce(); 

        if(use_service){
            auto_pilot_pkg::ComputeControl control_srv; 
            control_srv.request.pose   = pose; 
            control_srv.request.target = target; 
            
            std::vector<double> u; 
            
            if(control_client.call(control_srv)){
                
                u = control_srv.response.control;
                
                set_rotor_speed(1, u[0]);
                set_rotor_speed(2, u[1]);
                set_rotor_speed(3, u[2]);
                set_rotor_speed(4, u[3]);
            }
        }
    }

    ROS_WARN("[drone_controller] shutting down");
    ros::shutdown();
    return 0; 
}
