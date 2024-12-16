// energy consumption 
// exploration efficiency: all segments length 
// total exploration time

#include "frontier_evaluation/frontier_evaluation.h"
#include <ros/ros.h>
#include <matplotlibcpp.h>

// global variable
double energy_consumption = 0.0;
double dist_v = 0.0;
double dist_h = 0.0;
double start_time;
double time_limitation_;
float drag_h, drag_v, hover;
bool start = false;
Eigen::Vector3d last_pose;
std::vector<double> total_segments_lengths; 
std::vector<double> energy_consumption_set; 
std::vector<double> timestamps_seg;
std::vector<double> timestamps_neg;

namespace plt = matplotlibcpp;


void dronePosCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{   
    double duration = ros::Time::now().toSec() - start_time;
    if(!start) { 
     last_pose = Eigen::Vector3d(msg->pose.position.x,
                                msg->pose.position.y,
                                msg->pose.position.z);
    start = true;
    return;
    }

    Eigen::Vector3d cur_pose(msg->pose.position.x,
                             msg->pose.position.y,
                             msg->pose.position.z);
    
    dist_v += abs(cur_pose.z() - last_pose.z());
    dist_h += (cur_pose.head<2>() - last_pose.head<2>()).norm();

    last_pose = cur_pose;
    energy_consumption = hover * duration +
                            drag_h * dist_h + 
                            drag_v * dist_v;
    // ROS_INFO("energy_consumption:%f", energy_consumption);

    energy_consumption_set.push_back(energy_consumption);
    timestamps_neg.push_back(duration);

    plt::figure(1);
    plt::clf();
    plt::plot(timestamps_neg, energy_consumption_set); 
    plt::grid(true);
    plt::xlabel("Time (seconds)");  
    plt::ylabel("Energy consumption"); 
    plt::title("Energy consumption vs Time"); 
    // plt::xlim(0, (int)time_limitation_);  
    // plt::ylim(0, 1000);
    plt::pause(0.1);

    return;
}

void segmentsCallback(const path_planning::Segment::ConstPtr& msg) 
{
    double time_stamp;
    double total_segments_length = 0;
    int size = msg->starts.size();
    for(int i = 0; i < size; ++i){
        Eigen::Vector3d start_point(msg->starts[i].x, msg->starts[i].y, msg->starts[i].z);
        Eigen::Vector3d end_point(msg->ends[i].x, msg->ends[i].y, msg->ends[i].z);
        double length = (start_point - end_point).norm();
        total_segments_length += length;
        time_stamp = ros::Time::now().toSec() - start_time;
        // ROS_INFO("total_segments : %f", total_segments_length);
    }
    total_segments_lengths.push_back(total_segments_length);
    timestamps_seg.push_back(time_stamp);

    plt::figure(2);
    plt::clf();
    plt::plot(timestamps_seg, total_segments_lengths); 
    plt::grid(true);
    plt::xlabel("Time (seconds)");  
    plt::ylabel("Total Segment Length"); 
    plt::title("Total Segment Length vs Time"); 
    plt::xlim(0, (int)time_limitation_);  
    plt::ylim(0, 1000);
    plt::pause(0.1);

    return;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "metrics");
    ros::NodeHandle nh;
    start_time = ros::Time::now().toSec();

    nh.getParam("/drag_h", drag_h);
    nh.getParam("/drag_v", drag_v);
    nh.getParam("/hover", hover);
    nh.getParam("/time_limitation", time_limitation_);

    ros::Subscriber drone_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(
        "/mavros/local_position/drone_pose_with_correction", 
        10, dronePosCallback);

    ros::Subscriber segments_sub = nh.subscribe<path_planning::Segment>(
        "seg_point", 
        10, segmentsCallback);
   
    plt::ion(); 

    ros::spin();

    return 0;

}