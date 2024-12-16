#!/usr/bin/python3
"""
Trajectory node
---------------

This node can arm the drone if required. Set param /trajectory_node/arm = true. 
It can be used both in simulation and to control a real drone. 

Publish position targets along the specified trajectory.
Possible trajectory options are:
['none','hover','circle','eight','random','random_smooth','vertical','ramps','circle+ramps','mix'] 

Parameters:
- /trajectory_node/trajectory: trajectory to follow
- /trajectory_node/arm: arm the drone before starting the trajectory

Published topics:
- /mavros/setpoint_raw/local, type: mavros_msgs.msg.PositionTarget, frame: NWU 
- /trajectory/path, type: nav_msgs.msg.Path, frame: NWU 

Subscribed topics:
- /mavros/local_position/pose, type: geometry_msgs.msg.PoseStamped, frame: NWU

Used services:
- /mavros/cmd/arming, type: mavros.command.CommandBool 
"""

import sys 

from scipy.spatial.transform import Rotation as R 
import pandas as pd 

import rospy  
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget 
from nav_msgs.msg import Path

# from sensor_msgs.msg import PointCloud2, PointField
# import sensor_msgs.point_cloud2 as pc2
import autopilot.utils.trajectory as tr
from mavros_msgs.srv import CommandBool

RATE      = 20  # general rate of node 
RATE_PATH = 4   # path message update rate 
PATH_LEN  = 100 # pose buffer length 

rospy.init_node('trajectory_node')

# Parametric trajectories parameters 
p_trajectory:str      = rospy.get_param('/trajectory_node/trajectory', default='none') 
p_replay_file:str     = rospy.get_param('/trajectory_node/replay_file', default='')
p_arm:bool            = rospy.get_param('/trajectory_node/arm', default=True) 
p_min_altitude:float  = rospy.get_param('/trajectory_node/min_altitude', default=0.5)
p_max_altitude:float  = rospy.get_param('/trajectory_node/max_altitude', default=2.5)
p_hovering_hgt:float  = rospy.get_param('/trajectory_node/hovering_height', default=1.5)
p_max_disp:float      = rospy.get_param('/trajectory_node/max_displacement', default=2.0)
p_velocity:float      = rospy.get_param('/trajectory_node/velocity', default=1.0)
p_fixed_heading:bool  = rospy.get_param('/trajectory_node/fixed_heading', default=False)

# Vector following parameters
p_vector:list         = rospy.get_param('/trajectory_node/vector', default=[0.0, 1.0, 0.0, 0.0])  # [yaw,x,y,z]

# Waypoint parameters
p_plot:bool           = rospy.get_param('/trajectory_node/plot', default=True)
p_waypoints_file:str  = rospy.get_param('/trajectory_node/waypoints_file', default='')
p_nointerp:bool       = rospy.get_param('/trajectory_node/no_interpolation', default=False)
p_wp_vel:float        = rospy.get_param('/trajectory_node/waypoint_velocity', default=0.0)
p_mean_vel:float      = rospy.get_param('/trajectory_node/mean_velocity', default=1.0)
p_loop:bool           = rospy.get_param('/trajectory_node/loop', default=False)
p_vel_ctrl:bool       = rospy.get_param('/trajectory_node/velocity_control',default=False)

rospy.loginfo(f'[trajectory_node] max_displacement: {p_max_disp}, max_altitude: {p_max_altitude}')

# Read the waypoints (x,y,z,yaw,duration) from the file pointed to by the parameter waypoints
if p_trajectory == 'waypoints' and p_waypoints_file != '' and p_waypoints_file[-1] != '/':
    waypoints = pd.read_csv(p_waypoints_file, header=0)
    waypoints = waypoints[['yaw','x','y','z','duration']] # reorder columns
    waypoints[['yaw','y','z']] *= -1 # FLU to NED
    print('Received following waypoints (NED) to execute:\n',waypoints)
    tr.set_waypoints(waypoints.to_numpy(),p_mean_vel,p_wp_vel,p_loop)

    tr.minjerktraj.print_info()
    print(f'Interpolation: {"enabled" if not p_nointerp else "disabled"}')
    
    if p_plot:
        print('Waiting for plot...')
        tr.minjerktraj.plot()
        print('Plot closed, starting trajectory')

if p_trajectory in ['none','']:
    rospy.logwarn(f'No trajectory specified, exiting')
    exit(0) 
elif p_trajectory == 'replay' and p_replay_file == '':
    rospy.logerr(f'Replay option selected but no replay file specified, exiting')
    exit(1) 

def target2PositionTarget(target:list, stamp:rospy.Time) -> PositionTarget:
    
    msg = PositionTarget()
    msg.header.stamp = stamp 
    msg.header.frame_id = 'map' 

    # enforce position control 
    msg.type_mask = 0 | \
                    PositionTarget.IGNORE_VX | \
                    PositionTarget.IGNORE_VY | \
                    PositionTarget.IGNORE_VZ | \
                    PositionTarget.IGNORE_AFX | \
                    PositionTarget.IGNORE_AFY | \
                    PositionTarget.IGNORE_AFZ | \
                    PositionTarget.IGNORE_YAW_RATE 

    # target is NED but needs to be FLU 
    msg.yaw = -target[0] 
    msg.position.x =  target[1] 
    msg.position.y = -target[2] 
    msg.position.z = -target[3] 

    return msg 

def target2VelocityTarget(target:list, stamp:rospy.Time) -> PositionTarget:
    
    msg = PositionTarget()

    msg.header.stamp = stamp 
    msg.header.frame_id = 'map' 

    # enforce position control 
    msg.type_mask = 0 | \
                    PositionTarget.IGNORE_PX | \
                    PositionTarget.IGNORE_PY | \
                    PositionTarget.IGNORE_PZ | \
                    PositionTarget.IGNORE_AFX | \
                    PositionTarget.IGNORE_AFY | \
                    PositionTarget.IGNORE_AFZ | \
                    PositionTarget.IGNORE_YAW_RATE

    # target is NED but needs to be FLU 
    msg.yaw = -target[0] 
    msg.velocity.x =  target[1] 
    msg.velocity.y = -target[2] 
    msg.velocity.z = -target[3] 

    return msg 

def target2PoseStamped(target:list, stamp:rospy.Time) -> PoseStamped:
    
    posestamped = PoseStamped()

    posestamped.header.frame_id = 'map'
    posestamped.header.stamp = stamp

    posestamped.pose.position.x =  target[1]
    posestamped.pose.position.y = -target[2]
    posestamped.pose.position.z = -target[3]
    q:list = R.from_euler('z',-target[0]).as_quat() # (x, y, z, w) format
    posestamped.pose.orientation.x = q[0]
    posestamped.pose.orientation.y = q[1]
    posestamped.pose.orientation.z = q[2]
    posestamped.pose.orientation.w = q[3]

    return posestamped 

def target_from_velvector(current_pose:list, vector:list=[0,1,0,0]) -> list:
    # The target point is the current position plus the vector
    target_point = [current_pose[i] + vector[i] for i in range(1,4)]
    # The target heading is the same as the current heading
    target_heading = current_pose[0]
    # Return the target point in the same format as the current pose
    return [target_heading] + target_point

def compute_velvector(previous_vector:list, point_cloud:list) -> list:
    # Cureent vect init
    current_vector = [0,1,0,0] # velocity vector [yaw,x,y,z]   
    return current_vector

target_vector = [0]*4 # vector [yaw,x,y,z]

def point_cloud_cb(msg):
    global target_vector
    # Process the point cloud data to find the direction of the free space
    #target_vector = process_point_cloud(msg)
    pass


current_pose = [0]*4 # NED pose [h,x,y,z]
def pose_cb(msg:PoseStamped):
    global current_pose
    
    heading = R.from_quat([
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ]).as_euler('xyz',degrees=False)[2]
    current_pose = [
        -heading,
         msg.pose.position.x,
        -msg.pose.position.y,
        -msg.pose.position.z
    ]

# PositionTarget publisher 
target_pub = rospy.Publisher('/mavros/setpoint_raw/local',PositionTarget,queue_size=1) 

# Path publisher
path_pub = rospy.Publisher('/trajectory/path',Path,queue_size=1) 

# Pose subscriber
pose_sub = rospy.Subscriber('/mavros/local_position/pose',PoseStamped, pose_cb)

# Point cloud subscriber
# point_cloud_sub = rospy.Subscriber("/tof_pc", PointCloud2, point_cloud_cb)

# Arm the drone 
# $ rosservice call /mavros/cmd/arming "value: true"
if p_arm: 
    rospy.wait_for_service('/mavros/cmd/arming')
    try:
        arm_srv = rospy.ServiceProxy('/mavros/cmd/arming',CommandBool) 
        resp = arm_srv(True) 
    except:
        rospy.logerr('[trajectory_node] could not arm drone') 

rate = rospy.Rate(RATE) 

path:Path = Path()
path.header.frame_id = 'map'
prev_t = rospy.Time.now().to_sec() 

start_time = -1.0

while not rospy.is_shutdown():

    stamp = rospy.Time.now()

    # Get current time in seconds
    t = stamp.to_sec() 

    if start_time < 0.0 and t > 0.0:
        start_time = t

    # Get the current pose target [headind,x,y,z]
    if p_trajectory == 'vector':
        velvector = [0,1,0,0]  # velocity vector [yaw,x,y,z] hardcoded for now
        target = target_from_velvector(current_pose, velvector)
    elif p_trajectory == 'waypoints':
        target = tr.get_target(t-start_time,
                            trajectory=p_trajectory,
                            max_displacement=p_max_disp,
                            min_altitude=p_min_altitude,
                            max_altitude=p_max_altitude,
                            hovering_height=p_hovering_hgt,
                            velocity=p_velocity,
                            fixed_heading=p_fixed_heading,
                            replay_file=p_replay_file,
                            nointerp=p_nointerp)

    # Add to path message every T seconds 
    if t - prev_t > 1/RATE_PATH:
        path.poses.append(target2PoseStamped(target,stamp))
        path.header.stamp = stamp
        if len(path.poses) > PATH_LEN:
            path.poses = path.poses[-PATH_LEN:]
        path_pub.publish(path)
        prev_t = t 

    # Create the target message 
    if p_trajectory == 'waypoints' and p_vel_ctrl: 
        v_target = tr.get_target_velocity(t-start_time,current_pose)
        msg = target2VelocityTarget(v_target,stamp)

    elif p_trajectory == 'vector':

        msg = target2PositionTarget(target,stamp)

    else:
        msg = target2PositionTarget(target,stamp)
    
    target_pub.publish(msg) 
    
    rate.sleep() 