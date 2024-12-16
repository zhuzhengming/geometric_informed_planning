#!/usr/bin/python3
"""
Drone controller node
---------------------

Set this controller as <extern> controller in Webots. 

This controller enables the motors and other devices onboard the drone
and listens to MotorCommand messages (sent by the autopilot_node) to 
apply to the motors. 

Command line arguments: 
- `--port`: the port to use with Webots (default: 1234)
- `--robot_name`: the name of the robot in Webots (default: starling)
- `--mode`: the state estimate mode (use "mcs" to use the ground truth pose,
    or "vio" to use the VIO pose estimate, which adds the initial pose offset)

Subscribed topics:
- /autopilot/motor_command

Published topics:
- /mavros/imu/data (frame_id : base_link, FLU) 
- /mavros/local_position/pose (frame_id : map, FLU) 
- /macortex_bridge/mcs_pose/<robot_name> (frame_id : world, FLU)
- /clock 

Broadcasted transforms:
- /world (MCS frame) -> /map (VIO frame)
"""

import os, sys
import numpy as np 
from scipy.spatial.transform import Rotation as R
from typing import List

from controller import Motor, Supervisor, Node, Camera
from autopilot.utils.imu_webots import IMU 
from autopilot.utils.camera_webots import CameraWebots, to_image_msg
from autopilot.utils.rangefinder_webots import TimeOfFlight, to_pointcloud2_msg

import rospy 
from auto_pilot_pkg.msg import MotorCommand 
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, Image, PointCloud2
from geometry_msgs.msg import PoseStamped, Quaternion
from tf2_ros import TransformBroadcaster, TransformStamped


from tf.transformations import euler_from_quaternion, quaternion_from_euler

from argparse import ArgumentParser

ap = ArgumentParser()
ap.add_argument('-p','--port',type=int,default=1234,help='Port to use with Webots',required=False)
ap.add_argument('-n','--robot_name',type=str,default='starling',help='Name of the robot in Webots',required=False)
ap.add_argument('-m','--mode',type=str,default='mcs',help='State estimate mode: "mcs" or "vio"',required=False)
ap.add_argument('-f','--fps',type=int,default=10,help='Cameras frame rate',required=False)
ap.add_argument('--mocap_frame',type=str,default='mocap',help='MCS frame name',required=False)
ap.add_argument('--vio_frame',type=str,default='map',help='VIO frame name',required=False)

args, _ = ap.parse_known_args()

# Set the port to communicate with Webots
port = args.port
robot_name = args.robot_name
os.putenv('WEBOTS_CONTROLLER_URL', f'ipc://{port}/{robot_name}')

# Set the parameters
MODE        = args.mode         # mcs (default) or vio
FPS         = args.fps          # cameras frame rate (default: 10)
MOCAP_FRAME = args.mocap_frame  # MCS frame (default: mocap)
VIO_FRAME   = args.vio_frame    # VIO frame (default: map)


def init_motors(robot:Supervisor):
    ''' 
    Initialize the motors 
    '''
    motors:List[Motor] = []
    motors.append(robot.getDevice('rotor1'))
    motors.append(robot.getDevice('rotor2'))
    motors.append(robot.getDevice('rotor3'))
    motors.append(robot.getDevice('rotor4'))
    INFINITY = float('+inf')
    for i in range(4):
        motors[i].setPosition(INFINITY)
        motors[i].setVelocity(0.0)
    return motors

def get_pose(node:Node, t:float):
    '''
    Get the pose of the provided node in the world (MCS)
    and in the VIO frame. If `mode == mcs`, then both 
    poses are the same.

    ### Parameters
    - `node`: the node to get the pose from
    - `t`: the time at which to get the pose

    ### Returns
    A tuple containing:
    - `mcs`: the pose in the MCS frame (PoseStamped)
    - `vio`: the pose in the VIO frame (PoseStamped)
    '''

    # Broadcast the transform from map to world
    tr = TransformStamped()
    tr.header.stamp = rospy.Time.from_sec(t)
    tr.header.frame_id = VIO_FRAME
    tr.child_frame_id = MOCAP_FRAME
    tr.transform.translation.x = mcs2vio_t[0]
    tr.transform.translation.y = mcs2vio_t[1]
    tr.transform.translation.z = mcs2vio_t[2]
    tr.transform.rotation.w = mcs2vio_r.as_quat()[3]
    tr.transform.rotation.x = mcs2vio_r.as_quat()[0]
    tr.transform.rotation.y = mcs2vio_r.as_quat()[1]
    tr.transform.rotation.z = mcs2vio_r.as_quat()[2]
    get_pose.br.sendTransform(tr)
    
    # Get the pose of the robot node in the world (MCS)
    pose:np.ndarray = np.reshape(np.array(node.getPose()),(4,4)) # 4x4 matrix  

    # Pose in MCS frame
    p:np.ndarray = pose[:3,3].reshape((3,1))
    q:np.ndarray = R.from_matrix(pose[:3,:3]).as_quat() # returns as (x, y, z, w)

    mcs = PoseStamped()
    mcs.header.stamp = rospy.Time.from_sec(t)
    mcs.header.frame_id = MOCAP_FRAME
    mcs.pose.position.x = p[0]
    mcs.pose.position.y = p[1]
    mcs.pose.position.z = p[2]
    mcs.pose.orientation.w = q[3]
    mcs.pose.orientation.x = q[0]
    mcs.pose.orientation.y = q[1]
    mcs.pose.orientation.z = q[2] 

    if MODE == 'mcs':
        return mcs, mcs 

    # Pose in VIO frame
    p = mcs2vio_r.as_matrix() @ pose[:3,3].reshape((3,1)) + mcs2vio_t 
    q = (mcs2vio_r * R.from_matrix(pose[:3,:3])).as_quat()

    vio = PoseStamped()
    vio.header.stamp = rospy.Time.from_sec(t)
    vio.header.frame_id = VIO_FRAME
    vio.pose.position.x = p[0]
    vio.pose.position.y = p[1]
    vio.pose.position.z = p[2]
    vio.pose.orientation.w = q[3]
    vio.pose.orientation.x = q[0]
    vio.pose.orientation.y = q[1]
    vio.pose.orientation.z = q[2] 

    return mcs, vio 

get_pose.br = TransformBroadcaster()


np.random.seed(42)  # Set the random seed
def noisy_pose_generator(position_std: float = 0.0002):
    """
    Generator to simulate sensor noise on positional data of a robot or drone's pose.

    Parameters:
    - position_std (float): Standard deviation of the Gaussian noise added to the pose's position, defaulting to 0.0001. Represents the magnitude of sensor noise.

    Yields:
    - noisy_pose (PoseStamped): A new PoseStamped object with noisy position data and the original orientation.
    """
    pose_error = np.zeros(3, dtype=np.float64)

    while True:
        clean_pose = yield
        if clean_pose is None:
            continue

        # Add noise to position
        noisy_position = np.array([clean_pose.pose.position.x[0],
                                   clean_pose.pose.position.y[0],
                                   clean_pose.pose.position.z[0]])
        pose_error += np.random.normal(0, position_std, size=3)  # Increment the position error
        noisy_position += pose_error

        # Create the noisy pose
        noisy_pose = PoseStamped()
        noisy_pose.header.stamp = clean_pose.header.stamp
        noisy_pose.header.frame_id = clean_pose.header.frame_id
        noisy_pose.pose.position.x = noisy_position[0]
        noisy_pose.pose.position.y = noisy_position[1]
        noisy_pose.pose.position.z = noisy_position[2]
        noisy_pose.pose.orientation = clean_pose.pose.orientation  # Keeping the orientation the same

        yield noisy_pose


def noisy_pose_yaw_generator(position_std: float = 0.0001, yaw_std: float = 0.0001, y_drift=False):
    """
    Generator to simulate sensor noise on positional data and yaw orientation of a robot or drone's pose.

    Parameters:
    - position_std (float): Standard deviation of the Gaussian noise added to the pose's position, defaulting to 0.0001. Represents the magnitude of sensor noise.
    - yaw_std (float): Standard deviation of the Gaussian noise added to the pose's yaw (in radians), defaulting to 0.0001. Represents the magnitude of sensor noise.

    Yields:
    - noisy_pose (PoseStamped): A new PoseStamped object with noisy position and yaw orientation data.
    """
    pose_error = np.zeros(3, dtype=np.float64)
    yaw_error = 0.0

    while True:
        clean_pose = yield
        if clean_pose is None:
            continue

        # Add noise to position
        noisy_position = np.array([clean_pose.pose.position.x[0],
                                   clean_pose.pose.position.y[0],
                                   clean_pose.pose.position.z[0]])
        pose_error += np.random.normal(0, position_std, size=3)  # Increment the position error
        noisy_position += pose_error
        # Add noise to yaw orientation
        clean_orientation = [clean_pose.pose.orientation.x,
                             clean_pose.pose.orientation.y,
                             clean_pose.pose.orientation.z,
                             clean_pose.pose.orientation.w]
        euler_orientation = euler_from_quaternion(clean_orientation)
        if y_drift:
            yaw_error += 0.1*yaw_std
        else:
            yaw_error += np.random.normal(0, yaw_std)  # Increment the yaw error
        noisy_yaw = euler_orientation[2] + yaw_error  # Apply the cumulative yaw error
        noisy_euler_orientation = (euler_orientation[0], euler_orientation[1], noisy_yaw)
        noisy_orientation = quaternion_from_euler(*noisy_euler_orientation)
        # Create the noisy pose
        noisy_pose = PoseStamped()
        noisy_pose.header.stamp = clean_pose.header.stamp
        noisy_pose.header.frame_id = clean_pose.header.frame_id
        noisy_pose.pose.position.x = noisy_position[0]
        noisy_pose.pose.position.y = noisy_position[1]
        noisy_pose.pose.position.z = noisy_position[2]
        noisy_pose.pose.orientation = Quaternion(*noisy_orientation)

        yield noisy_pose



def motor_cmd_cb(msg:MotorCommand, motors:List[Motor]):
    '''
    MotorCommand callback 
    '''
    motors[0].setVelocity(msg.motor_speeds[0])
    motors[1].setVelocity(msg.motor_speeds[1])
    motors[2].setVelocity(msg.motor_speeds[2])
    motors[3].setVelocity(msg.motor_speeds[3])


# ROS init
rospy.init_node('drone_controller_node_py')

# Webots init
robot = Supervisor()
robot_name:str  = robot.getName()
robot_node:Node = robot.getSelf() 

# Get the MCS to VIO transform
if MODE == 'vio':
    vio_origin:np.ndarray = np.reshape(np.array(robot_node.getPose()),(4,4)) # robot starting point in world 
elif MODE == 'mcs':
    vio_origin:np.ndarray = np.eye(4)
else: 
    rospy.logerr(f'Invalid mode: {MODE}')
    raise ValueError(f'Invalid mode: {MODE}')
mcs2vio_r:R = R.from_matrix(vio_origin[:3,:3]).inv()
mcs2vio_t:np.ndarray = mcs2vio_r.as_matrix() @ -vio_origin[:3,3].reshape((3,1))

# ROS params 
imu_id:int = rospy.get_param('/drone_controller_node/imu_id', default=1) 
rospy.loginfo(f'[drone_controller_node] robot name: "{robot.getName()}", using IMU {imu_id}')

timestep = int(robot.getBasicTimeStep())
dt = timestep/1000.0

# Enable components 
motors = init_motors(robot) 

imu = IMU(robot)
imu.enable(str(imu_id))

if 'starling' in robot_name:
    hires = CameraWebots(robot,'highRes',fps=FPS) 
    hires.enable()

    tracking = CameraWebots(robot,'tracking',fps=FPS)
    tracking.enable() # HACK disable the tracking camera for now

    tof = TimeOfFlight(robot,'tof',fps=5)
    tof.enable()

# ROS subscribers 
rospy.Subscriber('/autopilot/motor_command', MotorCommand,
                    motor_cmd_cb,callback_args=motors,
                    queue_size=1,tcp_nodelay=True) 

# ROS publishers 
imu_pub     = rospy.Publisher('/mavros/imu/data',Imu,
                    queue_size=1,tcp_nodelay=True)
pose_pub    = rospy.Publisher('/mavros/local_position/pose', PoseStamped,
                    queue_size=1,tcp_nodelay=True)

noisy_pose_pub    = rospy.Publisher('/mavros/local_position/noisy_pose', PoseStamped,
                    queue_size=1,tcp_nodelay=True) 

gt_pose_pub = rospy.Publisher('/macortex_bridge/'+robot_name+'/pose', PoseStamped,
                    queue_size=1,tcp_nodelay=True)
clock_pub   = rospy.Publisher('/clock',Clock,
                    queue_size=1,tcp_nodelay=True)
track_pub   = rospy.Publisher('/tracking',Image,
                    queue_size=1,tcp_nodelay=True)
hires_pub   = rospy.Publisher('/hires',Image,
                    queue_size=1,tcp_nodelay=True)
pc_pub      = rospy.Publisher('/tof_pc',PointCloud2,
                    queue_size=1,tcp_nodelay=True)


sim_clock = Clock()
# pose_error = np.zeros(3, dtype=np.float64)
noisy_pose_gen = noisy_pose_yaw_generator(position_std=0.001, yaw_std=0.0005) # Uncomment to Choose between noisy_pose_gen and noisy_pose_yaw_gen
# noisy_pose_gen = noisy_pose_generator(position_std=0.001)

while not rospy.is_shutdown() and robot.step(timestep) != -1:
    next(noisy_pose_gen)  # Prime the generator

    t = robot.getTime() 

    sim_clock.clock = rospy.Time.from_sec(t)
    clock_pub.publish(sim_clock) 

    imu.get_measurement() 
    if imu_id == 1: imu.project_axes(proj=[1,0,2],sign=[1,-1,1]) # RFU -> FLU 
    imu_pub.publish(imu.to_imu_msg(t)) 

    if 'starling' in robot_name:
        if hires.ready(t):  
            hires_pub.publish(to_image_msg(hires, t, frame='pose'))

        if tracking.ready(t):
            track_pub.publish(to_image_msg(tracking, t, frame='tracking'))

        if 'starling' in robot_name and tof.ready(t):
            pc_pub.publish(to_pointcloud2_msg(tof, t, frame='world'))

    gt_pose, pose = get_pose(robot_node,t)
    pose_pub.publish(pose)
    #print('type of  pose.pose.position.x ', type(pose.pose.position.x))
    # noisy_pose = get_noisy_pose(pose, pose_error, position_std=0.001) # 0.002 noramlly
    noisy_pose = noisy_pose_gen.send(pose)  # Send the new pose to the generator and get the noisy pose
    noisy_pose_pub.publish(noisy_pose)
    gt_pose_pub.publish(gt_pose)