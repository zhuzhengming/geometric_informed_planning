"""
    auto_pilot_demo controller.

    This controller demonstrates how to use the AutoPilot class. 

    This controller must be used with the VOXL m500 drone model. 
    The drone is instructed to hover 1m above the ground and stay put. 
    The drone can be perturbed by applying forces onto it with ALT+right click to test the autopilot response. 
    The state estimate is generated by reading the pose of the drone directly with the supervisor. 
"""

import sys
import time
from random import random, seed 
import numpy as np 
from scipy.spatial.transform import Rotation as R
from matplotlib import pyplot as plt
from typing import List 

from controller import Motor, Supervisor, Node, LED, Camera

from autopilot.main import BasicPose, autopilot
from autopilot.utils import trajectory as tr 
from autopilot.utils.imu_webots import IMU 
from autopilot.utils.camera_webots import CameraWebots
from autopilot.utils.rangefinder_webots import TimeOfFlight 

def nwu2ned(v:list) -> list:
    '''
    Convert a NWU to a NED vector (Webots -> PX4 conversion)
    
    NED <- NWU:
    - x   <-  x
    - y   <- -y
    - z   <- -z
    '''
    return [v[0],-v[1],-v[2]]

def nwu2nedRot(m:list) -> list:
    '''
    Convert a rotation matrix in NWU representation as 
    roll pitch yaw in NED (Webots -> PX4 conversion)
    ''' 
    M = np.reshape(np.array(m),[3,3])
    r:R = R.from_matrix(M)
    euler:np.ndarray = r.as_euler('xyz',degrees=False)
    return nwu2ned(euler.tolist())

# Define some constants 
INFINITY = float('+inf') # motor position constant for velocity control 

# Create the Robot instance.
robot = Supervisor()

# Get the robot node and name 
robot_node:Node = robot.getSelf() 
robot_name:str = robot.getName()

# Get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
dt = timestep/1000.0

# Enable the m500 sensors
if robot_name != None and 'm500' in robot_name:
    led:LED = robot.getDevice('led')
    camera = CameraWebots(robot,'tracking',fps=10)
else:
    led:LED = None

# Enable the IMU  
if robot_name != None and 'crazy' in robot_name:
    imu  = IMU(robot)        # crazyflie
    imu.enable()
else:
    imu = IMU(robot) 
    imu.enable('0') # voxl m500 (same for IMU 1)

# Enable the starling sensors
if robot_name != None and 'starling' in robot_name:
    camera = CameraWebots(robot,'tracking',fps=10)
    tof = TimeOfFlight(robot,'tof',fps=5)
else:
    tof = None

# Initialize the motors 
motors:List[Motor] = []
motors.append(robot.getDevice('rotor1'))
motors.append(robot.getDevice('rotor2'))
motors.append(robot.getDevice('rotor3'))
motors.append(robot.getDevice('rotor4'))
for i in range(4):
    motors[i].setPosition(INFINITY)
    motors[i].setVelocity(0.0)

# Get the initial position
p_init:List[float] = nwu2ned(robot_node.getPosition())

# Read path to flight parameters file if provided 
fp_path:str = None 
if(len(sys.argv) > 1):
    fp_path = sys.argv[1]

# Initialize the autopilot
autopilot.init("Position",fp_path)
if int(autopilot.dt*1000) != int(robot.getBasicTimeStep()):
    print(f'\033[93m[auto_pilot_demo] Warning: basicTimeStep should be {int(autopilot.dt*1000)} ms but is instead {int(robot.getBasicTimeStep())} ms\033[0m')

# Arm the autopilot 
autopilot.arm(True)

# If using crazyflie, we need to modify the allocation matrix
# Motors: 1 FR - / 2 BR + / 3 BL - / 4 FL +
if robot_name != None and 'crazy' in robot_name:
    print('Setting allocation matrix for crazyflie')
    c_t, c_q, l = autopilot.c_t, autopilot.c_q, autopilot.l # simplify notation 
    A:list = [
            [ c_t,      c_t,      c_t,      c_t     ],
            [-c_t * l, -c_t * l, +c_t * l, +c_t * l ],
            [ c_t * l, -c_t * l, -c_t * l,  c_t * l ],
            [-c_q,      c_q,     -c_q,      c_q     ]
        ]
    autopilot.set_allocation_matrix(A,[-1,1,-1,1]) 

# Set a random seed for trajectory generation 
tr.set_rand_seed() 

seed(time.time())
t_offset = random()/0.06

# Generate a trajectory [yaw, x, y, z, duration]
waypoints = np.array([
    [0,         0,-0,-1,  np.nan],
    [np.pi/2,   1,-0,-1,  np.nan],
    [-np.pi/2,  2,-0,-1,  np.nan],
    [np.pi/2,   0,-1,-2,  np.nan],
    [-np.pi/2,  0,-1,-1,  np.nan],
])
waypoints = np.array([
    [1* 1.57, 0,0,-0,  np.nan],
    [1* 1.57, 0,0,-1,  np.nan],
    [1*-1.57, 1,0,-0,  np.nan],
    [1*-1.57, 1,0,-1,  np.nan],
    [1* 1.57, 0,1,-0,  np.nan],
    [1* 1.57, 0,1,-1,  np.nan],
    [1*-1.57, 1,1,-0,  np.nan],
    [1*-1.57, 1,1,-1,  np.nan],
])
# waypoints = np.concatenate((
#     np.random.uniform(-np.pi,np.pi,(10,1)), # heading
#     np.random.uniform(-1,1,(10,1)),         # x
#     np.random.uniform(-1,1,(10,1)),         # y
#     np.random.uniform(-1,1,(10,1)),         # z
#     np.ones((10,1))*np.nan                  # duration
# ),axis=1)
waypoints[:,1]*=2   # scale x
waypoints[:,1]+=-1  # shift x
waypoints[:,2]*=2   # scale y
waypoints[:,2]+=-1  # shift y
waypoints[:,3]*=2   # scale z
waypoints[:,3]+=-0.5  # shift z
tr.set_waypoints(waypoints,mean_vel=1.5,wp_vel=1.5,loop=True)

# Main loop
while robot.step(timestep) != -1:

    # Switch led color every second 
    t = int(np.floor(robot.getTime()))
    if led is not None: led.set(t%2+1) 

    # Get the IMU data
    imu_measurement:list = imu.get_measurement()

    # Get the camera data
    if robot_name != None and 'starling' in robot_name or 'm500' in robot_name:
        if camera is not None and camera.ready(robot.getTime()):
            #camera.get_image()
            pass
        if tof is not None and tof.ready(robot.getTime()):
            #tof.get_pointcloud()
            pass

    # Get the ToF data when the simulation is paused and plot it
    if robot.simulationGetMode() == robot.SIMULATION_MODE_PAUSE and tof is not None:
        
        pc = tof.get_pointcloud(flat=False)
        depth_image = tof.get_depth_image()
        
        fig = plt.figure()
        ax = plt.subplot2grid((1,2),(0,0), projection='3d')
        ax.set_title('ToF point cloud')
        ax.scatter(pc[:,0],pc[:,1],pc[:,2],c='r',marker='.',s=1)
        ax.set_xlim((0,4))
        ax.set_ylim((-2,2))
        ax.set_zlim((-2,2))
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax = plt.subplot2grid((1,2),(0,1))
        ax.set_title('ToF depth image')
        ax.imshow(depth_image,cmap='gray')
        plt.tight_layout()
        plt.show() 

    # Get the robot pose 
    p:List[float] = nwu2ned(robot_node.getPosition())
    o:List[float] = nwu2nedRot(robot_node.getOrientation())
    pose = BasicPose()
    pose.set(p[0],p[1],p[2],o[0],o[1],o[2]) 
    
    # Update the drone state 
    autopilot.state.update_with_pose(pose, dt) 

    # Set the target (position target [h,x,y,z], centered 1m above the ground, NED convention)
    target = tr.get_target(t=robot.getTime()+t_offset,trajectory='waypoints')

    # Compute the motor control 
    u = autopilot.get_u(target) 

    # Send motor commands 
    motors[0].setVelocity(u[0])
    motors[1].setVelocity(u[1])
    motors[2].setVelocity(u[2])
    motors[3].setVelocity(u[3])
