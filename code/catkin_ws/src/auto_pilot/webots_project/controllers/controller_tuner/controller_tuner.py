"""
Autopilot PID tuner controller. 

This controller will perform the control of the drone for the 
following controls: 
- Rate
- Attitude
- Velocity
- Position

Set the name of the drone being tuned to not let the controller
run for other robots in the controller_tuner world. 
"""
ROBOT_NAME = 'starling'
autopilot_control_type = "Position" # Rate, Attitude, Velocity, Position
TUNING_ATT = 'roll' # roll, yaw (for Rate and Attitude tuning)

import sys
from time import time
from math import pi 
import numpy as np 
from scipy.spatial.transform import Rotation as R
from typing import List 
from matplotlib import pyplot as plt 

from controller import Motor, Supervisor, Node 

from autopilot.main import BasicPose, autopilot

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

# Initialize the motors 
motors:List[Motor] = []
motors.append(robot.getDevice('rotor1'))
motors.append(robot.getDevice('rotor2'))
motors.append(robot.getDevice('rotor3'))
motors.append(robot.getDevice('rotor4'))
for i in range(4):
    motors[i].setPosition(INFINITY)
    motors[i].setVelocity(0.0)

# exit if the current robot is not the one being tuned 
if ROBOT_NAME not in robot_name: 
    print(f'Robot name is "{robot_name}" but "{ROBOT_NAME}" is required, exiting')
    exit() 

# Get the initial position
p_init:List[float] = nwu2ned(robot_node.getPosition())

# Read path to flight parameters file if provided 
fp_path:str = None 
if(len(sys.argv) > 1):
    fp_path = sys.argv[1]

# Initialize the autopilot 
autopilot.init("Position",fp_path) # Note: control type is overwritten by config file 
autopilot.autopilot_control_type = autopilot_control_type # override the control type
autopilot.arm(True)

# If using the crazyflie, we need to modify the allocation matrix
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

# State variables to conduct the experiment 
counter = 0 
pose_counter = 0
target_angle = pi/4
target_rate = 1.0
target_velocity = 0.5 
target_velocity_z = -0.02
target_pos = 1.0 
target_pos_z = -0.2
old_roll = 0
old_pitch = 0 
old_yaw  = 0

# Data collected to plot the system's response 
class Data:
    t = []
    roll = []
    pitch = []
    yaw  = []
    roll_rate = []
    pitch_rate = []
    yaw_rate = []
    velx = [] 
    velz = [] 
    posx = [] 
    posz = [] 
    p_int = []
    q_int = []
    r_int = []
    motor_commands = []
    target_rate = []
    target_angle = []
    target_velocity = []
    target_pos = [] 
data = Data() 

# Main loop
while robot.step(timestep) != -1:

    if counter >= 3:
        print(f"Completed control sequence")
        break 

    # Get the robot pose 
    p:List[float] = nwu2ned(robot_node.getPosition())
    o:List[float] = nwu2nedRot(robot_node.getOrientation())
    pose = BasicPose()
    pose.set(p[0],p[1],p[2],o[0],o[1],o[2]) 
    
    # Update the drone state 
    autopilot.state.update_with_pose(pose, dt) 

    # Get current roll angle and adapt target 
    roll = autopilot.state.phi 
    pitch = autopilot.state.theta 
    yaw  = autopilot.state.psi 
    velx = autopilot.state.vn 
    velz = autopilot.state.vd 
    posx = autopilot.state.pn 
    posz = autopilot.state.pd 
    p_int = autopilot.P_p_torque.integral
    q_int = autopilot.P_q_torque.integral
    r_int = autopilot.P_r_torque.integral

    # Set the target for the selected control scheme
    if autopilot.autopilot_control_type == 'Rate':
        if TUNING_ATT == 'roll': 
            if(abs(roll - target_angle) < 0.1):
                print(f'Switching target to {target_angle}')
                counter += 1
                target_rate *= -1
                target_angle *= -1
            target = [target_rate,0,0]
        elif TUNING_ATT == 'yaw':
            if(abs(yaw - target_angle) < 0.1):
                print(f'Switching target to {target_angle}')
                counter += 1
                target_rate *= -1
                target_angle *= -1
            target = [0,0,target_rate]
    elif autopilot.autopilot_control_type == 'Attitude':
        if TUNING_ATT == 'roll':
            if(abs(roll - target_angle) < 0.01 and abs((roll-old_roll)/dt) < 0.01):
                print(f'Switching target to {target_angle}')
                counter += 1
                target_angle *= -1 
            target = [target_angle,0,0]
        elif TUNING_ATT == 'yaw':
            if(abs(yaw - target_angle) < 0.01 and abs((yaw-old_yaw)/dt) < 0.01):
                print(f'Switching target to {target_angle}')
                counter += 1
                target_angle *= -1 
            target = [0,0,target_angle]
    elif autopilot.autopilot_control_type == 'Velocity':
        if(abs(posx) > abs(target_pos) and posx*target_pos > 0): 
            print(f'Switching target to {target_velocity}')
            counter += 1
            target_pos *= -1
            target_velocity *= -1  
        target = [0,target_velocity,0,target_velocity_z] 
    elif autopilot.autopilot_control_type == 'Position':
        if(abs(posx - target_pos) < 0.01): 
            pose_counter += 1
            if pose_counter >= 100:
                pose_counter = 0
                print(f'Switching target to {target_pos}')
                counter += 1
                target_pos *= -1
        target = [0,target_pos,p_init[1],target_pos_z] 
    else:
        print(f"Error: control type should be either 'Rate', 'Attitude', 'Velocity' or 'Position' got instead '{autopilot.autopilot_control_type}'")
        exit() 
    
    # Compute the motor control 
    u = autopilot.get_u(target) 

    # Collect data
    if len(data.t) == 0: 
        data.t.append(0)
    else: 
        data.t.append(data.t[-1] + dt)
    data.roll.append(roll)
    data.pitch.append(pitch)
    data.yaw.append(yaw)
    data.roll_rate.append((roll-old_roll)/dt)
    data.pitch_rate.append((pitch-old_pitch)/dt)
    data.yaw_rate.append((yaw-old_yaw)/dt)
    data.velx.append(velx)
    data.velz.append(velz)
    data.posx.append(posx) 
    data.posz.append(posz) 
    data.p_int.append(p_int)
    data.q_int.append(q_int)
    data.r_int.append(r_int)
    if TUNING_ATT == 'roll':
        data.target_rate.append(autopilot.p_c)
    elif TUNING_ATT == 'yaw':
        data.target_rate.append(autopilot.r_c)
    if TUNING_ATT == 'roll':
        data.target_angle.append(autopilot.phi_c)
    elif TUNING_ATT == 'yaw':
        data.target_angle.append(autopilot.psi_c)
    data.target_velocity.append(autopilot.vn_c)
    data.target_pos.append(autopilot.pn_c) 
    data.motor_commands.append([u[0],u[1],u[2],u[3]])
    old_roll = roll 
    old_pitch = pitch 
    old_yaw  = yaw

    # Send motor commands 
    motors[0].setVelocity(u[0])
    motors[1].setVelocity(u[1])
    motors[2].setVelocity(u[2])
    motors[3].setVelocity(u[3])

# Stop the motor
motors[0].setVelocity(0)
motors[1].setVelocity(0)
motors[2].setVelocity(0)
motors[3].setVelocity(0)

# plot the data (plot up to the tested level) 
levels = ['Rate','Attitude','Velocity','Position']
if autopilot.autopilot_control_type in levels[:]:
    plt.figure()
    ax = plt.subplot2grid((2,1),(0,0))
    ax.plot(data.t,data.target_rate)
    if TUNING_ATT == 'roll':
        ax.plot(data.t,data.roll_rate,label='roll rate')
        #ax.plot(data.t,data.pitch_rate,label='pitch rate')
    if TUNING_ATT == 'yaw':
        ax.plot(data.t,data.yaw_rate)
    ax.set_title(f'Rate control')
    ax.set_xlabel('time [s]')
    ax.set_ylabel('roll rate [rad/s]')
    ax.legend()
    ax = plt.subplot2grid((2,1),(1,0))
    if TUNING_ATT == 'roll':
        ax.plot(data.t,data.p_int,label='roll integral')
    if TUNING_ATT == 'yaw':
        ax.plot(data.t,data.r_int,label='yaw integral')
    ax.set_title(f'Rate control integral')
    ax.set_xlabel('time [s]')
    ax.set_ylabel('integral')
    ax.legend()
    plt.tight_layout()
if autopilot.autopilot_control_type  in levels[1:]:
    plt.figure()
    plt.plot(data.t,data.target_angle)
    if TUNING_ATT == 'roll':
        plt.plot(data.t,data.roll,label='roll')
        plt.plot(data.t,data.pitch,label='pitch')
    if TUNING_ATT == 'yaw':
        plt.plot(data.t,data.yaw)
    plt.title(f'Attitude control')
    plt.xlabel('time [s]')
    plt.ylabel('roll [rad]')
    plt.legend()
if autopilot.autopilot_control_type in levels[2:]:
    plt.figure()
    plt.plot(data.t[2:-1],data.target_velocity[2:-1])
    plt.plot(data.t[2:-1],data.velx[2:-1],label='vel x')
    plt.hlines(target_velocity_z,data.t[0],data.t[-1])
    plt.plot(data.t[2:-1],data.velz[2:-1],label='vel z')
    plt.title('Velocity control')
    plt.xlabel('time [s]')
    plt.ylabel('velocity [m/s]')
    plt.legend()
if autopilot.autopilot_control_type in levels[3:]:
    plt.figure()
    plt.plot(data.t,data.target_pos)
    plt.plot(data.t,data.posx,label='pos x')
    plt.hlines(target_pos_z,data.t[0],data.t[-1])
    plt.plot(data.t,data.posz,label='pos z')
    plt.title('Position control')
    plt.xlabel('time [s]')
    plt.ylabel('position [m]')
    plt.legend() 

robot.simulationResetPhysics()
robot.simulationSetMode(robot.SIMULATION_MODE_PAUSE)

plt.show()