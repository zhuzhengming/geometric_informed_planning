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

import sys
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
autopilot.autopilot_control_type = "Position" # override the control type
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

# Define a series of waypoints [h, x, y, z] in NED 
test_sequence = ('yaw','x/pitch','y/roll','z')
waypoints = np.array([
    [ 0,      0,0,-1],  # Yaw test 
    [ np.pi/2,0,0,-1],
    [-np.pi/2,0,0,-1],
    [ 0,0,0,-1],        # X / Pitch test
    [ np.pi/2, 1,0,-1],         
    [-np.pi/2,-1,0,-1],
    [ 0, 0,0,-1],       # Y / Roll test
    [ np.pi/2,0, 1,-1],         
    [-np.pi/2,0,-1,-1],
    [ 0,0,0,-1],        # Z / Roll test
    [ np.pi/2,0,0,-2],         
    [-np.pi/2,0,0,-0.5],
    [ 0,0,0,-1]         # final position
])
wp_idx = 0 # waypoint index
prev_idx = -1 # previous waypoint index

class Commands:
    pn_c = []
    pe_c = []
    pd_c = []
    vn_c = []
    ve_c = []
    vd_c = []
    an_c = []
    ae_c = []
    ad_c = []
    phi_c = []
    theta_c = []
    psi_c = []
    p_c = []
    q_c = []
    r_c = []
cmd = Commands()

class States:
    pn = []
    pe = []
    pd = []
    vn = []
    ve = []
    vd = []
    phi = []
    theta = []
    psi = []
    p = []
    q = []
    r = []
state = States()

t = []
test_times = []

# Main loop
while robot.step(timestep) != -1:

    # Get the robot pose 
    p:List[float] = nwu2ned(robot_node.getPosition())
    o:List[float] = nwu2nedRot(robot_node.getOrientation())
    
    # Update the drone state 
    pose = BasicPose()
    pose.set(p[0],p[1],p[2],o[0],o[1],o[2]) 
    autopilot.state.update_with_pose(pose, dt) 

    # Update the target
    target = waypoints[wp_idx,:].tolist()

    # Update waypoint index if needed
    de = np.linalg.norm(np.array(p) - np.array(target[1:])) # distance error
    da = np.abs(target[0] - o[2]) # angle error
    ve = np.linalg.norm(np.array([autopilot.state.vn,autopilot.state.ve,autopilot.state.vd])) # velocity error
    if de < 0.1 and ve < 0.1 and da < 0.04: # within tolerance
        wp_idx += 1
    if wp_idx >= waypoints.shape[0]:
        break # exit the loop

    if prev_idx != wp_idx:
        if wp_idx in [1,4,7,10]:
            print('Test sequence for',test_sequence[wp_idx//3])
            test_times.append(robot.getTime())
        prev_idx = wp_idx
    
    # Compute the motor control 
    u = autopilot.get_u(target) 

    # Get the current time 
    t.append(robot.getTime())

    # Send motor commands 
    motors[0].setVelocity(u[0])
    motors[1].setVelocity(u[1])
    motors[2].setVelocity(u[2])
    motors[3].setVelocity(u[3])

    # Store commands
    cmd.pn_c.append(autopilot.pn_c)      # position
    cmd.pe_c.append(autopilot.pe_c) 
    cmd.pd_c.append(autopilot.pd_c) 
    cmd.vn_c.append(autopilot.vn_c)      # velocity
    cmd.ve_c.append(autopilot.ve_c) 
    cmd.vd_c.append(autopilot.vd_c) 
    cmd.an_c.append(autopilot.an_c)      # acceleration
    cmd.ae_c.append(autopilot.ae_c) 
    cmd.ad_c.append(autopilot.ad_c) 
    cmd.phi_c.append(autopilot.phi_c)    # attitude
    cmd.theta_c.append(autopilot.theta_c) 
    cmd.psi_c.append(autopilot.psi_c) 
    cmd.p_c.append(autopilot.p_c)        # rate
    cmd.q_c.append(autopilot.q_c) 
    cmd.r_c.append(autopilot.r_c) 

    # Store states
    clip = lambda x: 0 if np.abs(x) > 20 else x
    state.pn.append(autopilot.state.pn)       # position
    state.pe.append(autopilot.state.pe)
    state.pd.append(autopilot.state.pd)
    state.vn.append(clip(autopilot.state.vn)) # velocity
    state.ve.append(clip(autopilot.state.ve))
    state.vd.append(clip(autopilot.state.vd))
    state.phi.append(autopilot.state.phi)     # attitude
    state.theta.append(autopilot.state.theta)
    state.psi.append(autopilot.state.psi)
    state.p.append(clip(autopilot.state.p))         # rate
    state.q.append(clip(autopilot.state.q))
    state.r.append(clip(autopilot.state.r))

    if not autopilot.armed:
        print('Drone disarmed')
        break 

# Stop the motor
motors[0].setVelocity(0)
motors[1].setVelocity(0)
motors[2].setVelocity(0)
motors[3].setVelocity(0)

# plot the data (plot up to the tested level) 
fig = plt.figure(figsize=(12,12))

fig.suptitle(f'Sequences: {test_sequence}\nDashed: command, Solid: state')
shape = (5,1)

ax = plt.subplot2grid(shape,(0,0))
ax.set_title('Position tracking')
ax.plot(t,cmd.pn_c,'r',linestyle='--',label='pn_c')
ax.plot(t,cmd.pe_c,'g',linestyle='--',label='pe_c')
ax.plot(t,cmd.pd_c,'b',linestyle='--',label='pd_c')
ax.plot(t,state.pn,'r',label='pn')
ax.plot(t,state.pe,'g',label='pe')
ax.plot(t,state.pd,'b',label='pd')
ax.vlines(test_times, *ax.get_ybound(), colors='k', linestyles='--', alpha=0.5, label='Test sequence')
ax.set_xlabel('Time [s]')
ax.set_ylabel('Position [m]')

ax = plt.subplot2grid(shape,(1,0))
ax.set_title('Velocity tracking')
ax.plot(t,cmd.vn_c,'r',linestyle='--',label='vn_c')
ax.plot(t,cmd.ve_c,'g',linestyle='--',label='ve_c')
ax.plot(t,cmd.vd_c,'b',linestyle='--',label='vd_c')
ax.plot(t,state.vn,'r',label='vn')
ax.plot(t,state.ve,'g',label='ve')
ax.plot(t,state.vd,'b',label='vd')
ax.vlines(test_times, *ax.get_ybound(), colors='k', linestyles='--', alpha=0.5, label='Test sequence')
ax.set_xlabel('Time [s]')
ax.set_ylabel('Velocity [m/s]')

ax = plt.subplot2grid(shape,(2,0))
ax.set_title('Acceleration')
ax.plot(t,cmd.an_c,'r',linestyle='--',label='an_c')
ax.plot(t,cmd.ae_c,'g',linestyle='--',label='ae_c')
ax.plot(t,cmd.ad_c,'b',linestyle='--',label='ad_c')
ax.vlines(test_times, *ax.get_ybound(), colors='k', linestyles='--', alpha=0.5, label='Test sequence')
ax.set_xlabel('Time [s]')
ax.set_ylabel('Acceleration [m/s^2]')

ax = plt.subplot2grid(shape,(3,0))
ax.set_title('Attitude tracking')
ax.plot(t,cmd.phi_c,'r',linestyle='--',label='phi_c')
ax.plot(t,cmd.theta_c,'g',linestyle='--',label='theta_c')
ax.plot(t,cmd.psi_c,'b',linestyle='--',label='psi_c')
ax.plot(t,state.phi,'r',label='phi')
ax.plot(t,state.theta,'g',label='theta')
ax.plot(t,state.psi,'b',label='psi')
ax.vlines(test_times, *ax.get_ybound(), colors='k', linestyles='--', alpha=0.5, label='Test sequence')
ax.set_xlabel('Time [s]')
ax.set_ylabel('Angle [rad]')

ax = plt.subplot2grid(shape,(4,0))
ax.set_title('Rate tracking')
ax.plot(t,cmd.p_c,'r',linestyle='--',label='p_c')
ax.plot(t,cmd.q_c,'g',linestyle='--',label='q_c')
ax.plot(t,cmd.r_c,'b',linestyle='--',label='r_c')
ax.plot(t,state.p,'r',label='p')
ax.plot(t,state.q,'g',label='q')
ax.plot(t,state.r,'b',label='r')
ax.vlines(test_times, *ax.get_ybound(), colors='k', linestyles='--', alpha=0.5, label='Test sequence')
ax.set_xlabel('Time [s]')
ax.set_ylabel('Rate [rad/s]')

plt.tight_layout()

robot.simulationResetPhysics()
robot.simulationSetMode(robot.SIMULATION_MODE_PAUSE)

plt.show()