#!/usr/bin/env python3
'''
Drone autopilot based on PX4 cascaded control. 

Inspired from Maxime Gardoni's semester project DISAL-SP149
https://disalw3.epfl.ch/teaching/student_projects/ay_2020-21/ss/DISAL-SP149_summary.pdf
'''

from math import fabs
from operator import add 
import numpy as np

from autopilot.opti import saturate_array, adjust_yaw_target, _pid, _acceleration_to_attitude, _detect_failure, _mixer 

class ColTxt:
    '''
    Convenience color definitions for printing colored text 
    '''
    HEADER = '\033[95m'
    BLUE = '\033[94m'
    CYAN = '\033[96m'
    GREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

class PIDState:
    '''
    This class allows to store the interal state of a PID controller. 
    '''
    def __init__(self):
        '''
        Initialize the PID inner state 
        '''
        self.integral = 0.0   # controller integral term
        self.derivative = 0.0 # controller derivative term
        self.error_prev = 0.0
        self.y_c_prev = 0.0

def pid(y_c, y, k, limit, Ts, dy, param, log=False, integLim=0.0 ):
    """
    PID control
    inspired by Beard and McLain, Small Unmanned Aircraft, Chapter 6.5, Princeton University Press, 2012, p.131

    ### Arguments
    - `y_c` setpoint
    - `y` current value
    - `k` gains [P, I, D]
    - `limit` saturation for the output limits
    - `Ts` refresh period
    - `dy` diffrential if available from higher levels, otherwise will be computed
    - `param` Memory variable
    - `log` Flag for printing the value, usefull for tuning
    - `integLim` max. limit on the integrator term, infinite if None

    ### Return 
    - `u` the ouput of the PID
    - `param` memory variable, to be fed at the next iteration
    """
    if type(limit) != list:
        lim_down = -limit
        lim_up   =  limit
    else: 
        lim_down = limit[0]
        lim_up   = limit[1]
    u, param.integral, param.derivative,  param.error_prev, param.y_c_prev = \
        _pid(float(y_c), float(y), float(k[0]), float(k[1]), float(k[2]), float(lim_down), float(lim_up), float(Ts), float(dy), 
             float(param.integral), float(param.derivative), float(param.error_prev), 
             float(param.y_c_prev), float(integLim))
    return u, param 


class BasicPose: 
    '''
    Simple class describing a 3D pose

    ### Attributes
    - `x` 
    - `y` 
    - `z` 
    - `roll` 
    - `pitch` 
    - `yaw` 
    '''
    def __init__(self) -> None:
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.roll  = 0.0
        self.pitch = 0.0
        self.yaw   = 0.0

    def __repr__(self) -> str:
        return 'p:[{0:.3f},{1:.3f},{2:.3f}], o:[{3:.3f},{4:.3f},{5:.3f}]'\
            .format(self.x,self.y,self.z,self.roll,self.pitch,self.yaw)
    
    def set(self,x,y,z,roll,pitch,yaw):
        self.x = x
        self.y = y
        self.z = z
        self.roll  = roll
        self.pitch = pitch
        self.yaw   = yaw

class Vector3D: 
    '''
    Simple 3D vector class
    '''
    def __init__(self) -> None:
        self.x = 0
        self.y = 0
        self.z = 0
    
    def set(self,x,y,z):
        self.x = x
        self.y = y
        self.z = z

class State: 
    '''
    Drone state. Stores all quantities describing the pose and velocities of the aircraft. 
    When updating the state, the new pose must be provided, as well as the elapsed time
    since the last update. The derivation of speed and acceleration are handled internally. 
    The provided dt is then used for control as well. 

    Note that the state follows the NED frame representation.
    '''
    def __init__(self) -> None:
        
        # Drone pose
        self.pn = 0.0 # position in x
        self.pe = 0.0 # position in y
        self.pd = 0.0 # position in z

        self.pn_old = 0.0 # previous position in x
        self.pe_old = 0.0 # previous position in y
        self.pd_old = 0.0 # previous position in z

        self.phi   = 0.0 # roll
        self.theta = 0.0 # pitch
        self.psi   = 0.0 # yaw 

        # Drone velocity
        self.vn = 0.0 # velocity in x
        self.ve = 0.0 # velocity in y
        self.vd = 0.0 # velocity in z

        self.p = 0.0 # roll rate
        self.q = 0.0 # pitch rate
        self.r = 0.0 # yaw rate

        # Drone acceleration
        self.an = 0.0
        self.ae = 0.0
        self.ad = 0.0

        self.update_with_pose(BasicPose(), 0.1)

    def update_with_pose(self, pose:BasicPose, dt:float):
        '''
        Update the state with a new pose 

        ### Arguments
        - `pose` BasicPose instance storing the current position and attitude of the drone
        - `dt` time elapsed since last update 
        '''

        # Numerical stability
        dt += 1e-10 
        self.dt = dt 

        ### Pose 

        # Store previous position
        self.pn_old = self.pn # previous position in x
        self.pe_old = self.pe # previous position in y
        self.pd_old = self.pd # previous position in z

        # Drone current position
        self.pn = pose.x # position in x
        self.pe = pose.y # position in y
        self.pd = pose.z # position in z

        # Store previous attitude 
        self.phi_old   = self.phi
        self.theta_old = self.theta
        self.psi_old   = self.psi

        # Drone current attitude 
        self.phi   = pose.roll
        self.theta = pose.pitch
        self.psi   = pose.yaw 


        ### Velocity

        # Store previous linear velocity 
        self.vn_old = self.vn 
        self.ve_old = self.ve 
        self.vd_old = self.vd 

        # Drone current linear velocity
        self.vn = (self.pn - self.pn_old) / dt 
        self.ve = (self.pe - self.pe_old) / dt 
        self.vd = (self.pd - self.pd_old) / dt 

        # Drone current angular velocity
        self.p = (self.phi - self.phi_old)     / dt 
        self.q = (self.theta - self.theta_old) / dt 
        self.r = (self.psi - self.psi_old)     / dt 


        ### Acceleration

        # Drone linear acceleration
        self.ae = (self.ve - self.ve_old) / dt
        self.an = (self.vn - self.vn_old) / dt
        self.ad = (self.vd - self.vd_old) / dt

    def update_with_velocity(self, vel:BasicPose, dt:float):
        '''
        Update the state with new 3D linear and angular velocities 

        ### Arguments
        - `vel` BasicPose instance storing the current linear and angular velocities of the drone
        - `dt` time elapsed since last update 
        '''

        # Numerical stability
        dt += 1e-10 
        self.dt = dt 

        ### Velocity

        # Store previous linear velocity 
        self.vn_old = self.vn 
        self.ve_old = self.ve 
        self.vd_old = self.vd 

        # Drone current linear velocity
        self.vn = vel.x
        self.ve = vel.y
        self.vd = vel.z 

        # Drone current angular velocity
        self.p = vel.roll
        self.q = vel.pitch
        self.r = vel.yaw


        ### Acceleration

        # Drone linear acceleration
        self.ae = (self.ve - self.ve_old) / dt
        self.an = (self.vn - self.vn_old) / dt
        self.ad = (self.vd - self.vd_old) / dt

    def update_with_attitude(self, att:Vector3D, dt):
        '''
        Update the state with a new attitude   

        ### Arguments
        - `att` Vector3D instance storing the current attitude of the drone
        - `dt` time elapsed since last update 
        '''
        
        # Numerical stability
        dt += 1e-10 
        self.dt = dt 

        # Store previous attitude 
        self.phi_old   = self.phi
        self.theta_old = self.theta
        self.psi_old   = self.psi

        # Drone current attitude 
        self.phi   = att.x 
        self.theta = att.y 
        self.psi   = att.z 

        # Drone current angular velocity
        self.p = (self.phi - self.phi_old)     / dt 
        self.q = (self.theta - self.theta_old) / dt 
        self.r = (self.psi - self.psi_old)     / dt 

    def update_with_pose_exp(self,x:float,y:float,z:float,roll:float,pitch:float,yaw:float,dt:float):
        '''
        Update the state with a new pose. Wrapper for update_with_pose() function.

        ### Arguments
        - `x`     position in x
        - `y`     position in y
        - `z`     position in z
        - `roll`  attitude in x 
        - `pitch` attitude in y
        - `yaw`   attitude in z 
        - `dt`  time elapsed since last update 
        '''
        pose = BasicPose()
        pose.set(x,y,z,roll,pitch,yaw)
        self.update_with_pose(pose,dt)

    def update_with_velocity_exp(self,x:float,y:float,z:float,roll:float,pitch:float,yaw:float,dt:float):
        '''
        Update the state with a new velocity. Wrapper for update_with_velocity() function.

        ### Arguments
        - `x`     velocity in x
        - `y`     velocity in y
        - `z`     velocity in z
        - `roll`  rate in x 
        - `pitch` rate in y
        - `yaw`   rate in z 
        - `dt`  time elapsed since last update 
        '''
        vel = BasicPose()
        vel.set(x,y,z,roll,pitch,yaw)
        self.update_with_velocity(vel,dt) 

    def update_with_attitude_exp(self,roll:float,pitch:float,yaw:float,dt:float):
        '''
        Update the state with a new attitude. Wrapper for update_with_attitude() function.

        ### Arguments
        - `roll`  attitude in x 
        - `pitch` attitude in y
        - `yaw`   attitude in z 
        - `dt`  time elapsed since last update 
        '''
        att = Vector3D()
        att.set(roll,pitch,yaw)
        self.update_with_velocity(att,dt) 


class AutoPilot():
    '''Auto pilot class used to control a quadrotor in simulation. 
    
    ## AutoPilot class
    This class contains all flight parameters for the control of the aircraft. 
    The frame definition used here is NED. 

    ### Arguments 
    - `autopilot_type`: type of control [`Position`,`Velocity`,`Acceleration`,`Attitude`,`Rate`,`Force`]
    - `path`: path to the flight parameters file 
    '''
    def __init__(self):
        pass 

    def init(self, autopilot_type:str, path:str=None):
        '''
        Initialize the auto-pilot 

        ### Arguments 
        - `autopilot_type`: type of control [`Position`,`Velocity`,`Acceleration`,`Attitude`,`Rate`,`Force`]
        - `path`: path to the flight parameters file 
        '''
        
        self.autopilot_control_type = autopilot_type
        self.command = [0,0,0,0]    # motors speed command 
        self.state = State()        # drone state 
        self.thrust = 0.0           # thrust command
        self.counter = 0            # delayed cascaded PID structure 
        self.armed = False          # arm status 
        self.wrn_unarmed = False    # indicate if an unarmed warning was already displayed 

        '''
        Physical parameters
        '''
        # drone physical parameters
        self.m = 1.37               # Drone mass in kg 
        self.l = 0.167              # Distance from CoM to rotor axis along x-axis in m

        # motor constants
        self.c_t = 0.0000103        # Thrust constant of motor
        self.c_q = 0.000000113      # Torque constant of motor
        self.vel_prop_max = 1100    # Rotor max velocity in rad/s 
        self.torque_max = (self.c_t * self.vel_prop_max ** 2) * 2 * self.l

        # allocation matrix (assumes strictly positive motor commands)
        c_t, c_q, l = self.c_t, self.c_q, self.l # simplify notation 
        self.A = np.array([
                [ c_t,      c_t,      c_t,      c_t     ],
                [-c_t * l, +c_t * l, +c_t * l, -c_t * l ],
                [ c_t * l, -c_t * l, +c_t * l, -c_t * l ],
                [ c_q,      c_q,     -c_q,     -c_q     ]
            ]) # M = A@u, standard PX4 quad config, where M = [T Mn Me Md]' 
        self.B:np.array # pseudo inverse of A, hence u = B@M with B = pinv(A)
        self.compute_allocation_matrix_pinv() 

        # rotation direction of the rotors (relative to upwards axis)
        self.rotor_sign = [1,1,-1,-1] 

        # simulation parameters
        self.dt = 0.016             # Simulation timestep in s (adapted online)

        # failure thresholds
        self.attitude_max = 1.2     # Maximum allowed attitude 

        ''' 
        Default PID gains and limits 
        '''
        # positiom to velocity
        self.k_pn_vn = [0.8,0.0,0.0]
        self.k_pe_ve = [0.8,0.0,0.0]
        self.k_pd_vd = [0.8,0.0,0.0]
        self.vn_max = 5
        self.ve_max = 5
        self.vd_max = [-3, 1]

        # velocity to acceleration
        self.k_vn_an = [3.0,1.0,0.0]
        self.k_ve_ae = [3.0,1.0,0.0]
        self.k_vd_ad = [4.5,0.05,0.0]
        self.an_max = 5
        self.ae_max = 5
        self.ad_max = 4
        
        # attitude to rate
        self.k_phi_p   = [7.0,0.0,0.0]
        self.k_theta_q = [7.0,0.0,0.0]
        self.k_psi_r   = [1.0,0.0,0.0]
        self.p_max = 220
        self.q_max = 220
        self.r_max = 200
        
        # rate to forces  
        self.k_p_torque = [0.1,0.1,0.005]
        self.k_q_torque = [0.1,0.1,0.005]
        self.k_r_torque = [0.05, 0.01, 0]
        self.k_p_int_lim = 0.3
        self.k_q_int_lim = 0.3
        self.k_r_int_lim = 0.3
        

        if path is not None and path != '':
            self.load_flight_parameters(path=path)

        self.reset_pid()

        print(ColTxt.GREEN + "[AutoPilot] initialized " + ("(using default flight parameters)"+ ColTxt.ENDC if path is None else "" + ColTxt.ENDC)) 

    def load_flight_parameters(self, path="./params.txt"):

        print("[AutoPilot] reading flight parameters from {}".format(path))

        success = True 
        
        with open(path, 'r') as file: 
            
            for line in file.readlines():

                # Ignore commented lines or empty lines 
                if line[0] == "#" or len(line.strip('\n').strip(' ')) == 0:
                    continue 

                # Read line content (extract words from line)
                line_values = [x for x in line.strip('\n').split(sep=" ") if x != '']
                attr = line_values[0]
                value = line_values[1]
                
                # If the attribute exists, get the value 
                if hasattr(self, attr):
                    # Parse a list 
                    if '[' in value:
                        try:
                            value = [float(x) for x in value.strip('[]').split(',')]
                            #print('[AutoPilot] {} = {}'.format(attr, value)) 
                            setattr(self,attr,value)
                        except: 
                            success = False
                            print(ColTxt.FAIL + "[AutoPilot] error: could not read '{}', are there spaces?".format(line_values[0]) + ColTxt.ENDC)

                    # Parse a string
                    elif '"' in value: 
                        try:
                            value = value.strip('"')
                            #print('[AutoPilot] {} = {}'.format(attr, value))  
                            setattr(self,attr,value)
                        except: 
                            success = False
                            print(ColTxt.FAIL + "[AutoPilot] error: could not read '{}'".format(line_values[0]) + ColTxt.ENDC)
                    
                    # Parse a float
                    else:
                        try:
                            value = float(value)
                            #print('[AutoPilot] {} = {}'.format(attr, value)) 
                            setattr(self,attr,value)
                        except: 
                            success = False
                            print(ColTxt.FAIL + "[AutoPilot] error: could not read parameter '{}'".format(line_values[0]) + ColTxt.ENDC)

                else: 
                    print(ColTxt.WARNING + "[AutoPilot] warning: unknown parameter '{}'".format(line_values[0]) + ColTxt.ENDC)
        
        if success: 
            print(ColTxt.GREEN + "[AutoPilot] flight parameters read successfully"+ColTxt.ENDC)
        else:
            print(ColTxt.FAIL + "[AutoPilot] error while reading parameters"+ColTxt.ENDC)

    def reset_pid(self):
        ''' 
        Reset all PIDs internal state variables
        '''
        # position
        self.P_pn_vn = PIDState()
        self.P_pe_ve = PIDState()
        self.P_pd_vd = PIDState()

        # velocity
        self.P_vn_an = PIDState()
        self.P_ve_ae = PIDState()
        self.P_vd_ad = PIDState()
        
        # attitude
        self.P_phi_p   = PIDState()
        self.P_theta_q = PIDState()
        self.P_psi_r   = PIDState()
        
        # rate
        self.P_p_torque = PIDState()
        self.P_q_torque = PIDState()
        self.P_r_torque = PIDState()

    def set_autopilot_type(self, control_type:str):
        '''
        Set the auto-pilot control type

        ### Arguments
        - `control_type`: ['Position','Velocity','Acceleration','Attitude','Rate','Force']
        '''

        allowed_types = ['Position','Velocity','Acceleration','Attitude','Rate','Force']
        
        if(not control_type in allowed_types):
            print(ColTxt.FAIL+"[AutoPilot] Error: illegal control type '{}'".format(control_type))
            print(ColTxt.WARNING+"[AutoPilot] Authorized types are ['Position','Velocity','Acceleration','Attitude','Rate','Force']")
            print("[AutoPilot] Defaulting to 'Position'"+ColTxt.ENDC)
            self.autopilot_control_type = "Position"
        
        self.autopilot_control_type = control_type

    def arm(self, cmd:bool):
        self.reset_pid() 
        self.armed = cmd 

    def acceleration_to_attitude(self, an, ae, ad):
        """
        Convert desired acceleration into attitude and thrust in the world frame (NED). 

        ### Arguments
        - `an` accel north
        - `ae` accel east
        - `ad` accel down

        ### Return
        - attitude and thrust: [`phi`,`theta`,`thrust`]
        """
        return _acceleration_to_attitude(float(self.m),float(an), float(ae), float(ad))

    def detect_failure(self):
        '''
        Detect if the control failed. 
        ### Return
        - `True` if attitude is beyond allowed range. 
        '''
        return _detect_failure(self.state.phi,self.state.theta,self.attitude_max)

    def detect_landing(self):
        '''
        Detect if the drone landed. 
        ### Return
        - `True` if landing detected. 
        '''

        eps = 0.0001

        if(self.state.pd > -1.0 
            and fabs(self.state.vn) < eps
            and fabs(self.state.ve) < eps
            and fabs(self.state.vd) < eps):
            return True  
        return False

    def get_u(self, target:list, superimpose:bool=False) -> list:
        """
        Compute the motor control based on the command. 

        ### Arguments 
        - `state` state object describing the current state of the aircraft
        - `target` set point 
        - `superimpose` whether to superimpose control with previous motor command

        ### Return
        - `u` list of motor velocities 

        ## Command types

        Note: no thrust setpoint is computed for the control types Attitude, 
        Rate and Force. 

        ### Position (global)
        `target` = [yaw, x, y, z]
        ### Velocity (global)
        `target` = [yaw, vx, vy, vz]
        ### Acceleration (global)
        `target` = [yaw, ax, ay, az] 
        ### Attitude (local)
        `target` = [roll, pitch, yaw]
        ### Rate (local)
        `target` = [roll_rate, pitch_rate, yaw_rate]
        ### Force (local)
        `target` = [roll_torque, pitch_torque, yaw_torque, thrust]
        """

        if(not self.armed):
            if(not self.wrn_unarmed):
                print(ColTxt.WARNING+"[AutoPilot] Warning: requiring motor command while not armed"+ColTxt.ENDC) 
                self.wrn_unarmed = True 
            return [0,0,0,0]
        self.wrn_unarmed = False 
        
        # Stop motors if control loss 
        if(self.armed and self.detect_failure()):
            print(ColTxt.FAIL+"[AutoPilot] Failure: excessive attitude, disarm"+ColTxt.ENDC)
            self.arm(False) 
            return [0,0,0,0]

        # Update the time delta 
        self.dt = self.state.dt 

        # Prevent unfeasible time steps yielding control failures 
        if(self.dt < 1e-3): 
            print(ColTxt.WARNING+"[AutoPilot] Warning: invalid time step dt = {:.2} s".format(self.dt)+ColTxt.ENDC) 
            return self.command # Reapply previous motor command 
        
        # Initialize the commands (will be overridden by the control type)
        self.pn_c = 0.0     # position
        self.pe_c = 0.0
        self.pd_c = 0.0
        self.vn_c = 0.0     # velocity
        self.ve_c = 0.0
        self.vd_c = 0.0
        self.an_c = 0.0     # acceleration
        self.ae_c = 0.0
        self.ad_c = 0.0
        self.phi_c = 0.0    # attitude
        self.theta_c = 0.0
        self.psi_c = 0.0
        self.p_c = 0.0      # rate
        self.q_c = 0.0
        self.r_c = 0.0
        
        # ________ process the command based on control type___________
        if self.autopilot_control_type == "Position":
            self.psi_c = float(target[0])  # commanded yaw [rad]
            self.pn_c = float(target[1])  # commanded north position [m]
            self.pe_c = float(target[2])  # commanded east position [m]
            self.pd_c = float(target[3])  # commanded down position [m]

        if self.autopilot_control_type == "Velocity":
            self.psi_c = float(target[0])  # commanded yaw [rad]
            self.vn_c = float(target[1])  # commanded north velocity [m/s]
            self.ve_c = float(target[2])  # commanded east velocity [m/s]
            self.vd_c = float(target[3])  # commanded down velocity [m/s]

        if self.autopilot_control_type == "Acceleration":
            self.psi_c = float(target[0])  # commanded yaw [rad]
            self.an_c = float(target[1])  # commanded north acceleration [m/s^2]
            self.ae_c = float(target[2])  # commanded east acceleration [m/s^2]
            self.ad_c = float(target[3])  # commanded down acceleration [m/s^2]

        if self.autopilot_control_type == "Attitude":
            self.phi_c = float(target[0])  # commanded roll [rad]
            self.theta_c = float(target[1])  # commanded pitch [rad]
            self.psi_c = float(target[2])  # commanded yaw [rad]

        if self.autopilot_control_type == "Rate":
            self.p_c = float(target[0])  # commanded rollrate [rad]
            self.q_c = float(target[1])  # commanded pitchrate [rad]
            self.r_c = float(target[2])  # commanded yawrate [rad] 

        if self.autopilot_control_type == "Force":
            torque_phi, torque_theta, torque_psi, thrust = float(target[0]), float(target[1]), float(target[2]), \
                                                           float(target[3])

        # ___________ apply PID Loops _________
        autopilot_type = self.autopilot_control_type

        inertial_frame_freq_divider=4 # this high level block runs with a lower freq.
        if autopilot_type == "Position":
            if self.counter%inertial_frame_freq_divider==0:
                [self.vn_c, self.P_pn_vn] = pid(self.pn_c, self.state.pn,
                                           self.k_pn_vn,
                                           self.vn_max, self.dt*inertial_frame_freq_divider, np.Inf, self.P_pn_vn, False)

                [self.ve_c, self.P_pe_ve] = pid(self.pe_c, self.state.pe,
                                           self.k_pe_ve,
                                           self.ve_max, self.dt*inertial_frame_freq_divider, np.Inf, self.P_pe_ve, False)

                [self.vd_c, self.P_pd_vd] = pid(self.pd_c, self.state.pd,
                                           self.k_pd_vd,
                                           self.vd_max, self.dt*inertial_frame_freq_divider, np.Inf, self.P_pd_vd, False)

            autopilot_type = "Velocity"

        if autopilot_type == "Velocity":

            if self.counter%inertial_frame_freq_divider==0:
                [self.an_c, self.P_vn_an] = pid(self.vn_c, self.state.vn,
                                           self.k_vn_an,
                                           self.an_max, self.dt*inertial_frame_freq_divider, np.Inf, self.P_vn_an, False)

                [self.ae_c, self.P_ve_ae] = pid(self.ve_c, self.state.ve,
                                           self.k_ve_ae,
                                           self.ae_max, self.dt*inertial_frame_freq_divider, np.Inf, self.P_ve_ae, False)

                [self.ad_c, self.P_vd_ad] = pid(self.vd_c, self.state.vd,
                                           self.k_vd_ad,
                                           self.ad_max, self.dt*inertial_frame_freq_divider, np.Inf, self.P_vd_ad, False)

            autopilot_type = "Acceleration"

        if autopilot_type == "Acceleration":
            if self.counter%inertial_frame_freq_divider==0:
                
                (self.phi_c_g, self.theta_c_g, self.thrust) = self.acceleration_to_attitude(self.an_c, self.ae_c, self.ad_c)

                # coordinate change to body frame
                self.phi_c = np.cos(self.state.psi) * self.phi_c_g + np.sin(self.state.psi) * self.theta_c_g
                self.theta_c = np.cos(self.state.psi) * self.theta_c_g - np.sin(self.state.psi) * self.phi_c_g

            autopilot_type = "Attitude"

        if autopilot_type == "Attitude":
            angle_freq_divider=2
            if self.counter%angle_freq_divider==0:

                [self.p_c, self.P_phi_p] = pid(self.phi_c, self.state.phi,
                                          self.k_phi_p,
                                          self.p_max, self.dt*angle_freq_divider, self.state.p, self.P_phi_p)

                [self.q_c, self.P_theta_q] = pid(self.theta_c, self.state.theta,
                                            self.k_theta_q,
                                            self.q_max, self.dt*angle_freq_divider, self.state.q, self.P_theta_q)

                # adjust yaw target to be as close as possible of current yaw
                self.psi_c = adjust_yaw_target(self.psi_c, self.state.psi)

                [self.r_c, self.P_psi_r] = pid(self.psi_c, self.state.psi,
                                                self.k_psi_r,
                                                self.r_max, self.dt*angle_freq_divider, self.state.r, self.P_psi_r)
            autopilot_type = "Rate"

        if autopilot_type == "Rate":
            [torque_phi, self.P_p_torque] = pid(self.p_c, self.state.p,
                                                self.k_p_torque,
                                                self.torque_max, self.dt, np.Inf, self.P_p_torque, False, self.k_p_int_lim)

            [torque_theta, self.P_q_torque] = pid(self.q_c, self.state.q,
                                                  self.k_q_torque,
                                                  self.torque_max, self.dt, np.Inf, self.P_q_torque, False, self.k_q_int_lim)

            [torque_psi, self.P_r_torque] = pid(self.r_c, self.state.r,
                                                self.k_r_torque,
                                                self.torque_max, self.dt, np.Inf, self.P_r_torque, False, self.k_r_int_lim)

        # if not hasattr(self, "thrust"): # if not initialized
        #     self.thrust = 0.0

        if autopilot_type == "Force":
            self.thrust = np.abs(thrust) 
        
        # mixer
        u = self.mixer(self.thrust, torque_phi, torque_theta, torque_psi)

        # Saturate motor speeds
        u = saturate_array(u,0,self.vel_prop_max)

        # Apply correct rotation directions
        u = [a*b for a,b in zip(u,self.rotor_sign)]

        # Store command 
        if superimpose:
            self.command = list( map(add, self.command, list(u)) )
        else: 
            self.command = list(u)

        return self.command

    def set_allocation_matrix(self, A:list, rotor_sign:list = None):
        '''
        Set the allocation matrix manually

        ### Arguments
        - `A` allocation matrix as 2D list
        - `rotor_sign` (optional) rotation sign of the rotor (can be set in the flight parameters also)
        '''
        self.A = np.array(A)
        self.compute_allocation_matrix_pinv()
        if rotor_sign is not None:
            self.rotor_sign = rotor_sign 

    def compute_allocation_matrix_pinv(self):
        '''
        Compute the allocation matrix pseudo-inverse 
        '''
        self.B = np.linalg.pinv(self.A)

    def mixer(self, thrust, torque_phi, torque_theta, torque_psi):
        """
        Mix the torque and thrust input to a motor command, cf report chapter 3.5.1
        
        ### Arguments
        - `thrust`
        - `torque_phi`
        - `torque_theta`
        - `torque_psi`

        ### Return
        - `U` motor speed commands 
        """
        return _mixer(self.B, thrust, torque_phi, torque_theta, torque_psi) 

# Create a class instance 
autopilot = AutoPilot() 