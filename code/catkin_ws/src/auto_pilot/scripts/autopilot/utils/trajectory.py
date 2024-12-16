#!/usr/bin/env python3
'''
Generate trajectories in NED convention. 
Each method returns a pose target with the format:
- [yaw, n, e ,d] 
'''

from math import pi, sin, cos, atan2, sqrt
from random import random, uniform, seed
import numpy as np 
import pandas as pd 
from scipy.signal import square 
import time

from autopilot.utils.min_jerk_traj import minjerktraj 

waypoints = np.array([]) # waypoints to follow (x,y,z,yaw,duration)

def set_fixed_seed():
    seed(42)
def set_rand_seed():
    seed(time.time())

p_init = [0.0,0.0,0.0] # initial position of the drone (x,y,z)
def set_init_pose(x:float,y:float,z:float):
    global p_init 
    p_init = [x,y,z] 

def set_waypoints(wp:np.array, mean_vel=1.0, wp_vel=0.0, loop=False) -> None:
    '''
    Waypoints defined as: ['yaw','x','y','z','duration'] (NED)

    ### Parameters
    - `wp`: array of waypoints
    - `mean_vel`: mean velocity between waypoints (m/s)
    - `vel`: velocity at waypoints (m/s)
    - `loop`: loop the trajectory (default: False)
    '''
    global waypoints 
    waypoints = wp

    minjerktraj.init(mean_vel)
    minjerktraj.integrate_waypoints(waypoints, wp_vel, loop)

def get_target(t:float, trajectory:str='hover', 
                max_displacement=1.0, 
                min_altitude=0.5, 
                max_altitude=2.0, 
                hovering_height=1.5,
                velocity=1.0,
                fixed_heading=False,
                dt:float=0.008, replay_file='',
                nointerp:bool=False) -> list:
    '''
    Target generation wrapper. Generate the next target for the selected trajectory. 

    ### Parameters
    - `t`: current time in seconds
    - `trajectory`: name of the trajectory to execute. Can be set as int as well. 
    - `max_displacement`: maximum allowed distance along x and y from the origin
    - `min_altitude`: minimum allowed altitude 
    - `max_altitude`: maximum allowed altitude 
    - `hovering_height`: height at which to hover
    - `velocity`: velocity at which to fly
    - `fixed_heading`: whether the heading should be fixed or computed from the velocity
    - `dt`: timestep used (for `replay` mode only)
    - `replay_file`: csv file containing a recorded trajectory (for `replay` mode only).
        The csv file must contain the following columns: 'time','x','y','z','yaw',
        and represent data in NWU/FLU. 
    - `nointerp`: whether the waypoints should be interpolated (for `waypoints` trajectory only, default is False)

    ### Options
    The `trajectory` parameter can be set to one of the following values:
    - `hover`
    - `circle`
    - `eight`
    - `random`
    - `random_vertical`
    - `vertical`
    - `ramps`
    - `circle+ramps`
    - `random_smooth`
    - `drag`
    - `thrust`
    - `replay`
    - `waypoints`
    
    ### Return 
    A target in NED containing the fields [yaw,x,y,z] 
    '''
    
    # Trajectory generation  
    if   trajectory == 'hover': 
        target = hover(height=hovering_height,heading=0.0) 
    elif trajectory == 'circle':
        target = circle(t,radius=max_displacement,vel=velocity,height=hovering_height,tilted=False,fixed_heading=fixed_heading)
    elif trajectory == 'eight': 
        target = eight(t,a=max_displacement,height=hovering_height,f=0.08)
    elif trajectory == 'random':
        target = rand(t,a=max_displacement,min_height=min_altitude,max_height=max_altitude,T=4,xy=True,yaw=True) 
    elif trajectory == 'random_vertical':
        target = rand(t,a=max_displacement,min_height=min_altitude,max_height=max_altitude,T=3,xy=False,yaw=False) 
    elif trajectory == 'vertical': 
        target = vertical_steps(t,height=1,f=1/8,heading=0.0)
    elif trajectory == 'ramps':
        target = vertical_ramps(t,height=1,vel=0.25,heading=0.0) 
    elif trajectory == 'circle+ramps':
        target = circle(t,a=max_displacement,f=0.06,height=-1,tilted=False)
        target[3] = vertical_ramps(t,height=-1,vel=0.25)[3] 
    elif trajectory == 'random_smooth':
        target = rand_smooth(t,a=max_displacement, min_height=min_altitude,max_height=max_altitude, T=6) # Horizontal flight 
    elif trajectory == 'drag':
        #target = fast_circle(t,min_height=min_altitude,max_height=max_altitude,radius=max_displacement)
        target = drag_trajectory(t,min_height=min_altitude,max_height=max_altitude,max_displacement=max_displacement)
    elif trajectory == 'thrust':
        target = thrust_trajectory(t,min_height=min_altitude,max_height=max_altitude,max_displacement=max_displacement)
    elif trajectory == 'replay':
        assert replay_file != '', 'Selected replay mode but no file specified' 
        if not is_replay_ready():
            try:
                set_replay(replay_file,'time','x','y','z','yaw',t_init=t,period=dt,nwu2ned=True) 
            except:
                print('Cannot load replay. Did you check that the file path is correct and that the column names are properly defined?') 
                exit(1) 
        target = replay(t) 
    elif trajectory == 'waypoints':
        target = minjerktraj(t, d=0, nointerp=nointerp)
    
    else: 
        raise ValueError(f'Unknown trajectory "{trajectory}"')
    
    return target 

def get_target_velocity(t:float, pose:list, K:float = 1.0, limit:float = 1.0):
    '''
    Get the target velocity to track the trajectory. 
    Warning: only for waypoints trajectory!

    ### Parameters
    - `t`: current time in seconds
    - `pose`: current pose in NED [heading,x,y,z]
    - `K`: pose tracking gain 

    ### Return
    A target velocity in NED [heading,dx/dt,dy/dt,dz/dt], combining the velocity 
    of the trajectory and the velocity to track the trajectory.
    The heading is not a velocity (expressed in [rad]).
    '''
    heading   = minjerktraj(t)[0]
    traj_vel  = np.array(minjerktraj(t,d=1))[1:]                      # Velocity of the trajectory
    track_vel = K*(np.array(minjerktraj(t))[1:] - np.array(pose)[1:]) # Velocity to track the trajectory
    track_vel = np.clip(track_vel,-limit,limit)                       # Clip the velocity to track the trajectory

    return [heading] + (traj_vel + track_vel).tolist()


def hover(height:float=1.0, heading:float=0.0) -> list: 
    '''
    ## Brief
    Fly in a circle
    ## Arguments
    - `height` defines at which average height to fly
    - `heading` define the heading at which to hover 
    ## Return 
    - Pose target in NED [heading,x,y,z] 
    '''
    return [heading,p_init[0],p_init[1],-height] 

def circle(t:float, radius:float=2.0, vel:float=1.0, 
            height:float=1.5, tilted:bool=True,
            fixed_heading:bool=False) -> list:
    '''
    ## Brief
    Fly in radius circle
    ## Arguments
    - `t` current time 
    - `radius` defines the circle radius 
    - `vel` defines the velocity in m/s at which the circle should be looped 
    - `height` defines at which average height to fly
    - `tilted` defines whether the circle should be tilted instead of horizontal (flies at height*(1+-0.3))
    - `fixed_heading` indicates whether the heading must remain constant (0), computed from the velocity otherwise
    ## Return 
    - Pose target in NED [heading,x,y,z]
    '''
    global p_init 

    # Compute the frequency from the desired velocity
    f = vel/(2*pi*radius)

    # Compute the circle position and velocity
    x  = radius*cos(2*pi*f*t) + p_init[0]
    y  = radius*sin(2*pi*f*t) + p_init[1]
    z  = -height*(1 + 0.3*cos(2*pi*f*t)*tilted)
    dx = -vel*sin(2*pi*f*t)
    dy =  vel*cos(2*pi*f*t)
    heading = 0 if fixed_heading else atan2(dy,dx)

    return [heading,x,y,z] 

def eight(t:float, a:float=0.4, f:float=0.06, height:float=0.3, tilted:bool=True) -> list:
    '''
    ## Brief
    Figure eight trajectory (see: https://mathworld.wolfram.com/EightCurve.html)
    ## Arguments
    - `t` current time 
    - `a` defines the maximum distance 
    - `f` defines the frequency at which the circle should be looped 
    - `height` defines at which average height to fly
    - `tilted` defines whether the circle should be tilted instead of horizontal (flies at height*(1+-0.3))
    ### Return 
    - Pose target in NED [heading,x,y,z]
    '''
    global p_init 
    x = a*sin(2*pi*f*t)               + p_init[0]
    y = a*sin(2*pi*f*t)*cos(2*pi*f*t) + p_init[1]
    dx = 2*pi*f*a*cos(2*pi*f*t)
    dy = 2*pi*f*a*(cos(2*pi*f*t)**2 - sin(2*pi*f*t)**2) 
    return [atan2(dy,dx),x,y,-height*(1 + 0.3*cos(2*pi*f*t)*tilted)] 

def vertical_steps(t:float,height:float=0.4,f:float=0.1,heading:float=0.0) -> list: 
    '''
    ## Brief
    Generate vertical commands
    ## Arguments 
    - `t` current time
    - `height` height around which to evolve. The targets will be set at (1+-0.2)*height
    - `f` is the frequency at which the new targets are sent 
    - `heading` allows to specify the desired heading 
    ### Return 
    - Pose target in NED [heading,x,y,z]
    '''
    global p_init 
    return [heading,p_init[0],p_init[1],-height*(1 + 0.2*square(4*pi*f*t))] 

def vertical_ramps(t:float,height:float=1.0,vel:float=1.0,heading:float=0.0) -> list:
    '''
    ## Brief
    Generate vertical commands
    ## Arguments 
    - `t` current time
    - `height` is the average altitude (flies at height*(1+-0.2))
    - `vel` is the vertical velocity at which the target moves 
    - `heading` allows to specify the desired heading 
    ### Return 
    - Pose target in NED [heading,x,y,z]
    '''
    global p_init 
    
    dt = t - vertical_ramps.prev_t 
    vertical_ramps.prev_t = t 
    
    d = 0.2*height 
    f = vel/(2*d) # travel back and forth 
    s = square(2*pi*f*t, duty=0.5)
    
    vertical_ramps.z += vel*s*dt
    if vertical_ramps.z < -d:
        vertical_ramps.z = -d
    elif vertical_ramps.z > d:
        vertical_ramps.z = d
    
    return [heading,p_init[0],p_init[1],-height-vertical_ramps.z] 
vertical_ramps.z = 0.0
vertical_ramps.prev_t = 0.0

def rand(t:float, a:float=0.4,min_height:float=0.5, max_height:float=2.5, T:float=4.0, xy:bool=True, yaw:bool=True) -> list:
    '''
    Send random position targets in arena every `T` seconds.
    - `t` current time
    - `a` is the maximal lateral distance
    - `min_height` defines a minimal altitude 
    - `max_height` is the maximal height variation (absolute value)
    - `T` is the time interval between poses 
    - `xy` allows to generate random xy locations if set to `True`
    - `yaw` allows to generate random heading commands if set to `True`
    ### Return 
    - Pose target in NED [heading,x,y,z]
    '''
    if t - rand.prev_t >= T:

        prev_target = rand.target
        
        for _ in range(100):
            rand.target = [  
                uniform(-pi,pi)*yaw,
                uniform(-a,a)*xy + p_init[0],
                uniform(-a,a)*xy + p_init[1],
                -uniform(min_height,max_height)
            ]
            if abs(rand.target[3]-prev_target[3]) > 0.3*(max_height-min_height): break

        rand.prev_t = t 
        
    return rand.target 
rand.prev_t = 0.0
rand.target = [0,p_init[0],p_init[1],-1.0]

def rand_smooth(t:float, a:float=1.4,min_height:float=0.4, max_height:float=None, v:float=2, T:float=4.0) -> list:
    '''
    Send random position targets in arena every `T` seconds.
    - `t` is the current time
    - `a` is the maximal lateral distance
    - `min_height` defines a minimal altitude 
    - `max_height` defines the vertical travel distance (defaults to `a`)
    - `v` velocity norm at target 
    - `T` is the time interval between poses 
    ### Return 
    - Pose target in NED [heading,x,y,z]
    '''
    if max_height == None: max_height = min_height + a*2 # make flight volume a cube 

    # Pick new target 
    if t - rand_smooth.ti >= T:
        rand_smooth.x1 = rand_smooth.x2
        while True:
            rand_smooth.x2 = [ (random()*2-1)*a,
                            (random()*2-1)*a,
                            -min_height - random()*(max_height-min_height) ]
            d_xy = sqrt(np.sum([(i-j)**2 for i,j in zip(rand_smooth.x1[:2],rand_smooth.x2[:2])]))
            d_z = abs(rand_smooth.x1[2]-rand_smooth.x2[2])
            if d_xy > a or d_z > max_height/2: break # wait for sufficiently far next target 
        rand_smooth.x1_dot = rand_smooth.x2_dot 
        s = 1 if random()*2-1 > 0 else -1 
        vel = np.array([rand_smooth.x2[1]*s,-rand_smooth.x2[0]*s,0])
        vel = vel/np.linalg.norm(vel)*v 
        rand_smooth.x2_dot = vel
        rand_smooth.ti = t 
        rand_smooth.te = rand_smooth.ti + T 

    return minjerktraj._minimum_jerk(t,rand_smooth.ti,rand_smooth.te,rand_smooth.x1,rand_smooth.x2,rand_smooth.x1_dot,rand_smooth.x2_dot) 
rand_smooth.ti = -1000.0
rand_smooth.te = 5.0 
rand_smooth.x1 = p_init
rand_smooth.x2 = p_init
rand_smooth.x1_dot = [0,0,0] 
rand_smooth.x2_dot = [0,0,0] 
rand_smooth.target = p_init 

def fast_circle(t:float,min_height:float=0.4, max_height:float=1.0, 
                radius:float=1.0, v_max:float=2.0, face_forward:bool=False) -> list:
    '''
    Fly along a cirlce with an increasing velocity.
    ### Parameters
    - `t` is the current time
    - `min_height` defines the minimal altitude 
    - `max_height` defines the maximal altitude
    - `radius` defines the circle radius 
    - `v_max` is the maximal velocity norm 
    ### Return 
    - Pose target in NED [heading,x,y,z]
    '''
    global p_init

    # Initialize starting time 
    if fast_circle.ti < 0: fast_circle.ti = t 

    r = radius
    f_min = 0.0
    f_max = v_max/(2*pi*r) # max frequency to ensure v_max is not exceeded
    f_dot = 0.005 # frequency change rate

    f = f_min + f_dot*(t-fast_circle.ti)
    if f > f_max: 
        return [0,p_init[0],p_init[1],-min_height] # stop at min height 

    x = r*cos(2*pi*f*t) + p_init[0]
    y = r*sin(2*pi*f*t) + p_init[1]
    dx = -2*pi*f*r*sin(2*pi*f*t)
    dy = 2*pi*f*r*cos(2*pi*f*t)
    ddx = -(2*pi*f)**2*r*cos(2*pi*f*t)
    ddy = -(2*pi*f)**2*r*sin(2*pi*f*t)
    heading = atan2(dy,dx) if face_forward else 0
    z = -min_height - (max_height-min_height)*(1+sin(2*pi*f*t))/2

    v = np.sqrt(dx**2 + dy**2)
    a = np.sqrt(ddx**2 + ddy**2)
    print(f'Frequency: {f:.3f} Hz, Velocity: {v:.3f} m/s, Acceleration: {a:.3f} m/s^2')

    return [heading,x,y,z] 
fast_circle.ti = -1.0

def drag_trajectory(t:float, min_height:float=0.4, max_height:float=1.0, 
                        max_displacement:float=1.0):
    '''
    Execute a series of waypoints at maximum velocity along the edges and diagonals of a square.
    ### Parameters
    - `t` is the current time
    - `wp` is a list of waypoints in NED [heading,x,y,z]
    - `min_height` defines a minimal altitude > 0
    - `max_height` defines a maximal altitude > min_height
    - `max_displacement` defines the maximum displacement from the origin
    ### Return 
    - Pose target in NED [heading,x,y,z]
    '''
    # Parameters
    d = max_displacement
    h = (max_height-min_height)/2

    dt = 2.0 # duration of each segment

    # Waypoints 
    wp = np.array([
        [0,p_init[0],p_init[1],-min_height,dt],
        [0,-d, d,-h,dt], 
        [0,-d,-d,-h,dt],
        [0, d, d,-h,dt],
        [0,-d, d,-h,dt],
        [0, d,-d,-h,dt],
        [0, d, d,-h,dt],
        [0,-d,-d,-h,dt],
        [0, d,-d,-h,dt],
        [0,p_init[0],p_init[1],-min_height,dt]
    ])
    if not drag_trajectory.init:
        drag_trajectory.init = True
        minjerktraj.init(mean_vel=2.0)
        minjerktraj.integrate_waypoints(wp,wp_vel=2.0,loop=False)
        drag_trajectory.t_start = t
    #return _execute_waypoints(t, wp, min_height,smooth=True)
    return minjerktraj(t-drag_trajectory.t_start)
drag_trajectory.init = False
drag_trajectory.t_start = 0.0

def thrust_trajectory(t:float, min_height:float=0.4, max_height:float=1.0, 
                        max_displacement:float=1.0):
    '''
    Execute a series of waypoints at maximum velocity along the edges and diagonals of a square.
    ### Parameters
    - `t` is the current time
    - `wp` is a list of waypoints in NED [heading,x,y,z]
    - `min_height` defines a minimal altitude > 0
    - `max_height` defines a maximal altitude > min_height
    - `max_displacement` defines the maximum displacement from the origin
    ### Return 
    - Pose target in NED [heading,x,y,z]
    '''

    target = rand_smooth(t, a=0.5, min_height=min_height, max_height=max_height,v=0,T=1.5) 
    target[0] = 0.0 # face forward
    return target 

def execute_waypoints(t:float, min_height:float=0.4):
    '''
    Execute waypoints (yaw,x,y,z,duration) in a sequence. 
    '''
    global waypoints

    #target = _execute_waypoints(t, waypoints[:,:-1], min_height, smooth=True)
    target = minjerktraj(t)
    return target

traj_df = pd.DataFrame() # Trajectory to replay 
ti  = 0.0 # Initial time of the simulation for replay 
def set_replay(traj_file:str, t:str, x:str, y:str, z:str, yaw:str, t_init:float, period:float=0.008, nwu2ned=True) -> None:
    '''
    Load trajectory data from a csv file to be replayed.

    The time will be set to start at 0s and the drone is assumed to be 
    on the ground at the beginning of the trajectory. 

    ### Arguments
    - `traj_file` : trajectory file with x,y,z,yaw and time data
    - `t`: name of the time column in s
    - `x`: name of the x data column in m
    - `y`: name of the y data column in m
    - `z`: name of the z data column in m
    - `yaw`: name of the yaw data column in rad 
    - `t_init`: initial time of the simulation
    - `period` : sampling period of the simulation used for resampling 
    - `nwu2ned` : whether NWU to NED conversion should be performed 
    '''
    global traj_df, ti, p_init

    ti = t_init 

    T = int(period*1e3)

    # Extract time and pose 
    df = pd.read_csv(traj_file)[[t,x,y,z,yaw]]

    # Convert NWU to NED if required 
    if nwu2ned:
        df[y] *= -1
        df[z] *= -1
        df[yaw] *= -1

    # Set starting position at origin and time to zero 
    df[t] -= df[t][0]
    df[x] -= df[x][0]
    df[y] -= df[y][0]
    df[z] -= df[z][0]

    # Resample data to match the simulation time step 
    df['timedelta'] = pd.to_timedelta(df[t],unit='s')
    df = df.set_index('timedelta')
    df = df.resample(f'{T}ms',axis='index').mean().dropna()
    df[t] = df.index.total_seconds()
    df = df.reset_index().drop(columns='timedelta')

    # Rearrange columns 
    cols = [t,yaw,x,y,z]
    traj_df = df[cols]
    traj_df = traj_df.rename(columns={t:'t',x:'x',y:'y',z:'z',yaw:'yaw'})

    print('Loaded trajectory file:\n\t',traj_file)

def is_replay_ready():
    '''
    Indicate whether the replay is ready. Otherwise `set_replay()` must be called.
    '''
    global traj_df
    return traj_df.size > 0

def is_replay_over(t:float) -> bool:
    '''
    Indicate whether the replay is complete based on current time `t`
    '''
    return t > traj_df.iloc[-1][0] # is current time beyond time of last stored pose 

def replay(t:float) -> np.ndarray:
    '''
    Get the next pose along the replayed trajectory. 
    The method `set_replay()` must be called first. 
    ### Arguments
    - `t`: current time 
    ### Return 
    - Pose target in NED [heading,x,y,z]
    '''
    global traj_df, ti

    # Find the previous closest matching position
    tar_ = traj_df[traj_df['t']<=t-ti].iloc[-1]
    if is_replay_over(t-ti) and not replay.warned:
        print('Trajectory replay complete') 
        replay.warned = True 
    return tar_[1:5].to_numpy()
replay.warned=False 



