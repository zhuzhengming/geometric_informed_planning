#!/usr/bin/env python3

from math import atan2
import numpy as np
from matplotlib import pyplot as plt

TOL = 1e-4 # time tolerance for checking if two segments overlap

def adjust_yaw_target(yaw_target:float, yaw:float):
    """
    Adjust the `yaw_target` to be in the range [-pi, pi] of the current state `yaw`
    ### Returns
    - `yaw_target` : adjusted target yaw
    """
    while yaw_target - yaw > np.pi:
        yaw_target -= 2 * np.pi
    while yaw_target - yaw < -np.pi:
        yaw_target += 2 * np.pi
    return yaw_target

class MinJerkTraj:
    '''
    Class for generating trajectories with minimal jerk.

    This class can be used to generate trajectories with
    minimal jerk between waypoints. Each segment of the trajectory
    is a polynomial of degree 5, which is the minimum degree
    to ensure continuity of position, velocity, and acceleration.

    A series of waypoints can be provided to generate a trajectory, 
    providing following information in the `init()` function:
    - `mean_vel`: mean velocity of the trajectory
    - `vel_lim`: velocity limits
    - `acc_lim`: acceleration limits

    A list of waypoints can be interpolated using the `integrate_waypoints()`
    function, with the following arguments:
    - `waypoints`: list of waypoints (format: [heading, x, y, z, durartion])
    - `wp_vel`: velocity at each waypoint
    - `loop`: loop trajectory (time is automatically adjusted to ensure continuity)

    Individual segments between each pair of waypoint are represented 
    using an internal class `Segment`.

    Note the `duration` of the waypoints defines how long it takes to
    reach the waypoint from the previous one. The `duration` can also 
    be set to `np.nan`, in which case the trajectory will be generated with
    the mean velocity.
    '''

    def __init__(self) -> None:
        pass # use the init() method to initialize the trajectory

    def init(self,  mean_vel = 1.0, 
                    vel_lim = np.inf,
                    acc_lim = np.inf,
                    loop = False) -> None:
        self.mean_vel:float = mean_vel  # mean velocity of the trajectory
        self.vel_lim:float = vel_lim    # velocity limits
        self.acc_lim:float = acc_lim    # acceleration limits
        self.waypoints = np.array([])   # waypoints
        self.loop = loop                # loop trajectory
        self.segments = []              # list of minimal jerk segments

    minjerk_pos  = lambda t, a: a[0] + a[1]*t + a[2]*t**2 +   a[3]*t**3 +    a[4]*t**4  +    a[5]*t**5
    minjerk_vel  = lambda t, a:        a[1] + 2*a[2]*t    + 3*a[3]*t**2 +  4*a[4]*t**3  +  5*a[5]*t**4
    minjerk_acc  = lambda t, a:               2*a[2]      + 6*a[3]*t    + 12*a[4]*t**2  + 20*a[5]*t**3
    minjerk_jerk = lambda t, a:                             6*a[3]      + 24*a[4]*t     + 60*a[5]*t**2
    minjerk = [minjerk_pos, minjerk_vel, minjerk_acc, minjerk_jerk] # List of minjerk functions

    class Segment:
        '''
        Segment of a trajectory with minimal jerk from time t1 to t2.

        ### Attributes
        - `t1`: start time of the segment
        - `t2`: end time of the segment
        - `duration`: duration of the segment
        - `coeffs`: list of coefficients for each dimension of the trajectory (3x6) or (4x6).
        '''

        def __init__(self, t1:float, t2:float, coeffs:list):
            '''
            Store the coefficients of a trajectory with minimal jerk running from t1 to t2.
            The coefficients are stored for each dimension of the trajectory and
            are stored in a Segment object, with the order [x,y,z] or [x,y,z,h]. 

            Call the segment with the desired time to get the target position, velocity, acceleration, and jerk.

            ### Parameters
            - `t1`: start time of the segment
            - `t2`: end time of the segment
            - `coeffs`: list of coefficients for each dimension of the trajectory (3x6) or (4x6).
            '''
            assert t1 < t2+TOL, 't1 must be smaller than t2'
            assert len(coeffs) in [3,4] and len(coeffs[0]) == 6, 'coeffs must be a list of 3 or 4 lists of 6 coefficients'

            self.t1 = t1 
            self.t2 = t2 
            self.duration = t2 - t1
            self.coeffs = coeffs

        def __call__(self, t:float, d:int = 0) -> list:
            '''
            Evaluate the segment 

            ### Parameters
            - `t`: time at which to evaluate the trajectory between [t1,t2]
            - `d`: derivative to evaluate (0: position, 1: velocity, 2: acceleration, 3: jerk)

            ### Return
            The value of the trajectory at time `t` and derivative `d` as target [heading, x, y, z]
            '''
            t1 = self.t1
            t2 = self.t2
            assert t1-TOL <= t <= t2+TOL, f't={t:.3f} must be between {t1:.3f} and {t2:.3f}'

            f = MinJerkTraj.minjerk[d] # function to evaluate a derivative of the segment 

            x = f((t-t1),self.coeffs[0])
            y = f((t-t1),self.coeffs[1])
            z = f((t-t1),self.coeffs[2])

            if len(self.coeffs) == 4:
                h = f((t-t1),self.coeffs[3])
            elif len(self.coeffs) == 3 and d == 0:
                x_dot = MinJerkTraj.minjerk[1]((t-t1),self.coeffs[1])
                y_dot = MinJerkTraj.minjerk[1]((t-t1),self.coeffs[2])
                h = atan2(y_dot,x_dot) 
            else: 
                h = 0.0 # heading is not implicitly defined for velocity, acceleration, or jerk
            
            return [h,x,y,z] 
        
        def __eq__(self, other):
            '''
            Check if two segments overlap in time
            '''
            return other.t1 < self.t2 or self.t1 < other.t2
        def __lt__(self, other):
            '''
            Check if this segment ends before the other starts
            '''
            return self.t2 <= other.t1
        def __gt__(self, other):
            '''
            Check if this segment starts after the other ends
            '''
            return other.t2 <= self.t1
        
        def start(self,d:int=0) -> list:
            '''
            Get the `d`th derivative at start of the segment with format [heading, x, y, z]
            '''
            return self(self.t1,d)
        
        def end(self,d:int=0) -> list:
            '''
            Get the`d`th derivative at end of the segment with format [heading, x, y, z]
            '''
            return self(self.t2,d)
        
        def get_max(self,d:int) -> float:
            '''
            Get the maximum specified derivative of the segment
            ### Parameters
            - `d`: derivative to evaluate (0: position, 1: velocity, 2: acceleration, 3: jerk)
            ### Return
            The maximum value of the derivative of the segment
            '''
            return max([np.linalg.norm(self(t,d)[1:]) for t in np.arange(self.t1,self.t2,0.01)])

    def __call__(self, t:float, d:int = 0, nointerp:bool=False) -> list:
        '''
        Evaluate the trajectory at time `t` to get the target position, velocity, acceleration, or jerk.

        ### Parameters
        - `t`: time at which to evaluate the trajectory
        - `d`: derivative to evaluate (0: position, 1: velocity, 2: acceleration, 3: jerk)
        - `nointerp`: if True, do not interpolate between segments, only send the target of the current segment

        ### Return
        The value of the trajectory at time `t` as target [heading, x, y, z]
        '''

        # If the trajectory loops, get the time in the first loop
        while self.loop and t > self.get_duration():
            t -= self.get_duration()
        assert t >= 0, 't must be positive'

        # If the trajectory does not loop, stop at the end
        if not self.loop and t > self.get_duration():
            t = self.get_duration()

        if len(self.segments) == 0:
            print('No segments in trajectory')
            return [0.0,0.0,0.0,0.0]
        else:
            for segment in self.segments:
                if segment.t1 <= t <= segment.t2:
                    if nointerp:
                        return segment.end(d)
                    else:
                        return segment(t,d)
                
    def convert_duration_(self,duration) -> float:
        '''
        Convert a duration to a float. If the duration is not specified, return 0.0. 
        '''
        # When leaving empty the duration, the field can either be ' ' or NaN, which is handled here
        try:
            duration = float(duration)
            if np.isnan(duration):
                duration = 0.0
        except: 
            duration = 0.0
        return duration

    def integrate_waypoints(self, waypoints:np.ndarray, wp_vel:float = 0.0, loop:bool = False) -> None:
        '''
        Set the Waypoints the trajectory must go through. A velocity can be specified at each waypoint 
        to not stop at each waypoint. 
        
        The trajectory is computed over the time interval [0, self.get_duration()].

        ### Parameters
        - `waypoints`: List of NED waypoints defined as: [['yaw','x','y','z','duration'], ...]
        - `wp_vel`: velocity at the waypoints to not stop at each waypoint (direction is computed from the waypoints)
        - `loop`: If True, the trajectory will loop through the waypoints indefinitely.
        '''
        self.loop = loop
        self.waypoints = waypoints
        n = waypoints.shape[0]

        # Ensure heading deltas are in the range [-pi,pi] for shortest turn 
        for i in range(1,waypoints.shape[0]):
            waypoints[i,0] = adjust_yaw_target(waypoints[i,0], waypoints[i-1,0])

        # Compute the velocity vector at each waypoints
        velocities = np.zeros((waypoints.shape[0],4)) # [yaw,x,y,z]
        for i in range(0,n):

            if wp_vel == 0: break # no velocity specified, stop at each waypoint
            
            if not loop and i == 0: continue
            if not loop and i == waypoints.shape[0]-1: continue

            wp0 = waypoints[i-1,     1:4].astype('float')
            wp1 = waypoints[i,       1:4].astype('float')
            wp2 = waypoints[(i+1)%n, 1:4].astype('float')

            e0 = wp0 - wp1
            e1 = wp2 - wp1

            # Handle stationary waypoints or waypoints with indicated duration 
            if np.linalg.norm(e0) < 1e-6 or np.linalg.norm(e1) < 1e-6 \
                or self.convert_duration_(waypoints[i,4]) > 0.0:
                v = np.zeros(3)
            
            # General case
            else:
                e0 /= np.linalg.norm(e0)
                e1 /= np.linalg.norm(e1)
            
                if np.linalg.norm(e0+e1) < 1e-6:
                    v = e1 * wp_vel # if the waypoints are colinear, use the direction to the next waypoint
                else:
                    v = np.cross((e0+e1)/np.linalg.norm(e0+e1),np.cross(e0,e1))

                if np.dot(e0,e1) < 0:
                    v /= np.linalg.norm(v)

                if np.dot(v,e1) < 0:
                    v *= -1

                v *= wp_vel

            velocities[i,1:] = v
        
        # Compute the trajectory segments
        self.segments = []
        self.tot_duration = 0.0
        for i in range(0,n-1):
            
            # Get the duration of the segment from waypoints i to i+1
            duration = self.convert_duration_(waypoints[i+1,4])
            if duration == 0.0:
                duration = np.linalg.norm(waypoints[i+1,1:4] - waypoints[i,1:4]) / self.mean_vel
            if duration == 0.0:
                continue # the waypoint i+1 is redundant (same as previous)

            # Add the new segment between waypoint i and i+1 
            seg_start = self.segments[-1].t2 if len(self.segments) > 0 else 0.0
            self.segments.append(
                MinJerkTraj.Segment(
                    t1=seg_start,
                    t2=seg_start+duration,
                    coeffs=self._minimum_jerk_3D_coeffs(
                        x1=waypoints[i,(1,2,3,0)], x2=waypoints[i+1,(1,2,3,0)],
                        x1_dot=velocities[i,(1,2,3,0)], x2_dot=velocities[i+1,(1,2,3,0)],
                        x1_ddot=[0,0,0,0], x2_ddot=[0,0,0,0],
                        T=duration
                    ))
            )
            self.tot_duration += duration
        
        # Add a segment to close the loop 
        if loop:

            # Get the duration of the segment from the last waypoint to the first
            dist = np.linalg.norm(waypoints[-1,1:4] - waypoints[0,1:4]) 
            if dist < 1e-6: 
                duration = 1.0
            else: 
                duration = dist / self.mean_vel

            # define segment start time 
            seg_start = self.segments[-1].t2 if len(self.segments) > 0 else 0.0

            # Add the new segment between the last waypoint and the first (loop closure)
            self.segments.append(
                self.compute_segment(
                    t1=seg_start,
                    t2=seg_start + duration,
                    x1=waypoints[-1,(1,2,3,0)], x2=waypoints[0,(1,2,3,0)],
                    x1_dot=velocities[-1,(1,2,3,0)], x2_dot=velocities[0,(1,2,3,0)],
                    x1_ddot=[0,0,0,0], x2_ddot=[0,0,0,0]
                )
            )
            self.tot_duration += self.segments[-1].duration

        print('Waypoints integration complete')
        # print('resulting trajectory:')
        # for s in self.segments:
        #     print(s.t1, s.t2, s.duration)

    def compute_segment(self,
                        t1:float, t2:float, 
                        x1:list, x2:list, 
                        x1_dot:list=[0,0,0], x2_dot:list=[0,0,0],
                        x1_ddot:list=[0,0,0], x2_ddot:list=[0,0,0]) -> Segment:
        '''
        Compute the coefficients of a trajectory with minimal jerk from position x1 to x2,
        running from time t1 to t2. The specified derivatives must be provided as [x, y, z]
        OR [x, y, z, h] if the heading is specified.

        The coefficients are computed for each dimension of the trajectory and
        are stored in a Segment object, with the order [x,y,z] or [x,y,z,h]. 
        ## Arguments
        - `t1` time at start of the trajectory
        - `t2` time at end of the trajectory 
        - `x1` starting location
        - `x2` ending location 
        - `x1_dot` starting velocity 
        - `x2_dot` ending velocity 
        - `x1_ddot` starting acceleration
        - `x2_ddot` ending acceleration

        ## Returns
        - `segment` a Segment object containing the coefficients of the trajectory
        '''
        assert t1 < t2+TOL, 'Start time should be less than end time'
        assert len(x1) in [3,4], 'x1 should be [x,y,z] (len=3) or [x,y,z,h] (len=4), got len={}'.format(len(x1))

        T = t2-t1 
        
        # Get the coeffs, either 3x6 (x,y,z) or 4x6 (x,y,z,h) if heading specified
        coeffs = self._minimum_jerk_3D_coeffs(x1,x2,x1_dot,x2_dot,x1_ddot,x2_ddot,T)
        return MinJerkTraj.Segment(t1,t2,coeffs)
        

    def add_segment(self, segment:Segment) -> None:
        '''
        Add a segment at the end of an existing trajectory.
        ### Parameters
        - `segment`: Segment to add to the trajectory
        '''
        segment.t1 = self.segments[-1].t2 if len(self.segments) > 0 else 0.0
        segment.t2 = segment.t1 + segment.duration
        self.segments.append(segment)

    def get_duration(self) -> float:
        '''
        Get the total duration of the trajectory
        '''
        duration = 0.0
        for segment in self.segments:
            duration += segment.duration
        if len(self.segments) > 0:
            assert abs(self.segments[-1].t2 - duration) < 1e-3, f'Total duration does not match sum of segment durations, delta={abs(self.segments[-1].t2 - duration):.6f}s'
        return duration
    
    def get_max(self):
        '''
        Get the maximum absolute values of the trajectory's derivatives.
        ### Returns
        A tuple containing:
        - `max_pos` maximum position of the trajectory
        - `max_vel` maximum velocity of the trajectory
        - `max_acc` maximum acceleration of the trajectory
        - `max_jrk` maximum jerk of the trajectory
        '''
        max_pos = 0.0
        max_vel = 0.0
        max_acc = 0.0
        max_jrk = 0.0
        for segment in self.segments:
            s:MinJerkTraj.Segment = segment
            max_pos = max(max_pos, s.get_max(0))
            max_vel = max(max_vel, s.get_max(1))
            max_acc = max(max_acc, s.get_max(2))
            max_jrk = max(max_jrk, s.get_max(3))
        return max_pos, max_vel, max_acc, max_jrk
    
    def print_info(self) -> str:
        # Print trajectory info
        max_pos, max_vel, max_acc, max_jrk = self.get_max()
        info = 'Trajectory info:\n' + \
                '----------------\n' + \
                'Duration: {:2.3f} s\n'.format(self.get_duration()) + \
                'Mean vel: {:2.3f} m/s\n'.format(self.mean_vel) + \
                'Max pos:  {:2.3f} m\n'.format(max_pos) + \
                'Max vel:  {:2.3f} m/s\n'.format(max_vel) + \
                'Max acc:  {:2.3f} m/s^2\n'.format(max_acc) + \
                'Max jrk:  {:2.3f} m/s^3'.format(max_jrk)
        print('#'*30)
        print(info)
        print('#'*30)
        return info 
    
    def save(self, filename:str='/tmp/trajectory.yaml') -> None:
        '''
        Save the computed trajectory to a yaml file.
        ### Parameters
        - `filename`: filename to save the trajectory to
        '''
        try:
            import yaml
        except:
            print('Could not import yaml, make sure it is installed')
            return
        print('Saving trajectory to {}'.format(filename))

        data = {
            'mean_vel': self.mean_vel,
            'vel_lim': self.vel_lim,
            'acc_lim': self.acc_lim,
            'loop': self.loop,
            'segments': {}
        }

        for i,segment in enumerate(self.segments):
            data['segments'][i] = {
                't1': float(segment.t1),
                't2': float(segment.t2),
                'duration': float(segment.duration),
                'coeffs': {}
            }
            for j,d in enumerate(['x','y','z','h']):
                data['segments'][i]['coeffs'][d] = [c.tolist() for c in segment.coeffs[j]]
        
        # Save the trajectory to a yaml file
        with open(filename, 'w') as outfile:
            yaml.dump(data, outfile, default_flow_style=False)
    
    def load(self, filename:str='/tmp/trajectory.yaml') -> None:
        '''
        Load a trajectory from a yaml file.
        ### Parameters
        - `filename`: filename to load the trajectory from
        '''
        try:
            import yaml
        except:
            print('Could not import yaml, make sure it is installed')
            return
        print('Loading trajectory from {}'.format(filename))

        with open(filename, 'r') as infile:
            
            data = yaml.load(infile, Loader=yaml.FullLoader)
            
            self.init(mean_vel=data['mean_vel'], 
                      vel_lim=data['vel_lim'], 
                      acc_lim=data['acc_lim'],
                      loop=data['loop'])

            for s_id in data['segments']:
                s = data['segments'][s_id]
                t1 = s['t1']
                t2 = s['t2']
                coeffs = np.array([
                    s['coeffs']['x'],
                    s['coeffs']['y'],
                    s['coeffs']['z'],
                    s['coeffs']['h']
                ])
                segment = self.Segment(t1,t2,coeffs.tolist())
                self.segments.append(segment)
        
    
    def plot(self):
        '''
        Plot the trajectory and print relevant information.
        '''

        from copy import deepcopy

        # Plot the trajectory
        
        dt = 0.05/self.mean_vel
        T = np.arange(0,self.get_duration()+self.mean_vel/10, dt)
        h = [-self(t)[0] for t in T]
        x = [ self(t)[1] for t in T]
        y = [-self(t)[2] for t in T]
        z = [-self(t)[3] for t in T]

        fig = plt.figure(figsize=(20,10))

        shape = (4,3)

        if self.loop:
            T = np.arange(0,self.get_duration()*2, dt)

        ax = plt.subplot2grid(shape,(0,0),rowspan=4,projection='3d')
        # The waypoints may not be available if loading a trajectory from a file
        try:
            waypoints = deepcopy(self.waypoints[:,:-1]).astype('float')
            waypoints[:,(0,2,3)] *= -1
            ax.scatter(waypoints[:,1],waypoints[:,2],waypoints[:,3],c='r',marker='o',s=10, alpha=1.0, label='waypoints')
            ax.quiver(waypoints[:,1],waypoints[:,2],waypoints[:,3],np.cos(waypoints[:,0]),np.sin(waypoints[:,0]),0,length=0.2,normalize=True,color='r')
        except: pass 
        ax.plot(x,y,z)
        ax.quiver(x,y,z,np.cos(h),np.sin(h),0,length=0.1,normalize=True,color='b',label='heading')
        ax.set_title('Loaded trajectory')
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_zlabel('z')
        ax.legend()

        for d,l in zip([0,1,2,3], ['p','v','a','j']):

            f = np.array([self(t,d=d) for t in T])
            
            # Plot time series 
            ax = plt.subplot2grid(shape,(d,1))
            ax.plot(T,-f[:,0],label=f'${l}_h$',color='k')
            ax.plot(T, f[:,1],label=f'${l}_x$',color='r')
            ax.plot(T,-f[:,2],label=f'${l}_y$',color='g')
            ax.plot(T,-f[:,3],label=f'${l}_z$',color='b')
            ax.set_title(f'{l}')
            ax.set_xlabel('t')
            ax.set_ylabel(f'{l}')
            ax.grid()
            ax.legend()

            # Add segment lines
            for s in self.segments:
                ax.axvline(s.t1,c='k',ls='--',linewidth=0.5)

            # Plot histograms
            ax = plt.subplot2grid(shape,(d,2))
            ax.hist(np.abs(f[:,0]),alpha=0.2,label=f'${l}_h$',color='k')
            ax.hist(np.abs(f[:,1]),alpha=0.5,label=f'${l}_x$',color='r')
            ax.hist(np.abs(f[:,2]),alpha=0.5,label=f'${l}_y$',color='g')
            ax.hist(np.abs(f[:,3]),alpha=0.5,label=f'${l}_z$',color='b')
            ax.set_title(f'{l} histogram')
            ax.set_xlabel(f'{l}')
            ax.set_ylabel('count')
            ax.grid()
            ax.legend()

        plt.tight_layout()
        plt.show()

    def _minimum_jerk_1D_coeffs(self, 
                                x1:float, x2:float, 
                                x1_dot:float =0.0, 
                                x2_dot:float =0.0, 
                                x1_ddot:float=0.0, 
                                x2_ddot:float=0.0,
                                T:float = 1.0) -> np.ndarray: 
        '''
        1D minimum jerk trajectory coefficients from x1 to x2 with t in [0,T] 

        A function whose 6th derivative is zero minimizes the jerk. 
        Ref: http://courses.shadmehrlab.org/Shortcourse/minimumjerk.pdf 

        ### Parameters
        - `x1` starting location
        - `x2` ending location
        - `x1_dot` starting velocity
        - `x2_dot` ending velocity
        - `x1_ddot` starting acceleration
        - `x2_ddot` ending acceleration
        - `T` duration of the trajectory (default: 1 second)

        ## Return 
        List of coefficients of a 6th degree polynomial 
        '''

        assert T > 0, f'T must be positive, got {T}'
        
        a = np.zeros(6)

        a[0] = x1
        a[1] = x1_dot 
        a[2] = 0.5*x1_ddot

        if T == 1.0:
            Cinv = np.array([[ 10. ,  -4. ,   0.5],
                             [-15. ,   7. ,  -1. ],
                             [  6. ,  -3. ,   0.5]]) 
        else: 
            C = [[1*T**3, 1*T**4, 1*T**5],
                 [3*T**2, 4*T**3, 5*T**4],
                 [6*T**1,12*T**2,20*T**3] ]
            Cinv = np.linalg.inv(C)
        
        y = np.array([  [x2 - a[0] - a[1]*T - a[2]*T**2],
                        [x2_dot - a[1] - 2*a[2]*T],
                        [x2_ddot- 2*a[2]]
                    ]) 

        a[3:] = (Cinv@y).flatten() 

        return a  

    def _minimum_jerk_3D_coeffs(self,
                                x1:list, x2:list, 
                                x1_dot:list =[0,0,0], x2_dot:list =[0,0,0],
                                x1_ddot:list=[0,0,0], x2_ddot:list=[0,0,0],
                                T:float = 1.0) -> np.ndarray:
        '''
        Compute the coefficients for all dimensions of a minimum jerk trajectory with t in [0,T].
        If the provided arrays are 4D, the last (fourth) set of coefficients is the yaw angle.
        Otherwise, only three sets of coefficients are returned.
        ## Arguments 
        - `x1` starting location
        - `x2` ending location 
        - `x1_dot` starting velocity 
        - `x2_dot` ending velocity 
        - `x1_ddot` starting acceleration
        - `x2_ddot` ending acceleration
        - `T` duration of the trajectory (default: 1 second)
        ### Return
        List of coefficients for each dimension of the trajectory (x, y, z [,h]).
        '''
        # Compute polynomials 
        ax = self._minimum_jerk_1D_coeffs(x1[0],x2[0],x1_dot[0],x2_dot[0],x1_ddot[0],x2_ddot[0],T)
        ay = self._minimum_jerk_1D_coeffs(x1[1],x2[1],x1_dot[1],x2_dot[1],x1_ddot[1],x2_ddot[1],T)
        az = self._minimum_jerk_1D_coeffs(x1[2],x2[2],x1_dot[2],x2_dot[2],x1_ddot[2],x2_ddot[2],T)

        if len(x1) == 4:
            h  = self._minimum_jerk_1D_coeffs(x1[3],x2[3],x1_dot[3],x2_dot[3],x1_ddot[3],x2_ddot[3],T)
            return np.vstack((ax,ay,az,h))
        
        return np.vstack((ax,ay,az))

    
    # DEPRECATED 
    def _minimum_jerk(self, 
                      t:float, 
                      t1:float, t2:float, 
                      x1:list, x2:list, 
                      x1_dot:list=[0,0,0], x2_dot:list=[0,0,0],
                      x1_ddot:list=[0,0,0], x2_ddot:list=[0,0,0]):
        '''
        DEPRECATED Create a smooth trajectory with minimal jerk from x1 to x2. 
        ## Arguments
        - `t1` time at start of the trajectory
        - `t2` time at end of the trajectory 
        - `x1` starting location
        - `x2` ending location 
        - `x1_dot` starting velocity 
        - `x2_dot` ending velocity 
        '''
        assert t1 < t2 and t1 <= t and t <= t2, 'Invalid time specified'

        ax,ay,az = self._minimum_jerk_3D_coeffs(x1,x2,x1_dot,x2_dot,x1_ddot,x2_ddot) 

        x = MinJerkTraj.minjerk_pos((t-t1)/(t2-t1),ax)
        y = MinJerkTraj.minjerk_pos((t-t1)/(t2-t1),ay)
        z = MinJerkTraj.minjerk_pos((t-t1)/(t2-t1),az)

        x_dot = MinJerkTraj.minjerk_vel((t-t1)/(t2-t1),ax)
        y_dot = MinJerkTraj.minjerk_vel((t-t1)/(t2-t1),ay)
        h = atan2(y_dot,x_dot) 

        return [h,x,y,z] 

# Create a class instance 
minjerktraj = MinJerkTraj()
minjerktraj.init() # initialize the class


##########
## Test ##
##########

def plot_min_jerk():

    N = 30
    phi = np.linspace(0,np.pi,N,endpoint=False)
    wp0 = np.array([-1.0,0.0,0.0])
    wp1 = np.array([0.0,0.0,0.0])
    wp2 = np.stack((np.cos(phi),np.sin(phi),np.zeros_like(phi)))*1.1
    vel = np.zeros_like(wp2)
    v1  = np.stack((np.cos(phi),np.sin(phi)))*1.0
    ind = 20

    for i in range(N):

        e0 = wp0 - wp1
        e1 = wp2[:,i] - wp1
        
        e0 /= np.linalg.norm(e0)
        e1 /= np.linalg.norm(e1)

        if np.linalg.norm(e0+e1) < 0.1:
            v = e1 * 1.0 # if the waypoints are colinear, use the direction to the next waypoint
        else:
            v = np.cross((e0+e1)/np.linalg.norm(e0+e1),np.cross(e0,e1))
        
        if np.dot(e0,e1) < 0:
            v /= np.linalg.norm(v)

        if np.dot(v,e1) < 0:
            v *= -1   

        vel[:,i] = v

    from autopilot.utils.min_jerk_traj import MinJerkTraj

    tr = MinJerkTraj()
    tr.init()
    tr.integrate_waypoints(np.array(
        [
            [0.0,*wp0,1.0],
            [0.0,*wp1,np.nan],
            [0.0,*wp2[:,ind],1.0],
        ]
    ), wp_vel=1.0)
    tr.print_info()
    t = np.linspace(0,tr.get_duration(),100)
    path = np.zeros((2,len(t)))
    for i,t_ in enumerate(t):
        path[:,i] = np.array([tr(t_)[1],tr(t_)[2]])

    plt.figure(figsize=(6,3))
    plt.scatter(wp0[0],wp0[1],c='r',marker='o', alpha=1.0, label='$\mathbf{w}_0$')
    plt.scatter(wp1[0],wp1[1],c='g',marker='o', alpha=1.0, label='$\mathbf{w}_1$')
    plt.scatter(wp2[0],wp2[1],c='b',marker='o', alpha=0.5, label='$\mathbf{w}_2$')
    plt.scatter(wp1[0]+vel[0],wp1[1]+vel[1],c='k',marker='x', label='$\mathbf{v}_1$')
    plt.plot(v1[0,:],v1[1,:],'k--',linewidth=0.5,label='$v_1$')
    plt.plot(path[0,:],path[1,:],'m-',linewidth=1.5,label='Path')
    
    # Plot a path and velocity vector
    plt.arrow(wp0[0],wp0[1], wp1[0]-wp0[0],wp1[1]-wp0[1],ec='red',linewidth=2.0)
    plt.arrow(wp1[0],wp1[1], wp2[0,ind]-wp1[0],wp2[1,ind]-wp1[1],fc='white',ec='green',linewidth=2.0)
    plt.arrow(wp1[0],wp1[1], vel[0,ind],vel[1,ind],fc='white',ec='black')

    # Link all wp2 with their associated velocity vector
    for i in range(N):
        plt.arrow(vel[0,i],vel[1,i], wp2[0,i]-vel[0,i],wp2[1,i]-vel[1,i],fc='white',ec='blue',alpha=0.2)

    plt.xlabel('x [m]')
    plt.ylabel('y [m]')
    plt.xlim([-1.7,1.2])
    # plt.ylim([-0.1,1.5])
    plt.legend(loc='upper left')
    plt.tight_layout()


if __name__ == '__main__':

    plot_min_jerk()

    ref_vel = 1.0 # m/s

    mjt = MinJerkTraj()
    mjt.init(mean_vel=ref_vel)
    
    # # Plot effect of various end velocities
    # plt.figure()
    # r = 5
    # t_ = np.arange(0,1.01,0.01)
    # for i,x in enumerate(range(r)):
    #     a = mjt._minimum_jerk_1D_coeffs(0,1,0,x)
    #     plt.plot(t_,MinJerkTraj.minjerk_pos(t_,a),c=[i/r,0,(r-i)/r,1],label=f'{x} m/s')
    # plt.title('minimum jerk polynomial with various end velocities')
    # plt.xlabel('t')
    # plt.ylabel('x')
    # plt.legend()

    # Plot trajectory generated from list of waypoints (yaw,x,y,z,duration)
    waypoints = np.array([
        [0,         0,0,1,  np.nan],
        [np.pi/2,   1,0,1,  np.nan],
        [-np.pi/2,  2,0,1,  np.nan],
        [-np.pi/2,  2,0,1,  2],    # hold pose for 2 seconds
        [np.pi/2,   0,1,2,  np.nan],
        [-np.pi/2,  0,1,1,  np.nan],
    ])
    waypoints = np.array([
        [1* np.pi/2, 0,0,0,  np.nan],
        [1* np.pi/2, 0,0,1,  np.nan],
        [1*-np.pi/2, 1,0,0,  np.nan],
        [1*-np.pi/2, 1,0,1,  np.nan],
        [1* np.pi/2, 0,1,0,  np.nan],
        [1* np.pi/2, 0,1,1,  np.nan],
        [1*-np.pi/2, 1,1,0,  np.nan],
        [1*-np.pi/2, 1,1,1,  np.nan],
    ])
    # waypoints = np.concatenate((
    #     np.random.uniform(-np.pi,np.pi,(10,1)), # heading
    #     np.random.uniform(-1,1,(10,1)),         # x
    #     np.random.uniform(-1,1,(10,1)),         # y
    #     np.random.uniform(-1,1,(10,1)),         # z
    #     np.ones((10,1))*np.nan                  # duration
    # ),axis=1)
    waypoints[:,1]*=2   # scale x
    waypoints[:,2]*=1   # scale y
    waypoints[:,3]*=1   # scale z
    waypoints[:,3]+=1   # shift z

    loop = True
    mjt.integrate_waypoints(waypoints,wp_vel=ref_vel,loop=loop)

    mjt.save()
    mjt.load()

    mjt.plot()
