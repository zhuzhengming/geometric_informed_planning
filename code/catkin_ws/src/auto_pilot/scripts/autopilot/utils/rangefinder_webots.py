#!/usr/bin/env python3
'''
Emulate a ToF camera using Webots' RangeFinder sensor.
This file implements a TimeOfFlight class that converts the depth image
from a RangeFinder into a point cloud.

Default ToF characteristics
---------------------------
ModalAI ToF camera specs: https://www.modalai.com/products/voxl-dk-tof 
- Rate 	5 - 45FPS in configurable option modes for distance / accuracy / framerate
- Exposure Time 	4.8 ms typ. @ 45 fps / 30 ms typ. @ 5 fps
- Resolution 	224 x 171 (38k) px
- FOV (H x V) 	117° x 117° (near field 100° x 80°)
- Range 	4 - 6m
- Depth Resolution 	<= 1% of distance (0.5 - 4m @ 5fps) <= 2% of distance (0.1 - 1m @ 45fps)
'''


from numba import njit 
import numpy as np 
import struct 

from controller import Supervisor, RangeFinder

@njit 
def depth_image_to_pc(k_matrix:np.ndarray, depth_image:np.ndarray) -> np.ndarray:
    '''
    Parameters
    ----------
    - `k_matrix` : 3x3 camera intrinsics matrix
    - `depth_image` : 2D array of depth values (in meters)
    '''

    inv_fx = 1.0 / k_matrix[0, 0]
    inv_fy = 1.0 / k_matrix[1, 1]
    ox = k_matrix[0, 2]
    oy = k_matrix[1, 2]
    image_height, image_width = depth_image.shape
    points = np.zeros((image_width * image_height * 3), dtype=np.float32)
    counter = 0
    for y in range(image_height):
        for x in range(image_width):
            dist = depth_image[y, x]
            points[counter + 1] = -np.float32((x - ox) * dist * inv_fx)
            points[counter + 2] = -np.float32((y - oy) * dist * inv_fy)
            points[counter + 0] = np.float32(dist)
            counter += 3
    return points.astype(np.float32)


class TimeOfFlight():
    '''
    Emulate a Time of Flight (ToF) sensor from the range-finder sensor. 
    '''
    def __init__(self, robot:Supervisor, name:str='range-finder', fps:int=5) -> None:
        self.robot = robot 
        self.timestep = int(robot.getBasicTimeStep())
        self.range_finder:RangeFinder = self.robot.getDevice(name)
        self.fps = fps
        self.k_matrix = self.get_calibration_matrix()

    def enable(self) -> None:
        self.range_finder.enable(self.timestep)  

    def disable(self) -> None:
        self.range_finder.disable()

    def ready(self, t:float) -> bool:
        '''
        Check if the frame rate is respected and a new measurement 
        is available before querying the camera. 
        Note, the measurement is only available for the timestep
        during which this method returns true.
        '''
        if not hasattr(self, 'last_time'):
            self.last_time = t
            self.is_ready = False
        
        # Check the rate 
        is_rate_ok = (t - self.last_time) >= 1/self.fps
        
        # The rate is ok and the camera is not ready, enable it for next frame
        if is_rate_ok and not self.is_ready: 
            self.enable()
            self.is_ready = True 
            return False 
        
        # The rate is ok and the camera is already enabled, data can be queried
        elif is_rate_ok and self.is_ready:
            self.last_time = t
            self.is_ready = False
            return True 
        
        # The rate is not ok, disable the camera for next frame
        elif not is_rate_ok:
            self.disable()
            self.is_ready = False
            return False 

        else: 
            raise ValueError('Unexpected case in camera ready check.')

    def get_calibration_matrix(self) -> np.ndarray:
        image_width = self.range_finder.getWidth()
        image_height = self.range_finder.getHeight()
        focal_length = 0.5 * image_width * (1 / np.math.tan(0.5 * self.range_finder.getFov()))
        k_matrix = np.array([
            [focal_length, 0, image_width / 2],
            [0, focal_length, image_height / 2],
            [0, 0, 0]
        ])
        return k_matrix

    def get_depth_image(self) -> np.ndarray:
        '''
        Return a height x width depth image in meters. 
        '''
        depth_image = np.asarray(self.range_finder.getRangeImage(data_type='list'), dtype=np.float32)
        return depth_image.reshape((-1, self.range_finder.getWidth()))

    def get_pointcloud(self, flat:bool=True) -> np.ndarray:
        '''
        Return a point cloud in the camera frame. 
        
        Can return the pointcloud in two formats:
        - If `flat=True`(default), return as flat array of floats (x, y, z, x, y, z, ...).
        - If `flat=False`, return as 2D array of floats (x, y, z) with shape (height*width, 3).
        '''
        pc = depth_image_to_pc(self.k_matrix, self.get_depth_image())
        if flat:
            return pc 
        else:
            return pc.reshape((-1, 3))


# Pointcloud to PointCloud2 ROS message conversion 
try:
    import rospy
    from sensor_msgs.msg import PointCloud2, PointField

    def to_pointcloud2_msg(tof:TimeOfFlight, t:float, frame:str='tof') -> PointCloud2:
        '''
        Get the depth image from the RangeFinder sensor and convert it to PointCloud2 message

        Parameters
        ----------
        - `tof` : TimeOfFlight object
        - `t` : current time in seconds
        - `frame` : frame name of the PointCloud2 message

        Returns
        -------
        - `msg` : PointCloud2 message
        '''
        
        pc = tof.get_pointcloud()

        msg = PointCloud2()

        msg.header.stamp = rospy.Time.from_sec(t) 
        msg.header.frame_id = frame

        msg.height:int = tof.range_finder.getHeight()
        msg.width:int  = tof.range_finder.getWidth()

        msg.fields.append(PointField(name='x',offset= 0,datatype=PointField.FLOAT32,count=1))
        msg.fields.append(PointField(name='y',offset= 4,datatype=PointField.FLOAT32,count=1))
        msg.fields.append(PointField(name='z',offset= 8,datatype=PointField.FLOAT32,count=1))

        #msg.is_bigendian = 
        msg.point_step = 3*4 # bytes (float32)
        msg.row_step = msg.width * msg.point_step

        msg.data = struct.pack('%sf' % len(pc), *pc)

        msg.is_dense = True 

        return msg 
except:
    print('[rangefinder_webots.py] ROS not installed')