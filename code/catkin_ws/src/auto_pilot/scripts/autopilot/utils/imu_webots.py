#!/usr/bin/env python3 

from controller import Supervisor, Accelerometer, Gyro 

import rospy 
from sensor_msgs.msg import Imu 

class IMU:
    '''
    IMU class to combine accelerometer and gyro data from Webots into 
    a single measurement. 
    '''
    def __init__(self, robot:Supervisor):
        self.robot = robot 
        self.angular_velocity = [0.0,0.0,0.0]
        self.linear_acceleration = [0.0,0.0,0.0]
        self.timestep = int(self.robot.getBasicTimeStep())

    def enable(self, id:str='') -> None:
        '''
        Enable the accelerometer and gyroscope devices, designated by the provided `id`.
        Their names must be respectively `accelerometer` and `gyroscope` followed by the `id`. 

        Parameters
        ----------
        - `id` : id of the IMU (empty string if only one IMU is present)
        '''
        self.accelerometer:Accelerometer = self.robot.getDevice('accelerometer'+id)
        self.gyroscope:Gyro              = self.robot.getDevice('gyroscope'+id)
        if self.accelerometer is None or self.gyroscope is None:
            return 
        self.accelerometer.enable(self.timestep)
        self.gyroscope.enable(self.timestep)

    def get_measurement(self) -> list:
        '''
        Get the measurement from the accelerometer and gyroscope
        and return it as a list of 6 floats.

        Return
        ------
        - list of 6 floats [ax,ay,az,wx,wy,wz]
        '''
        if self.accelerometer is None or self.gyroscope is None:
            return [] 
        self.angular_velocity = self.gyroscope.getValues()
        self.linear_acceleration = self.accelerometer.getValues() 
        return self.linear_acceleration + self.angular_velocity

    def project_axes(self, proj:list=[0,1,2], sign:list=[1,1,1]):
        '''
        Project axis if the IMU is not in FLU configuration. 
        The `proj` argument defines which axis is projected 
        on the FLU x,y,z axes.

        For instance, RFU gives proj=[1,0,2] and sign=[1,-1,1] 
        to project back onto FLU 

        ### Arguments
        - `proj` : projection to apply 
        - `sign` : axis sign 
        '''
        tmp_ = self.angular_velocity[:] # pass by value, not ref
        self.angular_velocity[0] = tmp_[proj[0]]*sign[0]
        self.angular_velocity[1] = tmp_[proj[1]]*sign[1]
        self.angular_velocity[2] = tmp_[proj[2]]*sign[2]
        
        tmp_ = self.linear_acceleration[:] # pass by value, not ref
        self.linear_acceleration[0] = tmp_[proj[0]]*sign[0] 
        self.linear_acceleration[1] = tmp_[proj[1]]*sign[1] 
        self.linear_acceleration[2] = tmp_[proj[2]]*sign[2] 

    def is_enabled(self) -> bool: 
        return not (self.accelerometer is None or self.gyroscope is None)

    def to_imu_msg(self, t:float) -> Imu:
        '''
        Convert current measurement into a ROS Imu message (sensors_msg.msg.Imu)
        at specified time `t` in seconds.

        Parameters
        ----------
        - `t` : time in seconds

        Return
        ------
        - ROS Imu message (sensors_msg.msg.Imu)
        '''
        msg = Imu()
        msg.header.stamp = rospy.Time.from_sec(t) 
        msg.header.frame_id = 'base_link' 
        msg.angular_velocity.x = self.angular_velocity[0]
        msg.angular_velocity.y = self.angular_velocity[1]
        msg.angular_velocity.z = self.angular_velocity[2]
        msg.linear_acceleration.x = self.linear_acceleration[0]
        msg.linear_acceleration.y = self.linear_acceleration[1]
        msg.linear_acceleration.z = self.linear_acceleration[2]
        return msg 
    
    accelerometer = None 
    gyroscope = None 