#!/usr/bin/env python3
'''
Transform a camera image into a sensor_msgs/Image 
'''

from controller import Supervisor, Camera

class CameraWebots:
    def __init__(self, robot:Supervisor, cam:str='tracking', fps:int=30) -> None:
        '''
        Enable the specified camera 
        '''
        self.robot = robot 
        self.timestep = int(self.robot.getBasicTimeStep())
        self.camera:Camera = robot.getDevice(cam) 
        self.fps = fps 
        
    def enable(self): 
        self.camera.enable(self.timestep)

    def disable(self):
        self.camera.disable()

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
    
    def get_image(self):
        '''
        (SLOW) Get the image from the camera as array 
        of RGB values `list<list<list<int>>>`
        '''
        return self.camera.getImageArray()

try: 
    import rospy
    from sensor_msgs.msg import Image

    def to_image_msg(camera:CameraWebots, t:float, frame:str='camera') -> Image:
        '''
        Convert a Webots camera image to a ROS Image message

        Parameters
        ----------
        - `camera`: Webots camera sensor
        - `t`: Current time in seconds
        - `frame`: Frame ID of the camera

        Returns
        -------
        - `ros_image`: ROS Image message
        '''

        # Retrieve image data from Webots camera sensor
        width = camera.camera.getWidth()
        height = camera.camera.getHeight()

        # Retreive the image data from the camera
        # Encoded as four bytes representing the blue, green, red and alpha (BGRA8) levels of a pixel
        # see https://cyberbotics.com/doc/reference/camera?tab-language=c#wb_camera_get_image
        image_data = camera.camera.getImage() # with size in bytes of camera_width * camera_height * 4 

        # Create ROS Image message
        ros_image = Image()
        ros_image.header.stamp = rospy.Time.from_sec(t)
        ros_image.header.frame_id = frame
        ros_image.width = width
        ros_image.height = height
        ros_image.step = width * 1 * 4 # image width, 1 byte per channel (8 bits), 4 channels (BGRA)
        ros_image.encoding = 'bgra8'
        ros_image.data = image_data
        
        return ros_image 
except:
    print('[camera_webots.py] ROS not installed')