#!/usr/bin/python3
'''
Control a drone in simulation. 

Subscribed topics:
- "mavros/local_position/pose", type: geometry_msgs.msg.PoseStamped,  frame: NWU 
- "mavros/setpoint_raw/local",  type: mavros_msgs.msg.PositionTarget, frame: NWU 

Advertised services:
- "mavros/cmd/arming", type: mavros.command.CommandBool
- "mavros/cmd/land",   type: mavros.command.CommandTOL

Published topics:
- "/autopilot/motor_command", type: auto_pilot_pkg.msg.MotorCommand
- "/autopilot/thrust",        type: geometry_msgs.msg.WrenchStamped

This node takes care of converting NWU data into NED before feeding it to the auto-pilot. 
'''

import sys 
from scipy.spatial.transform import Rotation 

import rospy 
from geometry_msgs.msg import PoseStamped, WrenchStamped
from mavros_msgs.msg import PositionTarget  
from mavros_msgs.srv import CommandTOL, CommandBool 

from auto_pilot_pkg.msg import MotorCommand
from auto_pilot_pkg.srv import ComputeControl, ComputeControlRequest, ComputeControlResponse
from autopilot.main import autopilot, BasicPose, ColTxt

# Pose of the drone NED
pose = BasicPose() 

# Control command for the autopilot 
target = [0,0,0,0]
motor_cmd_pub = None 
thrust_pub = None 

# Landing variables
landing_in_progress = False 
altitude_after_arm = 0.0 # global NED 

#############
##  Utils  ##
#############
def poseStamped2basicPose(pose:PoseStamped) -> BasicPose:
    '''
    Convert a PoseStamped object to a BasicPose object 
    '''
    
    bpose = BasicPose()
    
    bpose.x = pose.pose.position.x
    bpose.y = pose.pose.position.y
    bpose.z = pose.pose.position.z

    r = Rotation.from_quat([pose.pose.orientation.x,
                            pose.pose.orientation.y,
                            pose.pose.orientation.z,
                            pose.pose.orientation.w])
    bpose.roll, bpose.pitch, bpose.yaw = r.as_euler('xyz')

    return bpose  

#############
# Callbacks #
#############
def pose_cb(msg:PoseStamped):
    '''
    Perform the control each time a new pose is received. 
    The pose is received as NWU and is converted to NED. 
    '''
    global pose, target

    pose = poseStamped2basicPose(msg) 
    # Convert pose from NWU to NED (180° rotation around x axis)
    pose.y, pose.z = -pose.y, -pose.z 
    pose.pitch, pose.yaw = -pose.pitch, -pose.yaw

    # Compute elapsed time since last pose update 
    curr_time = msg.header.stamp.to_sec() 
    if(pose_cb.prev_time is None):
        pose_cb.prev_time = rospy.get_time()
    dt = curr_time - pose_cb.prev_time 

    # Compute control 
    autopilot.state.update_with_pose(pose,dt)
    if not autopilot.armed:
        return 
    u = autopilot.get_u(target)

    # Publish control 
    motor_cmd = MotorCommand()
    motor_cmd.header.stamp = rospy.Time.now()
    motor_cmd.motor_speeds = [u[0],u[1],u[2],u[3]]

    motor_cmd_pub.publish(motor_cmd)

    # Publish thrust
    thrust_msg = WrenchStamped()
    thrust_msg.header.stamp = rospy.Time.now()
    thrust_msg.wrench.force.z = autopilot.thrust 

    thrust_pub.publish(thrust_msg)

    pose_cb.prev_time = curr_time 
pose_cb.prev_time = None # make the variable static 

def positionTarget2Target(msg:PositionTarget) -> list:
    global target, landing_in_progress

    # Ignore commands if landing is in progress 
    if(landing_in_progress):
        return target 

    # Maintain yaw if completely ignored 
    if(msg.type_mask & msg.IGNORE_YAW):
        msg.yaw = pose.yaw 
    
    # Track position 
    if( not msg.type_mask & msg.IGNORE_PX and \
        not msg.type_mask & msg.IGNORE_PY and \
        not msg.type_mask & msg.IGNORE_PZ):
        autopilot.set_autopilot_type("Position")
        target = [-msg.yaw,msg.position.x,-msg.position.y,-msg.position.z] # [yaw, x, y, z]

    # Track velocity 
    elif(not msg.type_mask & msg.IGNORE_VX and \
         not msg.type_mask & msg.IGNORE_VY and \
         not msg.type_mask & msg.IGNORE_VZ):
        autopilot.set_autopilot_type("Velocity")
        target = [-msg.yaw,msg.velocity.x,-msg.velocity.y,-msg.velocity.z] # [yaw, vx, vy, vz]
    
    # Track acceleration 
    elif(not msg.type_mask & msg.IGNORE_AFX and \
         not msg.type_mask & msg.IGNORE_AFY and \
         not msg.type_mask & msg.IGNORE_AFZ and \
         not msg.type_mask & msg.FORCE):
        autopilot.set_autopilot_type("Acceleration")
        target = [-msg.yaw,msg.acceleration_or_force.x,-msg.acceleration_or_force.y,-msg.acceleration_or_force.z] # [yaw, ax, ay, az]

    else: 
        print(ColTxt.FAIL+"[autopilot_node] Error: mavros/setpoint_raw/local has bad type mask"+ColTxt.ENDC)
    
    return target 

def cmd_cb(msg:PositionTarget):
    '''
    Set the target based on received PositionTarget message.
    The target is received as NWU and is converted to NED. 
    '''
    positionTarget2Target(msg)

def arm_cb(req:CommandBool._request_class):
    global motor_cmd_pub, altitude_after_arm, state 

    # Publish control 
    motor_cmd = MotorCommand()
    motor_cmd.header.stamp = rospy.Time.now()

    if(req.value):
        motor_cmd.motor_speeds = [100,100,100,100] 
        autopilot.arm(True) 
        altitude_after_arm = autopilot.state.pd
        rospy.loginfo("[autopilot_node] armed")
    else:
        motor_cmd.motor_speeds = [0,0,0,0] 
        autopilot.arm(False)
        rospy.loginfo("[autopilot_node] disarmed")

    if motor_cmd_pub is not None: 
        motor_cmd_pub.publish(motor_cmd)

    return [True, True] # ['success', 'result']

def land_cb(req:CommandTOL._request_class):
    global state, target, landing_in_progress 

    landing_in_progress = True 

    autopilot.set_autopilot_type("Velocity")
    target = [req.yaw,0,0,0.5]

    while True:
        if(autopilot.detect_landing() or autopilot.state.pd >= altitude_after_arm):
            autopilot.arm(False)
            landing_in_progress = False 
            rospy.loginfo("[autopilot_node] landing complete")
            return [True, True] # ['success', 'result']

def control_cb(req:ComputeControlRequest) -> ComputeControlResponse:
    '''
    Perform the control based on the provided pose and target. 
    Both are received as NWU and are converted to NED.

    Almost identical to `pose_cb()` 
    '''
    global pose, target

    stamp = req.pose.header.stamp 

    #
    # Compute target from NWU to NED 
    #
    target = positionTarget2Target(req.target) 

    #
    # Compute pose from NWU to NED 
    #
    pose = poseStamped2basicPose(req.pose) 
    # Convert pose from NWU to NED (180° rotation around x axis)
    pose.y, pose.z = -pose.y, -pose.z 
    pose.pitch, pose.yaw = -pose.pitch, -pose.yaw

    #
    # Compute elapsed time since last pose update 
    #
    curr_time = stamp.to_sec() 
    if(control_cb.prev_time is None):
        control_cb.prev_time = rospy.get_time()
    dt = curr_time - control_cb.prev_time 

    #
    # Compute control 
    #
    autopilot.state.update_with_pose(pose,dt)
    if not autopilot.armed:
        u = [0,0,0,0]
        return ComputeControlResponse(u)
    u = autopilot.get_u(target)

    control_cb.prev_time = curr_time 

    return ComputeControlResponse(u)

control_cb.prev_time = None # make the variable static 

#############
##  Main   ##
#############
def main():
    global motor_cmd_pub, thrust_pub

    # Initialize the rosnode 
    rospy.init_node("autopilot_node", anonymous=False)  
    rospy.loginfo("Starting auto-pilot node")

    # Get the /use_service parameter
    ns:str = rospy.get_namespace()
    if ns == '/': 
        ns = ''
    node:str = rospy.get_name() 
    use_service:bool = rospy.get_param(ns+node+'/use_service',False)

    # Initialize autopilot 
    autopilot.init("Position")
    if(len(sys.argv) > 1):
        autopilot.load_flight_parameters(path=sys.argv[1])

    if not use_service:
        rospy.loginfo("[autopilot_node] using Subscribers/Publishers")

        # Subscribe to state estimate 
        rospy.Subscriber('mavros/local_position/pose', PoseStamped, callback=pose_cb, queue_size=1, tcp_nodelay=True) 

        # Subscribe to setpoints 
        rospy.Subscriber('mavros/setpoint_raw/local', PositionTarget, callback=cmd_cb, queue_size=1, tcp_nodelay=True)

        # Advertise motor commands 
        motor_cmd_pub = rospy.Publisher('autopilot/motor_command',MotorCommand, queue_size=1)

        # Advertise thrust
        thrust_pub = rospy.Publisher('autopilot/thrust',WrenchStamped, queue_size=1)
    else:
        rospy.loginfo("[autopilot_node] using ComputeControl service")

        # Advertise a control service  
        rospy.Service("/autopilot/compute_control",ComputeControl, control_cb)
    
    # Advertise services 
    rospy.Service("mavros/cmd/arming", CommandBool, arm_cb)
    rospy.Service("mavros/cmd/land", CommandTOL, land_cb)
    

    rospy.spin() 

if __name__ == "__main__":
    main()