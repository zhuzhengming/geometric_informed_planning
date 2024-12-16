# auto_pilot_pkg

This package implements a fully functional autopilot implemented in Python and optimized using `numba` to reach performances close to C code. 

## Installation
This repository is mostly implemented in Python. It only has a few dependencies that can be installed with the following script:
```bash
cd path/to/this/script
./init.sh
```
It will also add the requried environment variables at the end of your `~/.bashrc`. Don't forget to source your terminal again or launch a new terminal after completing this step! 

> **Note**: You may need to adapt the **python version** (default: `3.8`) or the **Webots installation path** (default: `/usr/local/bin/webots`). 

For more information about the environment variables set here, you can check Webots' [documentation](https://cyberbotics.com/doc/guide/running-extern-robot-controllers?tab-os=linux#introduction). 

### Dependencies
> Everything here is handled by the `.init.sh` script, but is presented for reference. 

The following python packages are required: 
- `numpy`
- `numba`
- `pandas`
- `scipy`
- `matplotlib`

They can be installed with: 
```bash
pip3 install <pkg>
```
You may need to upgrade the `colorama` package as well: 
```bash
pip3 install --upgrade colorama 
```

If `numba` fails to install, it is possible that the pip version is too low. It can be upgraded with: 
```bash
python3 -m pip install --upgrade pip
```
The `pip3 install numba`  command can then be run again and should succeed this time. 

The following ROS packages are required and can be installed with (replace `noetic` with your ROS version if needed, e.g. melodic, kinetic, ...):
```bash
sudo apt install ros-noetic-webots-ros
sudo apt install ros-noetic-mavros ros-noetic-mavros-extras
```

## Frame definitions
The autopilot works with the NED (FRD) convention. All pose information sent over ros is expressed in the NWU (FLU) convention and is converted to NED by the autopilot_node before querying the auto-pilot. Webots uses NWU (FLU) as well. 

## Available drone models
This package implements the following drone models (defined as Webots protos):
- m500
- Starling
- Crazyflie 

## Running provided examples
### `flying_arena.wbt`
This world contains several drone models. The m500, the Crazyflie and the Starling all use the auto-pilot and follow a figure-eight trajectory. The DJI Mavic uses its default controller with keyboard interactions. The generic drone model runs a simple open loop controller that lets it hover if no disturbance is applied. 

### `flying_arena_ros.wbt`
This world demonstrates how to use the auto-pilot with ROS.

Open the world `flying_arena_ros.wbt` in Webots and start all nodes at once with: 
```bash
# m500 - controller set to <none> by default
roslaunch auto_pilot_pkg all.launch flight_parameters:=m500.fp trajectory:=eight imu_id:=1
# starling - controller set to <extern> by default 
roslaunch auto_pilot_pkg all.launch flight_parameters:=starling.fp trajectory:=eight imu_id:=2
```
This includes the nodes:
- **drone_controller_node.py** (extern controller, publishes sensor and pose data, subscribes to the `/autopilot/motor_command` topic to send motor commands)
- **autopilot_node.py** (ROS interface to the auto-pilot, sends `MotorCommand` messages on the `/autopilot/motor_command` topic)
- **trajectory_node.py** (publishes position targets to follow a pre-defined trajectory (default: figure eight))

> **Note**: When using the `ros` controller, make sure to start the launch file before starting the simulation to make sure the `ros` controller also uses simulation time (ROS parameter `/use_sim_time` set to `true` within the launch file).

### Command line controls 
Following calls can be sent to arm the drone and send targets to it from the terminal: 
```bash
# Arm the drone (let the rotor start spinning)
rosservice call /mavros/cmd/arming "value: true"

# Send a position target for the drone to track (hover 1 meter above the ground)
rostopic pub /mavros/setpoint_raw/local mavros_msgs/PositionTarget "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
coordinate_frame: 0
type_mask: 0
position: {x: 0.0, y: 0.0, z: 1.0}
velocity: {x: 0.0, y: 0.0, z: 0.0}
acceleration_or_force: {x: 0.0, y: 0.0, z: 0.0}
yaw: 0.0
yaw_rate: 0.0" 

# Send a land command (can be unstable)
rosservice call /mavros/cmd/land "{min_pitch: 0.0, yaw: 0.0, latitude: 0.0, longitude: 0.0, altitude: 0.0}" 
```