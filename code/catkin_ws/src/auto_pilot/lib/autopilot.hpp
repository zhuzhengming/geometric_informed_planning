#pragma once 

#include <string>
#include <vector>

#define MODULE_NAME "auto_pilot"
#define PARAMETER_FILE "../config/default.fp" 

/**
 * Autopilot API namespace 
 */
namespace ap{

    /**
     * Initialize the python code. 
     * This imports the "auto_pilot" module.
     * @return whether the import was successful 
     */
    bool init();

    /**
     * Initialize the autopilot
     * @param autopilot_type desired control type ("Position","Velocity","Acceleration","Attitude","Rate","Force")
     * @param path configuration file path 
     */
    void init_autopilot(std::string autopilot_type = "Position", std::string path = "");

    /**
     * Set the auto-pilot control type
     * @param control_type "Position","Velocity","Acceleration","Attitude","Rate","Force"
     */
    void set_autopilot_type(std::string type="Position");

    /**
     * Load a configuration file
     * @param file_path path to the configuration file 
     */
    void load_flight_parameters(std::string file_path="");
    
    /**
     * Arm the drone 
     * @param cmd set to true to arm, false to disarm 
     */
    void arm(bool cmd=true);

    /**
     * Update the state with a new pose 
     * @param x     position in x
     * @param y     position in y
     * @param z     position in z
     * @param roll  attitude in x 
     * @param pitch attitude in y
     * @param yaw   attitude in z 
     * @param dt    time elapsed since last update 
     */    
    void update_state_with_pose(double x, double y, double z, double roll, double pitch, double yaw, double dt);
    
    /**
     * @brief Compute the motor control based on the command
     * 
     * ### Position (global)
     * `target` = [yaw, x, y, z]
     * ### Velocity (global)
     * `target` = [yaw, vx, vy, vz]
     * ### Acceleration (global)
     * `target` = [yaw, ax, ay, az] 
     * ### Attitude (local)
     * `target` = [roll, pitch, yaw]
     * ### Rate (local)
     * `target` = [roll_rate, pitch_rate, yaw_rate]
     * ### Force (local)
     * `target` = [roll_torque, pitch_torque, yaw_torque, thrust]
     * 
     * @param target set point 
     * @param superimpose (bool) whether to superimpose motor commands with previous 
     * 
     * @return list of motor velocities 
     */
    std::vector<double> get_u(const std::vector<double>& target, bool superimpose=false);
    
    /**
     * Code clean up (call before terminating)
     */
    void clean();
}