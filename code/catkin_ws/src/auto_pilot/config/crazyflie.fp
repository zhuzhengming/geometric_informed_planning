### Flight parameters file for the Crazyflie drone model
# Hover is achieved by setting the motor velocity to ~ 55.5 rad/s 

autopilot_control_type "Position"

### Physical parameters

# drone physical parameters
m                       0.05            # Drone mass in kg 
l                       0.031           # Distance from CoM to rotor axis along x-axis in m

# motor constants
c_t                     4e-05           # Thrust constant of motor
c_q                     2.4e-06         # Torque constant of motor
vel_prop_max            600             # Rotor max velocity in rad/s 
rotor_sign              [-1,1,-1,1]     # Rotation direction of the rotors 

# simulation parameters
dt                      0.008           # Simulation timestep in s (will be overwritten)

# failure thresholds
attitude_max            1.2             # Maximum allowed attitude 

 
### PID gains and limits 

# positiom to velocity (P controller)
k_pn_vn                 [0.7,0,0.01]
k_pe_ve                 [0.7,0,0.01] 
k_pd_vd                 [0.7,0,0.01]
vn_max                  1
ve_max                  1
vd_max                  [-3,1]

# velocity to acceleration (PD controller)
k_vn_an                 [6,0,0.1] 
k_ve_ae                 [6,0,0.1] 
k_vd_ad                 [6,0,1] 
an_max                  1
ae_max                  1
ad_max                  2

# attitude to rate  (PD controller)
k_phi_p                 [3,0,0.0]
k_theta_q               [3,0,0.0]
k_psi_r                 [3,0,0.0]
p_max                   220
q_max                   220
r_max                   200

# rate to force (PID controller)
k_p_torque              [0.01,0.01,0.001] 
k_q_torque              [0.01,0.01,0.001]
k_r_torque              [0.01,0.01,0.001]
k_p_int_lim             0.3
k_q_int_lim             0.3 
k_r_int_lim             0.3
