### Flight parameters file for the VOXL m500 drone model
# Linear and angular damping set to 0.2 

autopilot_control_type "Position"

### Physical parameters

# drone physical parameters
m                       1.37            # Drone mass in kg 
l                       0.167           # Distance from CoM to rotor axis along x-axis in m

# motor constants
c_t                     0.0000103       # Thrust constant of motor
c_q                     0.000000113     # Torque constant of motor
vel_prop_max            1100            # Rotor max velocity in rad/s 
rotor_sign              [1,1,-1,-1]     # Rotation direction of the rotors 

# simulation parameters
dt                      0.008           # Simulation timestep in s (will be overwritten)

# failure thresholds
attitude_max            1.2             # Maximum allowed attitude 

 
### PID gains and limits 

# positiom to velocity (P controller)
k_pn_vn                 [1.8,0,0]
k_pe_ve                 [1.8,0,0] 
k_pd_vd                 [1.8,0,0]
vn_max                  2
ve_max                  2           
vd_max                  [-3,1]

# velocity to acceleration (PI controller)
k_vn_an                 [5,0,0] 
k_ve_ae                 [5,0,0] 
k_vd_ad                 [5,0,0] 
an_max                  4
ae_max                  4
ad_max                  4

# attitude to rate  (P controller)
k_phi_p                 [7,0,0]
k_theta_q               [7,0,0]
k_psi_r                 [1,0,0]
p_max                   220
q_max                   220
r_max                   200

# rate to force (PID controller)
k_p_torque              [0.1,0.1,0.005] 
k_q_torque              [0.1,0.1,0.005]
k_r_torque              [0.05,0.01,0]
k_p_int_lim             0.3
k_q_int_lim             0.3 
k_r_int_lim             0.3
