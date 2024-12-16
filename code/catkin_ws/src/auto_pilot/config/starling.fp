### Flight parameters file for the VOXL m500 drone model
# Tuned with:
# - basicTimeStep: 8 ms 
# - defaultDamping NULL 

autopilot_control_type "Position"

### Physical parameters

# drone physical parameters
m                       0.295           # Drone mass in kg 
l                       0.065           # Distance from CoM to rotor axis along x-axis in m

# motor constants
c_t                     0.0000103       # Thrust constant of motor
c_q                     0.000000113     # Torque constant of motor
vel_prop_max            1100            # Rotor max velocity in rad/s 
rotor_sign              [1,1,-1,-1]     # Rotation direction of the rotors 

# simulation parameters
dt                      0.008           # Simulation timestep in s (will be overwritten)

# failure thresholds
attitude_max            1.5             # Maximum allowed attitude 

 
### PID gains and limits 

# positiom to velocity (P controller)
k_pn_vn                 [1,0,0]
k_pe_ve                 [1,0,0] 
k_pd_vd                 [1,0,0]
vn_max                  1.0
ve_max                  1.0          
vd_max                  [-1.0,1.0]

# velocity to acceleration (PI controller)
k_vn_an                 [4,0.1,0] 
k_ve_ae                 [4,0.1,0] 
k_vd_ad                 [4,0.1,0] 
an_max                  0.5
ae_max                  0.5
ad_max                  0.5

# attitude to rate  (P controller)
k_phi_p                 [10,0,0]
k_theta_q               [10,0,0]
k_psi_r                 [5,0,0]
p_max                   0.5
q_max                   0.5
r_max                   0.5

# rate to force (PID controller)
k_p_torque              [0.032,0,0.0001] 
k_q_torque              [0.032,0,0.0001]
k_r_torque              [0.01,0,0]
k_p_int_lim             0.3
k_q_int_lim             0.3 
k_r_int_lim             0.1
