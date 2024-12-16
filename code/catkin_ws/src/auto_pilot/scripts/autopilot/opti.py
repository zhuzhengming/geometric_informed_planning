#!/usr/bin/env python3
'''
Numba optimized functions. 

These functions are constently called by the auto-pilot to stabilize the drone. 
Numba pre-compiles them to reach C-like execution times, which allows to run
simulations much faster. 

If you do not want to use numba, comment out the numba import and all @jit decorators. 

Note: if numba complains that the installed colorama version is too old, upgrade it: 
>  pip3 install --upgrade colorama 
'''

import numpy as np
from numba import jit

@jit(nopython=True)
def saturate_scalar(u:float, lim_down:float, lim_up:float):
    """
    simple saturation of a signal between 2 limits
    """
    if u > lim_up:
        return lim_up
    elif u < lim_down:
        return lim_down
    else:
        return u

@jit(nopython=True)
def saturate_array(u:np.array, lim_down:float, lim_up:float):
    u[u > lim_up] = lim_up
    u[u < lim_down] = lim_down
    return u

@jit(nopython=True)
def restrict_angle(angle:float):
    """
    restrict the angle to be in the range [-pi, pi]
    """
    while angle > np.pi:
        angle -= 2 * np.pi
    while angle < -np.pi:
        angle += 2 * np.pi
    return angle

@jit(nopython=True)
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

@jit(nopython=True)
def _pid(y_c, y, kp, ki, kd, lim_down, lim_up, Ts, dy, integrator, differentiator, error_prev, y_c_prev, integLim=0.0):
    '''
    Optimized pid function 
    '''

    tau = 0.01  # filtering

    error = y_c - y

    integrator += error*Ts

    # differentiator
    if kd != 0 and np.isfinite(1 / kd):
        if np.isfinite(dy):
            dy_c = y_c - y_c_prev  # Compute command diff
            derror = dy_c - dy  # Compute error diff
            y_c_prev = y_c  # Update the command for next step
        else:
            derror = error - error_prev  # Compute error diff
            error_prev = error  # Update the error for next step

        differentiator = (2 * tau - Ts) / (2 * tau + Ts) * differentiator + 2 / (2 * tau + Ts) * derror


    if integLim != 0.0:
        integrator = saturate_scalar(integrator, -integLim, integLim)
    if np.sign(integrator) != np.sign(error):
        integrator = 0

    u_unsat = kp * error + ki * integrator + kd * differentiator
    
    u = saturate_scalar(u_unsat, lim_down, lim_up)
    
    # anti-windup for `integrator`
    if ki != 0:
        integrator = integrator + Ts / ki * (u - u_unsat)

    return u, integrator, differentiator, error_prev, y_c_prev

@jit(nopython=True)
def _acceleration_to_attitude(m:float, an:float, ae:float, ad:float):
    """
    Convert desired acceleration into attitude and thrust in the world frame (NED). 

    ### Arguments
    - `m`  drone mass 
    - `an` accel north
    - `ae` accel east
    - `ad` accel down

    ### Return
    - attitude and thrust: [`phi`,`theta`,`thrust`]
    """
    if ad > 9.81:
        ad = 9.81  # because the drone cannot push downward

    g = 9.81
    F = m * np.array([an, ae, ad - g])
    thrust = np.linalg.norm(F)
    eps = 1e-10  # num. stability
    phi = np.arcsin(F[1] / (thrust + eps))
    theta = np.arcsin(-F[0] / (thrust + eps))
    
    return phi, theta, thrust

@jit(nopython=True)
def _detect_failure(phi:float, theta:float, attitude_max:float):
    '''
    Detect if the control failed. 
    ### Return
    - `True` if attitude is beyond allowed range. 
    '''
    if(np.abs(phi) >= attitude_max or np.abs(theta) >= attitude_max):
        return True  

    return False

@jit(nopython=True)
def _mixer(B:np.array, thrust:float, torque_phi:float, torque_theta:float, torque_psi:float):
    """
    Mix the torque and thrust input to a motor command, based on allocation matrix pseudo-inverse 
    
    ### Arguments
    - `B` allocation matrix pseudo-inverse
    - `thrust`
    - `torque_phi`
    - `torque_theta`
    - `torque_psi`

    ### Return
    - `U` motor speed commands 
    """

    T = np.array([thrust, torque_phi, torque_theta, torque_psi]).transpose()

    U_sq = B @ T

    U_sq[U_sq < 0] = 0  # some values are very small negative

    U = np.sqrt(U_sq)

    return U