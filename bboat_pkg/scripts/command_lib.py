"""
Author: Luiz Henrique Marques Gonçalves
Creation Date: May - July 2024

Description:
This library was developed during my Projet de Recherche at ENSTA Bretagne from May to July 2024.
It provides a series of functions and control laws used for analyzing various nonlinear control techniques for autonomous maritime systems.
"""

import numpy as np
from scipy.interpolate import CubicSpline

# Model dependent
M = np.array([[80.2815,     0,       0],
             [       0, 157.5,      11],
             [       0,    11, 41.9327]], dtype=float)


# Model dependent
def C(v_state):
    u,v,r = v_state.flatten()
    Xudot = -5.2815
    Yvdot = -82.5

    c = np.array([[0         ,         0,  Yvdot * v],
                  [0         ,         0, -Xudot * u],
                  [-Yvdot * v, Xudot * u,          0]], dtype=float) 
    return c
      
# Model dependent      
def D(v):
    Xu = -77.5544
    Yv = -157.5000
    Nr = -41.9327
    return -np.array([[Xu, 0, 0],
                      [0, Yv, 0],
                      [0, 0, Nr]], dtype=float)

# Used in Matrix H command strategy
tracking_point_distance = 1

# Used in Matrix H command strategy
H = np.array([[1,0,0],
              [0,0,0],
              [0,1/tracking_point_distance,1]], dtype=float)

# Used in Matrix H command strategy
T = np.array([[1,0,0],
              [0,1,tracking_point_distance],
              [0,0,1]], dtype=float)

# Rotation Matrix
def J(n):
    phi = float(n[2])
    return np.array([[np.cos(phi), -np.sin(phi), 0],
                    [np.sin(phi),  np.cos(phi), 0],
                    [0, 0, 1]], dtype=float)


def command_h(n_state, target, d_target, state_error_integral, dt):

    # Calculate the new estimated state considering the tracking point distance
    ne_state = n_state + np.array([[tracking_point_distance*np.cos(n_state[2,0])], 
                                   [tracking_point_distance*np.sin(n_state[2,0])], 
                                   [0]])
    

    # Invert matrices T and J(ne_state)
    invT = np.linalg.inv(T)
    invJ = np.linalg.inv(J(ne_state))
    
    # Calculate the state error and update the integral of the state error
    state_error = target - ne_state
    state_error_integral += state_error * dt

    # print(state_error[0],state_error[1])


    # Proportional and integral gains
    Kp_state = np.array([[0.8], [0.8], [0]])
    Ki_state = np.array([[0], [0], [0]])

    # Calculate the correction term
    n_correction = d_target + Kp_state * state_error + Ki_state * state_error_integral

    # Calculate the reference velocity
    vref = H @ invT @ invJ @ n_correction

    return vref[0], vref[2], state_error_integral

def get_path_points():

    # Initial x and y coordinates
    x = np.array([32, 29, 22, 27, 32, 29, 22])
    y = np.array([-20, -10, 0, 10, 20, 30, 40])

    # Cubic spline interpolation
    cs = CubicSpline(y, x)

    # Generate 10,000 evenly spaced points and interpolate x
    y = np.linspace(y[0], y[-1], 1000)
    x = cs(y)

    # Calculate distances and arc length
    distances = np.sqrt(np.diff(x)**2 + np.diff(y)**2)
    s = np.zeros_like(x)
    s[1:] = np.cumsum(distances)
    ds = np.gradient(s)

    # Calculate derivatives
    dx = np.gradient(x) / ds
    dy = np.gradient(y) / ds
    ddx = np.gradient(dx) / ds
    ddy = np.gradient(dy) / ds

    # Calculate orientation angle and curvature
    phi_f = np.arctan2(dy, dx)
    curvature = (dx * ddy - dy * ddx) / (dx**2 + dy**2)**1.5
    dc = np.gradient(curvature)
    g_c = dc / ds

    # Return results
    return np.vstack((x, y, s, phi_f, curvature, g_c, dx, dy, ddx, ddy))

def path_interrogation(s_in, path_points):

    # Initialize the index and the current arc length value
    indice = 0
    s_values = path_points[2]
    s_atual = s_values[indice]

    # Find the index where s_values exceeds s_in
    while s_atual < s_in:
        indice += 1
        s_atual = s_values[indice]

    # Retrieve the path points at the found index
    res = path_points[:, indice]

    return res

def get_target_traj(s, dt, path_points):
    # Speed along the path
    ds = 1
    
    # Update the path parameter 's' based on the time step 'dt' and step increment 'ds'
    s = s + ds * dt

    # Interrogate the path at the updated 's' to obtain various parameters
    xs, ys, _, phif, curv, g_c, dx, dy, ddx, ddy = path_interrogation(s, path_points)

    # Create the final position vector, velocity and acceleration in path direction.
    final_position = np.array([[xs], [ys], [0]], dtype=float)
    first_derivative = np.array([[ds * dx], [ds * dy], [0]], dtype=float)
    second_derivative = np.array([[ds * ddx], [ds * ddy], [0]], dtype=float)

    # Return the final position, first derivative, second derivative, and updated path parameter 's'
    return final_position, first_derivative, second_derivative, s

def command_auv_model(v_state, error_state, u_target, path_points):

    u, v, r = v_state.flatten()  # Linear velocities (u, v) and angular velocity (r)
    s1, y1, phi, s = error_state.flatten()  # Errors in along-track (s1), cross-track (y1), orientation (phi), and path parameter (s)

    # Gains for control laws
    k1 = 1
    k2 = 1
    k3 = 1
    k4 = 1
    k5 = 1

    # Coriolis and damping matrices
    Cv = C(v_state)
    Dv = D(v_state)

    # Derivatives of velocities
    du = -k4 * (u - u_target)  
    dv = (-Cv[1, 2] * r - Dv[1, 1] * v) / M[1, 1]  

    # Total speed and its derivative
    v_t = np.sqrt(u**2 + v**2)  
    d_vt = (u * du + v * dv) / v_t if v_t != 0 else 0  

    # Compute the derivative of the path parameter 's'
    ds = v_t * np.cos(phi) + k2 * s1
    if s == 0 and ds < 0: 
        ds = 0

    # Path properties in 's'
    xs, ys, _, phif, curv, g_c = path_interrogation(s, path_points)[:6]

    # Sideslip angle 'beta' and its derivative
    beta = np.arctan2(v, u)
    d_beta = (dv * u - du * v) / (u**2 + v**2) if v_t != 0 else 0

    # Drivative of the orientation 'phi'
    d_phi = r + d_beta - curv * ds

    # Derivatives of the error states
    d_s1 = -ds * (1 - curv * y1) + v_t * np.cos(phi)
    d_y1 = -curv * ds * s1 + v_t * np.sin(phi)
    dd_s = d_vt * np.cos(phi) - v_t * np.sin(phi) * d_phi + k2 * d_s1

    # Desired heading change 'delta' and its derivative
    k_delta = 2
    phi_a = np.pi / 2
    delta = -np.arctan(k_delta * y1)  # Desired heading change based on cross-track error
    redutor = 1 - np.tanh(0.1 * np.sqrt(s1**2 + y1**2))  # Reduces effect of delta change

    d_delta = -k_delta * d_y1 / (1 + (k_delta * y1)**2)
    # d_delta = d_delta * redutor  # Uncomment if reduction is needed (discussed in report)

    # Desired yaw rate 'rd'
    rd = d_delta - 1 * d_beta - k1 * sawtooth(phi - delta) + curv * ds

    # Return the target speed, desired yaw rate, path parameter derivative, and target coordinates
    return u_target, rd, ds, xs, ys

def initiate_error_state(n_state, v_state, s0, path_points):
    # Unpack velocity components
    u, v, _ = v_state.flatten()
    
    # Get initial path coordinates and heading
    xs, ys, _, phif = path_interrogation(s0, path_points)[:4]
    
    # Rotation matrix to align with path heading
    R = np.array([[np.cos(phif), np.sin(phif), 0],
                  [-np.sin(phif), np.cos(phif), 0],
                  [0, 0, 1]], dtype=float)
    
    # Calculate new state relative to path
    new_state = n_state + np.array([[-xs], [-ys], [-phif + np.arctan2(v, u)]])

    # Transform to error state
    s1, y1, phi = (R @ new_state).flatten()
    phi = sawtooth(phi)
    
    # Return error state
    error_state = np.array([[s1], [y1], [phi], [s0]])
    return error_state

def update_error_state(error_state, v_state, n_state, ds, dt_ctr, path_points):
    # Unpack velocity components
    u, v, _ = v_state.flatten()

    # Update the path parameter 's'
    s = error_state[3, 0] + ds * dt_ctr
    if s < 0: 
        s = 0  # Ensure 's' does not go negative

    # Get path coordinates and heading at the updated 's'
    xs, ys, _, phif = path_interrogation(s, path_points)[:4]
    
    # Rotation matrix to align with path heading
    R = np.array([[np.cos(phif), np.sin(phif), 0],
                  [-np.sin(phif), np.cos(phif), 0],
                  [0, 0, 1]], dtype=float)
    
    # Calculate new state relative to path
    new_state = n_state + np.array([[-xs], [-ys], [-phif + np.arctan2(v, u)]])
    s1, y1, phi = (R @ new_state).flatten()
    phi = sawtooth(phi)

    # Return updated error state
    return np.array([[s1], [y1], [phi], [s]])

def sawtooth(phi):
    while phi >  np.pi:
        phi -= 2 * np.pi
    while phi < -np.pi:
        phi += 2 * np.pi
    return phi

def get_target_los(n_state, path_points):
    # Find the closest point on the path to the current state
    closest = s_closest(n_state[0, 0], n_state[1, 0], path_points)

    # Get path coordinates and heading at the closest point
    xs, ys, _, phif = path_interrogation(closest, path_points)[:4]

    # Rotation matrix to align with path heading
    R = np.array([[np.cos(phif), np.sin(phif)], 
                  [-np.sin(phif), np.cos(phif)]], dtype=float)

    # Calculate the tangent direction at the closest point
    tanx = (np.linalg.inv(R) @ [1, 0]).flatten()[0]
    tany = (np.linalg.inv(R) @ [1, 0]).flatten()[1]
    tangent = np.array([[tanx], [tany]])

    # Compute the target point based on the tangent direction
    closest_point = np.array([[xs], [ys]])
    target = closest_point + 10 * tangent

    # Return the target point
    return target

def s_closest(x, y, path_points):
    # Extract all path parameter values 's'
    all_s = path_points[2, :]

    # Initialize variables to track the closest point
    index = 0
    s_closest = 0

    # Get initial path coordinates and compute initial distance
    xs = path_points[0, index]
    ys = path_points[1, index]
    distance_closest = np.sqrt((xs - x)**2 + (ys - y)**2)

    # Iterate over all path points to find the closest one
    for s in all_s:
        xs = path_points[0, index]
        ys = path_points[1, index]
        distance = np.sqrt((xs - x)**2 + (ys - y)**2)
        
        # Update closest point if a closer one is found
        if distance < distance_closest:
            s_closest = s
            distance_closest = distance
            
        index += 1

    # Return the path parameter 's' corresponding to the closest point
    return s_closest

def command_los(n_state, target, u_target, dt, error_integral):
    # Unpack the current state and target coordinates
    x, y, theta = n_state.flatten()
    xd, yd = target.flatten()[:2]

    # Compute the desired heading angle to the target
    desired = np.arctan2(yd - y, xd - x)
    
    # Calculate the heading error and adjust with sawtooth function
    theta_d = sawtooth(desired - theta)

    # Update the integral of the heading error
    error_integral += theta_d * dt

    # Define control gains
    k = 1
    ki = 1

    # Compute the control input for heading
    u2 = k * theta_d + ki * error_integral

    # Return the target speed, control input, and updated error integral
    return u_target, u2, error_integral

def command_fblin(n_state, v_state, k, dt_ctr):
    # Unpack current state and velocity components
    x, y, phi = n_state.flatten()
    u, v, r = v_state.flatten()

    # Return default control values if forward velocity is low
    if u < 0.5:
        return 0.5, 0

    # Compute Coriolis and damping matrices
    Cv = C(v_state)
    Dv = D(v_state)

    # Compute the derivative of velocity
    dv = (-Cv[1, 2] * r - Dv[1, 1] * v) / M[1, 1]

    # Create matrices A and B for linear control system
    A = np.array([[np.cos(phi), -u * np.sin(phi) - v * np.cos(phi)],
                  [np.sin(phi), u * np.cos(phi) - v * np.sin(phi)]])
    
    B = np.array([[-dv * np.sin(phi)], [dv * np.cos(phi)]])

    # Compute the control input
    inv = np.linalg.inv(A)  # Inverse of matrix A
    dif = k - B
    ref = inv @ dif

    # Update control input with respect to time step
    ref[0] = u + ref[0] * dt_ctr

    # Return control inputs
    return ref[0, 0], ref[1, 0]

def reconstruct_spline_matrix(response):
    # Reconstruct the matrix based on service responses
    x = np.array(response.x)
    y = np.array(response.y)
    s = np.array(response.s)
    phi_f = np.array(response.phi_f)
    curvature = np.array(response.curvature)
    g_c = np.array(response.g_c)
    dx = np.array(response.dx)
    dy = np.array(response.dy)
    ddx = np.array(response.ddx)
    ddy = np.array(response.ddy)
    
    # Stack the arrays vertically to form the final matrix
    spline_matrix = np.vstack((x, y, s, phi_f, curvature, g_c, dx, dy, ddx, ddy))
    return spline_matrix