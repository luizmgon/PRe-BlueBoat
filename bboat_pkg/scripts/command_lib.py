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

tracking_point_distance = 1

H = np.array([[1,0,0],
              [0,0,0],
              [0,1/tracking_point_distance,1]], dtype=float)
T = np.array([[1,0,0],
              [0,1,tracking_point_distance],
              [0,0,1]], dtype=float)

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
    Kp_state = np.array([[1], [1], [0]])
    Ki_state = np.array([[0], [0], [0]])

    # Calculate the correction term
    n_correction = d_target + Kp_state * state_error + Ki_state * state_error_integral

    # Calculate the reference velocity
    vref = H @ invT @ invJ @ n_correction

    return vref[0], vref[2], state_error_integral


def get_path_points():

    # Initial x and y coordinates
    # x = np.array([2, -3, -8, -3, 2, -3, -8])
    x = np.array([32, 29, 22, 27, 32, 29, 22])

    y = np.array([-20, -10, 0, 10, 20, 30, 40])


    # Cubic spline interpolation
    cs = CubicSpline(y, x)

    # Generate 10,000 evenly spaced points and interpolate x
    y = np.linspace(y[0], y[-1], 10000)
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

def get_target_traj(time, path_points):
    ds = 2
    s = ds * time

    xs, ys, _, phif, curv, g_c, dx, dy, ddx, ddy = path_interrogation(s, path_points)

    final_position = np.array([[xs], [ys], [0]], dtype=float)
    first_derivative = np.array([[ds * dx], [ds * dy], [0]], dtype=float)
    second_derivative = np.array([[ds * ddx], [ds * ddy], [0]], dtype=float)

    return final_position, first_derivative, second_derivative

def command_auv_model(v_state, error_state, u_target, path_points):
    # State
    u,v,r = v_state.flatten()
    s1, y1, phi, s = error_state.flatten()

    # print(v_state)

    # Gains
    k1 = 1
    k2 = 1
    k3 = 1
    k4 = 1
    k5 = 1

    Cv = C(v_state)
    Dv = D(v_state)

    # Speed derivatives
    du = -k4*(u - u_target)
    dv = (-Cv[1,2]*r - Dv[1,1]*v)/M[1,1]
   
    v_t = np.sqrt(u**2 + v**2)
    d_vt = (u*du + v*dv) / v_t if v_t != 0 else 0


    ds = v_t * np.cos(phi) + k2 * s1
    # print(ds)


    # ds = np.clip(ds, -5, 5)

    if(s == 0 and ds < 0): ds = 0

    # xs, ys, phif, curv, dcurv_dt, g_c = path_desc(s, ds,s_values, mu_values, phi_values)
    xs, ys, _, phif, curv, g_c = path_interrogation(s, path_points)[:6]

    # dcurv_dt = 0

    #Beta
    beta = np.arctan2(v, u)
    d_beta = (dv*u - du*v) / (u**2 + v**2) if v_t != 0 else 0

    d_phi = r + d_beta - curv * ds

    # Error state derivatives
    d_s1 = -ds*(1-curv*y1) + v_t*np.cos(phi)
    d_y1 = -curv * ds * s1 + v_t * np.sin(phi)
    dd_s = d_vt * np.cos(phi) - v_t * np.sin(phi) * d_phi + k2 * d_s1
    # dd_s = np.clip(dd_s, -5, 5)

    if(dd_s > 10000):
        a = 1

    # Delta derivatives
    k_delta = 1
    phi_a = np.pi/2
    delta = -np.arctan(k_delta * y1)
    redutor = 1 - np.tanh(0.1* np.sqrt(s1**2 + y1 ** 2))

    d_delta = -k_delta * d_y1 / (1 + (k_delta * y1)**2)
    # d_delta = d_delta * redutor

    rd = d_delta - 1*d_beta - k1 * sawtooth(phi - delta) + curv * ds

    return u_target, rd, ds, xs, ys

def initiate_error_state(n_state, v_state, s0, path_points):
    u,v,_ = v_state.flatten()
    xs, ys, _ , phif = path_interrogation(s0, path_points)[:4]
    
    R = np.array([[np.cos(phif), np.sin(phif), 0],
                    [-np.sin(phif),  np.cos(phif), 0],
                    [0, 0, 1]], dtype=float)
    
    new_state = n_state + np.array([[-xs],[-ys],[-phif + np.arctan2(v , u)]])

    s1,y1,phi = (R @ new_state).flatten()
    phi = sawtooth(phi)
    error_state = np.array([[s1], [y1], [phi], [s0]])
    return error_state

def update_error_state(error_state,v_state, n_state, ds, dt_ctr, path_points):
    u,v,_ = v_state.flatten()

    s = error_state[3,0] + ds * dt_ctr

    if(s < 0): s = 0

    xs, ys, _ , phif = path_interrogation(s, path_points)[:4]
    
    R = np.array([[np.cos(phif), np.sin(phif), 0],
                    [-np.sin(phif),  np.cos(phif), 0],
                    [0, 0, 1]], dtype=float)
    
    new_state = n_state + np.array([[-xs],[-ys],[-phif + np.arctan2(v , u)]])
    s1,y1,phi = (R @ new_state).flatten()
    phi = sawtooth(phi)

    return np.array([[s1], [y1], [phi], [s]])

def sawtooth(phi):
    while phi >  np.pi:
        phi -= 2 * np.pi
    while phi < -np.pi:
        phi += 2 * np.pi
    return phi

def get_target_los(n_state, path_points):
    closest = s_closest(n_state[0,0], n_state[1,0], path_points)

    xs, ys, _, phif = path_interrogation(closest, path_points)[:4]

    R = np.array([[np.cos(phif), np.sin(phif)], 
                [-np.sin(phif), np.cos(phif)]],dtype=float,)

    tanx = (np.linalg.inv(R) @ [1, 0]).flatten()[0]
    tany = (np.linalg.inv(R) @ [1, 0]).flatten()[1]
    tangent = np.array([[tanx], [tany]])

    closest_point = np.array([[xs], [ys]])

    target = closest_point + 2 * tangent
    return target

def s_closest(x,y,path_points):
    all_s = path_points[2,:]

    index = 0
    s_closest = 0

    xs = path_points[0, index]
    ys = path_points[1, index]

    distance_closest = np.sqrt((xs-x)**2 + (ys-y)**2)

    for s in all_s:
        xs = path_points[0, index]
        ys = path_points[1, index]
        distance = np.sqrt((xs-x)**2 + (ys-y)**2)
        if(distance < distance_closest):
            s_closest = s
            distance_closest = distance
        index += 1

    return s_closest

def command_los(n_state, target, u_target):
    x,y,theta = n_state.flatten()
    xd, yd = target.flatten()[:2]

    desired = np.arctan2(yd - y, xd - x)
    theta_d = sawtooth(desired - theta)

    k = 1

    u2 = k * theta_d

    return u_target, u2

def command_fblin(n_state, v_state, k, dt_ctr):

    x,y,phi = n_state.flatten()
    u,v,r = v_state.flatten()

    if(u < 1): return 1, 0

    Cv = C(v_state)
    Dv = D(v_state)

    dv = (-Cv[1,2]*r - Dv[1,1]*v)/M[1,1]

    A = np.array([[np.cos(phi), -u*np.sin(phi)-v*np.cos(phi)],
                  [np.sin(phi), u*np.cos(phi)-v*np.sin(phi)]])
    
    B = [[-dv*np.sin(phi)], [dv*np.cos(phi)]]
   
    inv = np.linalg.inv(A)
    dif = (k - B)
    ref = inv @ dif

    ref[0] = u + ref[0] * dt_ctr


    return ref[0,0], ref[1,0]

def reconstruct_spline_matrix(response):
		# Reconstruir a matriz com base nas respostas do serviço
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
		
		# Empilhar as matrizes verticalmente para formar a matriz final
		spline_matrix = np.vstack((x, y, s, phi_f, curvature, g_c, dx, dy, ddx, ddy))
		return spline_matrix