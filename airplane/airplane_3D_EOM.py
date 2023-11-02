import numpy as np
from scipy.integrate import solve_ivp
from scipy.optimize import fsolve
import yaml



# these rotation matrixs take the angle between the earth and the body frame. 
# V_b = Rot_x V_e
# the rotation matrix can give you a vector in base frame if given vector in earth frame

# to get vector in earth frame given vector in base frame
# V_e = (Rot_x)^T V_b


def Rot_x(theta):
    rot_x = np.array([[1, 0, 0],
                      [0, np.cos(theta), np.sin(theta)],
                      [0, -np.sin(theta), np.cos(theta)]])
    return rot_x

def Rot_y(theta):
    rot_y = np.array([[np.cos(theta), 0, -np.sin(theta)],
                      [0, 1, 0],
                      [np.sin(theta), 0, np.cos(theta)]])
    return rot_y

def Rot_z(theta):
    rot_z = np.array([[np.cos(theta), np.sin(theta), 0],
                      [-np.sin(theta), np.cos(theta), 0],
                      [0, 0, 1]])
    return rot_z

def calculate_aero_forces(u,v,w, p,q,r, T,elevator,aileron, rudder, params):
    

    V_magnitude = np.sqrt(u*u + v*v + w*w)

    if w == 0 and u == 0:
        alpha = 0
    else:
        alpha = np.arctan2(w,u)

    if v == 0:
        beta = 0
    else:
        beta = np.arcsin(v/V_magnitude)

    # long coef and forces
    if V_magnitude == 0:
        L = 0
        D = 0
        M_moment = 0
        C_M_moment = 0
    else:
        C_L = params['long_coef']['C_L_0'] + params['long_coef']['C_L_alpha'] * alpha + \
            params['long_coef']['C_L_q']*(params['physical']['c_wing_chord']/(2 *V_magnitude))*q +\
            params['long_coef']['C_L_elevator'] * elevator
        
        C_D = params['long_coef']['C_D_0'] + params['long_coef']['C_D_alpha'] * alpha + \
            params['long_coef']['C_D_q']*(params['physical']['c_wing_chord']/(2 *V_magnitude))*q +\
            params['long_coef']['C_D_elevator'] * elevator
        
        C_M_moment = params['long_coef']['C_M_moment_0'] + params['long_coef']['C_M_moment_alpha'] * alpha + \
            params['long_coef']['C_M_moment_q']*(params['physical']['c_wing_chord']/(2 *V_magnitude))*q +\
            params['long_coef']['C_M_moment_elevator'] * elevator
        
    
        L = 0.5 * params['environmental']['rho_air_density'] * V_magnitude**2 * params['physical']['Surface_area_wings'] * C_L
        D = 0.5 * params['environmental']['rho_air_density'] * V_magnitude**2 * params['physical']['Surface_area_wings'] * C_D
        M_moment = 0.5 * params['environmental']['rho_air_density'] * V_magnitude**2 * params['physical']['Surface_area_wings']\
                    * params['physical']['c_wing_chord'] * C_M_moment
        
        # lat coef
        if V_magnitude == 0:
            F_y = 0
            L_moment = 0
            N_moment = 0
        else:

            C_y = params['lat_coef']['C_y_0'] + params['lat_coef']['C_y_beta'] * beta + \
                params['lat_coef']['C_y_p']*(params['physical']['b_wing_span']/(2 *V_magnitude))*p +\
                params['lat_coef']['C_y_r']*(params['physical']['b_wing_span']/(2 *V_magnitude))*r +\
                params['lat_coef']['C_y_aileron'] * aileron + params['lat_coef']['C_y_rudder'] * rudder
            
            C_L_moment = params['lat_coef']['C_L_moment_0'] + params['lat_coef']['C_L_moment_beta'] * beta + \
                params['lat_coef']['C_L_moment_p']*(params['physical']['b_wing_span']/(2 *V_magnitude))*p +\
                params['lat_coef']['C_L_moment_r']*(params['physical']['b_wing_span']/(2 *V_magnitude))*r +\
                params['lat_coef']['C_L_moment_aileron'] * aileron + params['lat_coef']['C_L_moment_rudder'] * rudder
            
            C_N_moment = params['lat_coef']['C_N_moment_0'] + params['lat_coef']['C_N_moment_beta'] * beta + \
                params['lat_coef']['C_N_moment_p']*(params['physical']['b_wing_span']/(2 *V_magnitude))*p +\
                params['lat_coef']['C_N_moment_r']*(params['physical']['b_wing_span']/(2 *V_magnitude))*r +\
                params['lat_coef']['C_N_moment_aileron'] * aileron + params['lat_coef']['C_N_moment_rudder'] * rudder
        
            F_y = 0.5 * params['environmental']['rho_air_density'] * V_magnitude**2 * params['physical']['Surface_area_wings'] * C_y
            L_moment = 0.5 * params['environmental']['rho_air_density'] * V_magnitude**2 * params['physical']['Surface_area_wings'] \
                        * params['physical']['b_wing_span'] * C_L_moment
            N_moment = 0.5 * params['environmental']['rho_air_density'] * V_magnitude**2 * params['physical']['Surface_area_wings']\
                        * params['physical']['b_wing_span'] * C_N_moment

    return [V_magnitude, alpha, beta, L, D, M_moment, F_y, L_moment, N_moment]


def find_steady_straight_flight(earth_to_wind_angle, V_magnitude):

    with open("aerosonde_parameters.yaml", 'r') as file:
            params = yaml.safe_load(file)
    g = params['environmental']['gravity']
    mass = params['physical']['mass']   

    def equations(vars):
        u, w, theta, alpha, elevator = vars
        
        C_L = params['long_coef']['C_L_0'] + params['long_coef']['C_L_alpha'] * alpha + \
                params['long_coef']['C_L_elevator'] * elevator
            
        C_D = params['long_coef']['C_D_0'] + params['long_coef']['C_D_alpha'] * alpha + \
            params['long_coef']['C_D_elevator'] * elevator
    
        L = 0.5 * params['environmental']['rho_air_density'] * V_magnitude**2 * params['physical']['Surface_area_wings'] * C_L
        D = 0.5 * params['environmental']['rho_air_density'] * V_magnitude**2 * params['physical']['Surface_area_wings'] * C_D

        C_M_moment = params['long_coef']['C_M_moment_0'] + params['long_coef']['C_M_moment_alpha'] * alpha + \
                params['long_coef']['C_M_moment_elevator'] * elevator
        w_dot = g*np.cos(theta) - D*np.sin(alpha)/mass - L*np.cos(alpha)/mass
        alpha_zero = alpha - np.arctan2(w,u)
        V_magnitude_zero = V_magnitude - np.sqrt(u**2 + w**2)
        earth_to_wind_angle_zero = earth_to_wind_angle - (theta-alpha)

        return [C_M_moment, w_dot, alpha_zero, V_magnitude_zero, earth_to_wind_angle_zero]
    
    u, w, theta, alpha, elevator = fsolve(equations, (1, 1, earth_to_wind_angle, 0, 0))

    # now solve for T
    V_magnitude = np.sqrt(u**2 + w**2)
    C_L = params['long_coef']['C_L_0'] + params['long_coef']['C_L_alpha'] * alpha + \
            params['long_coef']['C_L_elevator'] * elevator
            
    C_D = params['long_coef']['C_D_0'] + params['long_coef']['C_D_alpha'] * alpha + \
        params['long_coef']['C_D_elevator'] * elevator

    L = 0.5 * params['environmental']['rho_air_density'] * V_magnitude**2 * params['physical']['Surface_area_wings'] * C_L
    D = 0.5 * params['environmental']['rho_air_density'] * V_magnitude**2 * params['physical']['Surface_area_wings'] * C_D

    T = g*np.sin(theta)*mass + D*np.cos(alpha) -L*np.sin(alpha)
            
    return(u, w, theta, alpha, elevator, T)


def simulate_airplane(state_init, state_des, u_input, K_long, K_lat, duration, dt):

    x_des_list = []
    y_des_list = []
    z_des_list = []
    u_des_list = []
    v_des_list = []
    w_des_list = []
    phi_des_list = []
    theta_des_list = []
    psi_des_list = []
    p_des_list = []
    q_des_list = []
    r_des_list = []
    t_list_des = []


    def airplane_model(t,state, state_des, u_input, K_long, K_lat):


        x = state[0]
        y = state[1]
        z = state[2]
        u = state[3]
        v = state[4]
        w = state[5]

        phi = state[6]
        theta = state[7]
        psi = state[8]
        p = state[9]
        q = state[10]
        r = state[11]

        long_variable = np.array([[u], [w], [q], [theta]])
        lat_variable = np.array([[v], [p], [r], [phi], [psi]])

        x_error = state_des[0] - x
        y_error = state_des[1] - y
        z_error = state_des[2] - z

        psi_des = np.arctan2(y_error,x_error)
        earth_to_wind_angle_des = np.arctan2(-z_error, np.sqrt(x_error**2 + y_error**2))
        V_magnitude = 20

        u_des, w_des, theta_des, alpha_des, elevator_des, Thrust_des = find_steady_straight_flight(earth_to_wind_angle_des, V_magnitude)

        long_state_des = np.array([[u_des],[w_des],[state_des[10]],[theta_des]])
        lat_state_des = np.array([[state_des[4]],[state_des[9]],[state_des[11]],[state_des[6]], [psi_des]])

        x_des_list.append(state_des[0])
        y_des_list.append(state_des[1])
        z_des_list.append(state_des[2])
        u_des_list.append(u_des)
        v_des_list.append(state_des[4])
        w_des_list.append(w_des)
        phi_des_list.append(state_des[6])
        theta_des_list.append(theta_des)
        psi_des_list.append(psi_des)
        p_des_list.append(state_des[9])
        q_des_list.append(state_des[10])
        r_des_list.append(state_des[11])
        t_list_des.append(t)



        long_input = -K_long @ (long_variable - long_state_des)
        lat_input = -K_lat @ (lat_variable - lat_state_des)

        T = u_input[0] + long_input[0,0]
        elevator = u_input[1] + long_input[1,0]
        aileron = u_input[2] + lat_input[0,0]
        rudder = u_input[3] + lat_input[1,0]

        ### load in stored parameters and calculate other parameters###
        with open("aerosonde_parameters.yaml", 'r') as file:
            params = yaml.safe_load(file)
        
        V_magnitude, alpha, beta, L, D, M_moment, F_y, L_moment, N_moment = calculate_aero_forces(u,v,w, p,q,r, T,elevator,aileron, rudder, params)
        ##################################################################################
        

        # Translation Kinematics
        pos_earth_dot = (Rot_z(psi).transpose() @ Rot_y(theta).transpose() @ Rot_x(phi).transpose()) \
                    @ np.array([[u], [v], [w]])
        x_dot = pos_earth_dot[0,0]
        y_dot = pos_earth_dot[1,0]
        z_dot = pos_earth_dot[2,0]


        # rotation Kinematics

        rotation_dot = np.array([[1, np.sin(phi)*np.tan(theta), np.cos(phi)*np.tan(theta)],
                                    [0, np.cos(phi), -np.sin(phi)],
                                    [0, np.sin(phi)*(1/np.cos(theta)), np.cos(phi)*(1/np.cos(theta))]])\
                                    @ np.array([[p], [q], [r]])
        phi_dot = rotation_dot[0,0]
        theta_dot = rotation_dot[1,0]
        psi_dot = rotation_dot[2,0]

        # Translation dynamics
        g = params['environmental']['gravity']
        mass = params['physical']['mass']

        u_dot = -g*np.sin(theta) - D*np.cos(alpha)*np.cos(beta)/mass + L*np.sin(alpha)/mass + T/mass -q*w+r*v
        v_dot = g*np.cos(theta)*np.sin(phi) - D*np.sin(beta)/mass + F_y/mass -r*u+p*w
        w_dot = g*np.cos(theta)*np.cos(phi) - D*np.sin(alpha)*np.cos(beta)/mass - L*np.cos(alpha)/mass -p*v+q*u

        
        # rotation dynamics
        Ixx = params['physical']['Ixx']
        Iyy = params['physical']['Iyy']
        Izz = params['physical']['Izz']
        Ixz = params['physical']['Ixz']
        
        p_dot = ( -q*r*(Izz-Iyy) + N_moment*Ixz/Izz - p*q*Ixz*(Iyy-Ixx)/Izz - q*r*Ixz*Ixz/Izz +p*q*Ixz + L_moment)\
                                / (Ixx - Ixz*Ixz/Izz)
        
        q_dot = ( M_moment + p*r*(Izz-Ixx)-(p**2 - r**2)*Ixz ) / Iyy

        r_dot = ( -p*q*(Iyy-Ixx) - q*r*Ixz + L_moment*Ixz/Ixx - q*r*Ixz*(Izz-Iyy)/Ixx + p*q*Ixz*Ixz/Ixx + N_moment)\
                                / (Izz - Ixz*Ixz/Ixx)

        
        
        
        return(np.array([x_dot, y_dot, z_dot,
                         u_dot, v_dot, w_dot,
                         phi_dot, theta_dot, psi_dot,
                         p_dot, q_dot, r_dot]))

    t_span = [0, duration] 
    t_eval = np.arange(0, duration+dt/2, dt)
    sol = solve_ivp(lambda t, y: airplane_model(t, y, state_des, u_input, K_long, K_lat), t_span, state_init, t_eval=t_eval)

    state_des = [x_des_list, y_des_list, z_des_list, u_des_list, v_des_list, w_des_list, \
                 phi_des_list, theta_des_list, psi_des_list, p_des_list, q_des_list, r_des_list]
    return sol, state_des, t_list_des




def simulate_airplane6(state_init, state_des, u_input, K_long, K_lat, duration, dt):


    def airplane_model6(t,state, state_des, u_input, K_long, K_lat):


        x = state[0]
        y = state[1]
        z = state[2]
        u = state[3]
        v = state[4]
        w = state[5]

        phi = state[6]
        theta = state[7]
        psi = state[8]
        p = state[9]
        q = state[10]
        r = state[11]

        long_variable = np.array([[u], [w], [q], [theta],[x], [z]])
        lat_variable = np.array([[v], [p], [r], [phi], [psi], [y]])

        long_state_des = np.array([[state_des[3]],[state_des[5]],[state_des[10]],[state_des[7]], [state_des[0]], [state_des[2]]])
        lat_state_des = np.array([[state_des[4]],[state_des[9]],[state_des[11]],[state_des[6]], [state_des[8]], [state_des[1]]])


        long_input = -K_long @ (long_variable - long_state_des)
        lat_input = -K_lat @ (lat_variable - lat_state_des)

        T = u_input[0] + long_input[0,0]
        elevator = u_input[1] + long_input[1,0]
        aileron = u_input[2] + lat_input[0,0]
        rudder = u_input[3] + lat_input[1,0]

        ### load in stored parameters and calculate other parameters###
        with open("aerosonde_parameters.yaml", 'r') as file:
            params = yaml.safe_load(file)
        
        V_magnitude, alpha, beta, L, D, M_moment, F_y, L_moment, N_moment = calculate_aero_forces(u,v,w, p,q,r, T,elevator,aileron, rudder, params)
        ##################################################################################
        

        # Translation Kinematics
        pos_earth_dot = (Rot_z(psi).transpose() @ Rot_y(theta).transpose() @ Rot_x(phi).transpose()) \
                    @ np.array([[u], [v], [w]])
        x_dot = pos_earth_dot[0,0]
        y_dot = pos_earth_dot[1,0]
        z_dot = pos_earth_dot[2,0]


        # rotation Kinematics

        rotation_dot = np.array([[1, np.sin(phi)*np.tan(theta), np.cos(phi)*np.tan(theta)],
                                    [0, np.cos(phi), -np.sin(phi)],
                                    [0, np.sin(phi)*(1/np.cos(theta)), np.cos(phi)*(1/np.cos(theta))]])\
                                    @ np.array([[p], [q], [r]])
        phi_dot = rotation_dot[0,0]
        theta_dot = rotation_dot[1,0]
        psi_dot = rotation_dot[2,0]

        # Translation dynamics
        g = params['environmental']['gravity']
        mass = params['physical']['mass']

        u_dot = -g*np.sin(theta) - D*np.cos(alpha)*np.cos(beta)/mass + L*np.sin(alpha)/mass + T/mass -q*w+r*v
        v_dot = g*np.cos(theta)*np.sin(phi) - D*np.sin(beta)/mass + F_y/mass -r*u+p*w
        w_dot = g*np.cos(theta)*np.cos(phi) - D*np.sin(alpha)*np.cos(beta)/mass - L*np.cos(alpha)/mass -p*v+q*u

        
        # rotation dynamics
        Ixx = params['physical']['Ixx']
        Iyy = params['physical']['Iyy']
        Izz = params['physical']['Izz']
        Ixz = params['physical']['Ixz']
        
        p_dot = ( -q*r*(Izz-Iyy) + N_moment*Ixz/Izz - p*q*Ixz*(Iyy-Ixx)/Izz - q*r*Ixz*Ixz/Izz +p*q*Ixz + L_moment)\
                                / (Ixx - Ixz*Ixz/Izz)
        
        q_dot = ( M_moment + p*r*(Izz-Ixx)-(p**2 - r**2)*Ixz ) / Iyy

        r_dot = ( -p*q*(Iyy-Ixx) - q*r*Ixz + L_moment*Ixz/Ixx - q*r*Ixz*(Izz-Iyy)/Ixx + p*q*Ixz*Ixz/Ixx + N_moment)\
                                / (Izz - Ixz*Ixz/Ixx)

        
        
        
        return(np.array([x_dot, y_dot, z_dot,
                         u_dot, v_dot, w_dot,
                         phi_dot, theta_dot, psi_dot,
                         p_dot, q_dot, r_dot]))

    t_span = [0, duration] 
    t_eval = np.arange(0, duration+dt/2, dt)
    sol = solve_ivp(lambda t, y: airplane_model6(t, y, state_des, u_input, K_long, K_lat), t_span, state_init, t_eval=t_eval, method='Radau')
    return sol
# cvxpy


# def simulate_airplane(state_init, state_des, u_input, K_long, K_lat, duration, dt):


#     def airplane_model(t,state, state_des, u_input, K_long, K_lat):


#         x = state[0]
#         y = state[1]
#         z = state[2]
#         u = state[3]
#         v = state[4]
#         w = state[5]

#         phi = state[6]
#         theta = state[7]
#         psi = state[8]
#         p = state[9]
#         q = state[10]
#         r = state[11]

#         long_variable = np.array([[u], [w], [q], [theta]])
#         lat_variable = np.array([[v], [p], [r], [phi], [psi]])

#         long_state_des = np.array([[state_des[3]],[state_des[5]],[state_des[10]],[state_des[7]]])
#         lat_state_des = np.array([[state_des[4]],[state_des[9]],[state_des[11]],[state_des[6]], [state_des[8]]])

#         long_input = -K_long @ (long_variable - long_state_des)
#         lat_input = -K_lat @ (lat_variable - lat_state_des)

#         T = u_input[0] + long_input[0,0]
#         elevator = u_input[1] + long_input[1,0]
#         aileron = u_input[2] + lat_input[0,0]
#         rudder = u_input[3] + lat_input[1,0]

#         ### load in stored parameters and calculate other parameters###
#         with open("aerosonde_parameters.yaml", 'r') as file:
#             params = yaml.safe_load(file)
        
#         V_magnitude, alpha, beta, L, D, M_moment, F_y, L_moment, N_moment = calculate_aero_forces(u,v,w, p,q,r, T,elevator,aileron, rudder, params)
#         ##################################################################################
        

#         # Translation Kinematics
#         pos_earth_dot = (Rot_z(psi).transpose() @ Rot_y(theta).transpose() @ Rot_x(phi).transpose()) \
#                     @ np.array([[u], [v], [w]])
#         x_dot = pos_earth_dot[0,0]
#         y_dot = pos_earth_dot[1,0]
#         z_dot = pos_earth_dot[2,0]


#         # rotation Kinematics

#         rotation_dot = np.array([[1, np.sin(phi)*np.tan(theta), np.cos(phi)*np.tan(theta)],
#                                     [0, np.cos(phi), -np.sin(phi)],
#                                     [0, np.sin(phi)*(1/np.cos(theta)), np.cos(phi)*(1/np.cos(theta))]])\
#                                     @ np.array([[p], [q], [r]])
#         phi_dot = rotation_dot[0,0]
#         theta_dot = rotation_dot[1,0]
#         psi_dot = rotation_dot[2,0]

#         # Translation dynamics
#         g = params['environmental']['gravity']
#         mass = params['physical']['mass']

#         u_dot = -g*np.sin(theta) - D*np.cos(alpha)*np.cos(beta)/mass + L*np.sin(alpha)/mass + T/mass -q*w+r*v
#         v_dot = g*np.cos(theta)*np.sin(phi) - D*np.sin(beta)/mass + F_y/mass -r*u+p*w
#         w_dot = g*np.cos(theta)*np.cos(phi) - D*np.sin(alpha)*np.cos(beta)/mass - L*np.cos(alpha)/mass -p*v+q*u

        
#         # rotation dynamics
#         Ixx = params['physical']['Ixx']
#         Iyy = params['physical']['Iyy']
#         Izz = params['physical']['Izz']
#         Ixz = params['physical']['Ixz']
        
#         p_dot = ( -q*r*(Izz-Iyy) + N_moment*Ixz/Izz - p*q*Ixz*(Iyy-Ixx)/Izz - q*r*Ixz*Ixz/Izz +p*q*Ixz + L_moment)\
#                                 / (Ixx - Ixz*Ixz/Izz)
        
#         q_dot = ( M_moment + p*r*(Izz-Ixx)-(p**2 - r**2)*Ixz ) / Iyy

#         r_dot = ( -p*q*(Iyy-Ixx) - q*r*Ixz + L_moment*Ixz/Ixx - q*r*Ixz*(Izz-Iyy)/Ixx + p*q*Ixz*Ixz/Ixx + N_moment)\
#                                 / (Izz - Ixz*Ixz/Ixx)

        
        
        
#         return(np.array([x_dot, y_dot, z_dot,
#                          u_dot, v_dot, w_dot,
#                          phi_dot, theta_dot, psi_dot,
#                          p_dot, q_dot, r_dot]))

#     t_span = [0, duration] 
#     t_eval = np.arange(0, duration+dt/2, dt)
#     sol = solve_ivp(lambda t, y: airplane_model(t, y, state_des, u_input, K_long, K_lat), t_span, state_init, t_eval=t_eval)
#     return sol
