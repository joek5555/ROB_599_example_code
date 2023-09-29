import numpy as np
from scipy.integrate import solve_ivp
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


def simulate_airplane(state_init, input_u, duration):


    def airplane_model(t,state, input_u):

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

        T = input_u[0]
        elevator = input_u[1]
        aileron = input_u[2]
        rudder = input_u[3]

        ### load in stored parameters and calculate other parameters###
        with open("aerosonde_parameters.yaml", 'r') as file:
            params = yaml.safe_load(file)
        
        V_magnitude, alpha, beta, L, D, M_moment, F_y, L_moment, N_moment = calculate_aero_forces(u,v,w, p,q,r, T,elevator,aileron, rudder, params)
        ##################################################################################
        

        # print(f"L:{L}, D:{D}, F_y{F_y}")
        # print(f"L_moment:{L_moment}, M_moment:{M_moment}, N_moment{N_moment}")

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

        
        # if u_dot < 1e-7 and u_dot > -1e-7:
        #     u_dot = 0
        # if v_dot < 1e-7 and v_dot > -1e-7:
        #     v_dot = 0
        # if w_dot < 1e-7 and w_dot > -1e-7:
        #     w_dot = 0
        # if p_dot < 1e-7 and p_dot > -1e-7:
        #     p_dot = 0
        # if q_dot < 1e-7 and q_dot > -1e-7:
        #     q_dot = 0
        # if r_dot < 1e-7 and r_dot > -1e-7:
        #     r_dot = 0
        
        print(f"time: {t}")
        print(f"alpha:{alpha}, beta:{beta}, theta_dot:{theta_dot}, psi_dot:{psi_dot}")
        print(f"u_dot:{u_dot}, v_dot:{v_dot}, w_dot:{w_dot}")
        print(f"p_dot:{p_dot}, q_dot:{q_dot}, r_dot:{r_dot}")

        
        
        return(np.array([x_dot, y_dot, z_dot,
                         u_dot, v_dot, w_dot,
                         phi_dot, theta_dot, psi_dot,
                         p_dot, q_dot, r_dot]))

    t_span = [0, duration] 
    t_eval = np.arange(0, duration, 0.01)
    sol = solve_ivp(lambda t, y: airplane_model(t, y, input_u), t_span, state_init, t_eval=t_eval)
    return sol

