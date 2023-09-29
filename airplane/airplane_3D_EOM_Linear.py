import numpy as np
import yaml
from scipy.linalg import expm



def continuous_linearization(state_eq, input_eq):

    x_0 = state_eq[0]
    y_0 = state_eq[1]
    z_0 = state_eq[2]
    u_0 = state_eq[3]
    v_0 = state_eq[4]
    w_0 = state_eq[5]

    phi_0 = state_eq[6]
    theta_0 = state_eq[7]
    psi_0 = state_eq[8]
    p_0 = state_eq[9]
    q_0 = state_eq[10]
    r_0 = state_eq[11]

    T_0 = input_eq[0]
    elevator_0 = input_eq[1]
    aileron_0 = input_eq[2]
    rudder_0 = input_eq[3]

    ### load in stored parameters and calculate other parameters###
    with open("aerosonde_parameters.yaml", 'r') as file:
        params = yaml.safe_load(file)

    g = params['environmental']['gravity']
    rho = params['environmental']['rho_air_density']
    S = params['physical']['Surface_area_wings']
    m = params['physical']['mass']
    Ixx = params['physical']['Ixx']
    Iyy = params['physical']['Iyy']
    Izz = params['physical']['Izz']
    Ixz = params['physical']['Ixz']
    c_wing_chord = params['physical']['c_wing_chord']
    b_wing_span =  params['physical']['b_wing_span']

    C_L_0 = params['long_coef']['C_L_0']
    C_L_alpha = params['long_coef']['C_L_alpha']
    C_L_q = params['long_coef']['C_L_q']
    C_L_elevator = params['long_coef']['C_L_elevator']

    C_D_0 = params['long_coef']['C_D_0']
    C_D_alpha = params['long_coef']['C_D_alpha']
    C_D_q = params['long_coef']['C_D_q']
    C_D_elevator = params['long_coef']['C_D_elevator']

    C_M_moment_0 = params['long_coef']['C_M_moment_0']
    C_M_moment_alpha = params['long_coef']['C_M_moment_alpha']
    C_M_moment_q = params['long_coef']['C_M_moment_q']
    C_M_moment_elevator = params['long_coef']['C_M_moment_elevator']

    C_y_0 = params['lat_coef']['C_y_0']
    C_y_beta = params['lat_coef']['C_y_beta']
    C_y_p = params['lat_coef']['C_y_p']
    C_y_r = params['lat_coef']['C_y_r']
    C_y_aileron = params['lat_coef']['C_y_aileron']
    C_y_rudder = params['lat_coef']['C_y_rudder']

    C_L_moment_0 = params['lat_coef']['C_L_moment_0']
    C_L_moment_beta = params['lat_coef']['C_L_moment_beta']
    C_L_moment_p = params['lat_coef']['C_L_moment_p']
    C_L_moment_r = params['lat_coef']['C_L_moment_r']
    C_L_moment_aileron = params['lat_coef']['C_L_moment_aileron']
    C_L_moment_rudder = params['lat_coef']['C_L_moment_rudder']

    C_N_moment_0 = params['lat_coef']['C_N_moment_0']
    C_N_moment_beta = params['lat_coef']['C_N_moment_beta']
    C_N_moment_p = params['lat_coef']['C_N_moment_p']
    C_N_moment_r = params['lat_coef']['C_N_moment_r']
    C_N_moment_aileron = params['lat_coef']['C_N_moment_aileron']
    C_N_moment_rudder = params['lat_coef']['C_N_moment_rudder']



    alpha_0 = np.arctan2(w_0,u_0)
    V_0 = np.sqrt(u_0**2 +v_0**2 + w_0**2)

    # linearized longitudial components
    print(u_0)
    print(rho)
    print(S)
    print(m)
    print(C_D_0)
    print(C_D_alpha)
    print(alpha_0)
    print(C_D_elevator)
    print(elevator_0)
    print(w_0)
    print(-u_0*rho*S/m*(C_D_0 + C_D_alpha*alpha_0 + C_D_elevator*elevator_0))
    print(rho*S*w_0*C_D_alpha/(2*m))
    print(-rho*S*c_wing_chord*C_D_q*q_0* u_0/(4*m*V_0))
    a_u_u = -u_0*rho*S/m*(C_D_0 + C_D_alpha*alpha_0 + C_D_elevator*elevator_0) \
        + rho*S*w_0*C_D_alpha/(2*m) - rho*S*c_wing_chord*C_D_q*q_0* u_0/(4*m*V_0)
    a_u_w = -w_0*rho*S/m*(C_D_0 + C_D_alpha*alpha_0 + C_D_elevator*elevator_0) \
        - rho*S*u_0*C_D_alpha/(2*m) - rho*S*c_wing_chord*C_D_q*q_0* w_0/(4*m*V_0) - q_0
    a_u_q = -w_0 - rho*V_0*S*C_D_q*c_wing_chord/(4*m)
    a_u_theta = -g*np.cos(theta_0)
    b_u_elevator = -rho*V_0**2*S*C_D_elevator/(2*m)
    b_u_T = 1/m

    a_w_u = -u_0*rho*S/m*(C_L_0 + C_L_alpha*alpha_0 + C_L_elevator*elevator_0) \
        + rho*S*w_0*C_L_alpha/(2*m) - rho*S*c_wing_chord*C_L_q*q_0* u_0/(4*m*V_0) + q_0
    a_w_w = -w_0*rho*S/m*(C_L_0 + C_L_alpha*alpha_0 + C_L_elevator*elevator_0) \
        - rho*S*u_0*C_L_alpha/(2*m) - rho*S*c_wing_chord*C_L_q*q_0* w_0/(4*m*V_0) 
    a_w_q = u_0 - rho*V_0*S*C_L_q*c_wing_chord/(4*m)
    a_w_theta = -g*np.sin(theta_0)
    b_w_elevator = -rho*V_0**2*S*C_L_elevator/(2*m)
    
    a_q_u = u_0*rho*S*c_wing_chord/Iyy*(C_M_moment_0 + C_M_moment_alpha*alpha_0 +C_M_moment_elevator*elevator_0)\
            - rho*S*c_wing_chord*C_M_moment_alpha*w_0/(2*Iyy) + rho*S*c_wing_chord**2*C_M_moment_q*q_0*u_0/(4*Iyy*V_0)
    a_q_w = w_0*rho*S*c_wing_chord/Iyy*(C_M_moment_0 + C_M_moment_alpha*alpha_0 +C_M_moment_elevator*elevator_0)\
            + rho*S*c_wing_chord*C_M_moment_alpha*u_0/(2*Iyy) + rho*S*c_wing_chord**2*C_M_moment_q*q_0*w_0/(4*Iyy*V_0)
    a_q_q = rho*V_0*S*c_wing_chord**2*C_M_moment_q/(4*Iyy)
    b_q_elevator = rho*V_0**2*S*c_wing_chord*C_M_moment_elevator/(2*Iyy)

    a_theta_q = 1

    A_long4 = np.array([[a_u_u, a_u_w, a_u_q, a_u_theta],
                        [a_w_u, a_w_w, a_w_q, a_w_theta],
                        [a_q_u, a_q_w, a_q_q,     0],
                        [ 0   ,   0  , a_theta_q, 0]])
    B_long4 = np.array([[b_u_T, b_u_elevator],
                        [0    , b_w_elevator],
                        [0    , b_q_elevator],
                        [0    , 0           ]])
    
 
    return(A_long4, B_long4)

    #return(A_lat4, B_lat4, A_lat5, B_lat5, A_lat6, B_lat6, A_long4, B_long4, A_long5, B_long5, A_long6, B_long6)


def A_matrix_evalation(A, dt):
    A_continuous = A
    A_discrete = expm(A*dt)

    continuous_eigenvalues, _ = np.linalg.eig(A_continuous)

    # Compute discretized eigenvalues
    discretized_eigenvalues, _ = np.linalg.eig(A_discrete)

    print(f"continuous_eigenvalues: {continuous_eigenvalues}")
    print(f"discrete_eigenvalues: {discretized_eigenvalues}")

def discrete_simulation(state_init, input_init, input_new, dt, duration):
    # A_discrete = np.eye(A.shape[0]) + dt*A
    # B_discrete = dt*B

    num_steps = int(duration/dt)
    state_history = state_init
    delta_state = np.array([[0],[0],[0],[0]])
    delta_input = input_new - input_init
    for k in range (num_steps):
        current_state = state_history[:,[-1]]
        state_eq = np.array([[0], [0], [0], [current_state[0,0]], [0], [current_state[1,0]],
                    [0], [current_state[3,0]], [0], [0], [current_state[2,0]], [0]])
        input_eq = np.array([[input_init[0,0]], [input_init[1,0]], [0], [0]])
        A, B = continuous_linearization(state_eq.squeeze(), input_eq.squeeze())
        A_discrete = np.eye(A.shape[0]) + dt*A
        B_discrete = dt*B
        delta_state = A_discrete @ delta_state + B_discrete @ delta_input
        state_history = np.concatenate([state_history, current_state+delta_state], axis=1)
        delta_input = np.array([[0],[0]])
        print(A)


    return state_history