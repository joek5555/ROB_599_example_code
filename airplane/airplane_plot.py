
import numpy as np
import matplotlib.pyplot as plt

def plot_airplane_data(sol):
    num_ticks = 5

    # Translation kinematics
    plt.figure(figsize=(14, 10))
    plt.subplot(4, 3, 1)
    plt.plot(sol.t, sol.y[0], label='x')
    plt.xlabel('Time')
    plt.ylabel('x')
    if np.abs(plt.yticks()[0][-1] - plt.yticks()[0][0]) < 0.01:
        upper_lim = np.ceil(plt.yticks()[0][-1]*100)/100
        lower_lim = np.floor(plt.yticks()[0][0]*100)/100
        plt.ylim([lower_lim, upper_lim])
    plt.legend()
    plt.yscale("linear")

    plt.subplot(4, 3, 2)
    plt.plot(sol.t, sol.y[1], label='y', color='orange')
    plt.xlabel('Time')
    plt.ylabel('y')
    if np.abs(plt.yticks()[0][-1] - plt.yticks()[0][0]) < 0.01:
        upper_lim = np.ceil(plt.yticks()[0][-1]*100)/100
        lower_lim = np.floor(plt.yticks()[0][0]*100)/100
        plt.ylim([lower_lim, upper_lim])
    plt.legend()
    plt.yscale("linear")

    plt.subplot(4, 3, 3)
    plt.plot(sol.t, sol.y[2], label='z', color='green')
    plt.xlabel('Time')
    plt.ylabel('z')
    if np.abs(plt.yticks()[0][-1] - plt.yticks()[0][0]) < 0.01:
        upper_lim = np.ceil(plt.yticks()[0][-1]*100)/100
        lower_lim = np.floor(plt.yticks()[0][0]*100)/100
        plt.ylim([lower_lim, upper_lim])
    plt.legend()
    plt.yscale("linear")

    # Translation dynamics

    plt.subplot(4, 3, 4)
    plt.plot(sol.t, sol.y[3], label='u', color='red')
    plt.xlabel('Time')
    plt.ylabel('u')
    if np.abs(plt.yticks()[0][-1] - plt.yticks()[0][0]) < 0.1:
        upper_lim = np.ceil(plt.yticks()[0][-1]*10)/10
        lower_lim = np.floor(plt.yticks()[0][0]*10)/10
        plt.ylim([lower_lim, upper_lim])
    plt.legend()
    plt.yscale("linear")

    plt.subplot(4, 3, 5)
    plt.plot(sol.t, sol.y[4], label='v', color='red')
    plt.xlabel('Time')
    plt.ylabel('v')
    if np.abs(plt.yticks()[0][-1] - plt.yticks()[0][0]) < 0.1:
        upper_lim = np.ceil(plt.yticks()[0][-1]*10)/10
        lower_lim = np.floor(plt.yticks()[0][0]*10)/10
        plt.ylim([lower_lim, upper_lim])
    plt.legend()
    plt.yscale("linear")

    plt.subplot(4, 3, 6)
    plt.plot(sol.t, sol.y[5], label='w', color='red')
    plt.xlabel('Time')
    plt.ylabel('w')
    if np.abs(plt.yticks()[0][-1] - plt.yticks()[0][0]) < 0.1:
        upper_lim = np.ceil(plt.yticks()[0][-1]*10)/10
        lower_lim = np.floor(plt.yticks()[0][0]*10)/10
        plt.ylim([lower_lim, upper_lim])
    plt.legend()
    plt.yscale("linear")
    
    # Rotational kinematics

    plt.subplot(4, 3, 7)
    plt.plot(sol.t, sol.y[6], label='phi')
    plt.xlabel('Time')
    plt.ylabel('phi')
    step_size = np.round((np.abs(plt.yticks()[0][-1] - plt.yticks()[0][0]))/(num_ticks*np.pi) *4)/4
    if step_size == 0:
        step_size = 1/4
    start_tick = np.floor(plt.yticks()[0][0] / (np.pi*step_size)) * np.pi*step_size
    stop_tick = np.ceil(plt.yticks()[0][-1]/ (np.pi*step_size)) * np.pi*step_size
    plt.yticks(ticks= np.arange(start_tick, stop_tick+np.pi*step_size, np.pi*step_size), \
            labels= [r"$" + format(r/np.pi, ".2g")+ r"\pi$" for r in np.arange(start_tick, stop_tick+np.pi*step_size, np.pi*step_size)])
    plt.legend()

    plt.subplot(4, 3, 8)
    plt.plot(sol.t, sol.y[7], label='theta', color='orange')
    plt.xlabel('Time')
    plt.ylabel('theta')
    step_size = np.round((np.abs(plt.yticks()[0][-1] - plt.yticks()[0][0]))/(num_ticks*np.pi) *4)/4
    if step_size == 0:
        step_size = 1/4
    start_tick = np.floor(plt.yticks()[0][0] / (np.pi*step_size)) * np.pi*step_size
    stop_tick = np.ceil(plt.yticks()[0][-1]/ (np.pi*step_size)) * np.pi*step_size
    plt.yticks(ticks= np.arange(start_tick, stop_tick+np.pi*step_size, np.pi*step_size), \
            labels= [r"$" + format(r/np.pi, ".2g")+ r"\pi$" for r in np.arange(start_tick, stop_tick+np.pi*step_size, np.pi*step_size)])
    plt.legend()

    plt.subplot(4, 3, 9)
    plt.plot(sol.t, sol.y[8], label='psi', color='green')
    plt.xlabel('Time')
    plt.ylabel('psi')
    step_size = np.round((np.abs(plt.yticks()[0][-1] - plt.yticks()[0][0]))/(num_ticks*np.pi) *4)/4
    if step_size == 0:
        step_size = 1/4
    start_tick = np.floor(plt.yticks()[0][0] / (np.pi*step_size)) * np.pi*step_size
    stop_tick = np.ceil(plt.yticks()[0][-1]/ (np.pi*step_size)) * np.pi*step_size
    plt.yticks(ticks= np.arange(start_tick, stop_tick+np.pi*step_size, np.pi*step_size), \
            labels= [r"$" + format(r/np.pi, ".2g")+ r"\pi$" for r in np.arange(start_tick, stop_tick+np.pi*step_size, np.pi*step_size)])
    plt.legend()

    # Rotational Dynamics

    plt.subplot(4, 3, 10)
    plt.plot(sol.t, sol.y[9], label='p', color='red')
    plt.xlabel('Time')
    plt.ylabel('p')
    step_size = np.round((np.abs(plt.yticks()[0][-1] - plt.yticks()[0][0]))/(num_ticks*np.pi) *4)/4
    if step_size == 0:
        step_size = 1/4
    start_tick = np.floor(plt.yticks()[0][0] / (np.pi*step_size)) * np.pi*step_size
    stop_tick = np.ceil(plt.yticks()[0][-1]/ (np.pi*step_size)) * np.pi*step_size
    plt.yticks(ticks= np.arange(start_tick, stop_tick+np.pi*step_size, np.pi*step_size), \
            labels= [r"$" + format(r/np.pi, ".2g")+ r"\pi$" for r in np.arange(start_tick, stop_tick+np.pi*step_size, np.pi*step_size)])
    plt.legend()

    plt.subplot(4, 3, 11)
    plt.plot(sol.t, sol.y[10], label='q', color='red')
    plt.xlabel('Time')
    plt.ylabel('q')
    step_size = np.round((np.abs(plt.yticks()[0][-1] - plt.yticks()[0][0]))/(num_ticks*np.pi) *4)/4
    if step_size == 0:
        step_size = 1/4
    start_tick = np.floor(plt.yticks()[0][0] / (np.pi*step_size)) * np.pi*step_size
    stop_tick = np.ceil(plt.yticks()[0][-1]/ (np.pi*step_size)) * np.pi*step_size
    plt.yticks(ticks= np.arange(start_tick, stop_tick+np.pi*step_size, np.pi*step_size), \
            labels= [r"$" + format(r/np.pi, ".2g")+ r"\pi$" for r in np.arange(start_tick, stop_tick+np.pi*step_size, np.pi*step_size)])
    plt.legend()

    plt.subplot(4, 3, 12)
    plt.plot(sol.t, sol.y[11], label='r', color='red')
    plt.xlabel('Time')
    plt.ylabel('r')
    step_size = np.round((np.abs(plt.yticks()[0][-1] - plt.yticks()[0][0]))/(num_ticks*np.pi) *4)/4
    if step_size == 0:
        step_size = 1/4
    start_tick = np.floor(plt.yticks()[0][0] / (np.pi*step_size)) * np.pi*step_size
    stop_tick = np.ceil(plt.yticks()[0][-1]/ (np.pi*step_size)) * np.pi*step_size
    plt.yticks(ticks= np.arange(start_tick, stop_tick+np.pi*step_size, np.pi*step_size), \
            labels= [r"$" + format(r/np.pi, ".2g")+ r"\pi$" for r in np.arange(start_tick, stop_tick+np.pi*step_size, np.pi*step_size)])
    plt.legend()


    plt.tight_layout()
    plt.show()