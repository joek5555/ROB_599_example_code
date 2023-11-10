# Model and Control of a Aerosonde fixed wing drone 

This code is used to model and control a fixed wing drone in simulation. LQR and MPC methods will be used. This work is being done as a project for ROB 599: Autonomous Vehicles.

The Aerosonde fixed wing drone is and aircraft whose parameters and coefficients have been well defined and can be found in the book <em>Small Unmanned Aircraft: Theory and Practice </em>  by Randal W. Beard and Timothy W. McLain. 
The equations of motion for a fixed wing drone are derived in both Project Checkpoints attached. The parameters are taken from the book and stored in the aerosonde_parameters.yaml file. 

Currently the airplane_3D_LQR.ipynb is working. Given a list of waypoints, the LQR controller will seek to drive the aircraft to these waypoints in order. Two lists of waypoints are defined, one being a helix trajectory and the
other being a wave trajectory. The results for these trajectories can be seen below. 


### Helix

![helix trajectory gif](drone_helix_LQR.gif)

</br>
</br>
</br>


### Wave

![wave trajectory gif](drone_wave_wind_LQR.gif)
