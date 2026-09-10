import numpy as np
import math
import matplotlib.pyplot as plt
import ussa1976

from numerical_integrators import numerical_integration_methods
from governing_equations import flat_earth_eom
from tools.Interpolators import fastInterp1
from vehicle_models import spheres

from vehicle_models import CESSNA_172
from flights.flight_config import TRIMMED_STRAIGHT_LEVEL_50MPS, build_initial_state

"""Need to make variable names in build_initial_state(flight) consistent"""

#==================================================
# 1: Initialisation
#==================================================

#Atmosphere & gravity essentials
atmosphere=ussa1976.compute()

alt_m = atmosphere["z"].values
rho_kgpm3 = atmosphere["rho"].values
c_mps = atmosphere["cs"].values
g_mps2 = ussa1976.core.compute_gravity(alt_m)

amod={"alt_m" : alt_m, "rho_kgpm3": rho_kgpm3, "c_mps": c_mps,  "g_mps2": g_mps2 } #amod used to be for Aircraft, now its for atmosphere; see vmod


#Vehicle & flight definition, and intial conditions (Cessna)
vmod = CESSNA_172
flight = TRIMMED_STRAIGHT_LEVEL_50MPS

x0 = build_initial_state(flight)
nx0 = x0.size

t0_s = flight["t0_s"]
tf_s = flight["tf_s"]
h_s = flight["h_s"]


#Vehicle & flight definition, and intial conditions (Bowling Ball)
"""
vmod = spheres.bowling_ball()
x0 = spheres.x0
nx0 = spheres.nx0

t0_s = 0
tf_s = 185
h_s = 0.01
"""


#==================================================
# 2: Numerical approximation
#==================================================

#Preallocate the solution array (creates a list of times with step size h_s from t0_s to tf_s)
t_s = np.arange(t0_s, tf_s + h_s, h_s) 
nt_s = t_s.size #number of time points

x = np.zeros((nx0, nt_s), dtype = float) #creates empty solution array
x[:, 0] = x0 #Assign initial conditions (x0) to the solution array (x)

## Euler integration
t_s, x = numerical_integration_methods.forward_euler(flat_earth_eom.flat_earth_eom, t_s, x, h_s, vmod, amod)


## Data post-processing

#Airspeed
true_airspeed_mps = np.zeros((nt_s,1))
for i, element in enumerate(t_s):
    true_airspeed_mps[i,0] = np.sqrt(x[0,i]**2 + x[1,i]**2 + x[2,i]**2)

# Altitude, speed of sound, and air density
Altitude_m  = np.zeros((nt_s,1))
Cs_mps      = np.zeros((nt_s,1))
Rho_kgpm3   = np.zeros((nt_s,1))

for i, element in enumerate(t_s):
    Altitude_m[i,0] = -x[11,i]
    Cs_mps[i,0]     = fastInterp1(amod["alt_m"], amod["c_mps"],    Altitude_m[i,0])
    Rho_kgpm3[i,0]  = fastInterp1(amod["alt_m"], amod["rho_kgpm3"], Altitude_m[i,0])

# Angle of attack
Alpha_rad = np.zeros((nt_s,1))
for i, element in enumerate(t_s):
        
    if x[0,i] == 0 and x[2,i] == 0:
        w_over_v = 0
    else:
        w_over_v = x[2,i]/x[0,i]
        
    Alpha_rad[i,0] = np.atan(w_over_v)
    
# Angle of side slip
Beta_rad = np.zeros((nt_s,1))
for i, element in enumerate(t_s):
        
    if x[1,i] == 0 and true_airspeed_mps[i,0] == 0:
        v_over_VT = 0
    else:
        v_over_VT = x[1,i]/true_airspeed_mps[i,0]
        
    Beta_rad[i,0] = np.asin(v_over_VT)
    
# Mach Number
Mach = np.zeros((nt_s,1))
for i, element in enumerate(t_s):
    Mach[i,0] = true_airspeed_mps[i,0]/Cs_mps[i,0]
    
print(f"The numerical terminal velocity is {x[0,-1]:.2f} m/s.")

#==================================================
# 3: Plotting
#==================================================

# A. Create figure of translational and rotation states
fig, axes = plt.subplots(3, 3, figsize=(10, 6))
fig.set_facecolor('black')  
fig.suptitle(f"{vmod["V_name"]}: Velocity, angles & rotation", fontsize=14, fontweight='bold', color='yellow')

# x-axis (axial) velocity u^b_CM/n
axes[0, 0].plot(t_s, x[0,:], color='yellow')
axes[0, 0].set_xlabel('Time [s]', color='white')
axes[0, 0].set_ylabel('u [m/s]', color='white')
axes[0, 0].grid(True)
axes[0, 0].set_facecolor('black')
axes[0, 0].tick_params(colors = 'white')

# y-axis velocity v^b_CM/n
axes[0, 1].plot(t_s, x[1,:], color='yellow')
axes[0, 1].set_xlabel('Time [s]', color='white')
axes[0, 1].set_ylabel('v [m/s]', color='white')
axes[0, 1].grid(True)
axes[0, 1].set_facecolor('black')
axes[0, 1].tick_params(colors = 'white')

# z-axis velocity w^b_CM/n
axes[0, 2].plot(t_s, x[2,:], color='yellow')
axes[0, 2].set_xlabel('Time [s]', color='white')
axes[0, 2].set_ylabel('w [m/s]', color='white')
if np.linalg.norm(x[2,:]) < 1e-6:
    axes[0,2].set_ylim(-0.05,0.05)
axes[0, 2].grid(True)
axes[0, 2].set_facecolor('black')
axes[0, 2].tick_params(colors = 'white')

# Roll angle, phi
axes[1, 0].plot(t_s, x[6,:], color='yellow')
axes[1, 0].set_xlabel('Time [s]', color='white')
axes[1, 0].set_ylabel('phi [rad]', color='white')
axes[1, 0].grid(True)
axes[1, 0].set_facecolor('black')
axes[1, 0].tick_params(colors = 'white')

# Pitch angle, theta
axes[1, 1].plot(t_s, x[7,:], color='yellow')
axes[1, 1].set_xlabel('Time [s]', color='white')
axes[1, 1].set_ylabel('theta [rad]', color='white')
axes[1, 1].grid(True)
axes[1, 1].set_facecolor('black')
axes[1, 1].tick_params(colors = 'white')

# Yaw angle, psi
axes[1, 2].plot(t_s, x[8,:], color='yellow')
axes[1, 2].set_xlabel('Time [s]', color='white')
axes[1, 2].set_ylabel('psi [rad]', color='white')
axes[1, 2].grid(True)
axes[1, 2].set_facecolor('black')
axes[1, 2].tick_params(colors = 'white')

# Roll rate p^b_b/n
axes[2, 0].plot(t_s, x[3,:], color='yellow')
axes[2, 0].set_xlabel('Time [s]', color='white')
axes[2, 0].set_ylabel('p [r/s]', color='white')
axes[2, 0].grid(True)
axes[2, 0].set_facecolor('black')
axes[2, 0].tick_params(colors = 'white')

# Pitch rate q^b_b/n
axes[2, 1].plot(t_s, x[4,:], color='yellow')
axes[2, 1].set_xlabel('Time [s]', color='white')
axes[2, 1].set_ylabel('q [r/s]', color='white')
axes[2, 1].grid(True)
axes[2, 1].set_facecolor('black')
axes[2, 1].tick_params(colors = 'white')

# Yaw rate r^b_b/n
axes[2, 2].plot(t_s, x[5,:], color='white')
axes[2, 2].set_xlabel('Time [s]', color='white')
axes[2, 2].set_ylabel('r [r/s]', color='white')
axes[2, 2].grid(True)
axes[2, 2].set_facecolor('black')
axes[2, 2].tick_params(colors = 'white')

plt.tight_layout()
plt.show(block=False)
plt.show()


#B. Create figure of position states
fig, axes = plt.subplots(2, 3, figsize=(10, 6))
fig.set_facecolor('black') 
fig.suptitle(vmod["V_name"], fontsize=14, fontweight='bold', color='cyan') 

# North position p1^n_CM/T
axes[0,0].plot(t_s, x[9,:], color='cyan')
axes[0,0].set_xlabel('Time [s]', color='white')
axes[0,0].set_ylabel('North [m]', color='white')
if np.linalg.norm(x[9,:]) < 1e-6:
    axes[0,0].set_ylim(-0.05,0.05)
axes[0,0].grid(True)
axes[0,0].set_facecolor('black')
axes[0,0].tick_params(colors = 'white')

# East position p2^n_CM/T
axes[0,1].plot(t_s, x[10,:], color='cyan')
axes[0,1].set_xlabel('Time [s]', color='white')
axes[0,1].set_ylabel('East [m]', color='white')
if np.linalg.norm(x[10,:]) < 1e-6:
    axes[0,1].set_ylim(-0.05,0.05)
axes[0,1].grid(True)
axes[0,1].set_facecolor('black')
axes[0,1].tick_params(colors = 'white')

# Altitude
axes[0,2].plot(t_s, -x[11,:], color='cyan')
axes[0,2].set_xlabel('Time [s]', color='white')
axes[0,2].set_ylabel('Altitude [m]', color='white')
if np.linalg.norm(x[11,:]) < 1e-6:
    axes[0,2].set_ylim(-0.05,0.05)
axes[0,2].grid(True)
axes[0,2].set_facecolor('black')
axes[0,2].tick_params(colors = 'white')

# North vs East position p2^n_CM/T
axes[1,0].plot(x[10,:], x[9,:], color='cyan')
axes[1,0].set_xlabel('East [s]', color='white')
axes[1,0].set_ylabel('North [m]', color='white')
if np.linalg.norm(x[9,:]) < 1e-6:
    axes[1,0].set_ylim(-0.05,0.05)
if np.linalg.norm(x[10,:]) < 1e-6:
    axes[1,0].set_xlim(-0.05,0.05)
axes[1,0].grid(True)
axes[1,0].set_facecolor('black')
axes[1,0].tick_params(colors = 'white')

# Altitude vs East position p2^n_CM/T
axes[1,1].plot(x[10,:], -x[11,:], color='cyan')
axes[1,1].set_xlabel('East [s]', color='white')
axes[1,1].set_ylabel('Altitude [m]', color='white')
if np.linalg.norm(x[10,:]) < 1e-6:
    axes[1,1].set_xlim(-0.05,0.05)
if np.linalg.norm(x[11,:]) < 1e-6:
    axes[1,1].set_ylim(-0.05,0.05)
axes[1,1].grid(True)
axes[1,1].set_facecolor('black')
axes[1,1].tick_params(colors = 'white')

# Altitude vs North
axes[1,2].plot(x[9,:], -x[11,:], color='cyan')
axes[1,2].set_xlabel('North [s]', color='white')
axes[1,2].set_ylabel('Altitude [m]', color='white')
if np.linalg.norm(x[9,:]) < 1e-6:
    axes[1,2].set_xlim(-0.05,0.05)
if np.linalg.norm(x[11,:]) < 1e-6:
    axes[1,2].set_ylim(-0.05,0.05)
axes[1,2].grid(True)
axes[1,2].set_facecolor('black')
axes[1,2].tick_params(colors = 'white')

plt.tight_layout()
#plt.savefig('saved_figures/S1p4_Ex_5_Position_Plot.png')
plt.show(block=False)

# C. Create figure of air data
fig, axes = plt.subplots(1, 3, figsize=(10, 6))
fig.set_facecolor('black')  
fig.suptitle(vmod["V_name"], fontsize=14, fontweight='bold', color='magenta')

# Angle of attack
axes[0].plot(t_s, Alpha_rad*180/3.14, color='magenta')
axes[0].set_xlabel('Time [s]', color='white')
axes[0].set_ylabel('Angle of Attack [deg]', color='white')
axes[0].set_ylim(-90,90)
axes[0].grid(True)
axes[0].set_facecolor('black')
axes[0].tick_params(colors = 'white')

# Angle of side slip
axes[1].plot(t_s, Beta_rad*180/3.14, color='magenta')
axes[1].set_xlabel('Time [s]', color='white')
axes[1].set_ylabel('Angle of Side Slip [deg]', color='white')
axes[1].set_ylim(-90,90)
axes[1].grid(True)
axes[1].set_facecolor('black')
axes[1].tick_params(colors = 'white')

# Mach
axes[2].plot(t_s, Mach, color='magenta')
axes[2].set_xlabel('Time [s]', color='white')
axes[2].set_ylabel('Mach Number', color='white')
axes[2].grid(True)
axes[2].set_facecolor('black')
axes[2].tick_params(colors = 'white')

plt.tight_layout()
plt.show()