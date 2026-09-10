import numpy as np
import matplotlib.pyplot as plt
import ussa1976

from numerical_integrators import numerical_integration_methods
from governing_equations import flat_earth_eom

"""from tools.interpolators import fastInterp1
from vehcile_models.sphere import spheres"""



"""from flat_earth_eom import flat_earth_eom
from numerical_integration_methods import forward_euler"""

#==================================================
# 1: Initialisation
#==================================================

#Defining the vehicle (a sphere), and the corresponding amod
r_sphere_kgm2 = 0.08
m_sphere_kg = 5
J_sphere_kgm2 = 0.4 * m_sphere_kg * r_sphere_kgm2**2

amod={"m_kg":m_sphere_kg, "Jxz_b_kgm2":0, "Jxx_b_kgm2":J_sphere_kgm2, "Jyy_b_kgm2":J_sphere_kgm2, "Jzz_b_kgm2":J_sphere_kgm2}


#Initial conditions creation

u0_bf_mps = 0 # x axis body-fixed CS velocity
v0_bf_mps = 0 # y axis body-fixed CS velocity
w0_bf_mps = 0 # z axis body-fixed CS velocity
p0_bf_rps = 0 # roll rate 
q0_bf_rps = 0 # pitch rate
r0_bf_rps = 0 # yaw rate
phi0_rad = np.pi/2 # roll angle
theta0_rad = 0 * np.pi /180 # pitch angle
psi0_rad = 0 # psi angle
p10_n_m = 0 # x axis position wrt NED CS
p20_n_m = 0 # y axis position wrt NED CS
p30_n_m = 0 # z axis position wrt NED CS

x0= np.array([
    u0_bf_mps,
    v0_bf_mps, 
    w0_bf_mps,
    p0_bf_rps,
    q0_bf_rps,
    r0_bf_rps,
    phi0_rad,
    theta0_rad,
    psi0_rad,
    p10_n_m,
    p20_n_m,
    p30_n_m,
    ])

"x0= x0.transpose() #inital conditions now a collumn vector"
nx0= x0.size

#Define timings
t0_s = 0
tf_s = 10.0
h_s = 0.005

#==================================================
# 2: Numerical approximation
#==================================================

#Preallocate the solution array (creates a list of times with step size h_s from t0_s to tf_s)
t_s = np.arange(t0_s, tf_s + h_s, h_s) 
nt_s = t_s.size #number of time points

x = np.zeros((nx0, nt_s), dtype = float) #creates empty solution array

#Assign initial conditions (x0) to the solution array (x)
x[:, 0] = x0

t_s, x = numerical_integration_methods.forward_euler(lambda t, x: flat_earth_eom(t, x, amod), t_s, x, h_s)

#Data post-processing (tbc)

#==================================================
# 3: Plotting
#==================================================

fig, axes = plt.subplots(4, 3, figsize=(12,10))

# (x-) Axial velocity, u_b_mps
axes[0,0].plot(t_s, x[0,:], label='Axial velocity', color='yellow')
axes[0,0].set_xlabel('Time, s', color='black')
axes[0,0].set_ylabel('u, mps')
axes[0,0].grid(True)

# y-axis velocity, v_b_mps
axes[0,1].plot(t_s, x[1,:], label='y-axis velocity', color='yellow')
axes[0,1].set_xlabel('Time, s', color='black')
axes[0,1].set_ylabel('v, mps')
axes[0,1].grid(True)

# z-axis velocity, w_b_mps
axes[0,2].plot(t_s, x[2,:], label='z-axis velocity', color='yellow')
axes[0,2].set_xlabel('Time, s', color='black')
axes[0,2].set_ylabel('w, mps')
axes[0,2].grid(True)


# roll angular rate, p_b_rps
axes[1,0].plot(t_s, x[3,:], label='roll angular rate', color='orange')
axes[1,0].set_xlabel('Time, s', color='black')
axes[1,0].set_ylabel('p, rps')
axes[1,0].grid(True)

# pitch angular rate, q_b_rps
axes[1,1].plot(t_s, x[4,:], label='pitch angular rate', color='orange')
axes[1,1].set_xlabel('Time, s', color='black')
axes[1,1].set_ylabel('q, rps')
axes[1,1].grid(True)

# yaw angular rate, r_b_rps
axes[1,2].plot(t_s, x[5,:], label='yaw angular rate', color='orange')
axes[1,2].set_xlabel('Time, s', color='black')
axes[1,2].set_ylabel('r, rps')
axes[1,2].grid(True)


# roll angle, phi_r
axes[2,0].plot(t_s, x[6,:], label='roll angle', color='red')
axes[2,0].set_xlabel('Time, s', color='black')
axes[2,0].set_ylabel('phi, rad')
axes[2,0].grid(True)

# pitch angle, theta_r
axes[2,1].plot(t_s, x[7,:], label='pitch angle', color='red')
axes[2,1].set_xlabel('Time, s', color='black')
axes[2,1].set_ylabel('theta, rad')
axes[2,1].grid(True)

# yaw angle, psi_r
axes[2,2].plot(t_s, x[8,:], label='yaw angle', color='red')
axes[2,2].set_xlabel('Time, s', color='black')
axes[2,2].set_ylabel('psi, rad')
axes[2,2].grid(True)


#NED x-axis position, m
axes[3,0].plot(t_s, x[9,:], label='NED x-axis pos.', color='purple')
axes[3,0].set_xlabel('Time, s', color='black')
axes[3,0].set_ylabel('p1, m')
axes[3,0].grid(True)

#NED y-axis position, m
axes[3,1].plot(t_s, x[10,:], label='NED y-axis pos.', color='purple')
axes[3,1].set_xlabel('Time, s', color='black')
axes[3,1].set_ylabel('p2, m')
axes[3,1].grid(True)

#NED z-axis position, m
axes[3,2].plot(t_s, x[11,:], label='NED z-axis pos.', color='purple')
axes[3,2].set_xlabel('Time, s', color='black')
axes[3,2].set_ylabel('p3, m')
axes[3,2].grid(True)

plt.tight_layout()
plt.show()

print("done")


