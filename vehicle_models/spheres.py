import numpy as np
import math

def BowlingBall():
    vehicle_name = 'Bowling Ball'

    rho_bowlingball_kgpm3 = 1500 #density
    r_sphere_m = 4.4*0.0254 #radius
    CD_approx = 0.5 #Approx drag coeff.

    vol_sphere_m3, m_sphere_kg, J_sphere_kgm2, Aref_m2 = CalcSphere(r_sphere_m, rho_bowlingball_kgpm3)
    Vterm_mps = np.sqrt((2*m_sphere_kg*9.81) / (1.2*CD_approx*Aref_m2))

    vmod={"V_name": vehicle_name, "m_kg": m_sphere_kg, \
           "Jxz_b_kgm2": 0, "Jxx_b_kgm2": J_sphere_kgm2, "Jyy_b_kgm2": J_sphere_kgm2, "Jzz_b_kgm2": J_sphere_kgm2, \
            "r_sphere_m": r_sphere_m, "m_sphere_kg": m_sphere_kg, "CD_approx": CD_approx, "Aref_m2" : Aref_m2,  "Vterm_mps": Vterm_mps }
    
    return vmod
def CalcSphere(r_sphere_m, rho_sphere_kgpm3):

    vol_sphere_m3 = 4/3 * np.pi * r_sphere_m**3
    m_sphere_kg = rho_sphere_kgpm3 * vol_sphere_m3
    J_sphere_kg = 0.4* m_sphere_kg * r_sphere_m
    Aref_m2 = np.pi * r_sphere_m**2

    return vol_sphere_m3, m_sphere_kg, J_sphere_kg, Aref_m2



u0_bf_mps = 0.001  #Avoids division by 0
v0_bf_mps = 0 
w0_bf_mps = 0 
p0_bf_rps = 0 
q0_bf_rps = 0 
r0_bf_rps = 0 
phi0_rad = 0* math.pi
theta0_rad = -89.99*math.pi/180 
psi0_rad = 0 
p10_n_m = 0 
p20_n_m = 0 
p30_n_m = -20000

x0= np.array([
    u0_bf_mps, # x axis body-fixed CS velocity
    v0_bf_mps, # y axis body-fixed CS velocity
    w0_bf_mps, # z axis body-fixed CS velocity
    p0_bf_rps, # roll rate 
    q0_bf_rps, # pitch rate
    r0_bf_rps, # yaw rate
    phi0_rad, # roll angle
    theta0_rad, # pitch angle
    psi0_rad, # psi angle
    p10_n_m, # x axis position wrt NED CS
    p20_n_m, # y axis position wrt NED CS
    p30_n_m, # z axis position wrt NED CS
    ])

nx0= x0.size

#Define timings
t0_s = 0
tf_s = 185
h_s = 0.01