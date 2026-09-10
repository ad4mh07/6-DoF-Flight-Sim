import numpy as np

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
    