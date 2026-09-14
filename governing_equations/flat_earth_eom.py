import numpy as np
from tools.interpolators import fastInterp1

def flat_earth_eom(t, x, vmod, amod):
    """
    Variable naming: 
    <variable name>_(<coordinate system>)_<units>

    Arguments:
    t - time [s], scalar
    x- state vector at time t [various units], np.array
        x[0] - u_b_mps, x (in body CS) velocity of CoM (wrt inertial plane) resolved in body CS
        x[1] - v_b_mps, y velocity of CoM (wrt inertial plane) resolved in body CS
        x[2] - w_b_mps, z velocity of CoM (wrt inertial plane) resolved in body CS
        x[3] - p_b_mps, roll angular velocity of body CS wrt inertial CS
        x[4] - q_b_mps, pitch angular velocity of body CS wrt inertial CS
        x[5] - r_b_mps, yaw angular velocity of body CS wrt inertial CS
        x[6] - phi_rad, roll angle
        x[7] - theta_rad, pitch angle
        x[8] - psi_rad, yaw angle
        x[9] - p1_n_m, x_axis (in NED CS) position of aircraft resolved in NED CS
        x[10] - p2_n_m, y_axis position of aircraft resolved in NED CS
        x[11] - p3_n_m, z_axis position of aircraft resolved in NED CS
    amod- aircraft model data, stored as a dict. with various parameters

    Output:
    dx- the time derivative of each sub-argument in x. 
    """

    dx=np.zeros(12)
    
    u_b_mps = x[0] #assign state values 
    v_b_mps = x[1]
    w_b_mps = x[2]
    p_b_rps = x[3]
    q_b_rps = x[4]
    r_b_rps = x[5]
    phi_rad = x[6]
    theta_rad = x[7]
    psi_rad = x[8]
    p1_n_m = x[9]
    p2_n_m = x[10]
    p3_n_m = x[11]

    m_kg = vmod["m_kg"] # mass and moments from the vmod
    Jxz_b_kgm2 = vmod["Jxz_b_kgm2"]
    Jxx_b_kgm2 = vmod["Jxx_b_kgm2"]
    Jyy_b_kgm2 = vmod["Jyy_b_kgm2"]
    Jzz_b_kgm2 = vmod["Jzz_b_kgm2"]

    #trig calc on euler angles (for ease later)
    c_phi = np.cos(phi_rad)
    c_theta = np.cos(theta_rad)
    c_psi = np.cos(psi_rad)
    s_phi =np.sin(phi_rad)
    s_theta = np.sin(theta_rad)
    s_psi = np.sin(psi_rad)
    t_theta = np.tan(theta_rad)


    ## Atmosphere model

    #current altitude
    h_m = -p3_n_m

    #air density (rho_interp_kgpm3 = 1.2)
    rho_interp_kgpm3= fastInterp1(amod["alt_m"], amod["rho_kgpm3"], h_m) 

    #speed of sound
    c_interp_mps = fastInterp1(amod["alt_m"], amod["c_mps"], h_m) 

    #rue airspeed
    true_airspeed_mps = np.sqrt(u_b_mps**2 + v_b_mps**2 + w_b_mps**2)

    #air pressure
    qbar_kgpms2 = 0.5 *rho_interp_kgpm3 *true_airspeed_mps 

    #Avoiding div by 0 when calc.ing angle of attack etc
    if true_airspeed_mps == 0 and v_b_mps == 0:
        v_over_VT = 0
    else:
        v_over_VT = v_b_mps / true_airspeed_mps

    alpha_rad = np.atan2(w_b_mps, u_b_mps)
    beta_rad  = np.asin(v_over_VT)
    s_alpha   = np.sin(alpha_rad)
    c_alpha   = np.cos(alpha_rad)
    s_beta    = np.sin(beta_rad)
    c_beta    = np.cos(beta_rad)

    #Gravity (it acts normal to Earth tangent CS) (gz_interp_n_mps2 = 9.81)
    gz_interp_n_mps2 = fastInterp1(amod["alt_m"], amod['g_mps2'], h_m)

    gx_b_mps2 = -s_theta*gz_interp_n_mps2
    gy_b_mps2 =  s_phi*c_theta*gz_interp_n_mps2
    gz_b_mps2 =  c_phi*c_theta*gz_interp_n_mps2


    #Aerodynamic forces (used in calc of ext. forces)
    drag_kgmps2 = -s_theta * gz_interp_n_mps2
    side_kgmps2 = 0
    lift_kgmps2 = 0

    # External forces
    Fx_b_kgmps2 = -(c_alpha*c_beta*drag_kgmps2 - c_alpha*s_beta*side_kgmps2 - s_alpha*lift_kgmps2)
    Fy_b_kgmps2 = -(s_beta*drag_kgmps2 + c_beta*side_kgmps2)
    Fz_b_kgmps2 = -(s_alpha*c_beta*drag_kgmps2 - s_alpha*s_beta*side_kgmps2 + c_alpha*lift_kgmps2)

    

    #External momemnts (tbc)
    l_b_kgm2ps2= 0
    m_b_kgm2ps2= 0
    n_b_kgm2ps2= 0



    #x,y,z velocity equations resp.

    dx[0]= (1/m_kg * Fx_b_kgmps2) + (gx_b_mps2) - (w_b_mps * q_b_rps) + (v_b_mps * r_b_rps) #negative included on line 57

    dx[1]= (1/m_kg * Fy_b_kgmps2) + (gy_b_mps2) - (u_b_mps * r_b_rps) + (w_b_mps * p_b_rps)

    dx[2]= (1/m_kg * Fz_b_kgmps2) + (gz_b_mps2) - (v_b_mps * p_b_rps) + (u_b_mps * q_b_rps)

    #Roll, pitch, yaw equations resp.

    dx[3]= ( Jxz_b_kgm2 * (Jxx_b_kgm2 - Jyy_b_kgm2 + Jzz_b_kgm2)*p_b_rps * q_b_rps - \
            (Jzz_b_kgm2 * (Jzz_b_kgm2 - Jyy_b_kgm2) + Jxz_b_kgm2**2) * q_b_rps * r_b_rps + \
            Jzz_b_kgm2 * l_b_kgm2ps2 + Jxz_b_kgm2 * n_b_kgm2ps2 ) / (Jxx_b_kgm2 * Jzz_b_kgm2 - Jxz_b_kgm2**2)
    
    dx[4] = ( (Jzz_b_kgm2 - Jxx_b_kgm2) * r_b_rps * p_b_rps - Jxz_b_kgm2 * (p_b_rps**2 - r_b_rps**2) + m_b_kgm2ps2 ) / Jyy_b_kgm2

    dx[5]= ( -Jxz_b_kgm2 * (Jxx_b_kgm2 - Jyy_b_kgm2 + Jzz_b_kgm2) * q_b_rps * r_b_rps + \
        (Jxx_b_kgm2 * (Jxx_b_kgm2 - Jyy_b_kgm2) + Jxz_b_kgm2**2) * p_b_rps * q_b_rps + \
        Jxz_b_kgm2 * l_b_kgm2ps2 + Jxx_b_kgm2 * n_b_kgm2ps2) / (Jxx_b_kgm2 * Jzz_b_kgm2 - Jxz_b_kgm2**2)

    #Kinematic equations
    dx[6] = p_b_rps + s_phi * t_theta * q_b_rps + c_phi * t_theta * r_b_rps

    dx[7] = c_phi * q_b_rps + c_phi / c_theta * r_b_rps

    dx[8] = s_phi / c_theta * q_b_rps + c_phi / c_theta * r_b_rps

    #Position/nav equations
    dx[9] = c_theta*c_psi*u_b_mps + (s_phi*s_theta*c_psi - c_phi*s_psi)*v_b_mps + (c_phi*s_theta*c_psi + s_phi*s_psi)*w_b_mps
    
    dx[10] = c_theta*s_psi*u_b_mps + (c_phi*c_psi+s_phi*s_theta*s_psi)*v_b_mps + (-s_phi*c_psi + c_phi*s_theta*s_psi)*w_b_mps

    dx[11] = -s_theta*u_b_mps + s_phi*c_theta*v_b_mps + c_phi*c_theta*w_b_mps

    return dx
