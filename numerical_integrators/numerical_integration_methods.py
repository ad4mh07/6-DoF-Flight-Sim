import numpy as np

def forward_euler(f, t_s, x, h_s, vmod, amod):
    """
    Performs euler approximation as integration

    Inputs:
        f - a function, representing the rhs of the ODE
        t_s - a vector of points in time at which to approximate the solutions
        x -  numerically approximated solution data to f, the ODE
        h_s - step size, in seconds

    Outputs:
        t_s - a vector of points in time at which the soloutions were approximated
        x - numerically approximated solution data to f, the ODE
    """

    for i in range(1, len(t_s)):
        x[:,i] = x[:,i-1] + h_s * f(t_s[i-1], x[:,i-1], vmod, amod)

    return t_s, x