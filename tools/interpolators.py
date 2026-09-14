import numpy as np 

def fastInterp1 (x, y, xi):
    """
    Performs linear interpolation
    
    Inputs:
        x- array of independent data points
        y- array of dependent data points
        xi- x value to interpolate the y value of
    """

    x = x.reshape(-1,1) #ensures x is a column vector 

    if len(x) != len(y):
        raise ValueError("Lengths of x & y must be euqal")
    
    #Creates interval arrays
    iia_r1 = np.array( [-np.inf, x[0,0], y[0], y[0], 1])
    iia_mr = np.array( [x[:-1,0], x[1:,0], y[:-1], y[1:], 2 * np.ones(len(x)-1)] )
    iia_mr = iia_mr.transpose()
    iia_rn = np.array([ x[-1,0], np.inf, y[-1], y[-1], 3])
    iia = np.vstack([iia_r1, iia_mr, iia_rn])

    #Find relevant xi info
    xyc= iia[ (xi> iia[:,0]) & (xi <= iia[:,1]), :]

    #Extract data from the interval
    x0 = xyc[0,0]
    x1 = xyc[0,1]
    y0 = xyc[0,2]
    y1 = xyc[0,3]
    ic = xyc[0,4]
    #Perform interpolation

    if ic ==2:
        yi = (y0 * (x1 - xi) + y1 * (xi - x0)) / (x1 - x0)
    elif ic == 1:
      yi = y0
    elif ic == 3:
      yi = y1
    else:
      raise RuntimeError("Interpolation failure.")

    return yi
