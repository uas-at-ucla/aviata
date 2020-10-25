import numpy as np

# Based on https://jckantor.github.io/CBE30338/04.01-Implementing_PID_Control_with_Python_Yield_Statement.html
# Modified so that Kd is applied to the derivative of the measured value (PV) as in PX4, instead of the error.
def PID(Kp, Ki, Kd):
    # initial control
    PV, SP, dt = yield
    e = SP - PV
    
    P = Kp*e
    I = Ki*e*dt
    D = np.zeros(np.shape(Kd))
    
    MV = P + I + D
    
    PV_prev = PV
    
    while True:
        # yield MV, wait for new t, PV, SP
        PV, SP, dt = yield MV
        
        # PID calculations
        e = SP - PV
        
        P = Kp*e
        I = I + Ki*e*dt
        D = Kd*(PV_prev - PV)/dt #TODO could add low-pass filter
        
        MV = P + I + D
        
        # update stored data for next iteration
        PV_prev = PV
