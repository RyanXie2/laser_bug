import math

class PIDController:

    def __init__(self, Kp, Ki, Kd):
        self.E_d = 0
        self.E_i = 0

        # PID gains
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
    
    def step(self, error, dt = 0.01):

        # Error for the proportional term
        e_P = error
        
        # Error for the integral term.
        e_I = self.E_i + error * dt
                 
        # Error for the derivative term.
        e_D = (error - self.E_d)/dt

        w = self.Kp * e_P + self.Ki * e_I + self.Kd * e_D

        # Save errors for next time step
        self.E_i = e_I
        self.E_d = error

        return w