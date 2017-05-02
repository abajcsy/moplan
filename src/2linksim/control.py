import numpy as np

class PID(object):
    """
    The base class for PID controller.
    """
    def __init__(self, kp, ki, kd, start, target, grav_comp, goal=None, alpha=1.0):
        """
        kp float: the position error term gain value
        kv float: the velocity error term gain value
        """

        self.u = np.zeros((2,1)) # control signal

        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.target = target
        self.start = np.copy(start)

        # checks if should publish gravity term in all control
        self.grav_comp = grav_comp

        if goal is None:
            self.goal = np.copy(self.target)
        else:
            self.goal = goal
    
        self.p_error_last = None
        self.i_error = np.array([[0.0, 0.0]])


    def check_distance(self, arm):
        """Checks the distance to target"""
        return np.sum(abs(arm.q - self.target)) 

    def control(self, arm, dt, q_des=None):
        """
        Generate a control signal to move the arm through
        joint space to the desired joint angle position
        """ 
        # calculated desired joint angle acceleration
        if q_des is None:
            p_error = ((self.target.reshape(1,-1) - arm.q) + np.pi) % (np.pi*2) - np.pi
        else: 
            # if a desired location is specified on input
            p_error = q_des - arm.q

        if self.p_error_last is None:
            self.p_error_last = np.array([0.0, 0.0])
            
        # p-error
        p = self.kp * p_error

        # i-error
        self.i_error += p_error * dt 
        i = self.ki * self.i_error

        # d-error
        #d_error = (p_error - self.p_error_last) / dt
        d_error = -arm.dq        
        self.p_error_last = p_error
        d = self.kd * d_error

        # compute total command to send
        self.u = (p + i + d).reshape(-1,)

        # adds gravity compensation term if flag is set to true
        if self.grav_comp:
            Gq = arm.gen_Gq()
            self.u += Gq

        #q_des = (self.kp * p_error + self.kd * -arm.dq).reshape(-1,)
        #Mq = arm.gen_Mq()
        # tau = Mq * q_des + tau_grav
        #self.u = (np.dot(Mq, q_des) + Gq).reshape(-1,)
        #self.u = (q_des + Gq).reshape(-1,)
        
        return self.u

