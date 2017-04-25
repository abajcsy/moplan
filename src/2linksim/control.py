import numpy as np

class PID(object):
    """
    The base class for PID controller.
    """
    def __init__(self, kp, ki, kd, target):
        """
        kp float: the position error term gain value
        kv float: the velocity error term gain value
        """

        self.u = np.zeros((2,1)) # control signal

        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.target = target
        self.p_error_last = None

        # generalized coordinates
        self.target_gain = 2*np.pi
        self.target_bias = -np.pi

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

        print "perror: " + str(p_error)

        if self.p_error_last is None:
            self.p_error_last = np.array([0.0, 0.0])
            
        # p-error
        p = self.kp * p_error
    
        # d-error
        d_error = (p_error - self.p_error_last) / dt
        self.p_error_last = p_error
        #d_error = -arm.dq        
        d = self.kd * d_error

        # TODO integral term is always zero 
        self.u = (p + d).reshape(-1,)

        # TODO TEST
        #q_des = (self.kp * p_error + self.kd * -arm.dq).reshape(-1,)
        #Mq = arm.gen_Mq()
        # tau = Mq * q_des + tau_grav, but gravity = 0
        #self.u = np.dot(Mq, q_des).reshape(-1,)

        return self.u

    def gen_target(self, arm):
        """Generate a random target"""
        self.target = np.random.random(size=arm.DOF,) * \
            self.target_gain + self.target_bias

        return self.target
