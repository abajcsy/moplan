import numpy as np

class PID(object):
    """
    The base class for PID controller.
    """
    def __init__(self, kp, ki, kd, target, start, grav_comp, goal=None):
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
            self.goal = self.target
        else:
            self.goal = goal
    
        self.alpha = 1.0
        self.Tf = self.alpha*(np.linalg.norm(self.start-self.goal))

        self.reached_goal = False
        self.p_error_last = None
        self.i_error = np.array([[0.0, 0.0]])

        # generalized coordinates
        self.target_gain = 2*np.pi
        self.target_bias = -np.pi

    def check_distance(self, arm):
        """Checks the distance to target"""
        return np.sum(abs(arm.q - self.target)) 

    def is_at_goal(self, arm, epsilon=0.1):
        """Checks the if arm is within epsilon of dist to final goal"""
        dist_from_goal = np.fabs(arm.q - self.goal)
        #print "dist: " + str(dist_from_goal)

        # check if every joint is close enough to goal configuration
        close_to_goal = [dist_from_goal[i] < epsilon for i in range(len(dist_from_goal))]
        #print "close: " + str(close_to_goal)

        # if all joints are close enough, robot is at goal
        at_goal = all(close_to_goal)

        return at_goal

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
        d_error = (p_error - self.p_error_last) / dt
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

    def draw_lintraj(self, arm):
        """Returns sequence of (x,y) straight line trajectory pts"""
        qs = np.array([arm.q])
        X = np.array(0)
        Y = np.array(0)

        print "totalT: " + str(self.Tf)
        t = 0.0
        while t < self.Tf:
            target = (self.goal-self.start)*(1/self.Tf)*t + self.start
            qs = np.append(qs, [target], axis=0)
            t += 0.05 
        for q in qs:
            pos = arm.position(q)
            X = np.append(X, pos[0][2])
            Y = np.append(Y, pos[1][2])
        return X, Y

    def line_target(self, arm, dt=None, Fq=None):
        """Generates a linear in C-space trajectory target"""
        #self.start = arm.q

        #print Fq
        # if human exerting force, 
        # then determing which direction to scale alpha      
        if (Fq is not None) and np.linalg.norm(Fq) is not 0:
            magnitude = np.linalg.norm(Fq)
            # TODO: should this be (Fq - dirVecQ)? 
            direction = np.arctan2(Fq[0], Fq[1])  
            #print "mag, direction: " + str(magnitude) + ", " + str(direction)
        self.t_f = self.alpha*(np.linalg.norm(self.start-self.goal))

        if dt is None:
            t = arm.t    
        else: 
            t = dt
    
        #print "s: " + str(self.start)
        #print "g: " + str(self.goal)
        #print "original total time (Tf): " + str(self.Tf)
        #print "current total time (t_f): " + str(self.t_f) 
        #print "current t: " + str(t) 

        self.target = (self.goal-self.start)*(1/self.Tf)*t + self.start

        # if time after the final time, then just go to goal
        if t >= self.Tf or self.reached_goal or self.is_at_goal(arm):
            #print "is_at_goal"
            self.target = self.goal
            self.reached_goal = True

        return self.target

    def rand_target(self, arm):
        """Generate a random target"""
        self.target = np.random.random(size=arm.DOF,) * \
            self.target_gain + self.target_bias

        return self.target
