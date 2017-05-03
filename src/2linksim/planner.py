import numpy as np

class Planner(object):
    
    def __init__(self, start, goal, alpha=1.0):
        self.start = np.copy(start)
        self.goal = np.copy(goal)

        self.alpha = alpha
        self.Tf = self.alpha*(np.linalg.norm(self.start-self.goal))
        print "Original totalT: " + str(self.Tf)


        self.reached_goal = False

        # generalized coordinates
        self.target_gain = 2*np.pi
        self.target_bias = -np.pi

    def is_at_goal(self, arm, epsilon=0.1):
        """Checks the if arm is within epsilon of dist to final goal"""
        dist_from_goal = np.fabs(arm.q - self.goal)
        
        # check if every joint is close enough to goal configuration
        close_to_goal = [dist_from_goal[i] < epsilon for i in range(len(dist_from_goal))]
        
        # if all joints are close enough, robot is at goal
        at_goal = all(close_to_goal)

        return at_goal

    def draw_lintraj(self, arm):
        """Returns sequence of (x,y) straight line trajectory pts"""
        qs = np.array([arm.q])
        X = np.array(0)
        Y = np.array(0)

        t_f = self.alpha*(np.linalg.norm(self.start-self.goal))

        #print "totalT: " + str(t_f)

        t = 0.0
        while t < t_f:
            target = (self.goal-self.start)*(1/t_f)*t + self.start
            qs = np.append(qs, [target], axis=0)
            t += 0.05 
        for q in qs:
            pos = arm.position(q)
            X = np.append(X, pos[0][2])
            Y = np.append(Y, pos[1][2])
        return X, Y

    def line_target(self, arm, dt=None, Fq=None, replan=False):

        """Generates a linear in C-space trajectory target"""
        if replan:
            self.start = arm.q

        # if human exerting force, 
        # then determing which direction to scale alpha      
        if (Fq is not None) and (np.linalg.norm(Fq) != 0.0):
            norm_Fq = np.linalg.norm(Fq)
            norm_armq = np.linalg.norm(arm.q)
            direction  = np.dot(Fq, arm.q)
            
            print "alpha: " + str(self.alpha)
            #self.start = arm.q
            if np.sign(direction) < 0:
                if self.alpha + 0.01 > 2.0:
                    self.alpha = 2.0 
                else:
                    self.alpha += 0.01
            elif np.sign(direction) > 0:
                if self.alpha - 0.01 < 0.1:
                    self.alpha = 0.01
                else: 
                    self.alpha -= 0.01

        t_f = self.alpha*(np.linalg.norm(self.start-self.goal))

        if dt is None:
            t = arm.t    
        else: 
            t = dt
    
        #print "original total time (Tf): " + str(self.Tf)
        #print "current total time (t_f): " + str(t_f) 

        """
        print "s: " + str(self.start)
        print "g: " + str(self.goal)
        print "original total time (Tf): " + str(self.Tf)
        print "current total time (t_f): " + str(t_f) 
        print "current t: " + str(t) 
        """        
    
        self.target = (self.goal-self.start)*(1/t_f)*t + self.start

        # if time after the final time, then just go to goal
        if t >= t_f or self.reached_goal or self.is_at_goal(arm):
            #print "is_at_goal"
            self.target = self.goal
            self.reached_goal = True

        return self.target

    def rand_target(self, arm):
        """Generate a random target"""
        self.target = np.random.random(size=arm.DOF,) * \
            self.target_gain + self.target_bias

        return self.target
