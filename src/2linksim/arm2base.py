'''
2-link arm base class. 

Based off of studywolf_control library: 
https://github.com/studywolf/control
'''

from armbase import ArmBase
import numpy as np

class Arm2Base(ArmBase):
    """
    Base information for simulation of a two-link arm.
    """

    def __init__(self, init_q=[.75613, 1.8553], init_dq=[0.0,0.0],
                    l1=.31, l2=.27, dt=None):


        self.DOF = 2
        ArmBase.__init__(self, init_q=init_q, init_dq=init_dq, singularity_thresh=.00025)

        # length of arm links
        self.l1 = l1
        self.l2 = l2
        self.L = np.array([self.l1, self.l2])
        # mass of links
        self.m1 = 1.98
        self.m2 = 1.32
        # z axis inertia moment of links
        izz1 = 15.0; izz2 = 8.0
        # create mass matrices at COM for each link
        self.M1 = np.zeros((6,6))
        self.M2 = np.zeros((6,6))
        self.M1[0:3,0:3] = np.eye(3)*self.m1
        self.M1[3:,3:] = np.eye(3)*izz1
        self.M2[0:3,0:3] = np.eye(3)*self.m2
        self.M2[3:,3:] = np.eye(3)*izz2

        self.rest_angles = np.array([np.pi/4.0, np.pi/4.0])

        # calling reset sets up:
        #   self.q = self.init_q
        #   self.dq = self.init_dq
        #   self.t = 0.0
        self.reset()

        if dt is not None:
            self.dt = dt

        print "dt: " + str(self.dt)

        # compute non changing constants
        self.K1 = (self.m1 + self.m2) * self.l1**2.0 + self.m2 * self.l2**2.0
        self.K2 = 2.0 * self.m2 * self.l1 * self.l2
        self.K3 = self.m2 * self.l2**2.0
        self.K4 = self.m2 * self.l1 * self.l2

    def gen_jacCOM1(self, q=None):
        """
        Generates the Jacobian from the COM of the first
        link to the origin frame
        """
        q = self.q if q is None else q

        JCOM1 = np.zeros((6,2))
        JCOM1[0,0] = self.l1 * -np.sin(q[0])
        JCOM1[1,0] = self.l1 * np.cos(q[0])
        JCOM1[5,0] = 1.0

        return JCOM1

    def gen_jacCOM2(self, q=None):
        """
        Generates the Jacobian from the COM of the second
        link to the origin frame
        """
        q = self.q if q is None else q

        JCOM2 = np.zeros((6,2))
        # define column entries right to left
        JCOM2[0,1] = self.l2 * -np.sin(q[0]+q[1])
        JCOM2[1,1] = self.l2 * np.cos(q[0]+q[1])
        JCOM2[5,1] = 1.0

        JCOM2[0,0] = self.l1 * -np.sin(q[0]) + JCOM2[0,1]
        JCOM2[1,0] = self.l1 * np.cos(q[0]) + JCOM2[1,1]
        JCOM2[5,0] = 1.0

        return JCOM2

    def gen_jacEE(self, q=None):
        """
        Generates the Jacobian from end-effector to the origin frame
        """
        q = self.q if q is None else q

        JEE = np.zeros((2,2))
        # define column entries right to left
        JEE[0,1] = -self.l2 * np.sin(q[0]+q[1])
        JEE[1,1] = self.l2 * np.cos(q[0]+q[1])

        JEE[0,0] = -self.l1 * np.sin(q[0]) + JEE[0,1]
        JEE[1,0] = self.l1 * np.cos(q[0]) + JEE[1,1]

        return JEE

    def gen_Mq(self, q=None):
        """
        Generates the mass matrix for the arm in joint space
        """
        # get the instantaneous Jacobians
        JCOM1 = self.gen_jacCOM1(q=q)
        JCOM2 = self.gen_jacCOM2(q=q)
        # generate the mass matrix in joint space
        Mq = np.dot(JCOM1.T, np.dot(self.M1, JCOM1)) + \
             np.dot(JCOM2.T, np.dot(self.M2, JCOM2))

        return Mq

    def inv_kinematics(self, xy):
        """
        Calculate the joint angles for a given (x,y) hand position
        """
        import scipy.optimize
        # function to optimize
        def distance_to_target(q, xy, L):
            x = L[0] * np.cos(q[0]) + L[1] * np.cos(q[0] + q[1])
            y = L[0] * np.sin(q[0]) + L[1] * np.sin(q[0] + q[1])
            return np.sqrt((x - xy[0])**2 + (y - xy[1])**2)

        return scipy.optimize.minimize(fun=distance_to_target, x0=self.q,
                args=([xy[0], xy[1]], self.L))['x']

    def position(self, q=None):
        """
        Compute (x,y) position of the hand

        q np.array: a set of angles to return positions for

        Pos_Elbow = [ l1*cos(q0); <--(x1)
                      l1*sin(q0); <--(y1)
                      0 ]

        Pos_EE = [ l1*cos(q0) + l2*cos(q0+q1); <--(x2)
                   l1*sin(q0) + l2*sin(q0+q1); <--(y2)
                   0 ]
        """
        q = self.q if q is None else q

        x = np.cumsum([0,
                       self.l1 * np.cos(q[0]),
                       self.l2 * np.cos(q[0]+q[1])])
        y = np.cumsum([0,
                       self.l1 * np.sin(q[0]),
                       self.l2 * np.sin(q[0]+q[1])])
        return np.array([x, y])

    def apply_torque(self, u, dt=None):
        """
        Takes in a torque and time step and updates the
        arm simulation accordingly.
        
        Solves for ddq, dq, q given u: 
            u = M(q)*ddq + V(q,dq) + G(q)

        u np.array: the control signal to apply
        dt float: the time step
        """
        if dt is None:
            dt = self.dt

        # equations solved for angles
        C1 = np.cos(self.q[0])
        C2 = np.cos(self.q[1])
        S2 = np.sin(self.q[1])
        C12 = np.cos(self.q[0] + self.q[1])

        # generate entries of M(q) mass matrix
        M11 = (self.K1 + self.K2*C2)
        M12 = (self.K3 + self.K4*C2)
        M21 = M12
        M22 = self.K3

        # generate coriolis forces matrix V(q,dq)
        V1 = -self.K4*S2*(2.0*self.dq[0]*self.dq[1] +self.dq[1]**2.0) 
        V2 = self.K4*S2*self.dq[0]**2.0

        # generate gravity forces matrix G(q)
        g = -9.8
        G1 = (self.m1 + self.m2)*g*self.l1*C1 + self.m2*g*self.l2*C12
        G2 = self.m2*g*self.l2*C12

        ddq1 = (M11*u[1] - M21*u[0] + M21*V1 + M21*G1 - M11*V2 - M11*G2) / (M11*M22 - M12**2.0)
        ddq0 = (u[0] - M12*ddq1 - V1 - G1) / M11

        #ddq1 = ((V2*M11 - V1*M21 - M11*u[1] + M21*u[0]) / (M12**2.0 - M11*M22))
        #ddq0 = (-V2 + u[1] - M22*ddq1) / M21
        """
        print "Old state:"
        print "q: " + str(self.q)
        print "dq: " + str(self.dq)
        print "t: " + str(self.t)
        """
        self.dq += np.array([ddq0, ddq1]) * dt
        self.q += self.dq * dt

        # transfer to next time step
        self.t += dt
        """    
        print "New state after torque:"
        print "q: " + str(self.q)
        print "dq: " + str(self.dq)
        print "t: " + str(self.t)
        """

    def plot(self, plt):
        """
        Plots the links of the arm and the joints.

        plt matplitlib fig: plot to display arm on
        """
        pos = self.position()
        
        plt.plot([pos[0][0], pos[0][1]], [pos[1][0], pos[1][1]], 'b', linewidth=5)
        plt.plot([pos[0][1], pos[0][2]], [pos[1][1], pos[1][2]], 'b', linewidth=5)

        # plot joints
        plt.plot(0,0,'ko')
        plt.plot(pos[0][1], pos[1][1], 'ko') 
        plt.plot(pos[0][2], pos[1][2], 'ko')

