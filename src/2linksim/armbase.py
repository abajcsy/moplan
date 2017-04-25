'''
Base robotic arm class.

Based off of studywolf_control library: 
https://github.com/studywolf/control
'''
import numpy as np

class ArmBase:
    """ A base class for arm simulators. """

    def __init__(self, init_q=None, init_dq=None,
                 dt=1e-5, singularity_thresh=.00025, options=None):
        """
        dt float: the timestep for simulation
        singularity_thresh float: the point at which to singular values
                                  from the matrix SVD to zero.
        """

        self.dt = dt
        self.options = options
        self.singularity_thresh = singularity_thresh

        self.init_q = np.zeros(self.DOF) if init_q is None else init_q
        self.init_dq = np.zeros(self.DOF) if init_dq is None else init_dq

    def apply_torque(self, u, dt):
        """
        Takes in a torque and timestep and updates the
        arm simulation accordingly.

        u np.array: the control signal to apply
        dt float: the timestep
        """
        raise NotImplementedError

    def gen_jacEE(self):
        """
        Generates the Jacobian from end-effector to
        the origin frame
        """
        raise NotImplementedError

    def gen_Mq(self):
        """
        Generates the mass matrix for the arm in joint space
        """
        raise NotImplementedError

    def position(self, q=None):
        """
        Compute (x,y) position of the robot's hand

        q list: a list of the joint angles,
                if None use current system state
        """
        raise NotImplementedError

    def reset(self, q=[], dq=[]):
        """
        Resets the state of the arm

        q list: a list of the joint angles
        dq list: a list of the joint velocities
        """
        if isinstance(q, np.ndarray):
            q = q.tolist()
        if isinstance(dq, np.ndarray):
            dq = dq.tolist()

        if q:
            assert len(q) == self.DOF
        if dq:
            assert len(dq) == self.DOF

        self.q = np.copy(self.init_q) if not q else np.copy(q)
        self.dq = np.copy(self.init_dq) if not dq else np.copy(dq)
        self.t = 0.0

    def update_state(self):
        """
        Update the state
        """
        pass

    @property
    def x(self):
        """
        Returns (x,y) coord of hand
        """
        return self.position()[:, -1]
