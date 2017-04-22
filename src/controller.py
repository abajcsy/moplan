import numpy as np
from numpy import linalg

class Controller(object):

    def __init__(self, p_gain, i_gain, d_gain, i_min, i_max, dim):
        """Constructor, zeros out Pid values when created and
        initialize Pid-gains and integral term limits. All gains are 
		(dim) x (dim) matrices.

        Parameters:
          p_gain     The proportional gain.
          i_gain     The integral gain.
          d_gain     The derivative gain.
          i_min      The integral lower limit. 
          i_max      The integral upper limit.
        """
        self.set_gains(p_gain, i_gain, d_gain, i_min, i_max)
        self.reset()
        self.dim = dim;

    def reset(self):
        """  Reset the state of this PID controller """
        self._p_error_last = np.zeros((self.dim,1)) # Save position state for derivative
                                 # state calculation.
        self._p_error = np.zeros((self.dim,1))  # Position error.
        self._d_error = np.zeros((self.dim,1))  # Derivative error.
        self._i_error = np.zeros((self.dim,1))  # Integator error.
        self._cmd = np.zeros((self.dim,self.dim))  # Command to send.
        self._last_time = None # Used for automatic calculation of dt.
        
    def set_gains(self, p_gain, i_gain, d_gain, i_min, i_max): 
        """ Set PID gains for the controller. 

         Parameters:
          p_gain     The proportional gain.
          i_gain     The integral gain.
          d_gain     The derivative gain.
          i_min      The integral lower limit. 
          i_max      The integral upper limit.
        """ 
        self._p_gain = p_gain
        self._i_gain = i_gain
        self._d_gain = d_gain
        self._i_min = i_min
        self._i_max = i_max

    @property
    def p_gain(self):
        """ Read-only access to p_gain. """
        return self._p_gain

    @property
    def i_gain(self):
        """ Read-only access to i_gain. """
        return self._i_gain

    @property
    def d_gain(self):
        """ Read-only access to d_gain. """
        return self._d_gain

    @property
    def i_max(self):
        """ Read-only access to i_max. """
        return self._i_max

    @property
    def i_min(self):
        """ Read-only access to i_min. """
        return self._i_min

    @property
    def p_error(self):
        """ Read-only access to p_error. """
        return self._p_error

    @property
    def i_error(self):
        """ Read-only access to i_error. """
        return self._i_error

    @property
    def d_error(self):
        """ Read-only access to d_error. """
        return self._d_error

    @property
    def cmd(self):
        """ Read-only access to the latest command. """
        return self._cmd

    @property
    def last_time(self):
       """ Read-only access to the last time. """
       return self._last_time

    def __str__(self):
        """ String representation of the current state of the controller. """
        result = ""
        result += "p_gain:  " + str(self.p_gain) + "\n"
        result += "i_gain:  " + str(self.i_gain) + "\n"
        result += "d_gain:  " + str(self.d_gain) + "\n"
        result += "i_min:   " + str(self.i_min) + "\n"
        result += "i_max:   " + str(self.i_max) + "\n"
        result += "p_error: " + str(self.p_error) + "\n"
        result += "i_error: " + str(self.i_error) + "\n"
        result += "d_error: " + str(self.d_error) + "\n"
        result += "cmd:     " + str(self.cmd) + "\n"
        return result
        
    def update_PID(self, p_error, dt=None):
        """  Update the Pid loop with nonuniform time step size.

        Parameters:
          p_error  Error since last call (target - state)
          dt       Change in time since last call, in seconds, or None. 
                   If dt is None, then the system clock will be used to 
                   calculate the time since the last update. 
        """
        if dt == None:
            cur_time = time.time()
            if self._last_time is None:
                self._last_time = cur_time 
            dt = cur_time - self._last_time
            self._last_time = cur_time

        #print "in update_PID(): dt: " + str(dt)

        self._p_error = p_error 
        if dt == 0 or math.isnan(dt) or math.isinf(dt):
            return np.zeros((self.dim,self.dim)) # TODO or shold it be 0.0??

        # Calculate proportional contribution to command
        p_term = self._p_gain * self._p_error
		
        #print "in update_PID(): p_term: " + str(p_term)
        #print "in update_PID(): p_gain:" + str(self._p_gain)

        # Calculate the integral error
        self._i_error += dt * self._p_error 
        
        # Calculate integral contribution to command
        i_term = self._i_gain * self._i_error
        
        # Limit i_term so that the limit is meaningful in the output
        """
        if i_term > self._i_max and self._i_gain != 0:
            i_term = self._i_max
            self._i_error = i_term / self._i_gain
        elif i_term < self._i_min and self._i_gain != 0:
            i_term = self._i_min
            self._i_error = i_term / self._i_gain
        """
    
        # Calculate the derivative error
        self._d_error = (self._p_error - self._p_error_last) / dt
        self._p_error_last = self._p_error

        #print "in update_PID(): p_error: " + str(self._p_error)
        #print "in update_PID(): i_error: " + str(self._i_error)
        #print "in update_PID(): d_error: " + str(self._d_error)
        
        # Calculate derivative contribution to command 
        d_term = self._d_gain * self._d_error
        
        self._cmd = p_term + i_term + d_term

        return self._cmd

