import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation

import arm2base
from arm2base import Arm2Base
import control
from control import PID

import sys, select, os

RAND = "-r"
LINEAR = "-l"
INFERENCE = "-i"
FORCE = "-f"

INTERACTIVE = "-iact"

class Simulator(object):

    def __init__(self, title='', dt=1e-3, control_steps=1, 
                    display_steps=1, t_target=1.0, sim_type=RAND, iactive=False):
        self.dt = dt
        self.control_steps = control_steps
        self.display_steps = display_steps
        self.target_steps = int(t_target/float(dt*display_steps))

        self.title = title
        self.sim_step = 0
        self.sim_type = sim_type
        self.iactive = iactive

        print "sim_type: " + str(self.sim_type)

        self.tau = None

    def run(self, arm, control, target_xy, target_q):
        self.end_time = None

        self.controller = control
        self.arm = arm
        box = [-.55, .55, -.25, .85]
        #box = [-.6, .6, -.6, .6]

        fig = plt.figure(figsize=(6.1,6.1), dpi=None)
        fig.suptitle(self.title); 

        # set the padding of the subplot explicitly
        fig.subplotpars.left=.1; fig.subplotpars.right=.9
        fig.subplotpars.bottom=.1; fig.subplotpars.top=.9

        self.ax = fig.add_subplot(1, 1, 1, 
                             xlim=(box[0], box[1]), 
                             ylim=(box[2], box[3]))
        self.ax.xaxis.grid() 
        self.ax.yaxis.grid()
        # make it a square plot
        self.ax.set_aspect(1) 

        if self.sim_type == LINEAR:
            X, Y = self.controller.draw_lintraj(self.arm)
            self.traj, = self.ax.plot(X, Y, 'o', color='r', lw=1, alpha=0.5)

        self.force_vector = self.ax.quiver(0, 0, 0, 0, angles='xy', scale_units='xy', scale=1, color='r')

        #self.force_vector, = self.ax.plot([], [], '-', mew=4, color='r', lw=5)
        self.target_arm_line, = self.ax.plot([], [], 'o-', mew=4, color='b', lw=5, alpha=0.3)
        self.arm_line, = self.ax.plot([], [], 'o-', mew=4, color='b', lw=5)
        self.info = self.ax.text(box[0]+abs(.1*box[0]), \
                            box[3]-abs(.1*box[3]), \
                            '', va='top', fontsize=13)

        if self.sim_type == FORCE or self.sim_type == LINEAR:
            print "\nEnter <x-dir force> <y-dir force> to apply a force at the end-effector:"

        anim = animation.FuncAnimation(fig, self.anim_animate, 
                   init_func=self.anim_init, frames=5000, interval=0, blit=True)
        
        self.anim = anim

    def anim_init(self):
        self.info.set_text('')
        self.arm_line.set_data([], [])
        self.target_arm_line.set_data([], [])
        self.force_vector.set_offsets([0, 0])
        self.force_vector.set_UVC(0, 0)
        return self.info, self.target_arm_line, self.arm_line, self.force_vector

    def make_info_text(self):
        text = []
        text.append('$\mathregular{t = %1.4g}$'%(self.sim_step*self.dt))
        q_text = ' '.join('%4.3f,'%F for F in self.arm.q)
        text.append('$\mathregular{q = ['+q_text+']}$')
        u_text = ' '.join('%4.3f,'%F for F in self.controller.u)
        text.append('$\mathregular{u^{PID} = ['+u_text+']}$')
        Fq = self.arm.gen_Fq(forceEE=self.arm.fEE)
        uH_text = ' '.join('%4.3f,'%F for F in Fq)
        text.append('$\mathregular{u^{H} = ['+uH_text+']}$')

        return '\n'.join(text)    

    def anim_animate(self, i):
    
        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            line = raw_input()
            splt = line.split()
            if len(splt) == 2:
                fin = map(float, splt)
                self.arm.fEE = np.array(fin)
                print "Force applied: " + str(self.arm.fEE)
                f = self.arm.gen_Fq()
                q = self.arm.q
                normf = np.linalg.norm(f)
                normq = np.linalg.norm(q)
                direction  = np.dot(f, q)
                #print "Magnitude: " + str(np.linalg.norm(f))
                print "Direction: " + str(direction) 

        # if user entered a force at EE, apply it to torque
        self.Fq = self.arm.gen_Fq(forceEE=self.arm.fEE)

        # get target from controller
        if self.sim_type == LINEAR:
            self.target = self.controller.line_target(self.arm, self.arm.t, self.Fq)
            #self.target = self.controller.line_target(self.arm, self.dt*10)
            #self.target = self.controller.goal
        elif self.sim_type == RAND:
            # update target after specified period of time passes
            if self.sim_step % (self.target_steps*self.display_steps) == 0:
                self.target = self.controller.rand_target(self.arm)
        elif self.sim_type == FORCE:
            self.target = self.controller.goal

        # before drawing
        for j in range(self.display_steps):            
            # update control signal
            if (self.sim_step % self.control_steps) == 0 or self.tau is None:
                self.tau = self.controller.control(self.arm, self.dt)
            self.tau += self.Fq
            # apply total torque to robot
            self.arm.apply_torque(u=self.tau, dt=self.dt)
            self.sim_step += 1

        # update figure depending on what control we are running
        self.target_arm_line.set_data(*self.arm.position(self.controller.target))
        if self.sim_type == LINEAR or self.sim_type == FORCE:
            self.target_arm_line.set_data(*self.arm.position(self.controller.goal))
        
        # update figure
        pos = self.arm.position()
        X = pos[0][2]
        Y = pos[1][2]
        U = (self.arm.fEE[0]+X)/100
        V = (self.arm.fEE[1]+Y)/100
            
        # set (x,y) origin of force vector and vector direction (u,v)
        self.force_vector.set_offsets([X, Y])
        self.force_vector.set_UVC(U, V)

        self.arm_line.set_data(*self.arm.position())
        self.info.set_text(self.make_info_text())
            
        return self.info, self.target_arm_line, self.arm_line, self.force_vector

    def show(self):
        try:
            plt.show()
        except AttributeError:
            pass


if __name__ == '__main__':

    init_q = [0.75613, 1.8553] 
    init_dq = [0.5,0.5]
    l1 = 0.31
    l2 = 0.27
    dt = 1e-3

    # setup 2 link arm
    arm = arm2base.Arm2Base(init_q, init_dq, l1, l2, dt)

    # setup target position
    target_xy = np.array([-0.4, -0.1])
    target_q = arm.inv_kinematics(target_xy)  
    
    print "target_xy: " + str(target_xy)
    print "target_q: " + str(target_q)

    # setup controller
    kp = 60; ki = 0; kd = 10
    print "kp: " + str(kp) + ", ki: " + str(ki) + ", kd: " + str(kd)

    start_q = arm.q
    grav_comp = True
    controller = control.PID(kp, ki, kd, target_q, start_q, grav_comp)

    # start simulator
    sim_flag = sys.argv[1]
    iactive = True
    sim = Simulator(dt=dt, sim_type=sim_flag)
    sim.run(arm, controller, target_xy, target_q)
    sim.show()
