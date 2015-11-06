"""Demo of using so3tools in an attitude dynamics simulation and PD controller."""

# --- DETAILS
# The script below is divided into SETUP, SIMULATE, and DISPLAY. In the SETUP
# section you can adjust simulation length, timestep, initial conditions, desired
# state, mass properties, controller gains, and the display framerate. In the
# SIMULATE section, you will find a verbose description of exactly what is done
# to carry out the simulation. In short, it is an Euler forward integration, and
# the controller applied is a simple PD controller. The simulation and controller
# code is super short and neat thanks to so3tools. Once the simulation results
# have been stored, the script then goes into DISPLAY where the terror that is
# matplotlib is used to animate the results. You will notice that the animation is
# slower than real time - this is because matplotlib is garbage. Remember that it is
# only displaying results; the simulation has already been completed. In the animation,
# rigid body orientation is represented as 3 perpendicular lines. Dotted lines show the
# initial orientation, and dashed lines show the desired orientation. The animation loops
# when it reaches the end of the data, which depends on what you set the duration to in
# SETUP. Remember to keep the display window aspect ratio square so it doesn't distort
# the perpendicularity of the lines. You can change your view by click&dragging the plot.
# You probably want to do this a little or else the lines stop appearing 3D to the eye.

# --- AUTHOR
# Jason Nezvadovitz

################################################# IMPORTS

# standard
from __future__ import division
import time
# 3rd party
import numpy as np, numpy.linalg as npl
import transformations as trns
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import mpl_toolkits.mplot3d.axes3d as p3
# 1st party
import so3tools as so3

################################################# SETUP

# Define time:
dt = 0.01  # global time step (s)
T = 10  # simulation duration (s)
t_arr = np.arange(0,T,dt)  # time values array (s)

# Define state and initial conditions:
q = [0.5,0.5,0.5,0.5]  # orientation state quaternion representing a conversion ***from body frame to world frame***
q = trns.unit_vector(trns.random_vector(4)-0.5)  # actually let's start somewhere random instead
w = np.array([10,4,-2])  # angular velocity state (rad/s) in world frame
print('Initial Orientation: {}'.format(q))
print('Initial Ang. Velocity: {}'.format(w))

# Define body inertia:
I = np.array([[ 1   , 0.1  , 0.02],
              [0.01 ,  2   , 0.03],
              [0.02 , 0.03 ,  3 ]])  # inertia matrix in body frame (kg*m^2)
# I = np.diag([1,2,3])  # cool to compare to diagonal inertia matrix case (which would imply body frame is principal frame)
invI = npl.inv(I)  # store inverse for future use

# Controller setup (set gains to 0 if you want to test free spin):
q_des = [1,0,0,0]  # desired orientation state (need not be identity)
w_des = [0,0,0]  # desired angular velocity state (need not be zero)
kp = [140,140,140]  # proportional gain (roll, pitch, yaw)
kd = [170,170,170]  # derivative gain (roll, pitch, yaw)
print('Desired Orientation: {}'.format(q_des))
print('Desired Ang. Velocity: {}'.format(w_des))
print('Proportional Gains: {}'.format(kp))
print('Derivative Gains: {}'.format(kd))

# Animation setup:
animate = True  # Animate or just plot angles?
framerate = 20  # The current MatPlotLib implementation is slow and messed up- animation is never real time

# Visually represent the body as three perpendicular lines, each defined by its endpoints:
linlen = 0.5
xline = np.array([[0,linlen],[0,0],[0,0]])
yline = np.array([[0,0],[0,linlen],[0,0]])
zline = np.array([[0,0],[0,0],[0,linlen]])
body = np.concatenate((xline, yline, zline), axis=1)  # each column is a point in body

# Initialize histories for recording the simulation:
q_history = np.zeros((len(t_arr),4))
roll_history, pitch_history, yaw_history = np.zeros(len(t_arr)), np.zeros(len(t_arr)), np.zeros(len(t_arr))  # for plotting something understandable
body_world_history = np.zeros((body.shape[0], body.shape[1], len(t_arr)))  # will store all the body points expressed in world frame at each instant

################################################# SIMULATE

# Solve the rigid body rotation ODE with first-order integration:
# ---
# Equation 1:  Hnext = Hlast + torque*dt
# ---
# where in this case torque is the controller output and H is the 
# body's angular momentum in world frame,  H = I_world * w   ==>   w = inv(I_world)*H
# In differential equation form, this is the classic law  Hdot = torque  iff
# the origin of the body frame is at the center of mass, which is typically done.
# ---
# Equation 2:  qnext = dq "so3plus" q
# ---
# where dq represents ***nextbody to lastbody*** and q represents ***lastbody to world*** so that
# their so3sum is ***nextbody to world*** which then overwrites ***lastbody to world*** as the new q.
# The key here is that w*dt is the axle representing ***nextbody to lastbody***,
# and it has an equivalent quaternion expression dq. In differential equation form, the equation
# is  qdot = f(w,dt) = so3.quaternion_from_axle(w*dt)/dt. For clarity, let's express this
# equation as a "procedure":
# 1: Recall that q represents the conversion body_to_world. In rotation matrix form, we would
#    left multiply a vector in body frame in order to obtain that vector in world frame, that is, R*v_body = v_world.
# 2: Call the norm of w*dt, "angle"
# 3: Call the unitized version of w, "axis"
# 4: Form the quaternion defined by this angle and axis, call it "dq".  Note that
#    so3.quaternion_from_axle(w*dt) performs this operation. Also note that dq is
#    the small rotation that converts vectors from nextbody to lastbody.
# 5: Our goal is to find the conversion nextbody_to_world so we can make that the new q.
#    This is accomplished by doing  so3.plus(nextbody_to_lastbody, lastbody_to_world), because
#    when the result is applied to some vector, we want to first apply nextbody_to_lastbody and
#    then apply lastbody_to_world. Recall so3.plus is not commutative.
# 6: Let lasbody be the current body, and recall that nextbody_to_lastbody = dq.
#    Thus qnext = so3.plus(dq, qlast) = so3.plus(dq, q).
# 7: Finally, set q = qnext, and repeat for the next timestep.
# Unless it is designed to work on SO3, typical ODE solvers will try to use regular Euclidean addition
# when integrating. If you just tell it  qdot = f(w,dt) = so3.quaternion_from_axle(w*dt)/dt, even though
# that is correct, it will integrate by doing something analogous to  qnext = q + qdot*dt  (even RK4 is just
# a more involved version of that). Since the + used Euclidean, the result will be fundamentally wrong.
# To use a typical ODE solver, we would need to reformulate the quaternion state ODE into an approximate form where
# addition is possible, like the one given in http://ssl.mit.edu/spheres/library/ASS2011_11-033_spheres.pdf. On each
# iteration, the quaternion state will become less and less unit norm both due to computer roundoff (which plagues
# every method) AND due to local approximation of SO3 as Euclidean. This is because the euclidean sum of two unit quaternions
# is not a unit quaternion unless one of them represents an infinitesmal angle. That is, a hypothetical analytical solution
# to their equation would be exactly correct, but a numerical solution will be less accurate than a numerical solution using
# the angle-axis approach, which was explained above for the first order integration case. An equivalent but even more numerically
# accurate RK4 implementation of the angle axis method could and should be implemented.
# ---
# Simulate over t_arr:
for i,t in enumerate(t_arr):
    # Record current state:
    q_history[i,:] = np.copy(q)
    roll_history[i], pitch_history[i], yaw_history[i] = trns.euler_from_quaternion(q,'rxyz')
    body_world_history[:,:,i] = so3.apply(q, body)
    # Current values needed to compute next state:
    I_world = so3.apply(q, so3.apply(q, I).T)  # current inertia matrix in world frame noting I = I.T
    H = I_world.dot(w)  # current angular momentum
    dq = so3.quaternion_from_axle(w*dt)  # change in orientation for this timestep instant
    # PD controller... in ONLY THREE LINES BITCHES:
    q_err = so3.error(q, q_des)  # q_err is an axle
    w_err = w_des - w
    torque = kp*q_err + kd*w_err
    # Compute next state:
    q = so3.plus(dq, q)  # new orientation computed using dq and old q
    I_world = so3.apply(q, so3.apply(q, I).T)  # new I_world computed using new q
    H = H + torque*dt  # new H from old H and torque
    w = npl.inv(I_world).dot(H)  # new angular velocity computed using new I_world and new H

################################################# DISPLAY

fig = plt.figure()
fig.suptitle('Orientation State Evolution', fontsize=24)

if animate == True:
    ax4 = p3.Axes3D(fig)
    ax4.set_xlim3d([-1,1])
    ax4.set_ylim3d([-1,1])
    ax4.set_zlim3d([-1,1])
    ax4.set_xlabel('- World X +')
    ax4.set_ylabel('- World Y +')
    ax4.set_zlabel('- World Z +')
    ax4.grid(True)

    body_des = so3.apply(q_des, body)
    ax4.plot(2*body_des[0,:2], body_des[1,:2], body_des[2,:2], color='red', ls='--', linewidth=0.8)
    ax4.plot(body_des[0,2:4], 2*body_des[1,2:4], body_des[2,2:4], color='green', ls='--', linewidth=0.8)
    ax4.plot(body_des[0,4:6], body_des[1,4:6], 2*body_des[2,4:6], color='blue', ls='--', linewidth=0.8)

    ax4.plot(2*body_world_history[0,:2,0], body_world_history[1,:2,0], body_world_history[2,:2,0], color='red', ls=':', linewidth=0.8)
    ax4.plot(body_world_history[0,2:4,0], 2*body_world_history[1,2:4,0], body_world_history[2,2:4,0], color='green', ls=':', linewidth=0.8)
    ax4.plot(body_world_history[0,4:6,0], body_world_history[1,4:6,0], 2*body_world_history[2,4:6,0], color='blue', ls=':', linewidth=0.8)

    x = ax4.plot(body_world_history[0,:2,0], body_world_history[1,:2,0], body_world_history[2,:2,0], color='red', linewidth=4)
    y = ax4.plot(body_world_history[0,2:4,0], body_world_history[1,2:4,0], body_world_history[2,2:4,0], color='green', linewidth=4)
    z = ax4.plot(body_world_history[0,4:6,0], body_world_history[1,4:6,0], body_world_history[2,4:6,0], color='blue', linewidth=4)

    def update(arg, ii=[0]):
        i = ii[0]
        if np.isclose(t_arr[i], np.around(t_arr[i], 1)):
            fig.suptitle('Orientation State Evolution (Time: {})'.format(t_arr[i]), fontsize=24)
        x[0].set_data(body_world_history[0,:2,i], body_world_history[1,:2,i])
        x[0].set_3d_properties(body_world_history[2,:2,i])
        y[0].set_data(body_world_history[0,2:4,i], body_world_history[1,2:4,i])
        y[0].set_3d_properties(body_world_history[2,2:4,i])
        z[0].set_data(body_world_history[0,4:6,i], body_world_history[1,4:6,i])
        z[0].set_3d_properties(body_world_history[2,4:6,i])
        ii[0] += int(1/(dt*framerate))
        if ii[0] >= len(t_arr):
            ii[0] = 0
        return [x,y,z]

    ani = animation.FuncAnimation(fig, func=update, interval=dt*1000)
    print('Remember to keep the diplay window aspect ratio square!')
    print('')
    plt.show()

else:
    # Plot roll:
    ax1 = fig.add_subplot(3,1,1)
    ax1.plot(t_arr, roll_history*180/np.pi)
    ax1.set_ylabel('roll (deg)', fontsize=16)
    ax1.grid(True)

    # Plot pitch:
    ax2 = fig.add_subplot(3,1,2)
    ax2.plot(t_arr, pitch_history*180/np.pi)
    ax2.set_ylabel('pitch (deg)', fontsize=16)
    ax2.grid(True)

    # Plot yaw:
    ax3 = fig.add_subplot(3,1,3)
    ax3.plot(t_arr, yaw_history*180/np.pi)
    ax3.set_xlabel('time (s)', fontsize=16)
    ax3.set_ylabel('yaw (deg)', fontsize=16)
    ax3.grid(True)

    plt.show()
