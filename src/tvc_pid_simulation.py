import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.prev_error = 0

    def update(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative
    
def simulate():
    angles = []
    tvc_angles = [] 
    angular_velocity = 0
    angle = np.pi/2  #inital gimbal angle

    for _ in np.arange(0, sim_time, dt):
        error = -angle  
        control = pid.update(error, dt)
        gimbal_angle = np.clip(control, -np.radians(tvcAngleRange), np.radians(tvcAngleRange))
        tvc_angles.append(np.degrees(gimbal_angle))

        disturbance = np.random.normal(disturbance_mean, disturbance_std)
        torque = control * (rocket_length)/4 + disturbance
        angular_acceleration = torque / moment_of_inertia
        angular_velocity += angular_acceleration * dt
        angle += angular_velocity * dt
        angles.append(angle)
        
    return angles, tvc_angles

def animate(i):
    ax.clear()
    ax.set_xlim(-1.5, 1.5)
    ax.set_ylim(-1.5, 1.5)
    angle_rad = angles[i]
    rect_width = 0.1 * rocket_length
    rocket_rect = plt.Rectangle((-rect_width / 2, 0), rect_width, rocket_length, angle=np.degrees(angle_rad), color='gray', zorder=1)
    ax.add_patch(rocket_rect)
    # Motor dimensions
    motor_width = 0.05 * rocket_length
    motor_height = 0.2 * rocket_length
    motor_gimbal_angle = angles[i] + np.radians(tvc_angles[i])
    motor_rect = plt.Rectangle((-motor_width / 2, -motor_height / 2), motor_width, motor_height, angle=np.degrees(motor_gimbal_angle), color='red', zorder=2)
    ax.add_patch(motor_rect)

# Run Simulation ##################################################################################

# Rocket parameters
moment_of_inertia = 0.009365
rocket_length = 1.0
tvcAngleRange = 5  # deg

# PID parameters
kp = 1.5
ki = 0.05
kd = 0.75
pid = PIDController(kp, ki, kd)

# Sim parameters
sim_time = 20
dt = 0.01 
disturbance_mean = 0
disturbance_std = 0.05  # distrubance stdev

angles, tvc_angles = simulate()

# Rocket angle
fig, ax = plt.subplots(2, 1, figsize=(10, 8))
ax[0].plot(np.arange(0, sim_time, dt), angles, label='Rocket Angle')
ax[0].set_title('Rocket Angle vs. Time')
ax[0].set_xlabel('Time (s)')
ax[0].set_ylabel('Angle (rad)')
ax[0].legend()
ax[0].grid()

# TVC angle
ax[1].plot(np.arange(0, sim_time, dt), tvc_angles, label='Gimbal Angle', color='orange')
ax[1].set_title('PID Gimbal Angle vs. Time')
ax[1].set_xlabel('Time (s)')
ax[1].set_ylabel('Angle (deg)')
ax[1].legend()
ax[1].grid()
plt.tight_layout()
plt.show()

fig, ax = plt.subplots()
ani = FuncAnimation(fig, animate, frames=len(angles), interval=dt*1000)
plt.show()
