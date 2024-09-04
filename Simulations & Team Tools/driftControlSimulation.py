import random
import math
import matplotlib.pyplot as plt
import numpy as np

# Define constants
K = 2  # Motor constant (can be adjusted for different simulations)
KP = 8.96  # Proportional gain for PD controller
KD = 16  # Derivative gain for PD controller
VS = 5  # Vehicle speed (can be adjusted for initial testing)
RAND = 150/0.01

vel = 0

# Function to generate wind vector in polar form
def generate_wind(a, b, c, d, e, f, dt, wind_mag):
  wind_mag_change = (a * random.uniform(-1, 1) + b) / dt
  wind_mag += wind_mag_change * dt
  wind_mag = max(c, min(wind_mag, d))  # Limit magnitude
  wind_angle = e + (f - e) * random.uniform(0, 1)  # Limit angle
  wind_angle = wind_angle % 360  # Wrap around to 0-360 degrees
  return wind_mag * math.cos(math.radians(wind_angle)), wind_mag * math.sin(math.radians(wind_angle))

# Function to calculate goal angle
def calculate_goal_angle(position, target):
  dx = target[0] - position[0][-1]
  dy = target[1] - position[1][-1]

  # wind_x = wind_mag * math.cos(wind_angle)
  # wind_y = wind_mag * math.sin(wind_angle)

  # adjustedx = dx - wind_x
  # adjustedy = dy - wind_y

  goal_angle = math.degrees(math.atan2(dy, dx)) % 360

  return goal_angle

# System function (simulates PD controller)
def system(pos, targ, heading, past_error, dt):

  # Calculate goal angle
  goal_angle = calculate_goal_angle(pos, targ)

  # Calculate error terms
  angle_error = goal_angle - heading
  derror = (angle_error - past_error)/dt

  # Calculate control signal
  pwm = KP * angle_error + KD * derror

  return pwm, angle_error


# Update state function
def update_state(pwm, dt):
  global vel
  accel = K * pwm
  accel += RAND * random.uniform(-1, 1)
  vel += accel * dt

  return vel * dt


# Simulation loop
def simulate(target, steps, dt, rand_mag, offset, mag_lower, mag_upper, wind_lower, wind_upper):
  # Initialize state
  heading = [0]
  pos = [[0],[0]]
  past_error = 0
  wind_mag = 0
  total_time = 1

  # Simulation loop
  for _ in range(int(steps)):
    total_time += 1
    # Generate wind
    # wind_mag, wind_angle = generate_wind(0.1, 0, 0.5, 1, 270, 330, dt)
    wind_mag, wind_angle = generate_wind(rand_mag, offset, mag_lower, mag_upper, wind_lower, wind_upper, dt, wind_mag)


    # Get system output
    pwm, past_error = system(pos, target, heading[-1], past_error, dt)

    # Update state
    heading.append(heading[-1] + update_state(pwm, dt))
    

    pos[0].append(pos[0][-1] + (VS * math.cos(heading[-1]*math.pi/180) + wind_mag * math.cos(wind_angle*math.pi/180)) * dt)
    pos[1].append(pos[1][-1] + (VS * math.sin(heading[-1]*math.pi/180) + wind_mag * math.sin(wind_angle*math.pi/180)) * dt)

    if math.sqrt((pos[0][-1]-target[0])**2 + (pos[1][-1]-target[1])**2) < 0.1:
      break
  
  plt.figure()
  plt.title("Position")
  plt.plot(pos[0], pos[1], ls='-', color='red')

  plt.figure()
  plt.plot(np.arange(0,(total_time)*dt,dt), heading, ls = '-', color='blue')
  plt.show()


simulate([0,5], 10/0.01, 0.01, 0.1, 0, 0, 2, 90, 180)

