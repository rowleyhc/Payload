import random
import math
import matplotlib.pyplot as plt
import numpy as np

# Define constants
K = 2  # Motor constant (can be adjusted for different simulations)
KP = 8.96  # Proportional gain for PD controller
KD = 16  # Derivative gain for PD controller
VS = 5  # Vehicle speed (can be adjusted for initial testing)
DT = 0.01
RAND = 50/DT

ang_vel = 0
xvel = [0]
yvel = [0]

# Function to generate wind vector in polar form
def generate_wind(a, b, c, d, e, f, dt, wind_mag):
  wind_mag_change = (a * random.uniform(-1, 1) + b) / dt
  wind_mag += wind_mag_change * dt
  wind_mag = max(c, min(wind_mag, d))  # Limit magnitude
  wind_angle = e + (f - e) * random.uniform(0, 1)  # Limit angle
  wind_angle = wind_angle % 360  # Wrap around to 0-360 degrees
  return wind_mag, wind_angle

# Function to calculate goal angle
def calculate_goal_angle(position, target, wind_mag, wind_angle, correction):
  global xvel
  global yvel
  dx = target[0] - position[0][-1]
  dy = target[1] - position[1][-1]

  wind = np.array([wind_mag * math.cos(math.radians(wind_angle)), wind_mag * math.sin(math.radians(wind_angle))])

  print("Wind: ", wind)

  return_angle = math.degrees(math.atan2(dy, dx)) % 360

  if correction == True:

    direction = np.array([dx, dy])

    print("Direction: ", direction)

    direction_norm = np.linalg.norm(direction)

    wind_todirection = direction * np.dot(wind, direction) / direction_norm**2

    wind_cross = wind - wind_todirection

    h_prime = direction/direction_norm * np.sqrt(VS**2 - np.linalg.norm(wind_cross)**2)

    goal_vec = h_prime - wind_cross

    xvel.append(goal_vec[0])
    yvel.append(goal_vec[1])

    goal_angle = math.degrees(math.atan2(goal_vec[1], goal_vec[0])) % 360

    print(goal_angle, return_angle)

    return_angle = goal_angle

  return return_angle

# System function (simulates PD controller)
def system(pos, targ, heading, past_error, dt, wind_mag, wind_angle, correction):

  # Calculate goal angle
  goal_angle = calculate_goal_angle(pos, targ, wind_mag, wind_angle, correction)

  # Calculate error terms
  angle_error = goal_angle - heading
  derror = (angle_error - past_error)/dt

  # Calculate control signal
  pwm = KP * angle_error + KD * derror

  return pwm, angle_error


# Update state function
def update_state(pwm, dt):
  global ang_vel
  accel = K * pwm
  accel += RAND * random.uniform(-1, 1)
  ang_vel += accel * dt

  return ang_vel * dt


# Simulation loop
def simulate(target, steps, dt, rand_mag, offset, mag_lower, mag_upper, wind_lower, wind_upper, correction):
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
    wind_mag, wind_angle = generate_wind(rand_mag, offset, mag_lower, mag_upper, wind_lower, wind_upper, dt, wind_mag)

    # Get system output
    pwm, past_error = system(pos, target, heading[-1], past_error, dt, wind_mag, wind_angle, correction)

    # Update state
    heading.append(heading[-1] + update_state(pwm, dt))
    

    pos[0].append(pos[0][-1] + (VS * math.cos(math.radians(heading[-1])) + wind_mag * math.cos(math.radians(wind_angle))) * dt)
    pos[1].append(pos[1][-1] + (VS * math.sin(math.radians(heading[-1])) + wind_mag * math.sin(math.radians(wind_angle))) * dt)

    if math.sqrt((pos[0][-1]-target[0])**2 + (pos[1][-1]-target[1])**2) < 0.1:
      break
  
  return pos, heading, total_time


pos1, heading1, total_time1 = simulate([0,5], 10/DT, DT, 0.1, 0, 0, 2, 90, 180, True)
pos2, heading2, total_time2 = simulate([0,5], 10/DT, DT, 0.1, 0, 0, 2, 90, 180, False)


plt.figure()
plt.title("Position")
plt.plot(pos1[0], pos1[1], ls='-', color='red', label='wind corrected')
plt.plot(pos2[0], pos2[1], ls='-', color='pink', label='non-corrected')
plt.legend()

plt.figure()
plt.subplot(2, 1, 1)  # Create subplot 1 (top row)
plt.title("X Velocity")
plt.plot(np.arange(0, (total_time1)*DT, DT), xvel, ls='-', color='orange')

plt.subplot(2, 1, 2)  # Create subplot 2 (bottom row)
plt.title("Y Velocity")
plt.plot(np.arange(0, (total_time1)*DT, DT), yvel, ls='-', color='green')
plt.tight_layout()

plt.figure()
plt.title("Heading")
plt.plot(np.arange(0,(total_time1)*DT,DT), heading1, ls = '-', color='blue', label='wind corrected')
plt.plot(np.arange(0,(total_time2)*DT,DT), heading2, ls = '-', color='cyan', label='non-corrected')
plt.legend()
plt.show()
