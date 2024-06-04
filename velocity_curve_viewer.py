import numpy as np
import matplotlib.pyplot as plt

# Settings for the tanh curve
start_velocity = 0.0
end_velocity = -30.0
num_points = 10
range_velocity = end_velocity - start_velocity  # Calculate the range of velocities

# Initialize the list to store velocities
velocities = []

# Generate the tanh curve
step = 1.0 / num_points
for i in range(num_points + 1):
    t = i * step
    normalized_time = t * 2.0   # Normalize the time to [-1, 1]
    tanh_value = np.tanh(normalized_time)  # Calculate the tanh of these points
    velocity = start_velocity + (tanh_value) / 1 * range_velocity  # Scale and shift
    velocities.append(velocity)

# Plotting the tanh curve
plt.figure(figsize=(8, 4))
plt.plot(velocities, label='Velocity vs. Time')
plt.title('Tanh Curve for Velocity Transition')
plt.xlabel('Time Steps')
plt.ylabel('Velocity')
plt.legend()
plt.grid(True)
plt.show()
