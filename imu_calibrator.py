import numpy as np

def sample_a(sample_time_s=5.0) -> tuple[float, float, float]:
	"""
	Over sample_time, collect samples from the accelerometer and return the average as a tuple (x, y, z).
	All data in m/s^2
	"""
	return (0, 0, 0)

# calculate z
z_minus = np.array(sample_a(), dtype=float)
z_length = np.linalg.norm(z_minus)
if not np.isclose(z_length, 9.81, atol=0.5):
	raise ValueError(f"z_minus not accurately measuring gravity (length {z_length})")
z_plus_norm = -z_minus / z_length
# calculate x
input("Please begin pushing the car forwards then press enter, and continue pushing for 1 second")
x_and_z= np.array(sample_a(sample_time_s=1.0), dtype=float)
x_plus = x_and_z - np.dot(x_and_z, z_plus_norm) * z_plus_norm # remove any component of x in the z direction
x_length = np.linalg.norm(x_plus)
if x_length < 1e-3:
	raise ValueError(f"x_plus too small to estimate accurately (length {x_length})")
x_plus_norm = x_plus / x_length

y_plus_norm = np.cross(z_plus_norm, x_plus_norm)
y_length = np.linalg.norm(y_plus_norm)
if not np.isclose(y_length, 1.0, atol=1e-3):
	raise ValueError(f"y_plus_norm is not a unit vector (length {y_length})")
R = np.array([x_plus_norm, y_plus_norm, z_plus_norm]).T
print(R)