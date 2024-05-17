std_msgs/Header header

# Transformation from sensor (IMU) to camera.
# Transforms any point from IMU kinematic reference frame to the camera reference frame.
geometry_msgs/Transform t_sc

# Offset between the camera and IMU timestamps in microseconds (t_correct = t_imu - offset)
int64 time_offset_micros

# Variance of the offset of the angular velocity [rad/s]
float32 omega_offset_var

# Variance of the offset of the acceleration [m/s^2]
float32 acc_offset_var

# Noise density of the gyroscope [rad/s^s/sqrt(Hz)]
float32 omega_noise_density

# Noise density of the accelerometer [m/s^2/sqrt(Hz)]
float32 acc_noise_density

# Noise random walk of the gyroscope [rad/s^s/sqrt(Hz)]
float32 omega_noise_random_walk

# Noise random walk of the accelerometer [m/s^2/sqrt(Hz)]
float32 acc_noise_random_walk