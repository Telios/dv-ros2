std_msgs/Header header

# Name of the camera
string name

# Timestamp when the camera was opened
builtin_interfaces/Time startup_time

# Indicate whether a camera is a master camera
bool is_master

# Topic name for synchronization service server
string sync_service_topic

# Indicate whether node is publishing events
bool publishing_events

# Indicate whether node is publishing frames
bool publishing_frames

# Indicate whether node is publishing IMU data
bool publishing_imu

# Indicate whether node is publishing triggers
bool publishing_triggers
