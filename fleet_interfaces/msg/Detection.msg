# A single (noisy) sensor detection of a contact, reported by one UGV.
# The measurement is taken in the robot's sensor frame (range/bearing) and
# also provided pre-projected into the world frame for the HUB fusion layer.
std_msgs/Header header          # stamp + frame_id of the reporting sensor
string robot_id                 # which robot saw it (e.g. "chomp")
geometry_msgs/Point position    # measured position in the map frame (noisy)
float64 range                   # meters, robot -> contact
float64 bearing                 # radians, in robot base frame
float64 confidence              # 0..1, degrades with range
