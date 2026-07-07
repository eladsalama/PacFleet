# A fused global track maintained by the coordinator (Kalman MOT output).
uint8 TENTATIVE=0    # not enough hits yet to trust
uint8 CONFIRMED=1    # actively updated by detections
uint8 COASTING=2     # no recent detections; propagating on prediction only

std_msgs/Header header
uint32 track_id
uint8 status
geometry_msgs/Point position     # estimated position (map frame)
geometry_msgs/Vector3 velocity   # estimated velocity (map frame)
float64[16] covariance           # 4x4 row-major over state (x, y, vx, vy)
float64 threat_prob              # neural classifier output: P(power pellet), 0..1
string label                     # "coin" | "pellet"
string assigned_to               # robot currently chasing this track ("" = none)
