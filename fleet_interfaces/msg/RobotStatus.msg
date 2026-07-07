# Per-robot heartbeat/status, published by each UGV and aggregated by the HUB.
uint8 IDLE=0
uint8 PATROL=1
uint8 INVESTIGATE=2
uint8 RTB=3          # return to base (low battery)
uint8 LOST=4         # set by the HUB comms watchdog, not by the robot

string robot_id
uint8 state
float64 x            # map frame
float64 y
float64 yaw
float64 battery      # 0..100 percent
float32 speed        # current ground speed, m/s
uint32 task_track_id # track currently assigned by the HUB (0 = none)
