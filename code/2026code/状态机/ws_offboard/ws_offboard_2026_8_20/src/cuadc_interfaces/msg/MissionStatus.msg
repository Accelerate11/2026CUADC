uint8 IDLE=0
uint8 TAKEOFF=1
uint8 BASKET_SEARCH=2
uint8 PAYLOAD_DROP=3
uint8 HAZARD_WAYPOINT_RECON=4
uint8 RETURNING=5
uint8 LANDING=6
uint8 COMPLETE=7
uint8 ABORTED=8

std_msgs/Header header
string mission_id
uint8 state
bool active
bool safe_to_continue
string detail
