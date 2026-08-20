uint8 SAFE=0
uint8 BLOCKED=1
uint8 DEGRADED=2
uint8 FAULT=3

std_msgs/Header header
uint8 level
bool flight_enable
bool runtime_gate_open
bool mavros_command_ready
bool busy
bool interlock_active
bool fault_latched
string detail
