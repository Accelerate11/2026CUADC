uint8 UNKNOWN=0
uint8 STOWING=1
uint8 STOWED=2
uint8 RELEASING=3
uint8 RESTOWING=4
uint8 FAULT=5

std_msgs/Header header
uint8 payload_index
uint8 state
bool busy
bool last_command_accepted
string request_id
string detail
