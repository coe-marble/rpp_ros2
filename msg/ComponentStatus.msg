# Native RPP lifecycle states, including states without a ROS node-lifecycle equivalent.
uint8 UNLOADED=0
uint8 UNCONFIGURED=1
uint8 INACTIVE=2
uint8 ACTIVE=3
uint8 ERROR=4

string name
string component
string instance_id
uint8 state
string transition
bool callable
uint64 in_flight
bool last_success
string last_message
