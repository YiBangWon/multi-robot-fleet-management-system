string fleet_name
string task_id
string robot_id
string status

builtin_interfaces/Time processing_time
builtin_interfaces/Time remaining_time
builtin_interfaces/Time start_time
builtin_interfaces/Time end_time



uint32 state
uint32 STATE_QUEUED=0
uint32 STATE_ACTIVE=1
uint32 STATE_COMPLETED=2
uint32 STATE_FAILED=3
uint32 STATE_CANCELED=4
uint32 STATE_PENDING=5

fms_msgs/TaskInfo info


