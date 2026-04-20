string fleet_name
string robot_id
string task_id

uint32 mode
uint32 MODE_IDLE=0
uint32 MODE_MOVING_TASK=1
uint32 MODE_TASK_PROCESSING=2
uint32 MODE_MOVING=3
uint32 MODE_MOVING_CHARGING=4
uint32 MODE_CHARGING=5
uint32 MODE_MOVING_WAITING=6
uint32 MODE_WAITING=7
uint32 MODE_PAUSED=8
uint32 MODE_ERROR=9
uint32 MODE_ISOLATED=10

uint32 type
uint32 TYPE_AGF=0
uint32 TYPE_AGV=1
uint32 TYPE_AMR=2

fms_msgs/Node goal_node

geometry_msgs/Pose2D pose
geometry_msgs/Pose2D target_pose
geometry_msgs/Pose2D[] path

uint64 time_delay
uint64 progress
uint64 battery

