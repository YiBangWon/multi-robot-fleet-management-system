// Declarations for the multi-agent status monitoring module.

#ifndef MASM_H
#define MASM_H

#include <fleet_manager/task_states.hpp>
#include <fleet_manager/robot_states.hpp>
#include <fms_msgs/msg/robot_info.hpp>
#include <fms_msgs/srv/command_robot.hpp>
#include <fleet_manager/topological_map.h>

namespace FMS
{
/* MultiAgentStatusMonitor: Status Monitoring Module */
class MultiAgentStatusMonitor 
{
public:
    /*  Class constructor  */
    MultiAgentStatusMonitor(){};
    /*  Class destructor  */
    ~MultiAgentStatusMonitor(){};
    /* Initialize the MASM module */
    void initMASM(TaskStates* task_states_, RobotStates* robot_states_, TopologicalMap* topolomap_);
    /* Check errors on currently assigned tasks */
    bool checkTaskErros();
    /*  Check whether all tasks are complete
        (If all tasks are complete, idle robots move to their charging stations)  */
    bool checkTaskCompletion(std::vector<RobotAction>& action_list);      
    /*  Run multi-agent status monitor  
        - Update the robot and task state tables
        - Return the flag for path planning  */
    bool runMASM();
    /*  Update robot pose and info  */
    void updateRobotInfo(std::vector<geometry_msgs::msg::Pose2D> robot_pose_, std::vector<size_t> robot_info_);
    /*  Run the inserted action commands  */
    bool assignRobotActions(std::vector<RobotAction> action_list);

    /*********************************/
    /*  Task and Robot State Tables  */
    TaskStates* task_states; 
    RobotStates* robot_states;
    /*********************************/    
    TopologicalMap* topolomap;           // Topological map
    std::vector<bool> updated_robot;    // Robot flags for state update 
    int num_of_robots;                  // Number of robots
};
}
#endif // MASM_H
