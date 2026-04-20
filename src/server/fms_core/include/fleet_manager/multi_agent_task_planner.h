// Declarations for the multi-agent task allocation module.

#ifndef MATP_H
#define MATP_H

#include <fleet_manager/task_states.hpp>
#include <fleet_manager/robot_states.hpp>
#include <fleet_manager/topological_map.h>
#include <ECBS/path_planner.h>

namespace FMS
{
/* MultiAgentTaskPlanner: Task Planning Module */ 
class MultiAgentTaskPlanner 
{
public:
    /*  Class constructor  */
    MultiAgentTaskPlanner(){};
    /*  Class destructor  */
    ~MultiAgentTaskPlanner(){};
    /*  Initialize the MATP module  */
    void initMATP(TaskStates* task_states_, RobotStates* robot_states_, TopologicalMap* topolomap_,
                  PathPlanner* path_planner_, SolverMap* solver_map_);
    /*  Set IDs of task nodes  */
    void setTaskNodeIDs();
    /*  Get the distance of a path from start to goal  */
    double getDistance(int start, int goal);
    /* Update remaining & processing time for each task */
    void updateTimes();
    /*  Run multi-agent task planning   
        - Update the robot and task state tables according to the assigned tasks
        - Return the flag regarding updated  */
    bool runMATP();
    /*  Update assigned tasks from update topological map */
    bool updateMATP();
    /*  Sequential task assignment: simply assign tasks sequentially  */
    bool SequentialAssignment();
    /*  Greedy task assignment: assign the task with the minimum path distance  */
    bool GreedyAssignment();
    
    /*********************************/
    /*  Task and Robot State Tables  */
    TaskStates* task_states;
    RobotStates* robot_states;
    /*********************************/
    TopologicalMap* topolomap;           // Topological map
    SolverMap* solver_map;              // Solver map
	PathPlanner* path_planner;          // Single agent path planner (A* algorithm)
    int num_of_robots;                  // Number of robots
    
    // User parameters
    bool is_greedy_assignment = false;  // True: Greedy assignment or False: sequential assignment
    int num_assigned_tasks    = 10;     // number of assigned tasks
    double sec_per_node       = 3;      // Moving time for each node (sec)
    double sec_per_task       = 5;      // Task processing time (sec)
};

}

#endif // MATP_H
