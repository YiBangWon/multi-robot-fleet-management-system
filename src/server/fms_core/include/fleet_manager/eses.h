// #include <fleet_manager/route_table.h>
#include <iostream>
#include <memory>
#include <vector>
#include <algorithm>
#include <functional>
#include <stack>
#include <queue>
#include <eigen3/Eigen/Dense>

namespace FMS
{
    class VertexRT
    {
    public:
        VertexRT() = default;
        int robot_id;
        int timestep;
        bool is_waypoint;
        bool is_satisfied;
        int id_start;
        int id_end;
        Eigen::Vector3d start;      // Start pose
        Eigen::Vector3d end;        // End pose
        std::vector<std::pair<VertexRT*, bool>> preconditions; // precondition: <node, non-switchable>
    };

    typedef std::vector<std::vector<VertexRT*>> Graph;

    class ESES
    {
    public:
        double cost;
        int num_of_robots;
        std::vector<int> curr_steps;
        Graph ESES_graph;

        struct QueueNode
        {
            Graph route_table;
            std::vector<int> x;
            double g{};
            double h{};
            double f{};
        };
        struct CompareNodePtr
        {
            bool operator()(const QueueNode* a, const QueueNode* b) const
            { return a->f > b->f; }
        };
        using OpenList = std::priority_queue<QueueNode*, std::vector<QueueNode*>, CompareNodePtr>;

        struct branch_return
        {
            std::vector<int> x;
            double cost;
            std::vector<int> edge = std::vector<int>(4);
        };

        struct heuristic_return
        {
            double cost;
            std::vector<int> x;
        };

        ESES(Graph ESES_graph, std::vector<int> curr_steps, int num_of_robots){
            this->ESES_graph = ESES_graph;
            this->curr_steps = curr_steps;
            this->num_of_robots = num_of_robots;
        };
        ~ESES(){
            for (auto& robot_graph : ESES_graph) {
                for (auto& vertex : robot_graph) {
                    delete vertex; //   VertexRT  
                }
            }
        };
        void init(const Graph& route_table, std::vector<int>& x);
        std::pair<double, bool> step(const Graph& route_table, std::vector<int>& timestep);
        std::pair<int,int> get_delay(int robot_id, int timestep_a, std::vector<double> costs, const Graph& route_table, const std::vector<int>& timestep);
        heuristic_return heuristic(Graph& route_table, std::vector<int> x);
        branch_return branch(Graph& route_table, std::vector<int> x);
        bool is_cyclic(const Graph& route_table, int start_robot_id, int start_timestep, int target_robot_id, int target_timestep);
        std::vector<std::pair<int, int>> getSuccessors(const Graph& route_table, int robot_id, int timestep);
        std::pair<Graph, int> runESES();
        std::pair<int, int> find_switchable_nodeds(int robot_id, int timestep, int d_robot_id, int d_timestep, const Graph& route_table);
        double get_costs();


        Graph copy_graph(const Graph& original_graph);
    };
}
