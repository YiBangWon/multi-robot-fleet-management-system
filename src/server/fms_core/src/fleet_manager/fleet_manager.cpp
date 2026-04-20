// Core fleet manager node that coordinates monitoring, task allocation, routing, and visualization.

#include <fleet_manager/fleet_manager.h>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FMS::FleetManager>();
    std::thread spin_thread([&node]() { rclcpp::spin(node); });

    /*  Check if all robot information is subscribed  */
    rclcpp::WallRate loop_rate(1000ms);
    while (!node->isReady())
    {
        node->publishStateTables();
        loop_rate.sleep();
    }

    /*  Run FMS  */
    node->initFMS();
    node->Run(); // Thread for path planning
    while (rclcpp::ok() && node->running)
    {
        node->runIteration();
        node->publishStateTables();
        node->publishRouteTables();
        loop_rate.sleep();
    }
    node->Release();
    rclcpp::shutdown();
    if (spin_thread.joinable())
    {
        spin_thread.join();
    }
    return 0;
}

namespace FMS
{

    FleetManager::FleetManager() : Node("fleet_manager"), count_(0)
    {

        clock = this->get_clock();
        util.clock = clock;



        /* 0. Read ROS Parameters */
        this->declare_parameter<std::string>("filename_topolomap", " ");      // File name of topological map
        this->declare_parameter<std::string>("filename_task_list", "none");   // File name of task list
        this->declare_parameter<std::string>("filename_task_table", "none");  // File name of task state table
        this->declare_parameter<std::string>("filename_robot_table", "none"); // File name of robot state table
        this->declare_parameter<int>("nr_of_robots", 0);                      // Number of robots
        this->declare_parameter<bool>("run_RHCR", false);                     // True: RHCR algorithm (Li et al. AAAI-2021), False: Normal MAPF algorithm
        this->declare_parameter<bool>("directed_graph", false);               // Topological map is a directed graph or not
        this->declare_parameter<bool>("remove_stations", false);              // Remove station nodes
        this->declare_parameter<bool>("load_state_tables", false);            // Load task and robot state tables
        this->declare_parameter<double>("dist_bw_nodes", 1.5);                // Unit distance b/w nodes
        this->declare_parameter<double>("display_scale", 1.0);                // Visualization scale in rviz

        this->declare_parameter<int>("test_mode", -1);
        this->declare_parameter<std::string>("filename_result", " ");
        this->declare_parameter<int>("fms_mode", 1);
        this->declare_parameter<int>("optimizer", 1);

        filename_topolomap = this->get_parameter("filename_topolomap").as_string();
        filename_task_list = this->get_parameter("filename_task_list").as_string();
        filename_task_table = this->get_parameter("filename_task_table").as_string();
        filename_robot_table = this->get_parameter("filename_robot_table").as_string();
        num_of_robots = this->get_parameter("nr_of_robots").as_int();
        run_RHCR = this->get_parameter("run_RHCR").as_bool();
        is_directed_graph = this->get_parameter("directed_graph").as_bool();
        remove_stations = this->get_parameter("remove_stations").as_bool();
        load_state_tables = this->get_parameter("load_state_tables").as_bool();
        dist_bw_nodes = this->get_parameter("dist_bw_nodes").as_double();
        display_scale = this->get_parameter("display_scale").as_double();

        filename_result = this->get_parameter("filename_result").as_string();
        test_mode = this->get_parameter("test_mode").as_int();
        fms_mode = this->get_parameter("fms_mode").as_int();
        optimizer = this->get_parameter("optimizer").as_int();


        /* 1. Initialize Global Variables */
        is_global_route_updated = false;       // If true, global routes are updated
        ready.resize(num_of_robots, false);    // Ready for path planning
        robot_pose.resize(num_of_robots);      // Robot pose information
        local_routes.resize(num_of_robots);    // Computed local routes
        robot_steps.resize(num_of_robots);     // Current robot step of local route (index of waypoint in local route)
        pub_routes.resize(num_of_robots);      // Publishers for each robot's route
        pub_robot_state.resize(num_of_robots); // Publishers for robot state msg
        sub_robot_info.resize(num_of_robots);  // Subscribers for robot info msg
        robot_info.resize(num_of_robots);      // Inserted robot info msg
        util.setColor(num_of_robots);          // Robot colors for visualization
        util.display_scale = display_scale;    // Visualization scale in rviz

        pub_route_infos.resize(num_of_robots);
        sub_curr_steps.resize(num_of_robots);
        robot_curr_seg.assign(num_of_robots,{-1,-1});
        robot_locs.assign(num_of_robots, {-1, -1});

        /* 2. ROS Service Subscribers */
        srv_active_robot_server = this->create_service<fms_msgs::srv::ActiveRobot>("/fms/active_robot", std::bind(&FleetManager::robotActRobotSrvCallback, this, std::placeholders::_1, std::placeholders::_2)); // Service of robot command
        srv_command_robot_server = this->create_service<fms_msgs::srv::CommandRobot>("/fms/command_robot", std::bind(&FleetManager::robotCmdSrvCallback, this, std::placeholders::_1, std::placeholders::_2));   // Service of robot command
        srv_submit_task_server = this->create_service<fms_msgs::srv::SubmitTask>("/fms/submit_task", std::bind(&FleetManager::submitTaskSrvCallback, this, std::placeholders::_1, std::placeholders::_2));       // Service of task submit
        srv_cancel_task_server = this->create_service<fms_msgs::srv::CancelTask>("/fms/cancel_task", std::bind(&FleetManager::cancelTaskSrvCallback, this, std::placeholders::_1, std::placeholders::_2));       // Service of task cancel
        srv_terminate_server = this->create_service<fms_msgs::srv::Terminate>("/fms/terminate", std::bind(&FleetManager::terminateSrvCallback, this, std::placeholders::_1, std::placeholders::_2));             // Service of program termination
        // srv_disable_region_server= this->create_service<fms_msgs::srv::DisableRegion>("/fms/disable_region", &FleetManager::disableRegionSrvCallback); // Service of disable region

        /* 3. ROS Subscribers */
        for (int i = 0; i < num_of_robots; i++)
        {
            std::string robot_name = "carter" + std::to_string(i+1); // robot_0, robot_1, robot_2, ...
            robot_names.push_back(robot_name);                     // Set each robot name
            robot_name_index.insert({robot_names[i], i});
            // Set robot information subscriber for each robot
            sub_robot_info[i] = this->create_subscription<fms_msgs::msg::RobotInfo>(robot_names[i] + "/robot_info", 100, [this, i](typename fms_msgs::msg::RobotInfo::SharedPtr msg)
                                                                                    { this->robotInfoCallback(msg, i); });

            sub_curr_steps[i] = this->create_subscription<fms_msgs::msg::CurrStep>(robot_names[i] + "/curr_step", 10,[this,i](typename fms_msgs::msg::CurrStep::SharedPtr msg)
                                                                                    { this->robotStepCallback(msg, i); });
        }

        /* 4. ROS Publisher */
        pub_topolomap_viz = this->create_publisher<visualization_msgs::msg::MarkerArray>("graph_nodes", 1);         // Visualize the topological map
        pub_topolomap_node_viz = this->create_publisher<visualization_msgs::msg::MarkerArray>("topo_nodes", 1);         // Visualize the topological map
        pub_routes_viz = this->create_publisher<visualization_msgs::msg::MarkerArray>("routes_viz", 1);             // Visualize the planned global routes
        pub_local_routes_viz = this->create_publisher<visualization_msgs::msg::MarkerArray>("local_routes_viz", 1); // Visualize the planned local routes
        pub_poses_viz = this->create_publisher<visualization_msgs::msg::MarkerArray>("robot_poses", 1);             // Visualize the robot poses
        pub_task_viz = this->create_publisher<visualization_msgs::msg::MarkerArray>("task_viz", 1);                 // Visualize the tasks
        pub_temp_viz = this->create_publisher<visualization_msgs::msg::MarkerArray>("task_tmp", 1);                 // Visualize temp markers
        pub_task_table = this->create_publisher<fms_msgs::msg::Tasks>("/fms/tasks", 1);                             // Publish task state table
        pub_robot_table = this->create_publisher<fms_msgs::msg::Robots>("/fms/robots", 1);                          // Publish robot state table
        pub_route_table = this->create_publisher<fms_msgs::msg::RouteTable>("/fms/route_tables", 20);
        for (int i = 0; i < num_of_robots; i++)
        {
            pub_routes[i] = this->create_publisher<nav_msgs::msg::Path>(robot_names[i] + "/route", 20);                  // Set publisher for each planned route
            pub_robot_state[i] = this->create_publisher<fms_msgs::msg::RobotState>(robot_names[i] + "/robot_state", 1); // Publish a robot state (to robot manager)

            pub_route_infos[i] = this->create_publisher<fms_msgs::msg::RouteInfo>(robot_names[i] + "/route_info", 20);
        }

        /* 5. Load topological map */
        loadTopologicalMap();

        /* 6. Init Route, Task & Robot State Tables */
        route_table = new RouteTable();   // Generate the [Route Table]
        task_states = new TaskStates();   // Generate the [Task State Table]
        robot_states = new RobotStates(); // Generate the [Robot State Table]
        initStateTables();

        route_table->start_time = rclcpp::Clock().now();


        if(test_mode < 0){
            auto now = std::chrono::system_clock::now();
            std::time_t t = std::chrono::system_clock::to_time_t(now);
            std::tm tm;
            localtime_r(&t, &tm); // thread-safe

            std::ostringstream oss;
            oss << filename_result
                << std::put_time(&tm, "%Y_%m_%d_%H_%M_%S")
                << "/";

            result_test_dir = oss.str();
        }
        else{
            result_test_dir = filename_result + std::to_string(test_mode)+"/";
        }

        // std::cout << result_test_dir << std::endl;

        if (!std::filesystem::exists(result_test_dir)) {
            std::filesystem::create_directories(result_test_dir);
        }

    }

    FleetManager::~FleetManager()
    {
        if (load_state_tables)
        {
            // Save task and robot state tables
            // std::cout << "Save state tables" << std::endl;
            task_states->saveTaskStateTable(filename_task_table);
            robot_states->saveRobotStateTable(filename_robot_table);
        }
        condition = false;
        // Free pointers
        delete task_states;
        delete robot_states;
        delete route_table;
    }

    bool FleetManager::isReady()
    {
        /*  Check if all robot information is subscribed  */
        bool is_ready = true;
        for (int i = 0; i < num_of_robots; i++)
        {
            // If all robots are ready, start FMS
            if (!ready[i])
            {
                is_ready = false;
            }
        }
        if (robot_states->num_robots != num_of_robots)
            is_ready = false;
        return is_ready;
    }

    /*  Run the thread for MARP (path planning)  */
    void FleetManager::Run()
    {
        condition.store(true);
        tid = std::thread(std::bind(&FleetManager::Loop, this));
    }

    /*  Run a loop of MATP & MARP through a thread  */
    void FleetManager::Loop()
    {
        // Wait for ready
        while (!ready_planning)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(5000));
        }
        // Run MATP & MARP loop
        while (condition)
        {
            // std::cout<<"while"<<std::endl;
            // If we want to stop the program for debuging, we can set the disable parameter to true
            if (disable)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                continue;
            }
            // 1) Assign new tasks at regular time "time_thr_task_planning"
            static double last_task_planning = clock->now().seconds();

            if (need_task_replanning || last_task_planning + time_thr_task_planning < clock->now().seconds())
            {
                // std::cout<<"task replanning"<<std::endl;
                mtx_lock.lock();
                runMultiAgentStatusMonitor();                 // Run multi-agent status monitor: update robot & task state tables
                // std::cout<<"MASM done"<<std::endl;
                bool updated = runMultiAgentTaskAssignment(); // Run multi-agent task planning: assign new tasks to robots
                // std::cout<<"MATA done"<<std::endl;
                publishRobotState();                          // Publish robot states
                // std::cout<<"robot state published"<<std::endl;
                mtx_lock.unlock();
                need_task_replanning = false;
                last_task_planning = clock->now().seconds();
                // std::cout<<"task planning done"<<std::endl;
            }
            // 2) Check if replanning is needed (RHCR does not check the replanning. RHCR replans paths only at regular timesteps)
            if (check_replanning)
            {
                for (int i = 0; i < num_of_robots; i++)
                {
                    // Check need replanning or if a robot has waited longer than a certain amount of time "thr_delay"
                    if (robot_states->getNeedPlanning(i) || robot_states->getTimeDelay(i) > thr_delay)
                    {
                        need_path_replanning = true;
                        break;
                    }
                }
                // std::cout<<"need path replanning: "<<need_path_replanning<<std::endl;
            }

            // 3) Replan paths at regular time "time_thr_route_planning"
            static double last_route_planning = clock->now().seconds();
            if (need_path_replanning || clock->now().seconds() - last_route_planning > time_thr_route_planning)
            {
                mtx_lock.lock();
                runMultiAgentStatusMonitor(); // Run multi-agent status monitor: update robot & task state tables

                publishRobotState(); // Publish robot states

                runMultiAgentTrafficControl(); // Run multi-agent traffic control: update route tables

                mtx_lock.unlock();
                // std::cout<<"MARP"<<std::endl;
                runMultiAgentRoutePlanner(); // Run multi agent route planning
                // std::cout<<"MARP done"<<std::endl;
                last_route_planning = clock->now().seconds();
                // std::cout<<"clock check"<<std::endl;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            // std::cout<<"clock sleep"<<std::endl;
            // std::cout<<condition<<std::endl;
        }
    }

    /*  Main iteration function  */
    void FleetManager::runIteration()
    {

        // Run state monitoring modules (MASM, MATC) at regular time "time_thr_monitor"
        static double last_monitor = clock->now().seconds();
        if (last_monitor + time_thr_monitor < clock->now().seconds())
        {
            mtx_lock.lock();
            iter_monitor++;
            //  1. Run multi-agent status monitor: update robot & task state tables
            runMultiAgentStatusMonitor();
            publishRobotState(); // Publish robot states
            //  2. Run multi-agent traffic control: update route tables and return local routes (sub-routes)
            runMultiAgentTrafficControl();
            last_monitor += time_thr_monitor;
            mtx_lock.unlock();
            ready_planning = true;
        }
        // Plot robot states by publishing the visualizer topics
        util.plotLocalRoutes(local_routes, pub_local_routes_viz);
        util.plotPoses(robot_pose, robot_ids, pub_poses_viz);
        util.plotTasks(pub_task_viz);
        //// util.plotRouteTable(route_table->route_table, pub_temp_viz);
    }

    /* Initialize FMS */
    void FleetManager::initFMS()
    {
        // Parameter setting according to RHCR method or not
        if (run_RHCR)
        {                                                    // 1) RHCR algorithm (Li et al. AAAI-2021)
            check_replanning = false;                        // RHCR peplans paths only at regular time
            time_thr_task_planning = 10.0;                   // Task planning period
            time_thr_route_planning = 20.0;                  // Set a short planning period
            route_table->setParams(num_of_robots, true, 15); // route table params: add_init_table, init_step_size
            MARP.setParams(15, 20, 10, 5.0);                 // 20 30              // MARP params: window, window_extend, time_limit, suboptimal_bound
            route_table->set_result(result_test_dir);
            route_table->set_modes(fms_mode, optimizer);
            MARP.setModes(fms_mode);
        }
        else
        {                                                     // 2) Normal MAPF algorithm (planning whole paths with dummy goals)
            check_replanning = true;                          // Normal algorithm performs replanning whenever a state update
            time_thr_task_planning = 5.0;                    // Task planning period
            time_thr_route_planning = 10.0;                   // Set a longer planning period // 50.0
            time_thr_monitor = 3.0;
            thr_delay = 10.0;
            route_table->setParams(num_of_robots, false, 20);  // route table params: add_init_table, init_step_size
            MARP.setParams(30, 40, 10, 10.0);                // 50 60             // MARP params: window, window_extend, time_limit, suboptimal_bound
            // MARP.setParams(100, 110, 10, 2.0);                // 50 60             // MARP params: window, window_extend, time_limit, suboptimal_bound
            route_table->set_result(result_test_dir);
            route_table->set_modes(fms_mode, optimizer);
            MARP.setModes(fms_mode);
        }
        // Set the state tables, topological map, and solver in MARP, MASM, and MATP modules
        MARP.initMARP(task_states, robot_states, &topolomap);
        MASM.initMASM(task_states, robot_states, &topolomap);
        MATP.initMATP(task_states, robot_states, &topolomap, MARP.path_planner, &MARP.solver_map);
        util.plotTopologicalMap(topolomap, pub_topolomap_viz); // Plot the topological map
        util.setStateTables(task_states, robot_states);

        std::vector<Eigen::Vector2d> node_positions;
        std::vector<int> node_ids;
        for (int i = 0; i < topolomap.num_nodes; i++) {
            node_positions.push_back(topolomap.nodes[i].loc);
            node_ids.push_back(i);
        }
        util.plotNodes(node_positions, node_ids, pub_topolomap_node_viz);

        // Assign duck and charging station
        std::vector<Eigen::Vector3d> poses(num_of_robots);
        for (int i = 0; i < num_of_robots; i++)
        {
            poses[i][0] = robot_pose[i].x;
            poses[i][1] = robot_pose[i].y;
        }
        topolomap.assignDuckAndChargingStation(poses);
    }

    /* Initialize robot & task state tables */
    void FleetManager::initStateTables()
    {
        if (!load_state_tables)
        {
            initTasks(); // Initialize random tasks
            // initRobots();       // Initialize default robots
        }
        else
        { // Read saved files of robot & task state tables
            bool files_exist = std::filesystem::exists(filename_task_table) && std::filesystem::exists(filename_robot_table);
            if (files_exist)
            { // Load task and robot state tables
                task_states->loadTaskStateTable(filename_task_table);
                robot_states->loadRobotStateTable(filename_robot_table);
                fms_msgs::msg::Robots robots = robot_states->getRobotStateTable();
                for (int i = 0; i < num_of_robots; i++)
                {
                    robot_ids.push_back(robot_states->table[i].RobotID);
                    pub_robot_state[i]->publish(robots.robots[i]);
                }
            }
            else
            {                // The table files do not exist!
                initTasks(); // Initialize random tasks
                // initRobots();   // Initialize default robots
            }
        }
    }

    /* Active Robot Callback */
    bool FleetManager::robotActRobotSrvCallback(fms_msgs::srv::ActiveRobot::Request::SharedPtr req, fms_msgs::srv::ActiveRobot::Response::SharedPtr res)
    {

        std::string robot_name = req->robot_id;
        fms_msgs::msg::RobotModel robot_model = req->robot_model;
        robot_states->insertRobot(robot_model);
        robot_ids.push_back(robot_states->table.back().RobotID);
        // std::cout << "Robot activate [" << robot_name << "]" << std::endl;
        res->success = true;
        return true;
    }

    /* Robot Service Callback */
    bool FleetManager::robotCmdSrvCallback(fms_msgs::srv::CommandRobot::Request::SharedPtr req, fms_msgs::srv::CommandRobot::Response::SharedPtr res)
    {
        /*
        std::string robot_name = req.robot_id;
        std::string node_name  = req.goal_node;
        int r_ = robot_states->getRobotIndex(robot_name);
        int s_ = topolomap.getStationId(node_name);
        if (robot_name.empty() || r_ < 0 || (size_t)req.action == 0){
            res.success = false;
        }
        else if (req.action == fms_msgs::CommandRobot::Request::ACTION_MOVING ||
                req.action == fms_msgs::CommandRobot::Request::ACTION_WAITING){
            if (node_name.empty() || s_ < 0)    res.success = false;
            else                                res.success = true;
        }
        else {
            res.success = true;
        }

        if (res.success){
            FMS::RobotAction action(num_inserted_actions++, (size_t)req.action, req.robot_id, req.goal_node);
            action_list.push_back(action);
            std::cout << "[srv_robot_cmd] Get robot action command (action: " << action.action_id << ")" << std::endl;
        }
        */
        return true;
    }

    /* Insert Task Service Callback */
    bool FleetManager::submitTaskSrvCallback(fms_msgs::srv::SubmitTask::Request::SharedPtr req, fms_msgs::srv::SubmitTask::Response::SharedPtr res)
    {
        /*
        std::vector<TaskUnit> task_units;
        res.success = true;
        int num_nodes = req.task_info.task_nodes.size();
        for (int i = 0; i < num_nodes; i++){
            std::string node_name = req.task_info.task_nodes[i].name;
            int sid = topolomap.getStationId(node_name);
            if ( !node_name.empty() && sid >= 0 ){
                // Check location
                double range = 5.0;
                Eigen::Vector2d loc = topolomap.stations[sid].second;
                std::string name = topolomap.stations[sid].first;
                int id = topolomap.getNodeId(loc);

                TaskUnit task_unit;
                task_unit.node_name = name;
                task_unit.location[0] = loc[0];
                task_unit.location[1] = loc[1];
                task_unit.node_id = id;
                task_units.push_back(task_unit);
            }
            else {
                res.success = false;
                break;
            }
        }

        if (res.success){
            TaskStates::Task task;
            task = task_states->insertNewTask(task_units, (size_t)req.task_info.type, (size_t)req.task_info.priority);
            res.task_id = task.TaskID;
            std::cout << "[srv_submit_task] Success of task insertion (" << task.TaskID << ")" << std::endl;
        }
        else {
            std::cout << "[srv_submit_task] Fail of task insertion (invalid node)" << std::endl;
        }
        */
        return true;
    }

    /* Cancel Task Service Callback */
    bool FleetManager::cancelTaskSrvCallback(fms_msgs::srv::CancelTask::Request::SharedPtr req, fms_msgs::srv::CancelTask::Response::SharedPtr res)
    {

        /*
          bool succ = task_states->cancelTask(req.task_id);
          if (succ){
              res.success = true;
              std::cout << "[srv_cancel_task] Success of task cancellation (" << req.task_id << ")" << std::endl;
          }
          else {
              res.success = false;
              std::cout << "[srv_cancel_task] Fail of task cancellation" << std::endl;
          }
          */
        return true;
    }

    /* Termination Service Callback */
    bool FleetManager::terminateSrvCallback(fms_msgs::srv::Terminate::Request::SharedPtr req, fms_msgs::srv::Terminate::Response::SharedPtr res)
    {
        (void)req;
        running = false;
        condition.store(false);
        res->success = true;
        return true;
    }

    /* Subscribe the robot pose and state information (_topic: robot number) */
    // void FleetManager::robotInfoCallback(const fms_msgs::msg::RobotInfo::SharedPtr msg){
    void FleetManager::robotInfoCallback(const fms_msgs::msg::RobotInfo::SharedPtr msg, int index)
    {
        robot_pose[index] = msg->pose;      // Set the robot pose data
        robot_info[index] = msg->ctrl_mode; // Set the robot info
        robot_states->table[index].battery_percent = msg->battery_percent;
        ready[index] = true; // Ready for path planning
    }

    void FleetManager::robotStepCallback(const fms_msgs::msg::CurrStep::SharedPtr msg, int index)
    {
        robot_locs[index].first = msg->start_id;
        robot_locs[index].second = msg->goal_id;
    }

    /*
      Run the Multi Agent Status Monitor
        - Check the task progress (check if the robot has arrived at the target point)
        - Update the task & robot state tables
        - Check if any robots are stationary
    */
    bool FleetManager::runMultiAgentStatusMonitor()
    {
        // Run Multi Agent Status Monitor
        bool do_pp;
        MASM.updateRobotInfo(robot_pose, robot_info);
        if (MASM.checkTaskErros())
        { // Check task errors;
            do_pp = true;
        }
        if (action_list.size() > 0)
        { // Check action_list;
            // First process the actions
            do_pp = MASM.assignRobotActions(action_list);
            action_list.clear();
        }
        else if (MASM.checkTaskCompletion(action_list))
        { // Check whether all tasks are complete
            // std::cout << "All tasks are complete!" << std::endl;
            do_pp = MASM.assignRobotActions(action_list); // Idle robots move to their charging stations
            action_list.clear();
        }
        else
        {
            do_pp = MASM.runMASM();
        }

        return do_pp; // If true, do path planning
    }

    /*
      Run the Multi Agent Task Planner
        - Assign a task to a robot
    */
    bool FleetManager::runMultiAgentTaskAssignment()
    {
        /* Run Multi Agent Task Planning  */
        bool succ = MATP.runMATP(); // Task assignment
        if (succ)
        {
            std::fill(MASM.updated_robot.begin(), MASM.updated_robot.end(), true);
        }
        MATP.updateTimes();
        // True: one or more tasks are assigned to robots
        // False: there is no update
        return succ;
    }

    /*
      Plan the multi-agent routes
        - Run MAPF solver
        - Generate path segments from the planned paths
        - Set the routes (global routes) form the path segments
    */
    void FleetManager::runMultiAgentRoutePlanner()
    {
        mtx_lock.lock();

        // 1) Set the initial route table to be executed by the robots at computation time of MAPF
        //    (If the computation time of MAPF is less then 1~2 sec, the initial route table is not required)
        robot_states->updateNeedPlanningAll(thr_delay);

        std::vector<std::vector<Eigen::Vector3d>> global_routes(num_of_robots);

        std::vector<Eigen::Vector3d> curr_poses(num_of_robots);
        std::vector<int> loc_ids(num_of_robots);
        // std::cout << "============================" << std::endl;
        for (int i = 0; i < num_of_robots; i++)
        {
            curr_poses[i][0] = robot_pose[i].x;
            curr_poses[i][1] = robot_pose[i].y;
            curr_poses[i][2] = robot_pose[i].theta;

            // loc_ids[i] = topolomap.getNodeId(curr_poses[i]);

            // int curr_action_id = route_table->get_current_action(i);

            // std::cout << "current action id: " << curr_action_id << std::endl;



            if(robot_locs[i].first < 0)
            {
                loc_ids[i] = topolomap.getNodeId(curr_poses[i]);
            }
            else
            {
                // loc_ids[i] = curr_action_id;
                loc_ids[i] = robot_locs[i].first;
            }

            // loc_ids[i] = topolomap.getNodeId(curr_poses[i]);

            // // If true, get the closest node id in global path
            // if(is_global_route_updated)
            //     loc_ids[i] = topolomap.getRouteNodeId(i, global_routes, {curr_poses[i][0], curr_poses[i][1]});
            // else
            //     loc_ids[i] = topolomap.getNodeId(curr_poses[i]);

            // std::cout << "Robot " << i << " pose: " << curr_poses[i][0] << ", " << curr_poses[i][1] << ", " << curr_poses[i][2] << std::endl;
            // std::cout << "Robot " << i << " loc_id: " << loc_ids[i] << std::endl;
        }
        // std::cout << "============================" << std::endl;
        // std::cout << "done setting current poses and loc_ids" << std::endl;
        route_table->setInitRouteTable(robot_states->getNeedPlanningAll(), loc_ids);
        // std::cout << "done setting init RT" << std::endl;
        // 2) Set the start and goal for each robot from the robot state table
        std::vector<std::pair<bool, Eigen::Vector3d>> start_nodes = route_table->getStartNodes();
        // std::cout << "done setting SN" << std::endl;
        MARP.setStartsAndGoals(curr_poses, start_nodes);
        // std::cout << "done setting S&G" << std::endl;

        mtx_lock.unlock();
        MARP.updateHeuristics();         // Update A* heuristics for all goal locations
        // std::cout << "done update heuristics" << std::endl;
        MARP.correctOverlappingStarts(); // Correct overlapping start locations
        // std::cout << "done setting GN" << std::endl;

        // 3) Plan the multi-agent paths for all robots using ECBS solver
        bool succ = MARP.runMAPFSolver();
        // std::cout << "done running MAPF solver" << std::endl;
        if (!succ)
        {
            need_path_replanning = true;
            return;
        } // If it cannot find a solution, replan it again
        need_path_replanning = false;
        robot_states->clearAllNeedPlanning();
        // robot_states->clearAllDelay();

        // 4) Update the route table based on the planned paths (routes)
        mtx_lock.lock();
        route_table->setRouteTable(MARP.routes, loc_ids, curr_poses);

        if (remove_stations)
        {                                                      // If FMS removes the station nodes in topological map,
            route_table->setGoalStationNodes(MARP.goal_poses); // append the goal station nodes in route table
        }
        mtx_lock.unlock();

        // 5) Update global paths in robot state table
        for (int i = 0; i < num_of_robots; i++)
        {
            global_routes[i] = route_table->getGlobalRoute(i);
        }
        robot_states->setPaths(global_routes);
        is_global_route_updated = true;
    }

    /*
      Run Multi Agent Traffic Controler
        - Update the robot progress and local routes
        - Publish the local routes to each robot
    */
    void FleetManager::runMultiAgentTrafficControl()
    {

        // Update the node states in route table
        std::vector<Eigen::Vector2d> poses(num_of_robots);
        for (int i = 0; i < num_of_robots; i++)
        {
            poses[i] = Eigen::Vector2d(robot_pose[i].x, robot_pose[i].y);
        }

        route_table->runESES();
        route_table->updateRouteStates(poses);
        // std::cout << "============================" << std::endl;

        // route_table->switchRouteTableNodes(poses);

        // Update and publish the local routes
        for (int i = 0; i < num_of_robots; i++)
        {
            std::vector<Eigen::Vector3d> local_route;
            local_route = route_table->getLocalRoute(i, poses); // Get the local route from the route table
            local_routes[i].clear();
            local_routes[i] = local_route; // Set the local route
            publishRoute(local_route, i);  // Publish the local route
            publishRouteInfo(local_route, i);  // Publish the local route
        }
    }

    /* Load topological map from file (Read RMF map format) */
    void FleetManager::loadTopologicalMap()
    {
        // Read topological map (update nodes & lanes)
        topolomap.loadTopologicalMapFile(filename_topolomap, is_directed_graph, remove_stations, dist_bw_nodes);
        topolomap.setZone(); // Set the zone of nodes [Comment here]
        // std::vector<Eigen::Vector2d> node_positions;
        // std::vector<int> node_ids;
        // for (int i = 0; i < topolomap.num_nodes; i++) {
        //     node_positions.push_back(topolomap.nodes[i].loc);
        //     node_ids.push_back(i);
        // }
        // util.plotNodes(node_positions, node_ids, pub_topolomap_node_viz);
    }

    /* Initialize the tasks */
    void FleetManager::initTasks()
    {

        bool load_task_list = false;
        if (load_task_list)
        {
            if (!initTasksLoad())  // Load task list
                initTasksRandom(); // Initialize random tasks
        }
        else
        {
            initTasksRandom(); // Initialize random tasks
        }
    }

    /* Initialize the tasks: simply read task list and update the task state table */
    bool FleetManager::initTasksLoad()
    {
        // Create an input filestream
        std::ifstream file_task(filename_task_list);
        if (file_task.is_open())
        {                              // Make sure the file is open
            std::string line, colname; // Helper vars
            // Read the column names
            if (file_task.good())
            {
                std::getline(file_task, line); // Extract the first line in the file
            }
            // Read task data, line by line
            while (std::getline(file_task, line))
            {
                int num, type, priority;
                std::stringstream s(line); // Used for breaking words
                std::string val;
                getline(s, val, ',');
                num = atoi(val.c_str()); // 1) number
                getline(s, val, ',');
                type = atoi(val.c_str()); // 2) task type (1:TYPE_INBOUND, 2:TYPE_DELIEVERY, 3:TYPE_OUTBOUND)
                getline(s, val, ',');
                priority = atoi(val.c_str()); // 3) task priority
                std::vector<TaskUnit> task_units;
                while (std::getline(s, val, ','))
                {
                    TaskUnit task_unit;
                    task_unit.node_name = val; // 4) task station name
                    getline(s, val, ',');
                    task_unit.location[0] = std::stod(val.c_str()); // 5) task x location
                    getline(s, val, ',');
                    task_unit.location[1] = std::stod(val.c_str()); // 6) task y location
                    task_units.push_back(task_unit);
                    // std::cout << "[" << task_unit.node_name << ", " << task_unit.location[0] << ", " << task_unit.location[1] << "] ";
                }
                // std::cout << std::endl;
                // Insert the task into the task state table
                task_states->insertNewTask(task_units, (size_t)type, (size_t)priority);
            }
            file_task.close(); // Close file
            return true;
        }
        else
        {
            return false;
        }

        return true;
    }

    /* Initialize the tasks: generate random tasks */
    void FleetManager::initTasksRandom()
    {
        // std::cout << "generate randoom tasks" << std::endl;

        int task_option = 1; // 0: stacking, 1: order picking

        std::vector<int> st_inbounds, st_outbounds, st_racks;
        std::vector<int> op_inbounds, op_outbounds, op_racks;
        for (int i = 0; i < topolomap.stations.size(); i++)
        {
            Eigen::Vector2d loc = topolomap.stations[i].second;
            int zone = topolomap.nodes[topolomap.getNodeId(loc)].zone;
            zone = task_option;
            if (zone == 0)
            {
                if (topolomap.stations[i].first.find("I") != std::string::npos)
                    st_inbounds.push_back(i);
                else if (topolomap.stations[i].first.find("O") != std::string::npos)
                    st_outbounds.push_back(i);
                else if (topolomap.stations[i].first.find("R") != std::string::npos)
                    st_racks.push_back(i);
            }
            else if (zone == 1)
            {
                if (topolomap.stations[i].first.find("I") != std::string::npos)
                    op_inbounds.push_back(i);
                else if (topolomap.stations[i].first.find("O") != std::string::npos)
                    op_outbounds.push_back(i);
                else if (topolomap.stations[i].first.find("R") != std::string::npos)
                    op_racks.push_back(i);
            }
        }
        std::random_shuffle(st_inbounds.begin(), st_inbounds.end());
        std::random_shuffle(st_outbounds.begin(), st_outbounds.end());
        std::random_shuffle(st_racks.begin(), st_racks.end());
        std::random_shuffle(op_inbounds.begin(), op_inbounds.end());
        std::random_shuffle(op_outbounds.begin(), op_outbounds.end());
        std::random_shuffle(op_racks.begin(), op_racks.end());

        if (task_option == 0)
        {
            // 1) Stacking
            int st_num_task = 1;
            for (int i = 0; i < st_num_task; i++)
            {
                std::vector<TaskUnit> task_units;
                int rack_i = st_racks[rand() % st_racks.size()];
                int in_i = st_inbounds[i % st_inbounds.size()];
                TaskUnit task_unit1(Eigen::Vector3d(topolomap.stations[rack_i].second[0], topolomap.stations[rack_i].second[1], 0),
                                    topolomap.stations[rack_i].first);
                TaskUnit task_unit2(Eigen::Vector3d(topolomap.stations[in_i].second[0], topolomap.stations[in_i].second[1], 0),
                                    topolomap.stations[in_i].first);
                task_units.push_back(task_unit1);
                task_units.push_back(task_unit2);
                task_states->insertNewTask(task_units, fms_msgs::msg::TaskInfo::TYPE_STACKING, rand() % 3 + 1);
            }
        }
        else
        {
            // 2) Order Picking
            int op_num_task = num_of_robots;
            for (int i = 0; i < op_num_task; i++)
            {

                std::vector<TaskUnit> task_units;
                int num_subtasks = 1;
                for (int t = 0; t < num_subtasks; t++)
                {
                    int rack_i = op_racks[rand() % op_racks.size()];
                    TaskUnit task_unit(Eigen::Vector3d(topolomap.stations[rack_i].second[0], topolomap.stations[rack_i].second[1], 0),
                                       topolomap.stations[rack_i].first);
                    task_units.push_back(task_unit);
                }

                int in_i = op_inbounds[i % op_inbounds.size()];
                TaskUnit task_unit(Eigen::Vector3d(topolomap.stations[in_i].second[0], topolomap.stations[in_i].second[1], 0),
                                   topolomap.stations[in_i].first);
                task_units.push_back(task_unit);

                /*
                int rack_i1 = op_racks[ rand()%op_racks.size() ];
                int rack_i2 = op_racks[ rand()%op_racks.size() ];
                int in_i   = op_inbounds[i % op_inbounds.size()];
                TaskUnit task_unit1(Eigen::Vector3d(topolomap.stations[rack_i1].second[0], topolomap.stations[rack_i1].second[1], 0),
                                    topolomap.stations[rack_i1].first);
                TaskUnit task_unit2(Eigen::Vector3d(topolomap.stations[rack_i2].second[0], topolomap.stations[rack_i2].second[1], 0),
                                    topolomap.stations[rack_i2].first);
                TaskUnit task_unit3(Eigen::Vector3d(topolomap.stations[in_i].second[0], topolomap.stations[in_i].second[1], 0),
                                    topolomap.stations[in_i].first);
                task_units.push_back(task_unit1);
                task_units.push_back(task_unit2);
                task_units.push_back(task_unit3);
                */
                task_states->insertNewTask(task_units, fms_msgs::msg::TaskInfo::TYPE_ORDERPICK, rand() % 3 + 1);
            }
        }
        // std::cout << "generate randoom tasks - END" << std::endl;
    }

    /* Initialize the robot state table */
    void FleetManager::initRobots()
    {
        for (int i = 0; i < num_of_robots; i++)
        {
            robot_states->insertRobot(fms_msgs::msg::RobotModel());
            robot_ids.push_back(robot_states->table.back().RobotID);
        }
    }

    /* Publish the computed local route */
    void FleetManager::publishRoute(std::vector<Eigen::Vector3d> _p, int _topic)
    {
        nav_msgs::msg::Path path;
        // path.header.seq = 0;
        path.header.stamp = clock->now();
        path.header.frame_id = "map";
        for (const Eigen::Vector3d &p : _p)
        {
            geometry_msgs::msg::PoseStamped ps;
            // ps.header.seq = 0;
            ps.header.stamp = clock->now();
            ps.header.frame_id = "map";
            ps.pose.position.x = p[0];
            ps.pose.position.y = p[1];
            Eigen::Quaternion<float> q;
            q = Eigen::AngleAxisf(p[2], Eigen::Vector3f::UnitZ());
            ps.pose.orientation.x = q.x();
            ps.pose.orientation.y = q.y();
            ps.pose.orientation.z = q.z();
            ps.pose.orientation.w = q.w();
            path.poses.push_back(ps);
        }
        pub_routes[_topic]->publish(path);
    }

    void FleetManager::publishRouteInfo(std::vector<Eigen::Vector3d> _p, int _topic)
    {
        fms_msgs::msg::RouteInfo id_msg;
        for(const auto &p : _p){
            int nid = topolomap.getNodeId(Eigen::Vector2d(p[0],p[1]));
            id_msg.node_ids.push_back(nid);
        }
        pub_route_infos[_topic]->publish(id_msg);
    }

    /* Publish a robot state */
    void FleetManager::publishRobotState()
    {
        fms_msgs::msg::Robots robots = robot_states->getRobotStateTable();
        for (int i = 0; i < num_of_robots; i++)
        {
            if (MASM.updated_robot[i])
            {
                pub_robot_state[i]->publish(robots.robots[i]);
            }
        }
    }

    /* Publish the task state table to UI */
    void FleetManager::publishStateTables()
    {
        fms_msgs::msg::Tasks tasks = task_states->getTaskStateTable();
        fms_msgs::msg::Robots robots = robot_states->getRobotStateTable();
        pub_task_table->publish(tasks);
        pub_robot_table->publish(robots);
    }

    /* Publish the route table to UI */
    void FleetManager::publishRouteTables()
    {
        fms_msgs::msg::RouteTable route_tables = route_table->getRouteTable();
        pub_route_table->publish(route_tables);
    }

    void FleetManager::Release()
    {
        condition.store(false);
        running = false;
        if (tid.joinable())
        {
            tid.join();
        }
    }
}
