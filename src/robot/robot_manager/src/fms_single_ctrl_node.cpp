// Single-robot controller node used for simulation and focused controller debugging.

#include <robot_manager/fms_single_ctrl_node.h>
#include <chrono>
#define NSEC_2_SECS(A) ((float)A / 1000000000.0)

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<robot_manager::fms_single_ctrl_node>();
    std::thread spin_thread([&node]()
                            { rclcpp::spin(node); });

    rclcpp::WallRate loop_rate(5);
    while (!node->ready)
    {
        loop_rate.sleep();
    }

    node->Run();
    rclcpp::shutdown();

    if (spin_thread.joinable()) {
        spin_thread.join();
    }

    return 0;
}

namespace robot_manager
{

    fms_single_ctrl_node::fms_single_ctrl_node() : Node("fms_single_ctrl_node")
    {
        clock = this->get_clock();

        // Parameters for robot controller
        this->declare_parameter<std::string>("robot_name", "robot");
        this->declare_parameter<bool>("is_pure_rotation", true);
        this->declare_parameter<double>("max_v", 0.26);
        this->declare_parameter<double>("max_w", 1.82);

        // Parameters for robot model
        this->declare_parameter<int>("model_id", 0);
        this->declare_parameter<int>("type", 0);
        this->declare_parameter<std::string>("model_name", "robot_model_0");
        this->declare_parameter<double>("battery_life", 3600.0);
        this->declare_parameter<double>("weight", 50.0);
        this->declare_parameter<double>("max_weight", 100.0);
        this->declare_parameter<double>("size_w", 0.306);
        this->declare_parameter<double>("size_h", 0.141);
        this->declare_parameter<double>("size_d", 0.281);
        this->declare_parameter<double>("task_time", 5.0);

        // Parameters for mode option
        this->declare_parameter<bool>("run_stage", false);

        robot_name = this->get_parameter("robot_name").as_string();
        is_pure_rotation = this->get_parameter("is_pure_rotation").as_bool();

        run_stage = this->get_parameter("run_stage").as_bool();

        max_v = this->get_parameter("max_v").as_double();
        max_w = this->get_parameter("max_w").as_double();
        robot_model.model_id = this->get_parameter("model_id").as_int();
        robot_model.model_name = this->get_parameter("model_name").as_string();
        robot_model.max_v = this->get_parameter("max_v").as_double();
        robot_model.max_w = this->get_parameter("max_w").as_double();
        robot_model.type = this->get_parameter("type").as_int();
        robot_model.battery_life = this->get_parameter("battery_life").as_double();
        robot_model.weight = this->get_parameter("weight").as_double();
        robot_model.max_weight = this->get_parameter("max_weight").as_double();
        robot_model.size_w = this->get_parameter("size_w").as_double();
        robot_model.size_h = this->get_parameter("size_h").as_double();
        robot_model.size_d = this->get_parameter("size_d").as_double();
        robot_model.task_time = this->get_parameter("task_time").as_double();

        // Initialize global variables
        ready = false;
        update_time = clock->now();

        // ------- Communication topics b/w FMS and Robots -------
        // FMS -> Robot: robot_state (task_id, goal_node, target_pose, global_path, robot_mode)
        // FMS <- Robot: robot_info (robot_pose, ctrl_mode, battery)
        manager = new RobotManager(robot_name, 0, clock); // Initialize robot managers

        // Service Client

        actrobot_srv_client = this->create_client<fms_msgs::srv::ActiveRobot>("/fms/active_robot");
        global_clear_entire_Costmap_srv_client = this->create_client<nav2_msgs::srv::ClearEntireCostmap>(robot_name + "/global_costmap/clear_entirely_global_costmap");

        // Gazebo
        if(!run_stage){
            nav_act_client  = rclcpp_action::create_client<nav2_msgs::action::NavigateThroughPoses>(this, robot_name + "/navigate_through_poses");
            // sub_pose        = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(robot_name + "/amcl_pose", 10, std::bind(&fms_single_ctrl_node::subPoseCallback, this, std::placeholders::_1));
            sub_route       = this->create_subscription<nav_msgs::msg::Path>(robot_name + "/route", 20, std::bind(&fms_single_ctrl_node::subRouteCallback, this, std::placeholders::_1));
        }
        // Stage
        if(run_stage){
            pub_cmdvel      = this->create_publisher<geometry_msgs::msg::Twist>(robot_name + "/cmd_vel", 1);
            sub_path        = this->create_subscription<nav_msgs::msg::Path>(robot_name + "/route", 20, std::bind(&fms_single_ctrl_node::subPathCallback, this, std::placeholders::_1));
            // sub_gt_pose     = this->create_subscription<nav_msgs::msg::Odometry>("/ground_truth", 10, std::bind(&fms_single_ctrl_node::subGtPoseCallback, this, std::placeholders::_1));
        }
        sub_odom = this->create_subscription<nav_msgs::msg::Odometry>(robot_name + "/odom", 10, std::bind(&fms_single_ctrl_node::subOdomCallback, this, std::placeholders::_1));

        // Publishers
        pub_robot_info = this->create_publisher<fms_msgs::msg::RobotInfo>(robot_name + "/robot_info", 5);

        // Subscribers
        sub_robot_state = this->create_subscription<fms_msgs::msg::RobotState>(robot_name + "/robot_state", 10, std::bind(&fms_single_ctrl_node::subRobotStateCallback, this, std::placeholders::_1));

        // Set parmas & controller type
        controller.is_pure_rotation = is_pure_rotation;
        controller.robot_name = robot_name;
        controller.max_v = max_v;
        controller.max_w = max_w;
        manager->thr_task_time = robot_model.task_time;

        sleep(1);
        sendActivationSrv();
    }

    fms_single_ctrl_node::~fms_single_ctrl_node()
    {
        delete manager;
    }

    void fms_single_ctrl_node::Run()
    {
        // Run robot manager & controller
        rclcpp::WallRate loop_rate(5);
        while (rclcpp::ok())
        {
            updateRobotState(); // Update robot state
            sendClearmapSrv();
            checkGoalArrival();
            publishRobotInfo(); // Publish robot info

            loop_rate.sleep();

        }
    }

    void fms_single_ctrl_node::checkGoalArrival()
    {
        if (manager->is_goal_arrival)
        {
            if(run_stage){
                geometry_msgs::msg::Twist cmd_vel;
                cmd_vel.linear.x = 0;   cmd_vel.linear.y = 0;   cmd_vel.angular.z = 0;
                pub_cmdvel->publish(cmd_vel);   // Publish ctrl_cmd to robot hardware
            }
            else{
                // Navigation completion is handled by the Nav2 action result callback.
            }
            manager->is_goal_arrival = false;
        }
    }

    void fms_single_ctrl_node::sendActivationSrv()
    {

        if (robot_model.type == 1)
            sleep(2);
        else if (robot_model.type == 2)
            sleep(4);

        // Send ROS Service to active this robot
        auto actrobot_srv = std::make_shared<fms_msgs::srv::ActiveRobot::Request>();
        actrobot_srv->robot_id = robot_name;
        actrobot_srv->robot_model = robot_model;
        auto result = actrobot_srv_client->async_send_request(actrobot_srv);
    }

    void fms_single_ctrl_node::sendClearmapSrv()
    {
        // sleep(1);
        // Send ROS Service to active this robot
        send_request_entire_clear_global_();
    }

    // Update following robot states
    //   1) Goal arrival
    //   2) Completion of loading, unloading, charging, waiting
    //   3) Battery and error states
    void fms_single_ctrl_node::updateRobotState()
    {
        // Update robot states using robot manager
        manager->update_state(robot_pose);
    }

    // Publish robot information (ctrl_mode, robot_pose, battery)
    void fms_single_ctrl_node::publishRobotInfo()
    {
        fms_msgs::msg::RobotInfo ctrl_mode;
        ctrl_mode.ctrl_mode = manager->ctrl_mode;
        ctrl_mode.pose.x = manager->curr_pose[0];
        ctrl_mode.pose.y = manager->curr_pose[1];
        ctrl_mode.pose.theta = manager->curr_pose[2];
        ctrl_mode.battery_percent = manager->battery_percent;
        pub_robot_info->publish(ctrl_mode);
    }

    // Callback for robot state subscriber
    //    -  Update robot state based on the subscribed msg
    void fms_single_ctrl_node::subRobotStateCallback(const fms_msgs::msg::RobotState::SharedPtr msg)
    {
        manager->insert_state_msg(*msg); // Insert robot state msg and update the state
    }

    // Callback for odometry subscriber
    //    - Update robot pose and delta time
    //    - Compute velocity commands using robot controller
    void fms_single_ctrl_node::subOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        ready = true;

        // Update robot pose from odom msg
        Eigen::Vector3d pose_curr = getPoseVec(msg->pose.pose);
        robot_pose.x = pose_curr[0];
        robot_pose.y = pose_curr[1];
        robot_pose.theta = pose_curr[2];

        if(run_stage){
            // Compute delta time and set update time
            rclcpp::Time curr_time = clock->now();
            rclcpp::Duration d = curr_time - update_time;
            update_time = curr_time;                                    // Last update time
            float delta_t = d.seconds() + NSEC_2_SECS(d.nanoseconds()); // Delta time
            Eigen::Vector2d vw(msg->twist.twist.linear.x, msg->twist.twist.angular.z);

            if (manager->ctrl_mode == fms_msgs::msg::RobotInfo::CTRL_MOVING_TASK || // Check moving modes
                manager->ctrl_mode == fms_msgs::msg::RobotInfo::CTRL_MOVING_CHARGING ||
                manager->ctrl_mode == fms_msgs::msg::RobotInfo::CTRL_MOVING ||
                manager->ctrl_mode == fms_msgs::msg::RobotInfo::CTRL_MOVING_WAITING){
                    // controller.getCmdVel(pose_curr, delta_t, vw);
                    geometry_msgs::msg::Twist cmd_vel = controller.getCmdVel(pose_curr, delta_t, vw);
                    pub_cmdvel->publish(cmd_vel);   // Publish ctrl_cmd to robot hardware
                }
        }
    }

    void fms_single_ctrl_node::subPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        ready = true;

        // Update robot pose from odom msg
        Eigen::Vector3d pose_curr = getPoseVec(msg->pose.pose);
        robot_pose.x = pose_curr[0];
        robot_pose.y = pose_curr[1];
        robot_pose.theta = pose_curr[2];
    }

    void fms_single_ctrl_node::subRouteCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        if (msg->poses.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received an empty route for %s; skipping action dispatch.", robot_name.c_str());
            return;
        }

        // send new goal
        if (!nav_act_client->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "NavigateThroughPoses action server is unavailable for %s", robot_name.c_str());
            return;
        }

        auto goal_msg = nav2_msgs::action::NavigateThroughPoses::Goal();
        goal_msg.poses = msg->poses;  // set goals from /route data

        auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&fms_single_ctrl_node::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback =
            std::bind(&fms_single_ctrl_node::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback =
            std::bind(&fms_single_ctrl_node::result_callback, this, std::placeholders::_1);
        nav_act_client->async_send_goal(goal_msg, send_goal_options);
    }

    void fms_single_ctrl_node::goal_response_callback(rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::SharedPtr goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Nav2 rejected the route for %s", robot_name.c_str());
        } else {
            RCLCPP_INFO(this->get_logger(), "Nav2 accepted a route for %s", robot_name.c_str());
        }
    }

    void fms_single_ctrl_node::feedback_callback(
       rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::SharedPtr,
       const std::shared_ptr<const nav2_msgs::action::NavigateThroughPoses::Feedback> feedback)
    {
        RCLCPP_DEBUG(this->get_logger(), "%s remaining distance: %.2f", robot_name.c_str(), feedback->distance_remaining);
    }

    void fms_single_ctrl_node::result_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::WrappedResult &result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Navigation goal succeeded for %s", robot_name.c_str());
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Navigation goal was aborted for %s", robot_name.c_str());
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_WARN(this->get_logger(), "Navigation goal was canceled for %s", robot_name.c_str());
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Navigation goal finished with an unknown result for %s", robot_name.c_str());
                break;
        }
    }


    void fms_single_ctrl_node::send_request_entire_clear_global_()
    {

        static auto last_request_time = this->now();

        // Throttle costmap clearing to avoid sending a request every control tick.
        if ((this->now() - last_request_time).seconds() < 1.0) {
            return;
        }
        last_request_time = this->now();

        if (!global_clear_entire_Costmap_srv_client->wait_for_service(std::chrono::seconds(1))) {
            // RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
            return;
        }

        auto request = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();

        global_clear_entire_Costmap_srv_client->async_send_request(request,
            [this](rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedFuture future) {
                try {
                    future.get();
                    // RCLCPP_INFO(this->get_logger(), "Costmap cleared");
                } catch (const std::exception &e) {
                    RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
                }
            }
        );
    }

    // Callback for path subscriber
    //    - Update the local path in controller
    void fms_single_ctrl_node::subPathCallback(const nav_msgs::msg::Path::SharedPtr path){

        // If it is an empty path, reject this path msg
        // if (path->poses.size() == 0) return;

        std::vector<Eigen::Vector3d> local_path;
        for (int i = 0 ; i < (int)path->poses.size() ; i++){
            Eigen::Vector3d pt = getPoseVec(path->poses[i].pose);
            local_path.push_back(pt);
        }
        controller.setPath(local_path); // Update local path in controller
    }

    // Get pose vector from pose msg
    Eigen::Vector3d fms_single_ctrl_node::getPoseVec(geometry_msgs::msg::Pose pose)
    {
        double roll, pitch, yaw;
        tf2::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
        tf2::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);
        return Eigen::Vector3d(pose.position.x, pose.position.y, yaw); // 2D pose: x, y, yaw
    }

    void fms_single_ctrl_node::subGtPoseCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        auto pose = msg->pose.pose.position;
        for(auto p: controller.gt_pose){
            if(pose.x == p.x() && pose.y == p.y()){
                return;
            }
        }
        controller.gt_pose.push_back(Eigen::Vector2d(pose.x, pose.y));
    }
}
