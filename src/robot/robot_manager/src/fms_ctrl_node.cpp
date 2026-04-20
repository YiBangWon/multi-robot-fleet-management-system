// Multi-robot controller node that bridges fleet-manager routes to robot motion commands.

#include <robot_manager/fms_ctrl_node.h>

#define NSEC_2_SECS(A) ((float)A / 1000000000.0)


int main(int argc, char **argv){

    rclcpp::init(argc, argv);
    auto node = std::make_shared<robot_manager::fms_ctrl_node>();
    std::thread t = std::thread([&node]() {rclcpp::spin(node);});
    bool all_ready = false;

    while (!all_ready){
        all_ready = true;
        for (int i = 0; i < node->nr_of_robots; i++){
            if (!node->ready[i]) {all_ready = false; break;}
        }
    }
    node->sendActivationSrv();
    node->Run();
    rclcpp::shutdown();

    return 1;
}

namespace robot_manager{

fms_ctrl_node::fms_ctrl_node() : Node("fms_ctrl_node"){

    clock = this->get_clock();

    // Parameters for robot controller
    this->declare_parameter<int>("nr_of_robots", 0);
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

    nr_of_robots = this->get_parameter("nr_of_robots").as_int();
    is_pure_rotation = this->get_parameter("is_pure_rotation").as_bool();
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
    pub_robot_info.resize(nr_of_robots);      // Publisher for robot_info (robot_pose, ctrl_mode, battery)
    pub_cmdvel.resize(nr_of_robots);          // Publisher for cmd_vel (velocity commands)
    sub_odom.resize(nr_of_robots);            // Subscriber for odometry (robot pose)
    sub_path.resize(nr_of_robots);            // Subscriber for route (local path)
    sub_robot_state.resize(nr_of_robots);     // Subscriber for robot_state (current robot state)
    robot_names.resize(nr_of_robots);         // Robot name (e.g.: robot_0, robot_1, robot_2)
    controller.resize(nr_of_robots);          // Robot controllers
    robot_pose.resize(nr_of_robots);          // Robot pose
    update_time.resize(nr_of_robots);         // Last update time of robot pose
    ready.resize(nr_of_robots, false);

    // ------- Communication topics b/w FMS and Robots -------
    // FMS -> Robot: robot_state (task_id, goal_node, target_pose, global_path, robot_mode)
    // FMS <- Robot: robot_info (robot_pose, ctrl_mode, battery)
    actrobot_srv_client = this->create_client<fms_msgs::srv::ActiveRobot>("/fms/active_robot");
    for (int i = 0; i < nr_of_robots; i++){
        managers.push_back(RobotManager("AGF"+std::to_string(i), 0, clock));  // Initialize robot managers
        managers[i].clock = clock;
        robot_names[i] = "robot_" + std::to_string(i);
        robot_name_index.insert({robot_names[i], i});
        pub_robot_info[i]  = this->create_publisher<fms_msgs::msg::RobotInfo>(robot_names[i] + "/robot_info", 5);
        pub_cmdvel[i]      = this->create_publisher<geometry_msgs::msg::Twist>(robot_names[i] + "/cmd_vel", 1);
        sub_robot_state[i] = this->create_subscription<fms_msgs::msg::RobotState>(robot_names[i] + "/robot_state", 5, [this, i](typename fms_msgs::msg::RobotState::SharedPtr msg) {this->subRobotStateCallback(msg, i);});
        sub_odom[i] = this->create_subscription<nav_msgs::msg::Odometry>(robot_names[i] + "/odom", 5, [this, i](typename nav_msgs::msg::Odometry::SharedPtr msg) {this->subOdomCallback(msg, i);});
        sub_path[i] = this->create_subscription<nav_msgs::msg::Path>(robot_names[i] + "/route", 1, [this, i](typename nav_msgs::msg::Path::SharedPtr msg) {this->subPathCallback(msg, i);});
        update_time[i] = clock->now();
    }
    // Set parmas & controller type
    for (int i = 0; i < nr_of_robots; i++){
        controller[i].is_pure_rotation = is_pure_rotation;
        controller[i].robot_name = robot_names[i];
        controller[i].max_v = max_v;
        controller[i].max_w = max_w;
        managers[i].thr_task_time = robot_model.task_time;
    }

}

void fms_ctrl_node::Run(){
    // Run robot manager & controller
    rclcpp::WallRate loop_rate(5);
    while (rclcpp::ok()){
        updateRobotState(); // Update robot state
        publishRobotInfo(); // Publish robot info
        checkGoalArrival();
        loop_rate.sleep();
    }
}

void fms_ctrl_node::checkGoalArrival(){
    for (int i = 0; i < nr_of_robots; i++){
        if (managers[i].is_goal_arrival){
            geometry_msgs::msg::Twist cmd_vel;
            cmd_vel.linear.x = 0;   cmd_vel.linear.y = 0;   cmd_vel.angular.z = 0;
            pub_cmdvel[i]->publish(cmd_vel);   // Publish ctrl_cmd to robot hardware
            managers[i].is_goal_arrival = false;
        }
    }
}

void fms_ctrl_node::sendActivationSrv(){

    for (int i = 0; i < nr_of_robots; i++){
        // Send ROS Service to active this robot
        auto actrobot_srv = std::make_shared<fms_msgs::srv::ActiveRobot::Request>();
        actrobot_srv->robot_id = robot_names[i];
        actrobot_srv->robot_model = robot_model;
        auto result = actrobot_srv_client->async_send_request(actrobot_srv);
    }
}



// Update following robot states
//   1) Goal arrival
//   2) Completion of loading, unloading, charging, waiting
//   3) Battery and error states
void fms_ctrl_node::updateRobotState(){
    for (int i = 0; i < nr_of_robots; i++){
        // For each robot, update robot states using robot manager
        managers[i].update_state(robot_pose[i]);
    }
}

// Publish robot information (ctrl_mode, robot_pose, battery)
void fms_ctrl_node::publishRobotInfo(){
    for (int i = 0; i < nr_of_robots; i++){
        fms_msgs::msg::RobotInfo ctrl_mode;
        ctrl_mode.ctrl_mode  = managers[i].ctrl_mode;
        ctrl_mode.pose.x     = managers[i].curr_pose[0];
        ctrl_mode.pose.y     = managers[i].curr_pose[1];
        ctrl_mode.pose.theta = managers[i].curr_pose[2];
        ctrl_mode.battery_percent = managers[i].battery_percent;
        pub_robot_info[i]->publish(ctrl_mode);
    }
}


// Callback for robot state subscriber
//    -  Update robot state based on the subscribed msg
void fms_ctrl_node::subRobotStateCallback(const fms_msgs::msg::RobotState::SharedPtr msg, int index){
    managers[index].insert_state_msg(*msg); // Insert robot state msg and update the state
}


// Callback for odometry subscriber
//    - Update robot pose and delta time
//    - Compute velocity commands using robot controller
void fms_ctrl_node::subOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg, int index){

    ready[index] = true;

    // Update robot pose from odom msg
    Eigen::Vector3d pose_curr = getPoseVec(msg->pose.pose);
    robot_pose[index].x = pose_curr[0];
    robot_pose[index].y = pose_curr[1];
    robot_pose[index].theta = pose_curr[2];

    // Compute delta time and set update time
    rclcpp::Time curr_time = clock->now();
    rclcpp::Duration d = curr_time - update_time[index];
    update_time[index] = curr_time;                     // Last update time
    float delta_t = d.seconds() + NSEC_2_SECS(d.nanoseconds());         // Delta time
    Eigen::Vector2d vw(msg->twist.twist.linear.x, msg->twist.twist.angular.z);

    if ( managers[index].ctrl_mode == fms_msgs::msg::RobotInfo::CTRL_MOVING_TASK     ||    // Check moving modes
         managers[index].ctrl_mode == fms_msgs::msg::RobotInfo::CTRL_MOVING_CHARGING ||
         managers[index].ctrl_mode == fms_msgs::msg::RobotInfo::CTRL_MOVING          ||
         managers[index].ctrl_mode == fms_msgs::msg::RobotInfo::CTRL_MOVING_WAITING ){
            // Controller computes translation and angular velocities to follow a local route
            geometry_msgs::msg::Twist cmd_vel = controller[index].getCmdVel(pose_curr, delta_t, vw);
            pub_cmdvel[index]->publish(cmd_vel);   // Publish ctrl_cmd to robot hardware
         }
}


// Callback for path subscriber
//    - Update the local path in controller
void fms_ctrl_node::subPathCallback(const nav_msgs::msg::Path::SharedPtr msg, int index){

    // If it is an empty path, reject this path msg
    // if (path->poses.size() == 0) return;
    std::vector<Eigen::Vector3d> local_path;
    for (int i = 0 ; i < (int)msg->poses.size() ; i++){
        Eigen::Vector3d pt = getPoseVec(msg->poses[i].pose);
        local_path.push_back(pt);
    }
    controller[index].setPath(local_path); // Update local path in controller
}

// Get pose vector from pose msg
Eigen::Vector3d fms_ctrl_node::getPoseVec(geometry_msgs::msg::Pose pose){
    double roll, pitch, yaw;
    tf2::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    tf2::Matrix3x3 m(q);
    m.getRPY ( roll, pitch, yaw );
    return Eigen::Vector3d(pose.position.x, pose.position.y, yaw);  // 2D pose: x, y, yaw
}

}

