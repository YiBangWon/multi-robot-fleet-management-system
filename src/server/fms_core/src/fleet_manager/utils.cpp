// Visualization and helper utilities shared by the fleet manager runtime.

#include <fleet_manager/utils.h>

namespace FMS
{

void Util::plotRouteTable(std::vector<std::vector<NodeRT*>> route_table, rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_){

    // Clear node buffer
    visualization_msgs::msg::MarkerArray node_del;
    visualization_msgs::msg::Marker del;
    del.id = 0;
    del.header.frame_id = "map";
    del.header.stamp = clock->now();
    del.action = visualization_msgs::msg::Marker::DELETEALL;
    node_del.markers.push_back(del);
    pub_->publish(node_del);

    // Plublish paths
    visualization_msgs::msg::MarkerArray node_arr;
    int iter = 0;
    for (int i = 0; i < route_table.size(); i++) {
        for (int j = 0; j < route_table[i].size(); j++) {

            visualization_msgs::msg::Marker p;
            p.header.frame_id = "map";
            if (route_table[i][j]->init_node){
                p.type = visualization_msgs::msg::Marker::CUBE;
            }
            else {
                p.type = visualization_msgs::msg::Marker::SPHERE;
            }

            p.action = visualization_msgs::msg::Marker::ADD;
            p.pose.orientation.w = 1.0;
            p.scale.x = 0.3;    p.scale.y = 0.3;    p.scale.z = 0.3;
            if (route_table[i][j]->state == 0){
                p.color.r = 1.0; p.color.g = 0.0; p.color.b = 0.0;
            }
            else if (route_table[i][j]->state == 1){
                p.color.r = 0.0; p.color.g = 1.0; p.color.b = 0.0;
            }
            else if (route_table[i][j]->state == 2){
                p.color.r = 0.0; p.color.g = 0.0; p.color.b = 1.0;
            }
            else {
                p.color.r = 1.0; p.color.g = 1.0; p.color.b = 1.0;
            }
            p.color.a = 0.5;
            Eigen::Vector2d start = route_table[i][j]->start.head(2);
            Eigen::Vector2d end   = route_table[i][j]->end.head(2);
            for (double a = 0; a <= 1.0; a = a + 0.25){
                Eigen::Vector2d loc = start * a + end * (1 - a);
                p.id = iter;
                p.pose.position.x = loc[0];
                p.pose.position.y = loc[1];
                p.pose.position.z = 1.0 + 1.0 * (double)i;
                node_arr.markers.push_back(p);
                iter++;
            }
        }
    }
    pub_->publish(node_arr);

}

void Util::plotLocalRoutes(std::vector<std::vector<Eigen::Vector3d>> routes, rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_){

    // Clear node buffer
    visualization_msgs::msg::MarkerArray node_del;
    visualization_msgs::msg::Marker del;
    del.id = 0;
    del.header.frame_id = "map";
    del.header.stamp = clock->now();
    del.action = visualization_msgs::msg::Marker::DELETEALL;
    node_del.markers.push_back(del);
    pub_->publish(node_del);

    // Plublish paths
    visualization_msgs::msg::MarkerArray node_arr;
    int iter_node = 0;
    for (int i = 0 ; i < routes.size() ; i++) {
        for (int j = 1 ; j < routes[i].size() ; j++) {
            visualization_msgs::msg::Marker node_link;
            node_link.header.frame_id = "map";
            node_link.header.stamp = clock->now();
            node_link.id = iter_node;
            node_link.action = visualization_msgs::msg::Marker::ADD;
            node_link.pose.orientation.w = 1.0;
            if (color_path){
                node_link.color.r = robot_color[i][0];
                node_link.color.g = robot_color[i][1];
                node_link.color.b = robot_color[i][2];
            }
            else {
                node_link.color.r = 0.0;
                node_link.color.g = 1.0;
                node_link.color.b = 0.0;
            }
            node_link.color.a = 0.8;
            node_link.scale.x = 0.4 * display_scale;
            node_link.scale.y = 0.4 * display_scale;
            node_link.type = visualization_msgs::msg::Marker::LINE_STRIP;

            geometry_msgs::msg::Point start_p, end_p;
            start_p.x = routes[i][j-1][0];
            start_p.y = routes[i][j-1][1];
            start_p.z = 0.3;
            node_link.points.push_back(start_p);
            end_p.x = routes[i][j][0];
            end_p.y = routes[i][j][1];
            end_p.z = 0.3;
            node_link.points.push_back(end_p);
            node_arr.markers.push_back(node_link);
            iter_node++;
        }
    }
    pub_->publish(node_arr);
}

void Util::plotTopologicalMap(const TopologicalMap& topolomap, rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_){

    // Clear node buffer
    visualization_msgs::msg::MarkerArray node_del;
    visualization_msgs::msg::Marker del;
    del.id = 0;
    del.header.frame_id = "map";
    del.header.stamp = clock->now();
    del.action = visualization_msgs::msg::Marker::DELETEALL;
    node_del.markers.push_back(del);
    pub_->publish(node_del);

    visualization_msgs::msg::MarkerArray node_arr;
    int iter = 0;
    rclcpp::Time time_now = clock->now();
    for (int i = 0 ; i < topolomap.num_lanes ; i++) {
        Eigen::Vector2d entry = topolomap.nodes[topolomap.lanes[i].entry].loc;
        Eigen::Vector2d exit  = topolomap.nodes[topolomap.lanes[i].exit].loc;

        visualization_msgs::msg::Marker node_link;
        node_link.header.frame_id = "map";
        node_link.header.stamp = time_now;
        node_link.id = iter;
        node_link.action = visualization_msgs::msg::Marker::ADD;
        node_link.pose.orientation.w = 1.0;
        if (!topolomap.lanes[i].disabled){
            node_link.color.r = 1.0f;   node_link.color.g = 0.5f;   node_link.color.b = 0.0f;
        }
        else {
            node_link.color.r = 0.5f;   node_link.color.g = 0.5f;   node_link.color.b = 0.5f;
        }
        node_link.color.a = 0.3;
        node_link.scale.x = 1.0 * display_scale;
        node_link.scale.y = 1.0 * display_scale;
        node_link.type = visualization_msgs::msg::Marker::LINE_STRIP;

        geometry_msgs::msg::Point start_p, end_p;
        start_p.x = entry[0];
        start_p.y = entry[1];
        node_link.points.push_back(start_p);
        end_p.x = exit[0];
        end_p.y = exit[1];
        node_link.points.push_back(end_p);
        node_arr.markers.push_back(node_link);
        iter++;
    }

    // Plot station nodes
    for (int i = 0; i < topolomap.stations.size(); i++){
        Eigen::Vector2d node = topolomap.stations[i].second;
        visualization_msgs::msg::Marker p;
        p.header.stamp = time_now;
        p.id = iter;
        p.header.frame_id = "map";
        p.type = visualization_msgs::msg::Marker::SPHERE;
        p.action = visualization_msgs::msg::Marker::ADD;
        p.pose.position.x = node[0];
        p.pose.position.y = node[1];
        p.pose.position.z = 0.0;
        p.pose.orientation.x = 0.0;
        p.pose.orientation.y = 0.0;
        p.pose.orientation.z = 0.0;
        p.pose.orientation.w = 1.0;
        p.scale.x = 1.5 * display_scale;    p.scale.y = 1.5 * display_scale;    p.scale.z = 0.2;
        if (topolomap.stations[i].first.find("IO") != std::string::npos){           // Pink
            p.color.r = 1.0;    p.color.g = 0.0;    p.color.b = 1.0;
        }
        else if (topolomap.stations[i].first.find("I") != std::string::npos){       // Red
            p.color.r = 1.0;    p.color.g = 0.0;    p.color.b = 0.0;
        }
        else if (topolomap.stations[i].first.find("O") != std::string::npos){       // Blue
            p.color.r = 0.0;    p.color.g = 0.0;    p.color.b = 1.0;
        }
        else if (topolomap.stations[i].first.find("R") != std::string::npos){       // Gray
            p.color.r = 0.5;    p.color.g = 0.5;    p.color.b = 0.5;
        }
        else if (topolomap.stations[i].first.find("C") != std::string::npos){       // Green
            p.color.r = 0.0;    p.color.g = 1.0;    p.color.b = 0.0;
        }
        else if (topolomap.stations[i].first.find("W") != std::string::npos){       // Orange
            p.color.r = 1.0;    p.color.g = 0.5;    p.color.b = 0.0;
        }
        else {                                                                      // Purple
            p.color.r = 0.7;    p.color.g = 0.0;    p.color.b = 0.7;
        }
        p.color.a = 0.4;
        node_arr.markers.push_back(p);
        iter++;
    }

    // Plot station name
    for (size_t i = 0; i < topolomap.stations.size(); i++){
        std::string name     = topolomap.stations[i].first;
        Eigen::Vector2d node = topolomap.stations[i].second;
        visualization_msgs::msg::Marker p;
        p.header.frame_id = "map";
        p.header.stamp = time_now;
        p.id = iter;
        p.text = name;
        p.color.a = 1.0;
        p.scale.z = 0.8 * display_scale;
        p.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        p.action = visualization_msgs::msg::Marker::ADD;
        p.pose.orientation.w = 1.0;
        p.pose.position.x = node[0];
        p.pose.position.y = node[1];
        p.pose.position.z = 0.2;
        node_arr.markers.push_back(p);
        iter++;
    }

    pub_->publish(node_arr);
}


void Util::plotTasks(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_){

    // Clear node buffer
    rclcpp::Time time_now = clock->now();
    visualization_msgs::msg::MarkerArray node_del;
    visualization_msgs::msg::Marker del;
    del.id = 0;
    del.header.frame_id = "map";
    del.header.stamp = clock->now();
    del.action = visualization_msgs::msg::Marker::DELETEALL;
    node_del.markers.push_back(del);
    pub_->publish(node_del);


    visualization_msgs::msg::MarkerArray node_arr;
    int iter = 0;
    for (int i = 0 ; i < task_states->table.size() ; i++){

        if (task_states->table[i].state != fms_msgs::msg::TaskState::STATE_ACTIVE){
            continue;
        }

        visualization_msgs::msg::Marker p;
        p.header.stamp = time_now;
        p.id = iter;
        p.header.frame_id = "map";
        p.type = visualization_msgs::msg::Marker::SPHERE;
        p.action = visualization_msgs::msg::Marker::ADD;
        p.pose.position.x = task_states->table[i].curr_task_loc[0];
        p.pose.position.y = task_states->table[i].curr_task_loc[1];
        p.pose.position.z = 0.5;
        p.pose.orientation.x = 0.0;
        p.pose.orientation.y = 0.0;
        p.pose.orientation.z = 0.0;
        p.pose.orientation.w = 1.0;
        p.scale.x = 1.0*display_scale; p.scale.y = 1.0*display_scale; p.scale.z = 0.3;
        p.color.r = 1.0;    p.color.g = 0.0;    p.color.b = 0.0;
        p.color.a = 0.7;
        node_arr.markers.push_back(p);
        iter++;
    }
    pub_->publish(node_arr);
}

void Util::plotPoses(std::vector<geometry_msgs::msg::Pose2D> poses_, std::vector<std::string> robot_ids, rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_){

    // Plublish robot poses
    int iter = 0;
    rclcpp::Time time_now = clock->now();
    visualization_msgs::msg::MarkerArray node_arr;
    for (int i = 0 ; i < poses_.size() ; i++){
        visualization_msgs::msg::Marker p;
        p.header.stamp = time_now;
        // p.header.seq = iter;
        p.id = iter;
        p.header.frame_id = "map";
        p.type = visualization_msgs::msg::Marker::SPHERE;
        p.action = visualization_msgs::msg::Marker::ADD;
        p.pose.position.x = poses_[i].x;
        p.pose.position.y = poses_[i].y;
        p.pose.position.z = 0.5;
        p.pose.orientation.w = 1.0;

        if (robot_states->table[i].mode == fms_msgs::msg::RobotState::MODE_TASK_PROCESSING ||
            robot_states->table[i].mode == fms_msgs::msg::RobotState::MODE_MOVING_TASK){
            p.color.r = 0.0; p.color.g = 1.0; p.color.b = 0.0; p.color.a = 0.5;
        }
        else if (robot_states->table[i].mode == fms_msgs::msg::RobotState::MODE_MOVING ||
                 robot_states->table[i].mode == fms_msgs::msg::RobotState::MODE_MOVING_WAITING ||
                 robot_states->table[i].mode == fms_msgs::msg::RobotState::MODE_WAITING){
            p.color.r = 0.0; p.color.g = 0.0; p.color.b = 1.0; p.color.a = 0.5;
        }
        else if (robot_states->table[i].mode == fms_msgs::msg::RobotState::MODE_CHARGING ||
                 robot_states->table[i].mode == fms_msgs::msg::RobotState::MODE_MOVING_CHARGING){
            p.color.r = 1.0; p.color.g = 1.0; p.color.b = 0.0; p.color.a = 0.5;
        }
        else if (robot_states->table[i].mode == fms_msgs::msg::RobotState::MODE_PAUSED ||
                 robot_states->table[i].mode == fms_msgs::msg::RobotState::MODE_ERROR){
            p.color.r = 1.0; p.color.g = 0.0; p.color.b = 0.0; p.color.a = 0.5;
        }
        else {
            p.color.r = 0.0; p.color.g = 1.0; p.color.b = 1.0; p.color.a = 0.5;
        }
        p.scale.x = 1.8 * display_scale;    p.scale.y = 1.8 * display_scale;    p.scale.z = 0.2;
        node_arr.markers.push_back(p);
        iter++;

        // p.header.seq = iter;
        p.id = iter;
        p.scale.x = 0.8 * display_scale;    p.scale.y = 0.8 * display_scale;    p.scale.z = 0.6;
        p.color.r = 1.0;    p.color.g = 0.5;    p.color.b = 0.0;
        p.color.a = 1.0;
        node_arr.markers.push_back(p);
        iter++;

        visualization_msgs::msg::Marker p2;
        p2.header.stamp = time_now;
        // p2.header.seq = iter;
        p2.id = iter;
        p2.header.frame_id = "map";
        p2.type = visualization_msgs::msg::Marker::ARROW;
        p2.action = visualization_msgs::msg::Marker::ADD;
        p2.pose.position.x = poses_[i].x;
        p2.pose.position.y = poses_[i].y;
        p2.pose.position.z = 0.5;
        tf2::Quaternion quat;
        quat.setEuler(0.0, 0.0, poses_[i].theta);
        p2.pose.orientation.x = quat.x();
        p2.pose.orientation.y = quat.y();
        p2.pose.orientation.z = quat.z();
        p2.pose.orientation.w = quat.w();
        p2.scale.x = 1.2 * display_scale;   p2.scale.y = 0.3 * display_scale;   p2.scale.z = 0.3 * display_scale;
        p2.color.r = 1.0;   p2.color.g = 0.5;   p2.color.b = 0.0;
        p2.color.a = 1.0;
        node_arr.markers.push_back(p2);
        iter++;

        visualization_msgs::msg::Marker p3;
        p3.header.frame_id = "map";
        p3.header.stamp = time_now;
        p3.id = iter;
        p3.text = robot_ids[i];
        p3.color.a = 1.0;
        p3.scale.z = 0.8 * display_scale;
        p3.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        p3.action = visualization_msgs::msg::Marker::ADD;
        p3.pose.orientation.w = 1.0;
        p3.pose.position.x = poses_[i].x;
        p3.pose.position.y = poses_[i].y;
        p3.pose.position.z = 1.0;
        node_arr.markers.push_back(p3);
        iter++;

    }
    pub_->publish(node_arr);
}


float Util::getYaw ( const geometry_msgs::msg::Quaternion &_rot ) {
    double roll, pitch, yaw;
    tf2::Quaternion q ( _rot.x, _rot.y, _rot.z, _rot.w );
    tf2::Matrix3x3 m(q);
    m.getRPY ( roll, pitch, yaw );
    return yaw;
}

void Util::setColor(int num_of_robots){

    for (int i = 0 ; i < num_of_robots ; i++){
        double r_ = (double)rand() / (double)RAND_MAX;
        double g_ = (double)rand() / (double)RAND_MAX;
        double b_ = (double)rand() / (double)RAND_MAX;
        Eigen::Vector3d color(r_, g_, b_);
        robot_color.push_back(color);
    }

}

int getMinIndex(std::vector<double> vec){
    int min_index = 0;
    for (int i = 0; i < vec.size(); i++){
        if (vec[i] < vec[min_index]){
            min_index = i;
        }
    }
    return min_index;
}

int getMaxIndex(std::vector<double> vec){
    int max_index = 0;
    for (int i = 0; i < vec.size(); i++){
        if (vec[i] > vec[max_index]){
            max_index = i;
        }
    }
    return max_index;
}

void Util::plotNodes(const std::vector<Eigen::Vector2d>& nodes, const std::vector<int>& node_ids, rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_) {
    // Clear node buffer
    visualization_msgs::msg::MarkerArray node_del;
    visualization_msgs::msg::Marker del;
    del.id = 0;
    del.header.frame_id = "map";
    del.header.stamp = clock->now();
    del.action = visualization_msgs::msg::Marker::DELETEALL;
    node_del.markers.push_back(del);
    pub_->publish(node_del);

    // Publish nodes
    visualization_msgs::msg::MarkerArray marker_array;
    rclcpp::Time time_now = clock->now();
    for (size_t i = 0; i < nodes.size(); ++i) {
        // Text marker
        visualization_msgs::msg::Marker text_marker;
        text_marker.header.frame_id = "map";
        text_marker.header.stamp = time_now;
        text_marker.id = node_ids[i];
        text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::msg::Marker::ADD;
        text_marker.pose.position.x = nodes[i][0];
        text_marker.pose.position.y = nodes[i][1];
        text_marker.pose.position.z = 0.5; // Adjust height if necessary
        text_marker.scale.z = 0.4 * display_scale; // Text size
        text_marker.color.a = 1.0; // Alpha
        text_marker.color.r = 1.0;
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;
        text_marker.text = std::to_string(node_ids[i]);
        marker_array.markers.push_back(text_marker);

        // Sphere marker
        visualization_msgs::msg::Marker sphere_marker;
        sphere_marker.header.frame_id = "map";
        sphere_marker.header.stamp = time_now;
        sphere_marker.id = node_ids[i] + nodes.size(); // Ensure unique ID
        sphere_marker.type = visualization_msgs::msg::Marker::SPHERE;
        sphere_marker.action = visualization_msgs::msg::Marker::ADD;
        sphere_marker.pose.position.x = nodes[i][0];
        sphere_marker.pose.position.y = nodes[i][1];
        sphere_marker.pose.position.z = 0.0; // Ground level
        sphere_marker.scale.x = 0.1; // Sphere size
        sphere_marker.scale.y = 0.1;
        sphere_marker.scale.z = 0.1;
        sphere_marker.color.a = 1.0; // Alpha
        sphere_marker.color.r = 0.0;
        sphere_marker.color.g = 0.0;
        sphere_marker.color.b = 1.0; // Blue color
        marker_array.markers.push_back(sphere_marker);
    }
    pub_->publish(marker_array);
}

}
