// Shared local-control helpers used by the robot manager nodes.

#include "robot_manager/controller.h"

#define MAX_DIST 9999

namespace robot_manager
{
Controller::Controller()
{
  dwa = new DWA();
  is_pure_rotation = true;
  goal_range = 0.3;
  path_counter = -1;
  max_v = 1.0;
  max_w = 0.5;
}

// Update the local route
void Controller::setPath(const std::vector<Eigen::Vector3d>& path)
{
  path_counter = 0;  // Set the path counter to zero
  local_path.clear();
  local_path = samplePath(pose_curr, path);
  // local_path = path;
}

// Get velocity command for path following
geometry_msgs::msg::Twist Controller::getCmdVel(Eigen::Vector3d pose, float delta_t, Eigen::Vector2d vw)
{
  // if (robot_name == "robot_1"){
  //     std::cout << robot_name << ": getCmdVel" << std::endl;
  // }

  pose_curr = pose;  // Update robot pose

  geometry_msgs::msg::Twist cmd_vel;
  if (local_path.size() < 2 || checkGoalArrival(pose))
  {  // If empty path or arrived at goal,
    // if (robot_name == "robot_1")
    //     std::cout << robot_name << ": Goal Arrived" << std::endl;
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.angular.z = 0;  // return zero velos
    return cmd_vel;
  }
  updatePathCounter(pose);  // Update path counter (Index of current waypoint in local path)
  // cmd_vel = getCmdVel_Pure(pose);     // Compute cmd_vel based on PID controller

  // Compute command velocity with the DWA controller.
  // setGtPose();  // Set ground truth pose for DWA controller
  cmd_vel = getCmdVel_DWA(pose);

  // Restrict to maximum velocities
  if (cmd_vel.linear.x > max_v)
    cmd_vel.linear.x = max_v;
  if (cmd_vel.linear.y > max_v)
    cmd_vel.linear.y = max_v;
  if (abs(cmd_vel.angular.z) > max_w)
  {
    if (cmd_vel.angular.z > 0)
      cmd_vel.angular.z = max_w;
    else
      cmd_vel.angular.z = -max_w;
  }

  return cmd_vel;
}

// Check goal arrival
bool Controller::checkGoalArrival(Eigen::Vector3d pose)
{
  double dist = (pose.head(2) - local_path.back().head(2)).norm();
  if (dist < goal_range)
    return true;
  else
    return false;
}

// Update path counter (Index of current waypoint in local path)
void Controller::updatePathCounter(Eigen::Vector3d pose)
{
  double min_dist = MAX_DIST;
  int curr_index = 0;
  // Starting from previous path_counter, find the waypoint closet to current pose
  const int start_index = std::max(path_counter, 0);
  for (int i = start_index; i < static_cast<int>(local_path.size()) - 1; i++)
  {
    Eigen::Vector3d loc_preb = local_path[i];      // Previous waypoint
    Eigen::Vector3d loc_next = local_path[i + 1];  // Next waypoint
    double alpha = 0.0;  // 0.2;                             // Weight for distance of yaw angle
    double dist_preb = (pose.head(2) - loc_preb.head(2)).norm() + alpha * abs(pose[2] - loc_preb[2]);
    double dist_next = (pose.head(2) - loc_next.head(2)).norm() + alpha * abs(pose[2] - loc_next[2]);
    double dist = dist_preb + dist_next;  // Final distance to prev and next waypoints
    if (dist < min_dist)
    {
      min_dist = dist;
      curr_index = i;
    }
  }
  path_counter = curr_index;  // Update path counter
}

// Get cmd_vel of using pure controller
geometry_msgs::msg::Twist Controller::getCmdVel_Pure(Eigen::Vector3d pose)
{
  int look_ahead_index = 2;
  look_ahead_index = std::max(1, std::min(path_counter + look_ahead_index, (int)(local_path.size() - 1)));

  geometry_msgs::msg::Twist cmd_vel;
  double dt = 0.5;
  Eigen::Vector3d pose_target = local_path[look_ahead_index];
  Eigen::Vector2d vec = pose_target.head(2) - pose.head(2);
  double yaw = std::atan2(vec[1], vec[0]);
  pose_target[2] = yaw;

  extractVelocity(pose, pose_target, dt, cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);

  if (is_pure_rotation)
  {
    // double yaw_dir = pose[2] - pose_target[2];
    double yaw_dir = pose_target[2] - pose[2];
    if (yaw_dir > M_PI)
      yaw_dir -= 2.0 * M_PI;
    if (yaw_dir < -M_PI)
      yaw_dir += 2.0 * M_PI;
    if (abs(yaw_dir) > M_PI / 4)
    {
      cmd_vel.linear.x = 0;
      cmd_vel.linear.y = 0;

      // if (cmd_vel.angular.z > 0) cmd_vel.angular.z = max_w;
      // else                       cmd_vel.angular.z = -max_w;
      if (yaw_dir > 0)
        cmd_vel.angular.z = max_w;
      else
        cmd_vel.angular.z = -max_w;
    }
  }

  return cmd_vel;
}

// Get cmd_vel of using dwa controller
geometry_msgs::msg::Twist Controller::getCmdVel_DWA(Eigen::Vector3d pose)
{
  int look_ahead_index = 2;
  look_ahead_index = std::max(1, std::min(path_counter + look_ahead_index, (int)(local_path.size() - 1)));

  geometry_msgs::msg::Twist cmd_vel;
  Eigen::Vector3d goal = local_path[look_ahead_index];

  DWA::State x = { pose[0], pose[1], pose[2], 0.0, 0.0 };  // x, y, theta, v, omega
  auto dwa_plan = dwa->plan(x, {goal[0], goal[1]});

  cmd_vel.linear.x = dwa_plan.first[0];  // v
  cmd_vel.linear.y = 0;  // vy
  cmd_vel.angular.z = dwa_plan.first[1]; // omega

  return cmd_vel;
}

int Controller::sign(double x)
{
  if (x > 0)
    return 1;
  else if (x < 0)
    return -1;
  else
    return 0;
}

double Controller::normalize_theta(double theta)
{
  if (theta >= -M_PI && theta < M_PI)
    return theta;

  double multiplier = std::floor(theta / (2 * M_PI));
  theta = theta - multiplier * 2 * M_PI;
  if (theta >= M_PI)
    theta -= 2 * M_PI;
  if (theta < -M_PI)
    theta += 2 * M_PI;

  return theta;
}

void Controller::extractVelocity(Eigen::Vector3d& pose1, Eigen::Vector3d& pose2, double dt, double& vx, double& vy,
                                 double& omega)
{
  if (fabs(dt) <= FLT_EPSILON)
  {
    vx = 0;
    vy = 0;
    omega = 0;
    return;
  }

  Eigen::Vector2d deltaS = pose2.head(2) - pose1.head(2);

  bool holonomic_robot = true;
  if (holonomic_robot)
  {
    double cos_theta1 = std::cos(pose1[2]);
    double sin_theta1 = std::sin(pose1[2]);
    double p1_dx = cos_theta1 * deltaS.x() + sin_theta1 * deltaS.y();
    double p1_dy = -sin_theta1 * deltaS.x() + cos_theta1 * deltaS.y();
    vx = p1_dx / dt;
    vy = p1_dy / dt;
  }
  else
  {
    Eigen::Vector2d conf1dir(cos(pose1[2]), sin(pose1[2]));
    // translational velocity
    double dir = deltaS.dot(conf1dir);
    vx = (double)sign(dir) * deltaS.norm() / dt;
    vy = 0;
  }

  // rotational velocity
  double orientdiff = normalize_theta(pose2[2] - pose1[2]);
  omega = orientdiff / dt;
}

std::vector<Eigen::Vector3d> Controller::samplePath(Eigen::Vector3d start, const std::vector<Eigen::Vector3d>& path)
{
  if (path.empty())
    return std::vector<Eigen::Vector3d>();

  if (path.size() > 1)
    start = path.front();

  int num_add_sample = 3;
  std::vector<Eigen::Vector3d> ret;
  ret.push_back(start);
  Eigen::Vector2d prev = start.head(2);
  for (int i = 0; i < path.size(); i++)
  {
    Eigen::Vector2d curr = path[i].head(2);
    Eigen::Vector2d vec = curr - prev;

    if (vec.norm() < 0.01)
      continue;

    double yaw = std::atan2(vec[1], vec[0]);
    double disc = 1.0 / (double)num_add_sample;
    for (double it = disc; it <= 1.0; it += disc)
    {
      Eigen::Vector2d pt((1.0 - it) * prev[0] + it * curr[0], (1.0 - it) * prev[1] + it * curr[1]);

      if ((ret.back().head(2) - pt).norm() < 0.05)
        continue;

      ret.push_back(Eigen::Vector3d(pt[0], pt[1], yaw));
    }
    prev = curr;
  }
  return ret;
}

void Controller::setGtPose()
{
  dwa->gt_pose = gt_pose;
  gt_pose.clear();
}

}
