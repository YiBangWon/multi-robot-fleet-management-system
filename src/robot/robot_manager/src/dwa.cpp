// Dynamic-window local planner used by the robot manager.

#include "robot_manager/dwa.h"

namespace robot_manager
{
DWA::DWA()
{
  cfg_= Config();
  cfg_.ob.resize(2);
  loadObstacles("/home/air/obstacles.csv"); // Load obstacles from a CSV file
}

std::pair<std::array<double, 2>, DWA::Trajectory> DWA::plan(const State& x, const std::array<double, 2>& goal)
{
  // setObstacle();

  // Calculate dynamic window
  auto dw = calcDynamicWindow(x);

  auto result = calcControlAndTrajectory(x, dw, goal);

  std::cout << "best_u: " << result.first[0] << ", " << result.first[1] << std::endl;

  // Calculate control and trajectory
  return result;
}

DWA::State DWA::motion(const State& x, const std::array<double, 2>& u)
{
  State x_new = x;
  x_new[2] += u[1] * cfg_.dt;
  x_new[0] += u[0] * std::cos(x_new[2]) * cfg_.dt;
  x_new[1] += u[0] * std::sin(x_new[2]) * cfg_.dt;
  x_new[3] = u[0];
  x_new[4] = u[1];
  return x_new;
}

std::array<double, 4> DWA::calcDynamicWindow(const State& x)
{
  std::array<double, 4> Vs = { cfg_.min_speed, cfg_.max_speed, -cfg_.max_yaw_rate, cfg_.max_yaw_rate };
  std::array<double, 4> Vd = { x[3] - cfg_.max_accel * cfg_.dt, x[3] + cfg_.max_accel * cfg_.dt,
                               x[4] - cfg_.max_delta_yaw_rate * cfg_.dt, x[4] + cfg_.max_delta_yaw_rate * cfg_.dt };
  return { std::max(Vs[0], Vd[0]), std::min(Vs[1], Vd[1]), std::max(Vs[2], Vd[2]), std::min(Vs[3], Vd[3]) };
}

DWA::Trajectory DWA::predictTrajectory(const State& x_init, double v, double y)
{
  Trajectory traj;
  State x = x_init;
  traj.push_back(x);
  double time = 0.0;
  while (time <= cfg_.predict_time)
  {
    x = motion(x, { v, y });
    traj.push_back(x);
    time += cfg_.dt;
  }
  return traj;
}

double DWA::calcObstacleCost(const Trajectory& traj)
{
  double min_r = std::numeric_limits<double>::infinity();
  for (const auto& st : traj)
  {
    double px = st[0], py = st[1], yaw = st[2];
    for(auto& obs: cfg_.ob) {
      for (auto& o : obs)
      {
        double ox = o[0], oy = o[1];
        if (cfg_.robot_type == RobotType::Rectangle)
        {
          double dx = ox - px;
          double dy = oy - py;
          double cos_y = std::cos(yaw), sin_y = std::sin(yaw);
          double local_x = dx * cos_y + dy * sin_y;
          double local_y = -dx * sin_y + dy * cos_y;
          if (std::abs(local_x) <= cfg_.robot_length / 2.0 && std::abs(local_y) <= cfg_.robot_width / 2.0)
          {
            return std::numeric_limits<double>::infinity();
          }
        }
        else
        {
          double r = std::hypot(px - ox, py - oy);
          if (r <= cfg_.robot_radius)
          {
            return std::numeric_limits<double>::infinity();
          }
          min_r = std::min(min_r, r);
        }
      }
    }
  }
  return 1.0 / min_r;
}

double DWA::calcToGoalCost(const Trajectory& traj, const std::array<double, 2>& goal)
{
  const auto& last = traj.back();
  double dx = goal[0] - last[0];
  double dy = goal[1] - last[1];
  double error_angle = std::atan2(dy, dx);
  double cost_angle = error_angle - last[2];
  return std::abs(std::atan2(std::sin(cost_angle), std::cos(cost_angle)));
}

std::pair<std::array<double, 2>, DWA::Trajectory> DWA::calcControlAndTrajectory(const State& x,
                                                                                const std::array<double, 4>& dw,
                                                                                const std::array<double, 2>& goal)
{
  double min_cost = std::numeric_limits<double>::infinity();
  std::array<double, 2> best_u = { 0, 0 };
  Trajectory best_traj = { x };
  for (double v = dw[0]; v <= dw[1]; v += cfg_.v_resolution)
  {
    for (double y = dw[2]; y <= dw[3]; y += cfg_.yaw_rate_resolution)
    {
      auto traj = predictTrajectory(x, v, y);
      double to_goal = cfg_.to_goal_cost_gain * calcToGoalCost(traj, goal);
      double speed = cfg_.speed_cost_gain * (cfg_.max_speed - traj.back()[3]);
      double obc = cfg_.obstacle_cost_gain * calcObstacleCost(traj);
      double cost = to_goal + speed + obc;
      if (cost < min_cost)
      {
        min_cost = cost;
        best_u = { v, y };
        best_traj = traj;
      }
    }
  }
  return { best_u, best_traj };
}

void DWA::loadObstacles(const std::string& file_path)
{
  std::ifstream file(file_path);
    if (!file.is_open())
        throw std::runtime_error("Cannot open file: " + file_path);

    std::string line;

    // --------- (1)  :    -----------
    if (std::getline(file, line))
    {
        bool has_header = false;
        for (char c : line)
            if (std::isalpha(static_cast<unsigned char>(c))) { has_header = true; break; }

        //     line 
        if (!has_header)
        {
            std::istringstream ss(line);
            std::string token;
            std::array<double, 2> pt{};
            for (int i = 0; i < 2 && std::getline(ss, token, ','); ++i)
                pt[i] = std::stod(token);
            cfg_.ob[0].push_back(pt);
        }
    }

    // --------- (2)     -----------
    while (std::getline(file, line))
    {
        if (line.empty()) continue;        //   
        std::istringstream ss(line);
        std::string token;
        std::array<double, 2> pt{};
        for (int i = 0; i < 2 && std::getline(ss, token, ','); ++i)
            pt[i] = std::stod(token);
        cfg_.ob[0].push_back(pt);
    }
}

void DWA::setObstacle()
{
  cfg_.ob[1].clear();

  for (const auto& p : gt_pose)
  {
      cfg_.ob[1].push_back({p.x(), p.y()});
  }
  gt_pose.clear();
}

}
