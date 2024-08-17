// Copyright 2023 amsl

#include <algorithm>
#include <string>
#include <vector>

#include "point_follow_planner/point_follow_planner.h"

PointFollowPlanner::PointFollowPlanner(void)
    : private_nh_("~"), goal_subscribed_(false), footprint_subscribed_(false), odom_updated_(false),
      local_map_updated_(false), is_behind_obj_(false), has_reached_(false), turn_at_goal_flag_(true),
      local_map_not_sub_count_(0), odom_not_sub_count_(0)
{
  private_nh_.param<double>("hz", hz_, {20});
  private_nh_.param<std::string>("robot_frame", robot_frame_, {"base_link"});
  private_nh_.param<double>("target_velocity", target_velocity_, {0.55});
  private_nh_.param<double>("velocity_th_for_stop_behind_obj", velocity_th_for_stop_behind_obj_, {0.15});
  private_nh_.param<double>("max_velocity", max_velocity_, {0.55});
  private_nh_.param<double>("min_velocity", min_velocity_, {0.0});
  private_nh_.param<double>("max_yawrate", max_yawrate_, {1.0});
  private_nh_.param<double>("min_yawrate", min_yawrate_, {0.05});
  private_nh_.param<double>("min_in_place_yawrate", min_in_place_yawrate_, {0.3});
  private_nh_.param<double>("max_acceleration", max_acceleration_, {0.5});
  private_nh_.param<double>("max_deceleration", max_deceleration_, {1.0});
  private_nh_.param<double>("max_d_yawrate", max_d_yawrate_, {3.2});
  private_nh_.param<double>("angle_resolution", angle_resolution_, {0.2});
  private_nh_.param<double>("predict_time", predict_time_, {3.0});
  private_nh_.param<double>("dt", dt_, {0.1});
  private_nh_.param<double>("sleep_time_after_finish", sleep_time_after_finish_, {0.5});
  private_nh_.param<double>("angle_to_goal_th", angle_to_goal_th_, {0.26});
  private_nh_.param<double>("dist_to_goal_th", dist_to_goal_th_, {0.3});
  private_nh_.param<double>("turn_direction_th", turn_direction_th_, {0.1});
  private_nh_.param<double>("slow_velocity_th", slow_velocity_th_, {0.1});
  private_nh_.param<double>("dist_from_head_to_obj", dist_from_head_to_obj_, {0.5});
  private_nh_.param<int>("velocity_samples", velocity_samples_, {3});
  private_nh_.param<int>("yawrate_samples", yawrate_samples_, {20});
  private_nh_.param<int>("subscribe_count_th", subscribe_count_th_, {3});
  private_nh_.param<float>("recovery/stuck_time_th", recovery_params_.stuck_time_th, {5.0});
  private_nh_.param<float>("recovery/time", recovery_params_.time, {1.0});
  private_nh_.param<float>("recovery/goal_dist", recovery_params_.goal_dist, {5.0});
  private_nh_.param<float>("recovery/goal_angle", recovery_params_.goal_angle, {0.1});
  private_nh_.param<double>("recovery/max_velocity", recovery_params_.max_velocity, {0.5});
  private_nh_.param<std::string>("recovery/sound_file", recovery_params_.sound_file, {""});
  // set recovery params
  recovery_params_.stuck_count_th = static_cast<int>(recovery_params_.stuck_time_th * hz_);
  recovery_params_.max_recovery_count = static_cast<int>(recovery_params_.time * hz_);

  ROS_INFO("=== Point Followe Planner ===");
  ROS_INFO_STREAM("hz: " << hz_);
  ROS_INFO_STREAM("robot_frame: " << robot_frame_);
  ROS_INFO_STREAM("target_velocity: " << target_velocity_);
  ROS_INFO_STREAM("velocity_th_for_stop_behind_obj: " << velocity_th_for_stop_behind_obj_);
  ROS_INFO_STREAM("max_velocity: " << max_velocity_);
  ROS_INFO_STREAM("min_velocity: " << min_velocity_);
  ROS_INFO_STREAM("max_yawrate: " << max_yawrate_);
  ROS_INFO_STREAM("min_yawrate: " << min_yawrate_);
  ROS_INFO_STREAM("min_in_place_yawrate: " << min_in_place_yawrate_);
  ROS_INFO_STREAM("max_acceleration: " << max_acceleration_);
  ROS_INFO_STREAM("max_deceleration: " << max_deceleration_);
  ROS_INFO_STREAM("max_d_yawrate: " << max_d_yawrate_);
  ROS_INFO_STREAM("angle_resolution: " << angle_resolution_);
  ROS_INFO_STREAM("predict_time: " << predict_time_);
  ROS_INFO_STREAM("dt: " << dt_);
  ROS_INFO_STREAM("sleep_time_after_finish: " << sleep_time_after_finish_);
  ROS_INFO_STREAM("angle_to_goal_th: " << angle_to_goal_th_);
  ROS_INFO_STREAM("dist_to_goal_th: " << dist_to_goal_th_);
  ROS_INFO_STREAM("turn_direction_th: " << turn_direction_th_);
  ROS_INFO_STREAM("slow_velocity_th: " << slow_velocity_th_);
  ROS_INFO_STREAM("dist_to_obj_th_x: " << dist_to_obj_th_x_);
  ROS_INFO_STREAM("velocity_samples: " << velocity_samples_);
  ROS_INFO_STREAM("yawrate_samples: " << yawrate_samples_);
  ROS_INFO_STREAM("subscribe_count_th: " << subscribe_count_th_);
  ROS_INFO_STREAM("recovery/stuck_time_th: " << recovery_params_.stuck_time_th);
  ROS_INFO_STREAM("recovery/time: " << recovery_params_.time);
  ROS_INFO_STREAM("recovery/goal_dist: " << recovery_params_.goal_dist);
  ROS_INFO_STREAM("recovery/goal_angle: " << recovery_params_.goal_angle);
  ROS_INFO_STREAM("recovery/max_velocity: " << recovery_params_.max_velocity);
  ROS_INFO_STREAM("recovery/sound_file: " << recovery_params_.sound_file);

  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  best_trajectory_pub_ = private_nh_.advertise<visualization_msgs::Marker>("best_trajectory", 1);
  candidate_trajectories_pub_ = private_nh_.advertise<visualization_msgs::MarkerArray>("candidate_trajectories", 1);
  predict_footprints_pub_ = private_nh_.advertise<visualization_msgs::MarkerArray>("predict_footprints", 1);
  finish_flag_pub_ = private_nh_.advertise<std_msgs::Bool>("finish_flag", 1);

  footprint_sub_ = nh_.subscribe("/footprint", 1, &PointFollowPlanner::footprint_callback, this);
  goal_sub_ = nh_.subscribe("/local_goal", 1, &PointFollowPlanner::goal_callback, this);
  local_map_sub_ = nh_.subscribe("/local_map", 1, &PointFollowPlanner::local_map_callback, this);
  odom_sub_ = nh_.subscribe("/odom", 1, &PointFollowPlanner::odom_callback, this);
  target_velocity_sub_ = nh_.subscribe("/target_velocity", 1, &PointFollowPlanner::target_velocity_callback, this);
  dist_to_goal_th_sub_ = nh_.subscribe("/dist_to_goal_th", 1, &PointFollowPlanner::dist_to_goal_th_callback, this);

  turn_at_goal_flag_server_ =
      private_nh_.advertiseService("goal/turn", &PointFollowPlanner::turn_at_goal_flag_callback, this);
  recovery_mode_flag_server_ =
      private_nh_.advertiseService("recovery/available", &PointFollowPlanner::recovery_mode_flag_callback, this);
}

PointFollowPlanner::State::State(void) : x_(0.0), y_(0.0), yaw_(0.0), velocity_(0.0), yawrate_(0.0) {}

PointFollowPlanner::Window::Window(const double min_v, const double max_v, const double min_y, const double max_y)
    : min_velocity_(min_v), max_velocity_(max_v), min_yawrate_(min_y), max_yawrate_(max_y)
{
}

void PointFollowPlanner::goal_callback(const geometry_msgs::PoseStampedConstPtr &msg)
{
  goal_ = *msg;
  try
  {
    listener_.transformPose(robot_frame_, ros::Time(0), goal_, goal_.header.frame_id, goal_);
    goal_subscribed_ = true;
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
  }

  if (0 < recovery_params_.recovery_count)
  {
    const float goal_dist = target_velocity_ >= 0.0 ? recovery_params_.goal_dist : -recovery_params_.goal_dist;
    const float goal_angle = fabs(recovery_params_.goal_angle) < angle_to_goal_th_ ? recovery_params_.goal_angle
                                                                                   : angle_to_goal_th_ - DBL_EPSILON;
    goal_.pose.position.x = goal_dist * cos(goal_angle);
    goal_.pose.position.y = goal_dist * sin(goal_angle);
  }
}

void PointFollowPlanner::footprint_callback(const geometry_msgs::PolygonStampedPtr &msg)
{
  set_dist_to_obj_th(*msg);
  footprint_ = *msg;
  footprint_subscribed_ = true;
}

void PointFollowPlanner::local_map_callback(const nav_msgs::OccupancyGridConstPtr &msg)
{
  raycast(*msg);
  local_map_not_sub_count_ = 0;
  local_map_updated_ = true;
}

void PointFollowPlanner::odom_callback(const nav_msgs::OdometryConstPtr &msg)
{
  previous_velocity_ = current_velocity_;
  current_velocity_ = msg->twist.twist;
  odom_not_sub_count_ = 0;
  odom_updated_ = true;
}

void PointFollowPlanner::target_velocity_callback(const geometry_msgs::TwistConstPtr &msg)
{
  geometry_msgs::Twist target_velocity_msg = *msg;
  if (0 < recovery_params_.recovery_count)
  {
    target_velocity_msg.linear.x *= -1.0;
    target_velocity_msg.linear.x = target_velocity_msg.linear.x > 0
                                       ? std::min(target_velocity_msg.linear.x, recovery_params_.max_velocity)
                                       : std::max(target_velocity_msg.linear.x, -recovery_params_.max_velocity);
  }

  if (target_velocity_msg.linear.x >= 0.0)
  {
    if (min_velocity_ < 0.0)
    {
      const double tmp = min_velocity_;
      min_velocity_ = -max_velocity_;
      max_velocity_ = -tmp;
    }
    target_velocity_ = std::min(target_velocity_msg.linear.x, max_velocity_);
  }
  else
  {
    if (max_velocity_ > 0.0)
    {
      const double tmp = min_velocity_;
      min_velocity_ = -max_velocity_;
      max_velocity_ = -tmp;
    }
    target_velocity_ = std::max(target_velocity_msg.linear.x, min_velocity_);
  }
  ROS_INFO_THROTTLE(1.0, "target velocity was updated to %f [m/s]", target_velocity_);
}

void PointFollowPlanner::dist_to_goal_th_callback(const std_msgs::Float64ConstPtr &msg)
{
  dist_to_goal_th_ = msg->data;
  ROS_INFO_THROTTLE(1.0, "distance to goal threshold was updated to %f [m]", dist_to_goal_th_);
}

bool PointFollowPlanner::turn_at_goal_flag_callback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
  turn_at_goal_flag_ = req.data;
  res.success = true;
  if (turn_at_goal_flag_)
    res.message = "Enable turning at the goal";
  else
    res.message = "Disable turning at the goal";
  return true;
}

bool PointFollowPlanner::recovery_mode_flag_callback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
  recovery_params_.available = req.data;
  res.success = true;
  if (recovery_params_.available)
  {
    res.message = "Recovery mode is available..";
  }
  else
  {
    res.message = "Recovery mode is unavailable..";
    recovery_params_.recovery_count = 0;
    recovery_params_.stuck_count = 0;
  }
  return true;
}

void PointFollowPlanner::set_dist_to_obj_th(const geometry_msgs::PolygonStamped &footprint)
{
  float max_x = 0.0;
  float max_y = 0.0;
  for (const auto &point : footprint.polygon.points)
  {
    max_x = target_velocity_ >= 0.0 ? std::max(max_x, point.x) : std::min(max_x, point.x);
    max_y = std::max(max_y, point.y);
  }
  dist_to_obj_th_x_ = target_velocity_ >= 0.0 ? max_x + dist_from_head_to_obj_ : max_x - dist_from_head_to_obj_;
  dist_to_obj_th_y_ = max_y;
}

void PointFollowPlanner::raycast(const nav_msgs::OccupancyGrid &map)
{
  obs_list_.poses.clear();
  const double max_search_dist = hypot(map.info.origin.position.x, map.info.origin.position.y);

  for (float angle = -M_PI; angle <= M_PI; angle += angle_resolution_)
  {
    for (float dist = 0.0; dist <= max_search_dist; dist += map.info.resolution)
    {
      geometry_msgs::Pose pose;
      pose.position.x = dist * cos(angle);
      pose.position.y = dist * sin(angle);
      const int index_x = floor((pose.position.x - map.info.origin.position.x) / map.info.resolution);
      const int index_y = floor((pose.position.y - map.info.origin.position.y) / map.info.resolution);

      if ((0 <= index_x && index_x < map.info.width) && (0 <= index_y && index_y < map.info.height))
      {
        if (map.data[index_x + index_y * map.info.width] == 100)
        {
          obs_list_.poses.push_back(pose);
          if (target_velocity_ >= 0.0)
          {
            if ((0.0 < pose.position.x && pose.position.x < dist_to_obj_th_x_) &&
                fabs(pose.position.y) < dist_to_obj_th_y_)
              is_behind_obj_ = true;
          }
          else
          {
            if ((dist_to_obj_th_x_ < pose.position.x && pose.position.x < 0.0) &&
                fabs(pose.position.y) < dist_to_obj_th_y_)
              is_behind_obj_ = true;
          }
          break;
        }
      }
    }
  }
}

PointFollowPlanner::Window PointFollowPlanner::calc_dynamic_window(const geometry_msgs::Twist &current_velocity)
{
  Window window(min_velocity_, max_velocity_, -max_yawrate_, max_yawrate_);
  if (target_velocity_ >= 0.0)
  {
    window.min_velocity_ = std::max((current_velocity.linear.x - max_deceleration_ * dt_), min_velocity_);
    window.max_velocity_ = std::min((current_velocity.linear.x + max_acceleration_ * dt_), target_velocity_);
  }
  else
  {
    window.min_velocity_ = std::max((current_velocity.linear.x - max_acceleration_ * dt_), target_velocity_);
    window.max_velocity_ = std::min((current_velocity.linear.x + max_deceleration_ * dt_), max_velocity_);
  }
  window.min_yawrate_ = std::max((current_velocity.angular.z - max_d_yawrate_ * dt_), -max_yawrate_);
  window.max_yawrate_ = std::min((current_velocity.angular.z + max_d_yawrate_ * dt_), max_yawrate_);

  return window;
}

double PointFollowPlanner::calc_goal_cost(const std::vector<State> &traj, const Eigen::Vector3d &goal)
{
  Eigen::Vector3d last_position(traj.back().x_, traj.back().y_, traj.back().yaw_);
  return (last_position.segment(0, 2) - goal.segment(0, 2)).norm();
}

geometry_msgs::PolygonStamped PointFollowPlanner::transform_footprint(const State &target_pose)
{
  geometry_msgs::PolygonStamped footprint = footprint_;
  footprint.header.stamp = ros::Time::now();
  for (auto &point : footprint.polygon.points)
  {
    Eigen::VectorXf point_in(2);
    point_in << point.x, point.y;
    Eigen::Matrix2f rot;
    rot = Eigen::Rotation2Df(target_pose.yaw_);
    const Eigen::VectorXf point_out = rot * point_in;

    point.x = point_out.x() + target_pose.x_;
    point.y = point_out.y() + target_pose.y_;
  }
  return footprint;
}

bool PointFollowPlanner::is_inside_of_triangle(
    const geometry_msgs::Point &target_point, const geometry_msgs::Polygon &triangle)
{
  if (triangle.points.size() != 3)
  {
    ROS_ERROR("Not triangle");
    exit(1);
  }

  const Eigen::Vector3d vector_A(triangle.points[0].x, triangle.points[0].y, 0.0);
  const Eigen::Vector3d vector_B(triangle.points[1].x, triangle.points[1].y, 0.0);
  const Eigen::Vector3d vector_C(triangle.points[2].x, triangle.points[2].y, 0.0);
  const Eigen::Vector3d vector_P(target_point.x, target_point.y, 0.0);

  const Eigen::Vector3d vector_AB = vector_B - vector_A;
  const Eigen::Vector3d vector_BP = vector_P - vector_B;
  const Eigen::Vector3d cross1 = vector_AB.cross(vector_BP);

  const Eigen::Vector3d vector_BC = vector_C - vector_B;
  const Eigen::Vector3d vector_CP = vector_P - vector_C;
  const Eigen::Vector3d cross2 = vector_BC.cross(vector_CP);

  const Eigen::Vector3d vector_CA = vector_A - vector_C;
  const Eigen::Vector3d vector_AP = vector_P - vector_A;
  const Eigen::Vector3d cross3 = vector_CA.cross(vector_AP);

  if ((0 < cross1.z() && 0 < cross2.z() && 0 < cross3.z()) || (cross1.z() < 0 && cross2.z() < 0 && cross3.z() < 0))
    return true;

  return false;
}

bool PointFollowPlanner::is_inside_of_robot(const geometry_msgs::Pose &obstacle, const State &state)
{
  const geometry_msgs::PolygonStamped footprint = transform_footprint(state);
  geometry_msgs::Point32 state_point;
  state_point.x = state.x_;
  state_point.y = state.y_;

  for (int i = 0; i < footprint.polygon.points.size(); i++)
  {
    geometry_msgs::Polygon triangle;
    triangle.points.push_back(state_point);
    triangle.points.push_back(footprint.polygon.points[i]);

    if (i != footprint.polygon.points.size() - 1)
      triangle.points.push_back(footprint.polygon.points[i + 1]);
    else
      triangle.points.push_back(footprint.polygon.points.front());

    if (is_inside_of_triangle(obstacle.position, triangle))
      return true;
  }

  return false;
}

bool PointFollowPlanner::check_collision(const std::vector<State> &traj)
{
  for (const auto &state : traj)
    for (const auto &obs : obs_list_.poses)
      if (is_inside_of_robot(obs, state))
        return true;

  return false;
}

void PointFollowPlanner::motion(State &state, const double velocity, const double yawrate)
{
  state.yaw_ += yawrate * dt_;
  state.x_ += velocity * std::cos(state.yaw_) * dt_;
  state.y_ += velocity * std::sin(state.yaw_) * dt_;
  state.velocity_ = velocity;
  state.yawrate_ = yawrate;
}

void PointFollowPlanner::generate_trajectory(
    std::vector<State> &trajectory, const double velocity, const double yawrate)
{
  trajectory.clear();
  State state;
  for (float t = 0; t <= predict_time_; t += dt_)
  {
    motion(state, velocity, yawrate);
    trajectory.push_back(state);
  }
}

void PointFollowPlanner::generate_trajectory(
    std::vector<State> &trajectory, const double yawrate, const Eigen::Vector3d &goal)
{
  trajectory.clear();
  State state;
  const double angle_to_goal = atan2(goal.y(), goal.x());
  double predict_time;
  if (target_velocity_ >= 0.0)
    predict_time = fabs(angle_to_goal / (yawrate + DBL_EPSILON));
  else
    predict_time = fabs((M_PI - fabs(angle_to_goal)) / (yawrate + DBL_EPSILON));

  for (float t = 0; t <= predict_time; t += dt_)
  {
    motion(state, 0.0, yawrate);
    trajectory.push_back(state);
  }
}

void PointFollowPlanner::push_back_trajectory(
    std::vector<std::vector<State>> &trajectories, const double velocity, const double yawrate)
{
  std::vector<State> traj;
  generate_trajectory(traj, velocity, yawrate);
  trajectories.push_back(traj);
}

void PointFollowPlanner::search_optimal_cmd_vel_for_goal(
    double &optimal_velocity, double &optimal_yawrate, const Window dynamic_window, const Eigen::Vector3d &goal,
    std::vector<std::vector<State>> &trajectories)
{
  float min_cost = 1e6;
  const double velocity_resolution =
      (dynamic_window.max_velocity_ - dynamic_window.min_velocity_) / (velocity_samples_ - 1);
  const double yawrate_resolution =
      (dynamic_window.max_yawrate_ - dynamic_window.min_yawrate_) / (yawrate_samples_ - 1);

  for (int i = 0; i < velocity_samples_; i++)
  {
    const double velocity = target_velocity_ >= 0.0 ? dynamic_window.min_velocity_ + velocity_resolution * i
                                                    : dynamic_window.max_velocity_ - velocity_resolution * i;
    for (int j = 0; j < yawrate_samples_; j++)
    {
      double yawrate = dynamic_window.min_yawrate_ + yawrate_resolution * j;
      if (fabs(velocity) < slow_velocity_th_)
        yawrate = yawrate > 0 ? std::max(yawrate, min_yawrate_) : std::min(yawrate, -min_yawrate_);
      push_back_trajectory(trajectories, velocity, yawrate);
      const double goal_cost = calc_goal_cost(trajectories.back(), goal);
      if (goal_cost <= min_cost)
      {
        min_cost = goal_cost;
        optimal_velocity = velocity;
        optimal_yawrate = yawrate;
      }
    }

    if (dynamic_window.min_yawrate_ < 0.0 && 0.0 < dynamic_window.max_yawrate_)
    {
      push_back_trajectory(trajectories, velocity, 0.0);
      const double goal_cost = calc_goal_cost(trajectories.back(), goal);
      if (goal_cost <= min_cost)
      {
        min_cost = goal_cost;
        optimal_velocity = velocity;
        optimal_yawrate = 0.0;
      }
    }
  }
}

void PointFollowPlanner::search_safety_trajectory(
    std::vector<State> &optimal_traj, const double optimal_velocity, const double optimal_yawrate,
    const Window dynamic_window, const Eigen::Vector3d &goal)
{
  bool is_found_safety_traj = false;
  const double velocity_resolution = target_velocity_ >= 0.0
                                         ? (optimal_velocity - dynamic_window.min_velocity_) / (velocity_samples_ - 1)
                                         : (dynamic_window.max_velocity_ - optimal_velocity) / (velocity_samples_ - 1);

  for (int i = 0; i < velocity_samples_; i++)
  {
    const double velocity = target_velocity_ >= 0.0 ? dynamic_window.min_velocity_ + velocity_resolution * i
                                                    : dynamic_window.max_velocity_ - velocity_resolution * i;
    generate_trajectory(optimal_traj, velocity, optimal_yawrate);
    if (!check_collision(optimal_traj))
      is_found_safety_traj = true;
    else
      break;
  }

  if (!is_found_safety_traj)
    generate_trajectory(optimal_traj, 0.0, 0.0);
}

void PointFollowPlanner::planning(
    std::vector<State> &best_traj, std::vector<std::vector<State>> &trajectories, const Eigen::Vector3d &goal)
{
  const Window dynamic_window = calc_dynamic_window(current_velocity_);

  if (fabs(previous_velocity_.linear.x) < velocity_th_for_stop_behind_obj_ && is_behind_obj_)
  {
    ROS_WARN_THROTTLE(1.0, "##########################");
    ROS_WARN_THROTTLE(1.0, "### stop behind object ###");
    ROS_WARN_THROTTLE(1.0, "##########################");
    generate_trajectory(best_traj, 0.0, 0.0);
    trajectories.push_back(best_traj);
  }
  else if (fabs(dynamic_window.max_velocity_ - dynamic_window.min_velocity_) < DBL_EPSILON)
  {
    generate_trajectory(best_traj, 0.0, 0.0);
    trajectories.push_back(best_traj);
  }
  else
  {
    double optimal_velocity, optimal_yawrate;
    search_optimal_cmd_vel_for_goal(optimal_velocity, optimal_yawrate, dynamic_window, goal, trajectories);
    search_safety_trajectory(best_traj, optimal_velocity, optimal_yawrate, dynamic_window, goal);
  }
}

geometry_msgs::Twist PointFollowPlanner::calc_cmd_vel()
{
  geometry_msgs::Twist cmd_vel;
  std::vector<State> best_traj;
  std::vector<std::vector<State>> trajectories;
  const Eigen::Vector3d goal(goal_.pose.position.x, goal_.pose.position.y, tf::getYaw(goal_.pose.orientation));

  if (recovery_params_.available)
  {
    if (0 < recovery_params_.recovery_count && recovery_params_.recovery_count <= recovery_params_.max_recovery_count)
    {
      ROS_WARN_THROTTLE(1.0, "Recovery mode is activated");
      recovery_params_.recovery_count++;
    }
    else if (is_stuck())
    {
      recovery_params_.recovery_count = 0;
      recovery_params_.stuck_count++;
      if (recovery_params_.stuck_count_th <= recovery_params_.stuck_count)
      {
        recovery_params_.recovery_count++;
        sound(recovery_params_.sound_file);
      }
    }
    else
    {
      recovery_params_.recovery_count = 0;
      recovery_params_.stuck_count = 0;
    }
  }

  if (dist_to_goal_th_ < goal.segment(0, 2).norm() && !has_reached_)
  {
    if (can_adjust_robot_direction(goal))
    {
      const double angle_to_goal = atan2(goal.y(), goal.x());
      if (target_velocity_ >= 0.0)
      {
        cmd_vel.angular.z =
            angle_to_goal > 0 ? std::min(angle_to_goal, max_yawrate_) : std::max(angle_to_goal, -max_yawrate_);
      }
      else
      {
        cmd_vel.angular.z = angle_to_goal > 0 ? std::max(-M_PI + angle_to_goal, -max_yawrate_)
                                              : std::min(M_PI + angle_to_goal, max_yawrate_);
      }
      cmd_vel.angular.z = cmd_vel.angular.z > 0 ? std::max(cmd_vel.angular.z, min_in_place_yawrate_)
                                                : std::min(cmd_vel.angular.z, -min_in_place_yawrate_);
      generate_trajectory(best_traj, cmd_vel.angular.z, goal);
      trajectories.push_back(best_traj);
    }
    else
    {
      planning(best_traj, trajectories, goal);
      cmd_vel.linear.x = best_traj.front().velocity_;
      cmd_vel.angular.z = best_traj.front().yawrate_;
    }
  }
  else
  {
    has_reached_ = true;
    if (turn_direction_th_ < fabs(goal[2]) && turn_at_goal_flag_)
    {
      cmd_vel.angular.z = goal[2] > 0 ? std::min(goal[2], max_yawrate_) : std::max(goal[2], -max_yawrate_);
      cmd_vel.angular.z = cmd_vel.angular.z > 0 ? std::max(cmd_vel.angular.z, min_in_place_yawrate_)
                                                : std::min(cmd_vel.angular.z, -min_in_place_yawrate_);
    }
    else
    {
      has_finished_.data = true;
      has_reached_ = false;
    }
    generate_trajectory(best_traj, cmd_vel.linear.x, cmd_vel.angular.z);
    trajectories.push_back(best_traj);
  }

  visualize_trajectory(best_traj, 1, 0, 0, best_trajectory_pub_);
  visualize_trajectories(trajectories, 0, 1, 0, candidate_trajectories_pub_);
  visualize_footprints(best_traj, 0, 0, 1, predict_footprints_pub_);

  return cmd_vel;
}

bool PointFollowPlanner::is_stuck()
{
  return fabs(current_velocity_.linear.x) < DBL_EPSILON && fabs(current_velocity_.angular.z) < DBL_EPSILON;
}

void PointFollowPlanner::sound(const std::string &path)
{
  if (path == "")
    return;

  const std::string sound_command = "aplay " + path + " &";
  if (system(sound_command.c_str()) == -1)
    ROS_WARN("Failed to play sound");
}

bool PointFollowPlanner::can_adjust_robot_direction(const Eigen::Vector3d &goal)
{
  const double angle_to_goal = atan2(goal.y(), goal.x());
  double yawrate;
  if (target_velocity_ >= 0.0)
  {
    if (fabs(angle_to_goal) < angle_to_goal_th_)
      return false;
    yawrate = angle_to_goal > 0 ? std::min(angle_to_goal, max_yawrate_) : std::max(angle_to_goal, -max_yawrate_);
  }
  else
  {
    if (M_PI - fabs(angle_to_goal) < angle_to_goal_th_)
      return false;
    yawrate = angle_to_goal > 0 ? std::max(-M_PI + angle_to_goal, -max_yawrate_)
                                : std::min(M_PI + angle_to_goal, max_yawrate_);
  }

  yawrate = yawrate > 0 ? std::max(yawrate, min_in_place_yawrate_) : std::min(yawrate, -min_in_place_yawrate_);

  std::vector<State> traj;
  generate_trajectory(traj, yawrate, goal);

  if (!check_collision(traj))
    return true;
  else
    return false;
}

bool PointFollowPlanner::can_move()
{
  if (!goal_subscribed_)
    ROS_WARN_THROTTLE(1.0, "Local goal has not been updated");
  if (!footprint_subscribed_)
    ROS_WARN_THROTTLE(1.0, "Footprint has not been updated");
  if (subscribe_count_th_ < local_map_not_sub_count_)
    ROS_WARN_THROTTLE(1.0, "Local map has not been updated");
  if (subscribe_count_th_ < odom_not_sub_count_)
    ROS_WARN_THROTTLE(1.0, "Odom has not been updated");

  if (!local_map_updated_)
    local_map_not_sub_count_++;
  if (!odom_updated_)
    odom_not_sub_count_++;

  if (goal_subscribed_ && footprint_subscribed_ && local_map_not_sub_count_ <= subscribe_count_th_ &&
      odom_not_sub_count_ <= subscribe_count_th_)
    return true;
  else
    return false;
}

void PointFollowPlanner::process()
{
  ros::Rate loop_rate(hz_);

  while (ros::ok())
  {
    geometry_msgs::Twist cmd_vel;
    if (can_move())
      cmd_vel = calc_cmd_vel();
    cmd_vel_pub_.publish(cmd_vel);
    finish_flag_pub_.publish(has_finished_);
    if (has_finished_.data)
      ros::Duration(sleep_time_after_finish_).sleep();

    odom_updated_ = false;
    local_map_updated_ = false;
    is_behind_obj_ = false;
    has_finished_.data = false;

    ros::spinOnce();
    loop_rate.sleep();
  }
}

void PointFollowPlanner::visualize_trajectory(
    const std::vector<State> &trajectory, const double r, const double g, const double b, const ros::Publisher &pub)
{
  visualization_msgs::Marker v_trajectory;
  v_trajectory.header.frame_id = robot_frame_;
  v_trajectory.header.stamp = ros::Time::now();
  v_trajectory.color.r = r;
  v_trajectory.color.g = g;
  v_trajectory.color.b = b;
  v_trajectory.color.a = 0.8;
  v_trajectory.ns = pub.getTopic();
  v_trajectory.type = visualization_msgs::Marker::LINE_STRIP;
  v_trajectory.action = visualization_msgs::Marker::ADD;
  v_trajectory.lifetime = ros::Duration(1 / hz_);
  v_trajectory.scale.x = 0.05;
  geometry_msgs::Pose pose;
  pose.orientation.w = 1;
  v_trajectory.pose = pose;

  for (const auto &pose : trajectory)
  {
    geometry_msgs::Point point;
    point.x = pose.x_;
    point.y = pose.y_;
    v_trajectory.points.push_back(point);
  }

  pub.publish(v_trajectory);
}

void PointFollowPlanner::visualize_trajectories(
    const std::vector<std::vector<State>> &trajectories, const double r, const double g, const double b,
    const ros::Publisher &pub)
{
  visualization_msgs::MarkerArray v_trajectories;
  for (int i = 0; i < trajectories.size(); i++)
  {
    visualization_msgs::Marker v_trajectory;
    v_trajectory.header.frame_id = robot_frame_;
    v_trajectory.header.stamp = ros::Time::now();
    v_trajectory.color.r = r;
    v_trajectory.color.g = g;
    v_trajectory.color.b = b;
    v_trajectory.color.a = 0.8;
    v_trajectory.ns = pub.getTopic();
    v_trajectory.type = visualization_msgs::Marker::LINE_STRIP;
    v_trajectory.action = visualization_msgs::Marker::ADD;
    v_trajectory.lifetime = ros::Duration(1 / hz_);
    v_trajectory.id = i;
    v_trajectory.scale.x = 0.02;
    geometry_msgs::Pose pose;
    pose.orientation.w = 1;
    v_trajectory.pose = pose;
    geometry_msgs::Point p;
    for (const auto &pose : trajectories[i])
    {
      p.x = pose.x_;
      p.y = pose.y_;
      v_trajectory.points.push_back(p);
    }
    v_trajectories.markers.push_back(v_trajectory);
  }
  pub.publish(v_trajectories);
}

void PointFollowPlanner::visualize_footprints(
    const std::vector<State> &trajectory, const double r, const double g, const double b, const ros::Publisher &pub)
{
  visualization_msgs::MarkerArray v_footprints;
  for (int i = 0; i < trajectory.size(); i++)
  {
    visualization_msgs::Marker v_footprint;
    v_footprint.header.frame_id = robot_frame_;
    v_footprint.header.stamp = ros::Time::now();
    v_footprint.color.r = r;
    v_footprint.color.g = g;
    v_footprint.color.b = b;
    v_footprint.color.a = 0.8;
    v_footprint.ns = pub.getTopic();
    v_footprint.type = visualization_msgs::Marker::LINE_STRIP;
    v_footprint.action = visualization_msgs::Marker::ADD;
    v_footprint.lifetime = ros::Duration(1 / hz_);
    v_footprint.id = i;
    v_footprint.scale.x = 0.01;
    geometry_msgs::Pose pose;
    pose.orientation.w = 1;
    v_footprint.pose = pose;
    geometry_msgs::Point p;
    const geometry_msgs::PolygonStamped footprint = transform_footprint(trajectory[i]);
    for (const auto &point : footprint.polygon.points)
    {
      p.x = point.x;
      p.y = point.y;
      v_footprint.points.push_back(p);
    }
    p.x = footprint.polygon.points.front().x;
    p.y = footprint.polygon.points.front().y;
    v_footprint.points.push_back(p);
    v_footprints.markers.push_back(v_footprint);
  }
  pub.publish(v_footprints);
}
