// Copyright 2023 amsl

#ifndef POINT_FOLLOW_PLANNER_POINT_FOLLOW_PLANNER_H
#define POINT_FOLLOW_PLANNER_POINT_FOLLOW_PLANNER_H

#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_srvs/SetBool.h>
#include <string>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Dense>

class PointFollowPlanner
{
public:
  PointFollowPlanner(void);
  void process();

protected:
  class State
  {
  public:
    State(void);
    double x_;
    double y_;
    double yaw_;
    double velocity_;
    double yawrate_;

  private:
  };

  class Window
  {
  public:
    Window(const double min_v, const double max_v, const double min_y, const double max_y);
    double min_velocity_;
    double max_velocity_;
    double min_yawrate_;
    double max_yawrate_;

  private:
  };

  struct RecoveryParams
  {
    bool available = false;
    int stuck_count = 0;
    int stuck_count_th;
    int recovery_count = 0;
    int max_recovery_count;
    float stuck_time_th;
    float time;
    float goal_dist;
    float goal_angle;
    double max_velocity;
    std::string sound_file;
  };

  // callback function
  void goal_callback(const geometry_msgs::PoseStampedConstPtr &msg);
  void footprint_callback(const geometry_msgs::PolygonStampedPtr &msg);
  void local_map_callback(const nav_msgs::OccupancyGridConstPtr &msg);
  void odom_callback(const nav_msgs::OdometryConstPtr &msg);
  void target_velocity_callback(const geometry_msgs::TwistConstPtr &msg);
  void dist_to_goal_th_callback(const std_msgs::Float64ConstPtr &msg);
  void dist_from_head_to_obj_callback(const std_msgs::Float64ConstPtr &msg);
  bool turn_at_goal_flag_callback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
  bool recovery_mode_flag_callback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

  // others
  void motion(State &state, const double velocity, const double yawrate);
  void create_obs_list(const nav_msgs::OccupancyGrid &map);
  void generate_trajectory(std::vector<State> &trajectory, const double velocity, const double yawrate);
  void generate_trajectory_for_adjusting_robot_direction(
      std::vector<State> &trajectory, const double yawrate, const double yaw_diff);
  void push_back_trajectory(std::vector<std::vector<State>> &trajectories, const double velocity, const double yawrate);
  void search_optimal_cmd_vel_for_goal(
      double &optimal_velocity, double &optimal_yawrate, const Window dynamic_window, const Eigen::Vector3d &goal,
      std::vector<std::vector<State>> &trajectories);
  void search_safety_trajectory(
      std::vector<State> &optimal_traj, const double optimal_velocity, const double optimal_yawrate,
      const Window dynamic_window, const Eigen::Vector3d &goal);
  bool can_move();
  bool can_adjust_robot_direction(const Eigen::Vector3d &goal);
  bool can_add_to_obs_list(const geometry_msgs::Point &point);
  bool check_collision(const std::vector<State> &traj);
  bool is_inside_of_triangle(const geometry_msgs::Point &target_point, const geometry_msgs::Polygon &triangle);
  bool is_inside_of_robot(const geometry_msgs::Pose &obstacle, const State &state);
  double calc_goal_cost(const std::vector<State> &traj, const Eigen::Vector3d &goal);
  geometry_msgs::PolygonStamped transform_footprint(const State &target_pose);
  void set_dist_to_obj_th(const geometry_msgs::PolygonStamped &footprint);
  void
  planning(std::vector<State> &best_traj, std::vector<std::vector<State>> &trajectories, const Eigen::Vector3d &goal);
  geometry_msgs::Twist calc_cmd_vel();
  bool is_stuck();
  void sound(const std::string &path);
  Window calc_dynamic_window(const geometry_msgs::Twist &current_velocity);
  void visualize_trajectory(
      const std::vector<State> &trajectory, const double r, const double g, const double b, const ros::Publisher &pub);
  void visualize_trajectories(
      const std::vector<std::vector<State>> &trajectories, const double r, const double g, const double b,
      const ros::Publisher &pub);
  void visualize_footprints(
      const std::vector<State> &trajectory, const double r, const double g, const double b, const ros::Publisher &pub);

  // param
  double hz_;
  std::string robot_frame_;
  double target_velocity_;
  double velocity_th_for_stop_behind_obj_;
  double max_velocity_;
  double min_velocity_;
  double max_yawrate_;
  double min_yawrate_;
  double min_in_place_yawrate_;
  double max_acceleration_;
  double max_deceleration_;
  double max_d_yawrate_;
  double angle_resolution_;
  double predict_time_;
  double dt_;
  double sleep_time_after_finish_;
  double angle_to_goal_th_;
  double dist_to_goal_th_;
  double turn_direction_th_;
  double slow_velocity_th_;
  double dist_from_head_to_obj_;
  double dist_to_obj_th_x_;
  double dist_to_obj_th_y_;
  bool goal_subscribed_;
  bool footprint_subscribed_;
  bool odom_updated_;
  bool local_map_updated_;
  bool is_behind_obj_;
  bool has_reached_;
  bool turn_at_goal_flag_;
  int velocity_samples_;
  int yawrate_samples_;
  int subscribe_count_th_;
  int odom_not_sub_count_;
  int local_map_not_sub_count_;

  RecoveryParams recovery_params_;

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Publisher cmd_vel_pub_;
  ros::Publisher candidate_trajectories_pub_;
  ros::Publisher best_trajectory_pub_;
  ros::Publisher predict_footprints_pub_;
  ros::Publisher finish_flag_pub_;
  ros::Subscriber footprint_sub_;
  ros::Subscriber goal_sub_;
  ros::Subscriber local_map_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber target_velocity_sub_;
  ros::Subscriber dist_to_goal_th_sub_;
  ros::Subscriber dist_from_head_to_obj_sub_;
  ros::ServiceServer turn_at_goal_flag_server_;
  ros::ServiceServer recovery_mode_flag_server_;

  geometry_msgs::PoseStamped goal_;
  geometry_msgs::PoseArray obs_list_;
  geometry_msgs::PolygonStamped footprint_;
  geometry_msgs::Twist current_velocity_;
  geometry_msgs::Twist previous_velocity_;

  std_msgs::Bool has_finished_;

  tf::TransformListener listener_;
};

#endif  // POINT_FOLLOW_PLANNER_POINT_FOLLOW_PLANNER_H
