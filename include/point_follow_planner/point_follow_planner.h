#ifndef Point_Follow_PLANNER_H
#define Point_Follow_PLANNER_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>

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
        State(const double, const double, const double, const double, const double);

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
        Window(void);
        Window(const double, const double, const double, const double);
        double min_velocity_;
        double max_velocity_;
        double min_yawrate_;
        double max_yawrate_;
    private:
    };


    // callback function
    void local_goal_callback(const geometry_msgs::PoseStampedConstPtr& msg);
    void robot_footprint_callback(const geometry_msgs::PolygonStampedPtr& msg);
    void local_map_callback(const nav_msgs::OccupancyGridConstPtr& msg);
    void odom_callback(const nav_msgs::OdometryConstPtr& msg);

    // others
    void motion(State& state, const double velocity, const double yawrate);
    void raycast(const nav_msgs::OccupancyGrid& map);
    bool can_move();
    bool is_hit_obstacle(const std::vector<State>& traj);
    bool is_inside_of_triangle(const geometry_msgs::Point& target_point, const geometry_msgs::Polygon& triangle);
    bool is_inside_of_robot(const geometry_msgs::Pose& obstacle, const State & state);
    double calc_goal_cost(const std::vector<State>& traj, const Eigen::Vector3d& goal);
    geometry_msgs::PolygonStamped move_footprint(const State& target_pose);
    geometry_msgs::Twist planning(const Window dynamic_window, const Eigen::Vector3d goal);
    geometry_msgs::Twist calc_cmd_vel();
    Window calc_dynamic_window(const geometry_msgs::Twist& current_velocity);

    void visualize_trajectory(const std::vector<State>& trajectory, const double r, const double g, const double b, const ros::Publisher& pub);
    void visualize_trajectories(const std::vector<std::vector<State>>& trajectories, const double r, const double g, const double b, const int trajectories_size, const ros::Publisher& pub);


    // param
    bool local_goal_subscribed_;
    bool robot_footprint_subscribed_;
    bool odom_updated_;
    bool local_map_updated_;
    double hz_;
    double max_velocity_;
    double min_velocity_;
    double max_yawrate_;
    double max_acceleration_;
    double max_d_yawrate_;
    double max_dist_;
    double velocity_resolution_;
    double yawrate_resolution_;
    double angle_resolution_;
    double predict_time_;
    double dt_;

    std::string robot_frame_;

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Publisher velocity_pub_;
    ros::Publisher candidate_trajectories_pub_;
    ros::Publisher selected_trajectory_pub_;
    ros::Subscriber local_map_sub_;
    ros::Subscriber local_goal_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber robot_footprint_sub_;

    geometry_msgs::PoseStamped local_goal_;
    geometry_msgs::PoseArray obs_list_;
    geometry_msgs::PolygonStamped robot_footprint_;
    geometry_msgs::Twist current_velocity_;

    tf2_ros::Buffer tf_buffer_;
};

#endif // Point_Follow_PLANNER_H
