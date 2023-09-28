#include "point_follow_planner/point_follow_planner.h"

PointFollowPlanner::PointFollowPlanner(void):
    private_nh_("~"),
    goal_subscribed_(false),
    footprint_subscribed_(false),
    odom_updated_(false),
    local_map_updated_(false),
    is_behind_obj_(false),
    local_map_not_sub_count_(0),
    odom_not_sub_count_(0)
{
    private_nh_.param<double>("hz", hz_, {20});
    private_nh_.param<std::string>("robot_frame", robot_frame_, {"base_link"});
    private_nh_.param<double>("target_velocity", target_velocity_, {0.55});
    private_nh_.param<double>("max_velocity", max_velocity_, {0.55});
    private_nh_.param<double>("min_velocity", min_velocity_, {0.0});
    private_nh_.param<double>("max_yawrate", max_yawrate_, {1.0});
    private_nh_.param<double>("max_yawrate_in_situ_turns", max_yawrate_in_situ_turns_, max_yawrate_);
    private_nh_.param<double>("max_acceleration", max_acceleration_, {1.0});
    private_nh_.param<double>("max_d_yawrate", max_d_yawrate_, {3.2});
    private_nh_.param<double>("angle_resolution", angle_resolution_, {0.2});
    private_nh_.param<double>("predict_time", predict_time_, {3.0});
    private_nh_.param<double>("dt", dt_, {0.1});
    private_nh_.param<double>("angle_to_goal_th", angle_to_goal_th_, {0.26});
    private_nh_.param<double>("goal_threshold", goal_threshold_, {0.3});
    private_nh_.param<double>("turn_direction_threshold", turn_direction_threshold_, {0.1});
    private_nh_.param<double>("obs_dist_th", obs_dist_th_, {1.0});
    private_nh_.param<int>("velocity_samples", velocity_samples_, {3});
    private_nh_.param<int>("yawrate_samples", yawrate_samples_, {20});
    private_nh_.param<int>("sub_count_th", sub_count_th_, {3});

    ROS_INFO("=== Point Followe Planner ===");
    ROS_INFO_STREAM("hz: " << hz_);
    ROS_INFO_STREAM("robot_frame: " << robot_frame_);
    ROS_INFO_STREAM("max_velocity: " << max_velocity_);
    ROS_INFO_STREAM("min_velocity: " << min_velocity_);
    ROS_INFO_STREAM("max_yawrate: " << max_yawrate_);
    ROS_INFO_STREAM("max_acceleration: " << max_acceleration_);
    ROS_INFO_STREAM("max_d_yawrate: " << max_d_yawrate_);
    ROS_INFO_STREAM("angle_resolution: " << angle_resolution_);
    ROS_INFO_STREAM("predict_time: " << predict_time_);
    ROS_INFO_STREAM("dt: " << dt_);
    ROS_INFO_STREAM("angle_to_goal_th: " << angle_to_goal_th_);
    ROS_INFO_STREAM("obs_dist_th: " << obs_dist_th_);

    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    best_trajectory_pub_ = private_nh_.advertise<visualization_msgs::Marker>("best_trajectory", 1);
    candidate_trajectories_pub_ = private_nh_.advertise<visualization_msgs::MarkerArray>("candidate_trajectories", 1);
    predict_footprint_pub_ = private_nh_.advertise<geometry_msgs::PolygonStamped>("predict_footprint", 1);
    finish_flag_pub_ = private_nh_.advertise<std_msgs::Bool>("finish_flag", 1);

    footprint_sub_ = nh_.subscribe("/footprint", 1, &PointFollowPlanner::footprint_callback, this);
    goal_sub_ = nh_.subscribe("/local_goal", 1, &PointFollowPlanner::goal_callback, this);
    local_map_sub_ = nh_.subscribe("/local_map", 1, &PointFollowPlanner::local_map_callback, this);
    odom_sub_ = nh_.subscribe("/odom", 1, &PointFollowPlanner::odom_callback, this);
    target_velocity_sub_ = nh_.subscribe("/target_velocity", 1, &PointFollowPlanner::target_velocity_callback, this);
}


PointFollowPlanner::State::State(void)
    :x_(0.0), y_(0.0), yaw_(0.0), velocity_(0.0), yawrate_(0.0)
{
}


PointFollowPlanner::Window::Window(const double min_v, const double max_v, const double min_y, const double max_y)
    :min_velocity_(min_v), max_velocity_(max_v), min_yawrate_(min_y), max_yawrate_(max_y)
{
}


void PointFollowPlanner::goal_callback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    goal_ = *msg;
    try
    {
        listener_.transformPose(robot_frame_, ros::Time(0), goal_, goal_.header.frame_id, goal_);
        goal_subscribed_ = true;
    }
    catch(tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
    }
}


void PointFollowPlanner::footprint_callback(const geometry_msgs::PolygonStampedPtr& msg)
{
    footprint_ = *msg;
    footprint_subscribed_ = true;
}


void PointFollowPlanner::local_map_callback(const nav_msgs::OccupancyGridConstPtr& msg)
{
    raycast(*msg);
    local_map_not_sub_count_ = 0;
    local_map_updated_ = true;
}


void PointFollowPlanner::odom_callback(const nav_msgs::OdometryConstPtr& msg)
{
    current_velocity_ = msg->twist.twist;
    current_velocity_.linear.x = std::max(current_velocity_.linear.x, 0.0);
    odom_not_sub_count_ = 0;
    odom_updated_ = true;
}


void PointFollowPlanner::target_velocity_callback(const geometry_msgs::TwistConstPtr& msg)
{
    target_velocity_ = std::min(msg->linear.x, max_velocity_);
    ROS_INFO_THROTTLE(1.0, "target velocity was updated to %f [m/s]", target_velocity_);
}


void PointFollowPlanner::raycast(const nav_msgs::OccupancyGrid& map)
{
    obs_list_.poses.clear();
    const double max_search_dist = hypot(map.info.origin.position.x, map.info.origin.position.y);

    for(float angle=-M_PI; angle <= M_PI; angle += angle_resolution_)
    {
        for(float dist = 0.0; dist <= max_search_dist; dist += map.info.resolution)
        {
            geometry_msgs::Pose pose;
            pose.position.x = dist * cos(angle);
            pose.position.y = dist * sin(angle);
            const int index_x = int(floor((pose.position.x - map.info.origin.position.x) / map.info.resolution));
            const int index_y = int(floor((pose.position.y - map.info.origin.position.y) / map.info.resolution));

            if((0<=index_x and index_x<map.info.width) and (0<=index_y and index_y<map.info.height))
            {
                if(map.data[index_x + index_y*map.info.width] == 100)
                {
                    obs_list_.poses.push_back(pose);
                    if((fabs(angle) < 2.0*angle_to_goal_th_) and (dist <= obs_dist_th_))
                        is_behind_obj_ = true;
                    break;
                }
            }
        }
    }
}


PointFollowPlanner::Window PointFollowPlanner::calc_dynamic_window(const geometry_msgs::Twist& current_velocity)
{
    Window window(min_velocity_, max_velocity_, -max_yawrate_, max_yawrate_);
    window.min_velocity_ = std::max((current_velocity.linear.x - max_acceleration_*dt_), min_velocity_);
    window.max_velocity_ = std::min((current_velocity.linear.x + max_acceleration_*dt_), target_velocity_);
    window.min_yawrate_  = std::max((current_velocity.angular.z - max_d_yawrate_*dt_), -max_yawrate_);
    window.max_yawrate_  = std::min((current_velocity.angular.z + max_d_yawrate_*dt_),  max_yawrate_);

    return window;
}


double PointFollowPlanner::calc_goal_cost(const std::vector<State>& traj, const Eigen::Vector3d& goal)
{
    Eigen::Vector3d last_position(traj.back().x_, traj.back().y_, traj.back().yaw_);
    return (last_position.segment(0, 2) - goal.segment(0, 2)).norm();
}


geometry_msgs::PolygonStamped PointFollowPlanner::transform_footprint(const State& target_pose)
{
    geometry_msgs::PolygonStamped footprint = footprint_;
    footprint.header.stamp = ros::Time::now();
    for(auto& point : footprint.polygon.points)
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


bool PointFollowPlanner::is_inside_of_triangle(const geometry_msgs::Point& target_point, const geometry_msgs::Polygon& triangle)
{
    if(triangle.points.size() != 3)
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

    if ((0<cross1.z() and 0<cross2.z() and 0<cross3.z()) or (cross1.z()<0 and cross2.z()<0 and cross3.z()<0))
        return true;

    return false;
}


bool PointFollowPlanner::is_inside_of_robot(const geometry_msgs::Pose& obstacle, const State & state)
{
    const geometry_msgs::PolygonStamped footprint = transform_footprint(state);
    geometry_msgs::Point32 state_point;
    state_point.x = state.x_;
    state_point.y = state.y_;

    for(int i=0; i<footprint.polygon.points.size(); i++)
    {
        geometry_msgs::Polygon triangle;
        triangle.points.push_back(state_point);
        triangle.points.push_back(footprint.polygon.points[i]);

        if(i != footprint.polygon.points.size()-1)
            triangle.points.push_back(footprint.polygon.points[i+1]);
        else
            triangle.points.push_back(footprint.polygon.points.front());

        if(is_inside_of_triangle(obstacle.position, triangle))
            return true;
    }

    return false;
}


bool PointFollowPlanner::check_collision(const std::vector<State>& traj)
{
    for(const auto& state : traj)
        for (const auto& obs : obs_list_.poses)
            if(is_inside_of_robot(obs, state))
                return true;

    return false;
}


void PointFollowPlanner::motion(State& state, const double velocity, const double yawrate)
{
    state.yaw_ += yawrate*dt_;
    state.x_ += velocity*std::cos(state.yaw_)*dt_;
    state.y_ += velocity*std::sin(state.yaw_)*dt_;
    state.velocity_ = velocity;
    state.yawrate_ = yawrate;
}


void PointFollowPlanner::generate_trajectory(std::vector<State>& trajectory, const double velocity, const double yawrate)
{
        trajectory.clear();
        State state;
        for(float t=0; t<=predict_time_; t+=dt_)
        {
            motion(state, velocity, yawrate);
            trajectory.push_back(state);
        }
}


void PointFollowPlanner::push_back_trajectory(std::vector<std::vector<State>>& trajectories, const double velocity, const double yawrate)
{
    std::vector<State> traj;
    generate_trajectory(traj, velocity, yawrate);
    trajectories.push_back(traj);
}


void PointFollowPlanner::search_optimal_cmd_vel_for_goal(
        double& optimal_velocity,
        double& optimal_yawrate,
        const Window dynamic_window,
        const Eigen::Vector3d& goal,
        std::vector<std::vector<State>>& trajectories)
{
    float min_cost = 1e6;
    const double velocity_resolution = (dynamic_window.max_velocity_ - dynamic_window.min_velocity_) / velocity_samples_;
    const double yawrate_resolution = (dynamic_window.max_yawrate_ - dynamic_window.min_yawrate_) / yawrate_samples_;

    for(double velocity=dynamic_window.min_velocity_; velocity<=dynamic_window.max_velocity_; velocity+=velocity_resolution)
    {
        for(double yawrate=dynamic_window.min_yawrate_; yawrate<=dynamic_window.max_yawrate_; yawrate+=yawrate_resolution)
        {
            push_back_trajectory(trajectories, velocity, yawrate);
            const double goal_cost = calc_goal_cost(trajectories.back(), goal);
            if(goal_cost <= min_cost)
            {
                min_cost  = goal_cost;
                optimal_velocity = velocity;
                optimal_yawrate = yawrate;
            }
        }

        if (dynamic_window.min_yawrate_ < 0.0 and 0.0 < dynamic_window.max_yawrate_)
        {
            push_back_trajectory(trajectories, velocity, 0.0);
            const double goal_cost = calc_goal_cost(trajectories.back(), goal);
            if(goal_cost <= min_cost)
            {
                min_cost  = goal_cost;
                optimal_velocity = velocity;
                optimal_yawrate = 0.0;
            }
        }
    }
}


void PointFollowPlanner::search_safety_trajectory(
        std::vector<State>& optimal_traj,
        const double optimal_velocity,
        const double optimal_yawrate,
        const Window dynamic_window,
        const Eigen::Vector3d& goal)
{
    bool is_found_safety_traj = false;
    const double velocity_resolution = (dynamic_window.max_velocity_ - dynamic_window.min_velocity_) / velocity_samples_;

    for(double velocity=dynamic_window.min_velocity_; velocity<=optimal_velocity; velocity+=velocity_resolution)
    {
        generate_trajectory(optimal_traj, velocity, optimal_yawrate);
        if(!check_collision(optimal_traj))
            is_found_safety_traj = true;
        else
            break;
    }

    if(!is_found_safety_traj) generate_trajectory(optimal_traj, 0.0, 0.0);
}


geometry_msgs::Twist PointFollowPlanner::planning(const Window dynamic_window, const Eigen::Vector3d goal)
{
    std::vector<std::vector<State>> trajectories;
    std::vector<State> traj;

    // reaching goal
    const double angle_to_goal = atan2(goal.y(), goal.x());
    const double dist_to_goal = hypot(goal.x(), goal.y());
    if(dist_to_goal <= goal_threshold_)
    {
        geometry_msgs::Twist cmd_vel;
        if(turn_direction_threshold_ < fabs(goal[2]))
            cmd_vel.angular.z = std::min(std::max(goal[2], -max_yawrate_in_situ_turns_), max_yawrate_in_situ_turns_);
        else
            has_finished.data = true;

        generate_trajectory(traj, cmd_vel.linear.x, cmd_vel.angular.z);
        trajectories.push_back(traj);
        visualize_trajectories(trajectories, 0.0, 1.0, 0.0, 1000, candidate_trajectories_pub_);
        visualize_trajectory(traj, 1.0, 0.0, 0.0, best_trajectory_pub_);
        predict_footprint_pub_.publish(transform_footprint(traj.back()));

        return cmd_vel;
    }

    // turning in place
    if(angle_to_goal_th_ < fabs(angle_to_goal))
    {
        geometry_msgs::Twist cmd_vel;
        cmd_vel.angular.z = std::min(std::max(angle_to_goal, -max_yawrate_in_situ_turns_), max_yawrate_in_situ_turns_);
        generate_trajectory(traj, cmd_vel.linear.x, cmd_vel.angular.z);
        if(!check_collision(traj))
        {
            trajectories.push_back(traj);
            visualize_trajectories(trajectories, 0.0, 1.0, 0.0, 1000, candidate_trajectories_pub_);
            visualize_trajectory(traj, 1.0, 0.0, 0.0, best_trajectory_pub_);
            predict_footprint_pub_.publish(transform_footprint(traj.back()));
            return cmd_vel;
        }
    }

    // stop behind object
    if(previous_velocity_.linear.x <= 0.2 and is_behind_obj_)
    {
        ROS_WARN_THROTTLE(1.0, "##########################");
        ROS_WARN_THROTTLE(1.0, "### stop behind object ###");
        ROS_WARN_THROTTLE(1.0, "##########################");

        geometry_msgs::Twist cmd_vel;
        generate_trajectory(traj, cmd_vel.linear.x, cmd_vel.angular.z);
        trajectories.push_back(traj);
        visualize_trajectories(trajectories, 0.0, 1.0, 0.0, 1000, candidate_trajectories_pub_);
        visualize_trajectory(traj, 1.0, 0.0, 0.0, best_trajectory_pub_);
        predict_footprint_pub_.publish(transform_footprint(traj.back()));

        return cmd_vel;
    }

    // planing
    geometry_msgs::Twist cmd_vel;
    if(fabs(dynamic_window.max_velocity_) < DBL_EPSILON)
    {
        generate_trajectory(traj, 0.0, 0.0);
        trajectories.push_back(traj);
    }
    else
    {
        double optimal_velocity, optimal_yawrate;
        search_optimal_cmd_vel_for_goal(optimal_velocity, optimal_yawrate, dynamic_window, goal, trajectories);
        search_safety_trajectory(traj, optimal_velocity, optimal_yawrate, dynamic_window, goal);

        cmd_vel.linear.x  = traj.front().velocity_;
        cmd_vel.angular.z = traj.front().yawrate_;
    }

    visualize_trajectories(trajectories, 0.0, 1.0, 0.0, 1000, candidate_trajectories_pub_);
    visualize_trajectory(traj, 1.0, 0.0, 0.0, best_trajectory_pub_);
    predict_footprint_pub_.publish(transform_footprint(traj.back()));

    return cmd_vel;
}


geometry_msgs::Twist PointFollowPlanner::calc_cmd_vel()
{
    const Window dynamic_window = calc_dynamic_window(current_velocity_);
    const Eigen::Vector3d goal(goal_.pose.position.x, goal_.pose.position.y, tf::getYaw(goal_.pose.orientation));

    return planning(dynamic_window, goal);
}


bool PointFollowPlanner::can_move()
{
    if(!goal_subscribed_) ROS_WARN_THROTTLE(1.0, "Local goal has not been updated");
    if(!footprint_subscribed_) ROS_WARN_THROTTLE(1.0, "Footprint has not been updated");
    if(sub_count_th_ < local_map_not_sub_count_) ROS_WARN_THROTTLE(1.0, "Local map has not been updated");
    if(sub_count_th_ < odom_not_sub_count_) ROS_WARN_THROTTLE(1.0, "Odom has not been updated");

    if(!local_map_updated_) local_map_not_sub_count_++;
    if(!odom_updated_) odom_not_sub_count_++;

    if(goal_subscribed_
        and footprint_subscribed_
        and local_map_not_sub_count_ <= sub_count_th_
        and odom_not_sub_count_ <= sub_count_th_)
        return true;
    else
        return false;
}


void PointFollowPlanner::process()
{
    ros::Rate loop_rate(hz_);

    while(ros::ok())
    {
        geometry_msgs::Twist cmd_vel;
        if(can_move()) cmd_vel = calc_cmd_vel();
        cmd_vel_pub_.publish(cmd_vel);
        finish_flag_pub_.publish(has_finished);

        previous_velocity_ = cmd_vel;
        odom_updated_ = false;
        local_map_updated_ = false;
        is_behind_obj_ = false;
        has_finished.data = false;

        ros::spinOnce();
        loop_rate.sleep();
    }
}


void PointFollowPlanner::visualize_trajectory(
        const std::vector<State>& trajectory,
        const double r,
        const double g,
        const double b,
        const ros::Publisher& pub)
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
    v_trajectory.lifetime = ros::Duration();
    v_trajectory.scale.x = 0.05;
    geometry_msgs::Pose pose;
    pose.orientation.w = 1;
    v_trajectory.pose = pose;

    for(const auto& pose : trajectory)
    {
        geometry_msgs::Point point;
        point.x = pose.x_;
        point.y = pose.y_;
        v_trajectory.points.push_back(point);
    }

    pub.publish(v_trajectory);
}


void PointFollowPlanner::visualize_trajectories(
        const std::vector<std::vector<State>>& trajectories,
        const double r,
        const double g,
        const double b,
        const int trajectories_size,
        const ros::Publisher& pub)
{
    visualization_msgs::MarkerArray v_trajectories;
    int count = 0;

    for(; count<trajectories.size(); count++)
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
        v_trajectory.lifetime = ros::Duration();
        v_trajectory.id = count;
        v_trajectory.scale.x = 0.02;
        geometry_msgs::Pose pose;
        pose.orientation.w = 1;
        v_trajectory.pose = pose;
        geometry_msgs::Point p;
        for(const auto& pose : trajectories[count]){
            p.x = pose.x_;
            p.y = pose.y_;
            v_trajectory.points.push_back(p);
        }
        v_trajectories.markers.push_back(v_trajectory);
    }

    for(; count<trajectories_size; count++)
    {
        visualization_msgs::Marker v_trajectory;
        v_trajectory.header.frame_id = robot_frame_;
        v_trajectory.header.stamp = ros::Time::now();
        v_trajectory.ns = pub.getTopic();
        v_trajectory.type = visualization_msgs::Marker::LINE_STRIP;
        v_trajectory.action = visualization_msgs::Marker::DELETE;
        v_trajectory.lifetime = ros::Duration();
        v_trajectory.id = count;
        v_trajectories.markers.push_back(v_trajectory);
    }

    pub.publish(v_trajectories);
}
