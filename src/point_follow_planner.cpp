#include "point_follow_planner/point_follow_planner.h"
#include "jsk_recognition_msgs/PolygonArray.h"

PointFollowPlanner::PointFollowPlanner(void)
    :private_nh_("~"), local_goal_subscribed_(false), robot_footprint_subscribed_(false), odom_updated_(false), local_map_updated_(false)
{
    private_nh_.param("hz", hz_, {10});
    private_nh_.param("robot_frame", robot_frame_, {"base_link"});
    private_nh_.param("max_velocity", max_velocity_, {1.0});
    private_nh_.param("min_velocity", min_velocity_, {0.0});
    private_nh_.param("max_yawrate", max_yawrate_, {0.8});
    private_nh_.param("max_acceleration", max_acceleration_, {1.0});
    private_nh_.param("max_d_yawrate", max_d_yawrate_, {2.0});
    private_nh_.param("max_dist", max_dist_, {10.0});
    private_nh_.param("velocity_resolution", velocity_resolution_, {0.1});
    private_nh_.param("yawrate_resolution", yawrate_resolution_, {0.1});
    private_nh_.param("angle_resolution", angle_resolution_, {0.2});
    private_nh_.param("predict_time", predict_time_, {3.0});

    dt_ = 1.0 / hz_;

    ROS_INFO("=== Point Followe Planner ===");
    ROS_INFO_STREAM("hz: " << hz_);
    ROS_INFO_STREAM("dt: " << dt_);
    ROS_INFO_STREAM("robot_frame: " << robot_frame_);
    ROS_INFO_STREAM("max_velocity: " << max_velocity_);
    ROS_INFO_STREAM("min_velocity: " << min_velocity_);
    ROS_INFO_STREAM("max_yawrate: " << max_yawrate_);
    ROS_INFO_STREAM("max_acceleration: " << max_acceleration_);
    ROS_INFO_STREAM("max_d_yawrate: " << max_d_yawrate_);
    ROS_INFO_STREAM("max_dist: " << max_dist_);
    ROS_INFO_STREAM("velocity_resolution: " << velocity_resolution_);
    ROS_INFO_STREAM("yawrate_resolution: " << yawrate_resolution_);
    ROS_INFO_STREAM("angle_resolution: " << angle_resolution_);
    ROS_INFO_STREAM("predict_time: " << predict_time_);

    velocity_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    candidate_trajectories_pub_ = private_nh_.advertise<visualization_msgs::MarkerArray>("candidate_trajectories", 1);
    selected_trajectory_pub_ = private_nh_.advertise<visualization_msgs::Marker>("selected_trajectory", 1);
    robot_footprints_pub_ = private_nh_.advertise<jsk_recognition_msgs::PolygonArray>("robot_footprints", 1);

    local_goal_sub_ = nh_.subscribe("/local_goal", 1, &PointFollowPlanner::local_goal_callback, this);
    robot_footprint_sub_ = nh_.subscribe("/robot_footprint", 1, &PointFollowPlanner::robot_footprint_callback, this);
    local_map_sub_ = nh_.subscribe("/local_map", 1, &PointFollowPlanner::local_map_callback, this);
    odom_sub_ = nh_.subscribe("/odom", 1, &PointFollowPlanner::odom_callback, this);
}


PointFollowPlanner::State::State(const double x, const double y, const double yaw, const double velocity, const double yawrate)
    :x_(x), y_(y), yaw_(yaw), velocity_(velocity), yawrate_(yawrate)
{
}


PointFollowPlanner::Window::Window(void)
    :min_velocity_(0.0), max_velocity_(0.0), min_yawrate_(0.0), max_yawrate_(0.0)
{
}


PointFollowPlanner::Window::Window(const double min_v, const double max_v, const double min_y, const double max_y)
    :min_velocity_(min_v), max_velocity_(max_v), min_yawrate_(min_y), max_yawrate_(max_y)
{
}


void PointFollowPlanner::local_goal_callback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    local_goal_ = *msg;
    try
    {
        listener_.transformPose(robot_frame_, ros::Time(0), local_goal_, local_goal_.header.frame_id, local_goal_);
        local_goal_subscribed_ = true;
    }
    catch(tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
    }
}


void PointFollowPlanner::robot_footprint_callback(const geometry_msgs::PolygonStampedPtr& msg)
{
    robot_footprint_ = *msg;
    robot_footprint_subscribed_ = true;
}


void PointFollowPlanner::local_map_callback(const nav_msgs::OccupancyGridConstPtr& msg)
{
    raycast(*msg);
    local_map_updated_ = true;
}


void PointFollowPlanner::odom_callback(const nav_msgs::OdometryConstPtr& msg)
{
    current_velocity_ = msg->twist.twist;
    odom_updated_ = true;
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
                if(map.data[index_x + index_y*map.info.width] == 100)
                    obs_list_.poses.push_back(pose);
        }
    }
}


PointFollowPlanner::Window PointFollowPlanner::calc_dynamic_window(const geometry_msgs::Twist& current_velocity)
{
    Window window(min_velocity_, max_velocity_, -max_yawrate_, max_yawrate_);
    window.min_velocity_ = std::max((current_velocity.linear.x - max_acceleration_*dt_), min_velocity_);
    window.max_velocity_ = std::min((current_velocity.linear.x + max_acceleration_*dt_), max_velocity_);
    window.min_yawrate_  = std::max((current_velocity.angular.z - max_d_yawrate_*dt_), -max_yawrate_);
    window.max_yawrate_  = std::min((current_velocity.angular.z + max_d_yawrate_*dt_),  max_yawrate_);

    return window;
}


double PointFollowPlanner::calc_goal_cost(const std::vector<State>& traj, const Eigen::Vector3d& goal)
{
    Eigen::Vector3d last_position(traj.back().x_, traj.back().y_, traj.back().yaw_);
    return (last_position.segment(0, 2) - goal.segment(0, 2)).norm();
}


geometry_msgs::PolygonStamped PointFollowPlanner::move_footprint(const State& target_pose)
{
    geometry_msgs::PolygonStamped robot_footprint = robot_footprint_;
    robot_footprint.header.stamp = ros::Time::now();
    for(auto& point : robot_footprint.polygon.points)
    {
        Eigen::VectorXf point_in(2);
        point_in << point.x, point.y;
        Eigen::Matrix2f rot;
        rot = Eigen::Rotation2Df(target_pose.yaw_);
        const Eigen::VectorXf point_out = rot * point_in;

        point.x = point_out.x() + target_pose.x_;
        point.y = point_out.y() + target_pose.y_;
    }
    return robot_footprint;
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
    const geometry_msgs::PolygonStamped robot_footprint = move_footprint(state);

    geometry_msgs::Point32 state_point;
    state_point.x = state.x_;
    state_point.y = state.y_;

    for(int i=0; i<robot_footprint.polygon.points.size(); i++)
    {
        geometry_msgs::Polygon triangle;
        triangle.points.push_back(state_point);
        triangle.points.push_back(robot_footprint.polygon.points[i]);

        if(i != robot_footprint.polygon.points.size()-1)
            triangle.points.push_back(robot_footprint.polygon.points[i+1]);
        else
            triangle.points.push_back(robot_footprint.polygon.points[0]);

        if(is_inside_of_triangle(obstacle.position, triangle))
            return true;
    }

    return false;
}


bool PointFollowPlanner::is_hit_obstacle(const std::vector<State>& traj)
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


geometry_msgs::Twist PointFollowPlanner::planning(const Window dynamic_window, const Eigen::Vector3d goal)
{
    float min_cost = 1e6;
    std::vector<std::vector<State>> trajectories;
    double optimal_yawrate;

    // search optimal yawrate
    for(double velocity=dynamic_window.max_velocity_; dynamic_window.min_velocity_<=velocity; velocity-=velocity_resolution_){
        for(double yawrate=dynamic_window.max_yawrate_; dynamic_window.min_yawrate_<=yawrate; yawrate-=yawrate_resolution_){
            State state(0.0, 0.0, 0.0, current_velocity_.linear.x, current_velocity_.angular.z);
            std::vector<State> traj;

            // predict robot motion
            for(float t=0; t<=predict_time_; t+=dt_)
            {
                motion(state, velocity, yawrate);
                traj.push_back(state);
            }
            trajectories.push_back(traj);

            // calc goal cost
            const double goal_cost = calc_goal_cost(traj, goal);

            // update min cost & optimal yawrate
            if(goal_cost <= min_cost)
            {
                min_cost  = goal_cost;
                optimal_yawrate = yawrate;
            }
        }
    }

    // search safety trajectory
    std::vector<State> optimal_traj;
    bool is_found_safety_traj = false;
    for(double velocity=dynamic_window.max_velocity_; dynamic_window.min_velocity_<=velocity; velocity-=velocity_resolution_){
        State state(0.0, 0.0, 0.0, current_velocity_.linear.x, current_velocity_.angular.z);
        std::vector<State> traj;

        // predict robot motion
        for(float t=0; t<=predict_time_; t+=dt_)
        {
            motion(state, velocity, optimal_yawrate);
            traj.push_back(state);
        }

        // judge safety trajectory
        if(!is_hit_obstacle(traj))
        {
            optimal_traj = traj;
            is_found_safety_traj = true;
            break;
        }
    }

    if(!is_found_safety_traj)
    {
        std::vector<State> traj;
        State state(0.0, 0.0, 0.0, 0.0, optimal_yawrate);
        traj.push_back(state);
        optimal_traj = traj;
    }

    visualize_trajectories(trajectories, 0.0, 1.0, 0.0, 1000, candidate_trajectories_pub_);
    visualize_trajectory(optimal_traj, 1.0, 0.0, 0.0, selected_trajectory_pub_);

    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x  = optimal_traj[0].velocity_;
    cmd_vel.angular.z = optimal_traj[0].yawrate_;

    return cmd_vel;
}


geometry_msgs::Twist PointFollowPlanner::calc_cmd_vel()
{
    const Window dynamic_window = calc_dynamic_window(current_velocity_);
    const Eigen::Vector3d goal(local_goal_.pose.position.x, local_goal_.pose.position.y, tf::getYaw(local_goal_.pose.orientation));

    return planning(dynamic_window, goal);
}


bool PointFollowPlanner::can_move()
{
    if(!local_goal_subscribed_) ROS_ERROR_THROTTLE(1.0, "Local goal has not been updated");
    if(!robot_footprint_subscribed_) ROS_ERROR_THROTTLE(1.0, "Robot footprint has not been updated");
    if(!local_map_updated_) ROS_ERROR_THROTTLE(1.0, "Local map has not been updated");
    if(!odom_updated_) ROS_ERROR_THROTTLE(1.0, "Odom has not been updated");

    if(local_goal_subscribed_
        and robot_footprint_subscribed_
        and local_map_updated_
        and odom_updated_)
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
        velocity_pub_.publish(cmd_vel);

        odom_updated_ = false;
        local_map_updated_ = false;
        ros::spinOnce();
        loop_rate.sleep();
    }
}


void PointFollowPlanner::visualize_trajectory(const std::vector<State>& trajectory, const double r, const double g, const double b, const ros::Publisher& pub)
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


void PointFollowPlanner::visualize_trajectories(const std::vector<std::vector<State>>& trajectories, const double r, const double g, const double b, const int trajectories_size, const ros::Publisher& pub)
{
    visualization_msgs::MarkerArray v_trajectories;

    for(int count = 0; count<trajectories.size(); count++)
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

    for(int count = 0; count<trajectories_size; count++)
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

void PointFollowPlanner::visualize_footprints(const std::vector<State>& trajectory, const ros::Publisher& pub)
{
    jsk_recognition_msgs::PolygonArray robot_footprints;
    robot_footprints.header.frame_id = robot_frame_;
    robot_footprints.header.stamp = ros::Time::now();

    for (const auto& state : trajectory)
        robot_footprints.polygons.push_back(move_footprint(state));

    robot_footprints_pub_.publish(robot_footprints);
}
