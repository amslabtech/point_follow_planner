#include "point_follow_planner/point_follow_planner.h"

class RobotOutlineCreator
{
public:
    RobotOutlineCreator(void);
    void process();

protected:
    void set_corners_of_rectangle();
    void visualize_outline();

    int hz_;
    std::string robot_frame_;

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Publisher  outline_pub_;
    geometry_msgs::PolygonStamped robot_outline_;

    // each side length from origin of robot_frame
    // (for Rectangular Robot)
    double front_side_distance_;
    double rear_side_distance_;
    double right_side_distance_;
    double left_side_distance_;
};


RobotOutlineCreator::RobotOutlineCreator():private_nh_("~")
{
    // param
    private_nh_.param("hz", hz_, 5);
    private_nh_.param("robot_frame", robot_frame_, {"base_link"});
    private_nh_.param("front_side_distance", front_side_distance_, 0.5);
    private_nh_.param("rear_side_distance", rear_side_distance_, 0.5);
    private_nh_.param("right_side_distance", right_side_distance_, 0.5);
    private_nh_.param("left_side_distance", left_side_distance_, 0.5);

    // publisher
    outline_pub_ = nh_.advertise<geometry_msgs::PolygonStamped>("robot_outline", 1);

    // frame id
    robot_outline_.header.frame_id = robot_frame_;

    set_corners_of_rectangle();


    ROS_INFO_STREAM("=== Robot Outline Creator ===");
    ROS_INFO_STREAM("hz: " << hz_);
    ROS_INFO_STREAM("robot_frame: " << robot_frame_);
    ROS_INFO_STREAM("front_side_distance: " << front_side_distance_);
    ROS_INFO_STREAM("rear_side_distance: " << rear_side_distance_);
    ROS_INFO_STREAM("right_side_distance: " <<  right_side_distance_);
    ROS_INFO_STREAM("left_side_distance: " << left_side_distance_);
}


void RobotOutlineCreator::set_corners_of_rectangle()
{
    geometry_msgs::Point32 point;

    // right front
    point.x =  front_side_distance_;
    point.y = -right_side_distance_;
    robot_outline_.polygon.points.push_back(point);

    // right rear
    point.x = -rear_side_distance_;
    robot_outline_.polygon.points.push_back(point);

    // left rear
    point.y = left_side_distance_;
    robot_outline_.polygon.points.push_back(point);

    // left front
    point.x = front_side_distance_;
    robot_outline_.polygon.points.push_back(point);
}


void RobotOutlineCreator::visualize_outline()
{
    robot_outline_.header.stamp = ros::Time::now();
    outline_pub_.publish(robot_outline_);
}


void RobotOutlineCreator::process()
{
    ros::Rate loop_rate(hz_);

    while(ros::ok())
    {
        visualize_outline();
        ros::spinOnce();
        loop_rate.sleep();
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_outline_creator");
    RobotOutlineCreator robot_outline_creator;
    robot_outline_creator.process();

    return 0;
}
