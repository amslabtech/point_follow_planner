#include <ros/ros.h>
#include <geometry_msgs/PolygonStamped.h>


class RobotFootprintPublisher
{
public:
    RobotFootprintPublisher(void);
    void process();

protected:
    void set_corners_of_rectangle();
    void visualize_footprint();

    int hz_;
    std::string robot_frame_;

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Publisher  footprint_pub_;
    geometry_msgs::PolygonStamped robot_footprint_;

    // each side length from origin of robot_frame
    // (for Rectangular Robot)
    double front_side_distance_;
    double rear_side_distance_;
    double right_side_distance_;
    double left_side_distance_;
};


RobotFootprintPublisher::RobotFootprintPublisher():private_nh_("~")
{
    // param
    private_nh_.param("hz", hz_, 5);
    private_nh_.param("robot_frame", robot_frame_, {"base_link"});
    private_nh_.param("front_side_distance", front_side_distance_, 0.5);
    private_nh_.param("rear_side_distance", rear_side_distance_, 0.5);
    private_nh_.param("right_side_distance", right_side_distance_, 0.5);
    private_nh_.param("left_side_distance", left_side_distance_, 0.5);

    // publisher
    footprint_pub_ = nh_.advertise<geometry_msgs::PolygonStamped>("robot_footprint", 1);

    // frame id
    robot_footprint_.header.frame_id = robot_frame_;

    set_corners_of_rectangle();


    ROS_INFO_STREAM("=== Robot Footprint Publisher ===");
    ROS_INFO_STREAM("hz: " << hz_);
    ROS_INFO_STREAM("robot_frame: " << robot_frame_);
    ROS_INFO_STREAM("front_side_distance: " << front_side_distance_);
    ROS_INFO_STREAM("rear_side_distance: " << rear_side_distance_);
    ROS_INFO_STREAM("right_side_distance: " <<  right_side_distance_);
    ROS_INFO_STREAM("left_side_distance: " << left_side_distance_);
}


void RobotFootprintPublisher::set_corners_of_rectangle()
{
    geometry_msgs::Point32 point;

    // right front
    point.x =  front_side_distance_;
    point.y = -right_side_distance_;
    robot_footprint_.polygon.points.push_back(point);

    // right rear
    point.x = -rear_side_distance_;
    robot_footprint_.polygon.points.push_back(point);

    // left rear
    point.y = left_side_distance_;
    robot_footprint_.polygon.points.push_back(point);

    // left front
    point.x = front_side_distance_;
    robot_footprint_.polygon.points.push_back(point);
}


void RobotFootprintPublisher::visualize_footprint()
{
    robot_footprint_.header.stamp = ros::Time::now();
    footprint_pub_.publish(robot_footprint_);
}


void RobotFootprintPublisher::process()
{
    ros::Rate loop_rate(hz_);

    while(ros::ok())
    {
        visualize_footprint();
        ros::spinOnce();
        loop_rate.sleep();
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_footprint_publisher");
    RobotFootprintPublisher robot_footprint_publisher;
    robot_footprint_publisher.process();

    return 0;
}
