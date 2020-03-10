#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/tf.h>

// http://wiki.ros.org/navigation/Tutorials/RobotSetup/TF

/* Assignment:

Compute transformation matrices between frames attached to the pioneer robot.
To this end, you will subscribe to the topic /tf_static and /tf.

/tf_static sends static transformation between various frames attached to
the robot that do not move with respect to each other.

Through rosmsg and rostopic, you can see that it sends an array of
elements with type geometry_msgs/TransformStamped.

rosmsg show tf/tfMessage


Each transform includes two strings -  frame_id and child_frame_id -
as well as a transformation given in terms of translation and rotation
(represented in quaternion).

For [AB]T, frame_id is A and child_frame_id is B. This is the transformation
matrix that represents frame B with respect to frame A.

Write our node such that:

PROBLEM 1:
- Computes transformation matrix between frames front_sonar and back_sonar
(that is, the transformation matrix describing front_sonar in the back_sonar
frame). After computation, it should print to screen as a 4x4 matrix.
(Use tf static I presume)

PROBLEM 2:
- Computes transformation between frame front_sonar and odom, then
print this into the screen.
(Use tf)

Note that, depending on what you query, the same frame can be called
pioneer/base_link or base_link. This is the same frame. Similarly, odom
is also called as pioneer/odom. These are the same frames.
*/

// # GLOBALS #

// # CALLBACKS #

//given a TransformListener, this takes a point in "front_sonar" and transforms it to the "pioneer/odom" frame
//It is then used in that Timer line of code in int main
void transformPoint(const tf::TransformListener& listener)
{
    //create a point in front_sonar frame that we'd like to transform to odom frame
    geometry_msgs::PointStamped sonar_point;
    sonar_point.header.frame_id = "front_sonar";

    //use most recent transform available
    sonar_point.header.stamp = ros::Time();

    //arbitrary point in space, USE SUBSCRIBE INFO TO GET REAL VALUES?
    sonar_point.point.x = 1.0;
    sonar_point.point.y = 0.2;
    sonar_point.point.z = 0.0;

    try
    {
        geometry_msgs::PointStamped odom_point;
        listener.transformPoint("pioneer/odom", sonar_point, odom_point);

        ROS_INFO("front_sonar: (%.2f, %.2f, %.2f) ---> pioneer/odom: (%.2f, %.2f, %.2f) at time %.2f", 
        sonar_point.point.x, sonar_point.point.y, sonar_point.point.z, 
        odom_point.point.x, odom_point.point.y, odom_point.point.z, odom_point.header.stamp.toSec());
    }

    catch(tf::TransformException& ex)
    {
        ROS_ERROR("Received an exception trying to transform a point from \"sonar_point\" to \"pioneer\\odom\": %s", ex.what());
    }
}

void callbackTransformation(const tf2_msgs::TFMessage& msg)
{

}

// # MAIN #
int main(int argc, char ** argv)
{
    ros::init(argc, argv, "transformations");
    ros::NodeHandle nh;

    ros::Subscriber subTrans = nh.subscribe("/tf", 1000, &callbackTransformation);
    ros::Subscriber subTransStatic = nh.subscribe("/tf_static", 1000, &callbackTransformation);

    ros::Rate rate(10);

    tf::TransformBroadcaster broadcaster;
    //automatically subscribes to the transform topic for us
    tf::TransformListener listener(ros::Duration(10));

    //transform a point once every second
    ros::Timer timer = nh.createTimer(ros::Duration(1.0), boost::bind(&transformPoint, boost::ref(listener)));

    //let's try PROBLEM 2 first
    //pioneer/odom as parent, front_sonar as child
    while(nh.ok())
    {
        broadcaster.sendTransform(
            tf::StampedTransform(
                tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)), 
                ros::Time::now(), "pioneer/odom", "front_sonar"));
        rate.sleep();

        ros::spinOnce();
    }
}