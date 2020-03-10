#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h> //detective'd through rostopic list and rostopic info, data type


/*
ASSIGNMENT:
write a node moverobot that moves the robot by publishing to the topic
/pioneer/cmd_vel. The node should move the robot forward for 1 second
at (1 m/s), then rotate clockwise for 1 second (at 0.5 rad/sec) and then
repeat. The source code for the node must be stored in a file called
moverobot.cpp
*/

int main(int argc, char **argv)
{
    //skeleton based off 2.2: Chatter Node

    //ros::init must always be the first ROS fn called
    //default name shall be moverobot, put in quotes
    ros::init(argc, argv, "moverobot");
    ros::NodeHandle nh; // when instantiated, starts the node

    // use ros::Publisher objects to publish messages to a topic
    // data type for moving robot is geometry_msgs/Twist??? 2.8
    // publishes to topic named /pioneer/cmd_vel
    ros::Publisher pubMove = nh.advertise<geometry_msgs::Twist>("/pioneer/cmd_vel", 1000);
    ros::Publisher pubRot = nh.advertise<geometry_msgs::Twist>("/pioneer/cmd_vel", 1000);

    // object Rate is used for running loops at a given frequency
    // Here, our frequency is 1 second forward, 1 second rotate, repeat
    // 1 hz = 1 cycle per second
    ros::Rate rate(1);
    float moveValue = 1; // we want 1 m/s
    float rotateValue = 0.5; // we want 0.5 rad/sec
    
    geometry_msgs::Twist moveRobot;
    moveRobot.linear.x = moveValue;
    moveRobot.angular.x = 0;

    geometry_msgs::Twist rotateRobot;
    rotateRobot.linear.x = 0;
    rotateRobot.angular.x = rotateValue;

    // understanding how to use geometry_msgs data type
    // https://stackoverflow.com/questions/43515772/subscribing-and-publishing-geometry-twist-messages-from-turtlesim

    while (ros::ok())
    {
        pubMove.publish(moveRobot);
        rate.sleep();
        pubRot.publish(rotateRobot);
        rate.sleep();
    }
}