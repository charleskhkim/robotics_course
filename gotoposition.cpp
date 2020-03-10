#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <math.h> // atan
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/tf.h>
#include <stdlib.h> //abs

/*
ASSIGNMENT:
write a node that moves the robot to a arget pose, i.e. a desired position and orientation. For simplicity,
define three constants X, Y, THETA at the top of your file (X and Y to be expressed in meters, THETA in radians)
and let that be your target location. The following lines must be in your node to declare the global variable
(in the beginning of the file, right after the #includes but before any written code)
*/







// ##########################################################
// # GLOBALS ################################################
// ##########################################################

// TARGET LOCATION; this is where we want the robot to go/its waypoint
float x = 5.5;
float y = -6;
float THETA = -0.8; // Must be a value -1 > THETA > 1

float marginOfErrorDistance = 0.1;
float marginOfErrorTHETA = 0.05;

geometry_msgs::Pose robotOdom; // this stores robot's current information

//ros::Publisher rpy_publisher;
ros::Subscriber quat_subscriber;

const float PI = 3.141592653;

// STRATEGY:
// subscribe to gain CURRENT info of robot's position (requires use of globals)
// // let's use odom instead of base_pose_ground_truth, because the latter can be inaccurate in practice
// publish to send info to make the robot turn
// while loop that makes robot keep turning until reaching desired angle
// while loop that runs until robot meets target location
// AFTER reaching location, turn the robot until it matches the desired THETA

// because conditions cannot be perfect, we will want margins of error when calibrating
// position and angle for the while loops

// #############
// # CALLBACKS #
// #############

/*void callbackOdom(const nav_msgs::Odometry& msg)
{
    //copied from drift.cpp

    //# SHOWS COORDINATES
    //ROS_INFO_STREAM("Current /pioneer/odom coordinates.\nx: " << msg.pose.pose.position.x << 
    //"\ny: " << msg.pose.pose.position.y << "\nangle: " << msg.pose.pose.orientation.x);

    // now actually transfer the msg's position onto robotOdom so we can use and calc it in main()
    robotOdom.position.x = msg.pose.pose.position.x; //notice how robotDom doesnt need to write out pose.pose. because of data type
    robotOdom.position.y = msg.pose.pose.position.y;
}*/


//https://answers.ros.org/question/11545/plotprint-rpy-from-quaternion/#17106
//http://www.theconstructsim.com/treating-quaternions-2d-navigation/
//https://answers.ros.org/question/50113/transform-quaternion/
//source code and references for quaternion to rpy conversion
void callbackOdom(const nav_msgs::Odometry& msg)
{
    robotOdom.position.x = msg.pose.pose.position.x; //notice how robotDom doesnt need to write out pose.pose. because of data type
    robotOdom.position.y = msg.pose.pose.position.y;

    // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
    tf::Quaternion quat(
        msg.pose.pose.orientation.x,
         msg.pose.pose.orientation.y,
          msg.pose.pose.orientation.z,
           msg.pose.pose.orientation.w);
    //tf::quaternionMsgToTF(msg, quat);

    // the tf::Quaternion has a method to acess roll pitch and yaw
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    // the found angles are written in a geometry_msgs::Vector3
    /*geometry_msgs::Vector3 rpy;
    rpy.x = roll;
    rpy.y = pitch;
    rpy.z = yaw;
    */

    robotOdom.orientation.z = yaw / PI;

    // this Vector is then published:
    //rpy_publisher.publish(rpy);
    //ROS_INFO_STREAM("\nrpy.z: " << rpy.z);
    ROS_INFO_STREAM("\nrobot Angle: " << robotOdom.orientation.z);
}

// #############
// # FUNCTIONS #
// #############

// robot rotates until facing towards desired waypoint
void seekDesiredCoordinate(ros::Publisher pubRot, geometry_msgs::Twist rotateRobot,
 float rotateValue, float desiredX, float desiredY, bool &targetAcquired, ros::Rate rate)
{
    // time to google math formulas I have long forgotten
    // given two coordinates (x,y) & (x2,y2), find the angle that the robot at (x,y) will need to turn
    // in order to face (x2,y2)

    // slope m = (y2 - y1) / (x2 - x1)
    //float m = (desiredY - robotOdom.position.y) / (desiredX - robotOdom.position.x);
    //float targetOrientation = atan2(m) / (PI/2) ; //this is a THETA, returned in -1 < THETA < 1 format
    //arctan only returns positive angles, this means sometimes the robot will face the opposite direction intended
    //and then zoom off into infinity
    
    //atan2 does the job for us though, thanks google!
    //https://math.stackexchange.com/questions/1201337/finding-the-angle-between-two-points
    //atan(y, x)
    float calcX = desiredX - robotOdom.position.x;
    float calcY = desiredY - robotOdom.position.y;
    float targetOrientation = atan2(calcY, calcX) / (PI);
    //this is the hardest part of coding this program...
    //robot's full 360 goes from 0 to 1 to 0, THEN it goes from -0 to -1 to -0...
    //(X,Y)
    //(1, 0) = 0
    //(0, 1) = 0.5
    //(-1, 0) = 1
    //(0, -1) = 0.5 <--- this is the problem, the robot will probably be going towards (0,1 instead)

    //(-1.5, 1) = 0.8
    //(-1.5, -1) = 0.8 instead of -0.8
    //(1.5, -1) = 0.2 instead of -0.2
    if(calcX >= 0 && calcX <= 0)
    {
        targetOrientation *= -1;
    }

    geometry_msgs::Twist rotateRobotCW;
    rotateRobotCW.angular.z = rotateValue * -1;

    geometry_msgs::Twist robotStopRotate;
    robotStopRotate.angular.z = 0;

    //ROS_INFO_STREAM("\nDEBUG I: Currently in seekDesiredCoordinate, THETA = " << targetOrientation << "\nrobotOri = " << robotOdom.orientation.z);

    while(robotOdom.orientation.z > targetOrientation + marginOfErrorTHETA 
    || robotOdom.orientation.z < targetOrientation - marginOfErrorTHETA) //spin until facing target
    {
        ros::spinOnce();
        /*pubRot.publish(rotateRobot);
        ROS_INFO_STREAM("\nSpinning counter-clockwise.\nRobot angle = " << robotOdom.orientation.z << "\nTarget angle = " << targetOrientation);
        rate.sleep();*/

        if(fabs(robotOdom.orientation.z) > fabs(targetOrientation))
        {
            pubRot.publish(rotateRobot);
            ROS_INFO_STREAM("\nSpinning counter-clockwise.\nRobot angle = " << robotOdom.orientation.z << "\nTarget angle = " << targetOrientation);
            rate.sleep();
        }
        else
        {
            pubRot.publish(rotateRobotCW);
            ROS_INFO_STREAM("\nSpinning clockwise.\nRobot angle = " << robotOdom.orientation.z << "\nTarget angle = " << targetOrientation);
            rate.sleep();
        }
    }

    if(robotOdom.orientation.z < targetOrientation + marginOfErrorTHETA &&
     robotOdom.orientation.z > targetOrientation - marginOfErrorTHETA)
    {
        ros::spinOnce();
        targetAcquired = true;
        pubRot.publish(robotStopRotate);
        ROS_INFO_STREAM("\nTarget acquired.\nRobot angle = " << robotOdom.orientation.z << "\nTarget angle = " << targetOrientation);
    }

    ROS_INFO_STREAM("\nSTATUS:\nRobot - x: " << robotOdom.position.x << ", y: " << robotOdom.position.y << 
    "\nTarget - x: " << x << ", y: " << y);    

    rate.sleep();
}

//checks current position to see if robot has reached waypoint
void checkCurrentPos(float desiredX, float desiredY, bool &waypointAchieved)
{
    ROS_INFO_STREAM("\nChecking current position");
    if((robotOdom.position.x <= desiredX + marginOfErrorDistance && robotOdom.position.x >= desiredX - marginOfErrorDistance)
     && (robotOdom.position.y <= desiredY + marginOfErrorDistance && robotOdom.position.y >= desiredY - marginOfErrorDistance))
     {
        waypointAchieved = true;
        ROS_INFO_STREAM("\nReached waypoint.\nRobot - x: " << robotOdom.position.x << ", y: " << robotOdom.position.y << 
        "\nTarget - x: " << x << ", y: " << y);
     }
}



// ########
// # MAIN #
// ########

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gotoposition");
    ros::NodeHandle nh;

    // use ros::Publisher objects to publish messages to a topic
    // data type for moving robot is geometry_msgs/Twist??? 2.8
    // publishes to topic named /pioneer/cmd_vel
    ros::Publisher pubMove = nh.advertise<geometry_msgs::Twist>("/pioneer/cmd_vel", 1000);
    ros::Publisher pubRot = nh.advertise<geometry_msgs::Twist>("/pioneer/cmd_vel", 1000);
    //rpy_publisher = nh.advertise<geometry_msgs::Vector3>("rpy_angles", 1000);

    //ros::Subscriber subOdom = nh.subscribe("/pioneer/odom", 1000, &callbackOdom);
    quat_subscriber = nh.subscribe("/pioneer/odom", 1000, &callbackOdom);

    // rate(1) = 1 hz = 1 cycle per second
    ros::Rate rate(10);
    ros::Rate slowerRate(2);
    float moveValue = 1; // # m/s
    float rotateValue = 0.5; // # rad/sec
    
    geometry_msgs::Twist moveRobot; // move robot linearly, no turning
    moveRobot.linear.x = moveValue;
    moveRobot.angular.z = 0;

    geometry_msgs::Twist rotateRobot;
    rotateRobot.linear.x = 0;
    rotateRobot.angular.z = rotateValue;

    geometry_msgs::Twist stopRobot;
    stopRobot.linear.x = 0;
    stopRobot.angular.z = 0;

    bool waypointAchieved = false; // we have arrived at our waypoint! but it may not necessarily be oriented properly
    bool targetAcquired = false; // robot has turned towards the waypoint.
    bool desiredPosAndAngleAchieved = false; // we are both AT the waypoint AND turned to the correct angle

    while (ros::ok() && waypointAchieved == false)
    {
        // keep spinning robot until it is facing towards target
        ros::spinOnce(); // updates intel on our robot's position and angle by using subscribers and the callback I wrote up there
        seekDesiredCoordinate(pubRot, rotateRobot, rotateValue, x, y, targetAcquired, rate);

        //ROS_INFO_STREAM("DEBUG II: testing if seekDesireCoordinate is ever left");

        // accelerate straight
        ros::spinOnce();
        checkCurrentPos(x, y, waypointAchieved);
        if(targetAcquired == true && waypointAchieved == false)
        {
            //ROS_INFO_STREAM("DEBUG III: Am I moving forward?");
            pubMove.publish(moveRobot);
            slowerRate.sleep();
        }
        pubMove.publish(stopRobot);
        ros::spinOnce();
        ROS_INFO_STREAM("\nProceedin'... Robot - x: " << robotOdom.position.x << ", y: " << robotOdom.position.y << 
        "\nTarget - x: " << x << ", y: " << y);
        targetAcquired = false;
        rate.sleep();
    }

    // robot should be at correct position
    while(ros::ok() && desiredPosAndAngleAchieved == false)
    {
        ros::spinOnce();
        pubMove.publish(rotateRobot);
        rate.sleep();
        if(robotOdom.orientation.z <= THETA + marginOfErrorTHETA && robotOdom.orientation.z >= THETA - marginOfErrorTHETA)
        {
        pubMove.publish(stopRobot);
        desiredPosAndAngleAchieved = true;

        ROS_INFO_STREAM("\nFINAL REPORT:\nRobot - x: " << robotOdom.position.x << ", y: " << robotOdom.position.y << 
        ", angle: " << robotOdom.orientation.z << "\nTarget - x: " << x << ", y: " << y << ", angle: " << THETA);
        }
    }
}