#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>

// http://wiki.ros.org/navigation/Tutorials/RobotSetup/TF

/* Assignment:

Compute transformation matrices between frames attached to the pioneer robot.
To this end, you will subscribe to the topic /tf_static and /tf.

/tf_static sends static transformation between various frames attached to
the robot that do not move with respect to each other.

Through rosmsg and rostopic, you can see that it sends an array of
elements with type geometry_msgs/TransformStamped.

rosmsg show tf/tfMessage
rosmsg show tf2_msgs/TFMessage
rostopic echo /tf (gives us frame_id: pioneer/odom, child_frame_id: pioneer/base_link )
rostopic echo /tf_static (gives us frame_id: base_link, child_frame_id: back_sonar/front_sonar)

so I can obtain frame_id: pioneer/odom, child_frame: one of the sonars via
[Odom Base]T * [Base Sonar]T = [Odom Sonar]T

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
[back_sonar front_sonar] = [back_sonar * base] * [base front_sonar], must invert back_sonar's T

PROBLEM 2:
- Computes transformation between frame front_sonar and odom, then
print this into the screen.
(Use tf)

Note that, depending on what you query, the same frame can be called
pioneer/base_link or base_link. This is the same frame. Similarly, odom
is also called as pioneer/odom. These are the same frames.
*/

/*
      frame_id: "base_link"
    child_frame_id: "back_sonar"
    transform: 
      translation: 
        x: 0.109
        y: 0.0
        z: 0.209
      rotation: 
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0

    [base backsonar] = [0.109, 0.0, 0.209]

    quaternion expression for rotation by default
    quaternion can be converted to 3x3 rotation matrices (HOW?)

  - 
    header: 
      seq: 0
      stamp: 
        secs: 0
        nsecs:  41000000
      frame_id: "base_link"
    child_frame_id: "front_sonar"
    transform: 
      translation: 
        x: -0.198
        y: 0.0
        z: 0.208
      rotation: 
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0
    [base frontsonar] = [-0.198, 0.0, 0.208]

      - 
    header: 
      seq: 0
      stamp: 
        secs: 9382
        nsecs: 831000000
      frame_id: "pioneer/odom"
    child_frame_id: "pioneer/base_link"
    [odom base]

    we want [odom base] * [base front] = [odom front] for problem 2
*/

// # GLOBALS #

//for learning purposes
geometry_msgs::PointStamped front_sonar_point;
geometry_msgs::PointStamped back_sonar_point;
geometry_msgs::PointStamped base_point;

//use these for transform matrices math
tf2::Matrix3x3 rotation_base_Front;
tf2::Matrix3x3 rotation_base_Back;
tf2::Matrix3x3 rotation_odom_Base;
tf2::Vector3 translation_base_Front;
tf2::Vector3 translation_base_Back;
tf2::Vector3 translation_odom_Base;

tf2::Matrix3x3 rotationYPR_base_Front;
tf2::Matrix3x3 rotationYPR_base_Back;
tf2::Matrix3x3 rotationYPR_odom_Base;


//geometry_msgs::TransformStamped front_sonar_global;

// # CALLBACKS #

//THIS IS AN APPENDIX/LEGACY FUNCTION LEFT FOR ME TO LOOK BACK TO, NO USE IN THE PROGRAM/LAB ASSIGNMENT
//for learning purposes:
//given a TransformListener, this takes a point in "front_sonar" and transforms it to the "pioneer/odom" frame
//It is then used in that Timer line of code in int main
void transformPoint(const tf::TransformListener& listener)
{
    //create a point in front_sonar frame that we'd like to transform to base_link frame
    //geometry_msgs::PointStamped front_sonar_point;
    front_sonar_point.header.frame_id = "front_sonar";

    //use most recent transform available
    front_sonar_point.header.stamp = ros::Time();

    //arbitrary point in space, USE SUBSCRIBE INFO TO GET REAL VALUES?
    //for front_sonar, we should get:
    //x = -0.198, y = 0.0, z = 0.208
    //front_sonar_point.point.x = 1.0;
    //front_sonar_point.point.y = 0.2;
    //front_sonar_point.point.z = 0.0;

    try
    {
        geometry_msgs::PointStamped base_point;
        listener.transformPoint("base_link", front_sonar_point, base_point);

        //ROS_INFO("\nfront_sonar: (%.2f, %.2f, %.2f) ---> base_link: (%.2f, %.2f, %.2f) at time %.2f", 
        //front_sonar_point.point.x, front_sonar_point.point.y, front_sonar_point.point.z, 
        //base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
    }

    catch(tf::TransformException& ex)
    {
        ROS_ERROR("Received an exception trying to transform a point from \"front_sonar_point\" to \"pioneer\\odom\": %s", ex.what());
    }
}

void callbackTransformation(const tf2_msgs::TFMessage& msg)
{
    // use this for the non-static base_link + odom relationship

    //[parent: pioneer/odom, child: base_link]
    //ROS_INFO_STREAM("\nchild_frame_id: " << msg.transforms[0].child_frame_id);
    //ROS_INFO_STREAM("\ntranslation.x = " << msg.transforms[0].transform.translation.x);
    //ROS_INFO_STREAM("\ntranslation.y = " << msg.transforms[0].transform.translation.y);
    //ROS_INFO_STREAM("\ntranslation.z = " << msg.transforms[0].transform.translation.z);
    base_point.point.x = msg.transforms[0].transform.translation.x;
    base_point.point.y = msg.transforms[0].transform.translation.y;
    base_point.point.z = msg.transforms[0].transform.translation.z;

    rotation_odom_Base.setRotation(tf2::Quaternion(
        msg.transforms[0].transform.rotation.x, 
        msg.transforms[0].transform.rotation.y, 
        msg.transforms[0].transform.rotation.z, 
        msg.transforms[0].transform.rotation.w));

    translation_odom_Base = tf2::Vector3(
        msg.transforms[0].transform.translation.x, 
        msg.transforms[0].transform.translation.y,
        msg.transforms[0].transform.translation.z);

    double roll, pitch, yaw;
    rotation_odom_Base.getEulerYPR(yaw, pitch, roll);
    rotationYPR_odom_Base.setEulerYPR(yaw, pitch, roll);
}

void callbackStaticTransformation(const tf2_msgs::TFMessage& msg)
{
    //[parent: base_link, child: front_sonar]
    //ROS_INFO_STREAM("\nchild_frame_id: " << msg.transforms[1].child_frame_id);
    //ROS_INFO_STREAM("\ntranslation.x = " << msg.transforms[1].transform.translation.x);
    //ROS_INFO_STREAM("\ntranslation.y = " << msg.transforms[1].transform.translation.y);
    //ROS_INFO_STREAM("\ntranslation.z = " << msg.transforms[1].transform.translation.z);
    front_sonar_point.point.x = msg.transforms[1].transform.translation.x;
    front_sonar_point.point.y = msg.transforms[1].transform.translation.y;
    front_sonar_point.point.z = msg.transforms[1].transform.translation.z;

    //[parent: base_link, child: back_sonar]
    //ROS_INFO_STREAM("\nchild_frame_id: " << msg.transforms[0].child_frame_id);
    //ROS_INFO_STREAM("\ntranslation.x = " << msg.transforms[0].transform.translation.x);
    //ROS_INFO_STREAM("\ntranslation.y = " << msg.transforms[0].transform.translation.y);
    //ROS_INFO_STREAM("\ntranslation.z = " << msg.transforms[0].transform.translation.z);
    back_sonar_point.point.x = msg.transforms[0].transform.translation.x;
    back_sonar_point.point.y = msg.transforms[0].transform.translation.y;
    back_sonar_point.point.z = msg.transforms[0].transform.translation.z;

    rotation_base_Front.setRotation(tf2::Quaternion(
        msg.transforms[1].transform.rotation.x, 
        msg.transforms[1].transform.rotation.y, 
        msg.transforms[1].transform.rotation.z, 
        msg.transforms[1].transform.rotation.w));
    
    rotation_base_Back.setRotation(tf2::Quaternion(
        msg.transforms[0].transform.rotation.x, 
        msg.transforms[0].transform.rotation.y, 
        msg.transforms[0].transform.rotation.z, 
        msg.transforms[0].transform.rotation.w));
    
    translation_base_Front = tf2::Vector3(
        msg.transforms[1].transform.translation.x, 
        msg.transforms[1].transform.translation.y,
        msg.transforms[1].transform.translation.z);

    
    translation_base_Back = tf2::Vector3(
        msg.transforms[0].transform.translation.x, 
        msg.transforms[0].transform.translation.y,
        msg.transforms[0].transform.translation.z);

    double roll_baseFront, pitch_baseFront, yaw_baseFront;
    rotation_base_Front.getEulerYPR(yaw_baseFront, pitch_baseFront, roll_baseFront);
    rotationYPR_base_Front.setEulerYPR(yaw_baseFront, pitch_baseFront, roll_baseFront);

    double roll_baseBack, pitch_baseBack, yaw_baseBack;
    rotation_base_Back.getEulerYPR(yaw_baseBack, pitch_baseBack, roll_baseBack);
    rotationYPR_base_Back.setEulerYPR(yaw_baseBack, pitch_baseBack, roll_baseBack);

    //ROS_INFO_STREAM("\ntesting translation vector[0]: " << translation_base_Front[0]); // offsetx
    //ROS_INFO_STREAM("\ntesting translation vector[1]: " << translation_base_Front[1]); // offsety
    //ROS_INFO_STREAM("\ntesting translation vector[2]: " << translation_base_Front[2]); // offsetz

    //ROS_INFO_STREAM("\n\ntesting rotation[0][0]]: " << rotation_base_Front[0][0]); // = 1, xx
    //ROS_INFO_STREAM("\ntesting rotation[0][1]]: " << rotation_base_Front[0][1]); // = 0, xy
    //ROS_INFO_STREAM("\ntesting rotation[2][2]]: " << rotation_base_Front[2][2]); // = 1, zz
}

// # MAIN #
int main(int argc, char ** argv)
{
    ros::init(argc, argv, "transformations");
    ros::NodeHandle nh;

    ros::Subscriber subTrans = nh.subscribe("/tf", 1000, &callbackTransformation);
    ros::Subscriber subTransStatic = nh.subscribe("/tf_static", 1000, &callbackStaticTransformation);

    ros::Rate rate(10);

    tf::TransformBroadcaster broadcaster;

    //LEGACY/APPENDIX CODE
    //automatically subscribes to the transform topic for us
    tf::TransformListener listener(ros::Duration(10));

    //LEGACY/APPENDIX CODE
    //transform a point once every second
    ros::Timer timer = nh.createTimer(ros::Duration(1.0), boost::bind(&transformPoint, boost::ref(listener)));
    
    while(nh.ok())
    {
        broadcaster.sendTransform(
            tf::StampedTransform(
                tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)), 
                ros::Time::now(), "base_link", "front_sonar"));
        rate.sleep();

        ros::spinOnce();
    
        tf2::Transform transform_base_Front(rotation_base_Front, translation_base_Front);
        tf2::Transform transform_odom_Base(rotation_odom_Base, translation_odom_Base);
        tf2::Transform transform_odom_Front = transform_odom_Base * transform_base_Front;
        geometry_msgs::TransformStamped frame_odom_Front;
        frame_odom_Front.header.stamp = ros::Time::now();
        frame_odom_Front.header.frame_id = "base_link";
        frame_odom_Front.child_frame_id = "front_sonar";
        frame_odom_Front.transform = tf2::toMsg(transform_odom_Front);

        /*ROS_INFO_STREAM("\nTRANSFORMATION front_sonar WITH RESPECT TO base_link: ");
        ROS_INFO_STREAM("\ntranslation x: " << frame_odom_Front.transform.translation.x);
        ROS_INFO_STREAM("\ntranslation y: " << frame_odom_Front.transform.translation.y);
        ROS_INFO_STREAM("\ntranslation z: " << frame_odom_Front.transform.translation.z);
        ROS_INFO_STREAM("\nrotation x: " << frame_odom_Front.transform.rotation.x);
        ROS_INFO_STREAM("\nrotation y: " << frame_odom_Front.transform.rotation.y);
        ROS_INFO_STREAM("\nrotation z: " << frame_odom_Front.transform.rotation.z);
        ROS_INFO_STREAM("\nrotation w: " << frame_odom_Front.transform.rotation.w);*/
        // convert quaternion to 3x3 rotation matrix
        tf2::Matrix3x3 rotation_odom_Front;

        rotation_odom_Front.setRotation(tf2::Quaternion(
            frame_odom_Front.transform.rotation.x, 
            frame_odom_Front.transform.rotation.y, 
            frame_odom_Front.transform.rotation.z, 
            frame_odom_Front.transform.rotation.w));
        
        double roll, pitch, yaw;
        rotation_odom_Front.getEulerYPR(yaw, pitch, roll); // rotation_odom_Front still using quat's
        tf2::Matrix3x3 rotationYPR_odom_Front;
        rotationYPR_odom_Front.setEulerYPR(yaw, pitch, roll);
        // 4x4 Matrix of [Odom Front_Sonar] Transformation
        ROS_INFO_STREAM("\n\n4x4 Matrix of \"front_sonar\" with respect to \"odom\": ");
        ROS_INFO_STREAM("\n" << rotationYPR_odom_Front[0][0] << " " << rotationYPR_odom_Front[0][1] << " " << rotationYPR_odom_Front[0][2] << " " << frame_odom_Front.transform.translation.x);
        ROS_INFO_STREAM("\n" << rotationYPR_odom_Front[1][0] << " " << rotationYPR_odom_Front[1][1] << " " << rotationYPR_odom_Front[1][2] << " " << frame_odom_Front.transform.translation.y);
        ROS_INFO_STREAM("\n" << rotationYPR_odom_Front[2][0] << " " << rotationYPR_odom_Front[2][1] << " " << rotationYPR_odom_Front[2][2] << " " << frame_odom_Front.transform.translation.z);
        ROS_INFO_STREAM("\n" << "0      " << " " << "0      " << " " << "0      " << " " << "1      ");


        // [back_sonar front_sonar] = [back_sonar * base] * [base front_sonar], must invert back_sonar's T
        tf2::Transform transform_back_Base(rotation_base_Back, translation_base_Back); // is not yet proper
        transform_back_Base.inverse(); // is now proper
        tf2::Transform transform_back_Front = transform_back_Base * transform_base_Front;
        geometry_msgs::TransformStamped frame_back_Front;
        frame_back_Front.header.stamp = ros::Time::now();
        frame_back_Front.header.frame_id = "back_sonar";
        frame_back_Front.child_frame_id = "front_sonar";
        frame_back_Front.transform = tf2::toMsg(transform_back_Front);

        tf2::Matrix3x3 rotation_back_Front;

        rotation_back_Front.setRotation(tf2::Quaternion(
            frame_back_Front.transform.rotation.x, 
            frame_back_Front.transform.rotation.y, 
            frame_back_Front.transform.rotation.z, 
            frame_back_Front.transform.rotation.w));

        double roll2, pitch2, yaw2;
        rotation_back_Front.getEulerYPR(yaw2, pitch2, roll2);
        tf2::Matrix3x3 rotationYPR_back_Front;
        rotationYPR_back_Front.setEulerYPR(yaw2, pitch2, roll2);
        // 4x4 Matrix of [Back Front] Transformation

        ROS_INFO_STREAM("\n\n4x4 Matrix of \"front_sonar\" with respect to \"back_sonar\": ");
        ROS_INFO_STREAM("\n" << rotationYPR_back_Front[0][0] << " " << rotationYPR_back_Front[0][1] << " " << rotationYPR_back_Front[0][2] << " " << frame_back_Front.transform.translation.x);
        ROS_INFO_STREAM("\n" << rotationYPR_back_Front[1][0] << " " << rotationYPR_back_Front[1][1] << " " << rotationYPR_back_Front[1][2] << " " << frame_back_Front.transform.translation.y);
        ROS_INFO_STREAM("\n" << rotationYPR_back_Front[2][0] << " " << rotationYPR_back_Front[2][1] << " " << rotationYPR_back_Front[2][2] << " " << frame_back_Front.transform.translation.z);
        ROS_INFO_STREAM("\n" << "0" << " " << "0" << " " << "0" << " " << "   1   ");
    }
}