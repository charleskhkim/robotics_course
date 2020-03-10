#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h> //notice this is needed
#include <nav_msgs/Odometry.h> //as is this data type


/*
ASSIGNMENT:
write a node drift that subscribes to the topics /pioneer/odom and
/base_pose_ground_truth and computes the distance between the pose
returned by the two topics. Once the difference is computed, the
node publishes it to a topic called /posedrift. For pose, you can
ignore the z coordinate. The pose of the robot is returned in
the position.(x,y,z) portion of the message. The source code for
the node must be stored in a file called drift.cpp.
*/

//just like in the listener node example, we want to obtain data from the topics
//pioneer/odom and baseposegroundtruth, which transmits x y z values
//and refer to & of these when we make our subscribers for this drift.cpp Node
//
//rostopic echo shows that these topics odom and groundtruth are constantly
//emitting string messages showing the positions, this means there is data to extract
//and use in main()

//https://gist.github.com/PrieureDeSion/77c109a074573ce9d13da244e5f82c4d#file-sim-cpp-L65
//examine how he uses sensors to obtain data and writes sub/pub data accordingly

//using globals so we can obtain and store data in the callbacks, then use them in main()
geometry_msgs::Pose robotOdom;
geometry_msgs::Pose robotBase;
std_msgs::Float32 finalDestination;


//https://robocodingwithros.wordpress.com/2016/07/06/a-c-code-to-listen-odometry-messages-from-a-pioneer-p3at-robot-using-ros-subscriber/
//shows this similar concept as well
//http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom
void callbackOdom(const nav_msgs::Odometry& msg)
{
    //rostopic echo /pioneer/odom and /base_pose_ground_truth
    //shows us twist: twist: linear and angular's x y and z values in real-time
    //and pose: pose: position (!!!) and orientation

    ROS_INFO_STREAM("Received /pioneer/odom coordinates.\nx: " << msg.pose.pose.position.x << 
    "\ny: " << msg.pose.pose.position.y << "\nz: " << msg.pose.pose.position.z);

    // now actually transfer the msg's position onto robotOdom so we can use and calc it in main()
    robotOdom.position.x = msg.pose.pose.position.x; //notice how robotDom doesnt need to write out pose.pose. because of data type
    robotOdom.position.y = msg.pose.pose.position.y;
    robotOdom.position.z = msg.pose.pose.position.z;
}

void callbackBase(const nav_msgs::Odometry &msg)
{
    ROS_INFO_STREAM("Received /base_pose_ground_truth coordinates.\nx: " << msg.pose.pose.position.x << 
    "\ny: " << msg.pose.pose.position.y << " \nz: " << msg.pose.pose.position.z);

    robotBase.position.x = msg.pose.pose.position.x;
    robotBase.position.y = msg.pose.pose.position.y;
    robotBase.position.z = msg.pose.pose.position.z;
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "drift");
    ros::NodeHandle nh;

    //rostopic echo /pioneer/odom and /base_pose_ground_truth
    //shows us twist: twist: linear and angular's x y and z values in real-time
    //and pose: pose: position (!!!) and orientation

    ros::Subscriber subOdom = nh.subscribe("/pioneer/odom", 1000, &callbackOdom);
    ros::Subscriber subBasePoseGroundTruth = nh.subscribe("/base_pose_ground_truth", 1000, &callbackBase);


    ros::Publisher pubDist = nh.advertise<std_msgs::Float32>("/posedrift", 1000);

    //while ros ok
    while(ros::ok())
    {
        ros::spinOnce();

        //distance = sqrt of (x1-x2)^2 + (y1-y2)^2
        float squaredX = pow((robotOdom.position.x - robotBase.position.x), 2); // (x1 - x2)^2
        float squaredY = pow((robotOdom.position.y - robotBase.position.y), 2); // (y1 - y2)^2
        // we are ignoring Z values for distance computation, code it in here if needed
        float distance = sqrt(squaredX + squaredY); // pub this ofc

        finalDestination.data = distance;

        pubDist.publish(finalDestination);
        
        //odom msg/info //float
        ROS_INFO_STREAM("Distance between: " << distance);
    }
}