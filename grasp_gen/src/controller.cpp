#include <ros/ros.h>
// Services
#include "moveit_planner/MovePose.h"
#include "moveit_planner/MoveCart.h"
#include "franka_gripper_gazebo/GripMsg.h"
#include "franka_pos_grasping_gazebo/GripPos.h"
#include "moveit_planner/MoveJoint.h"
#include "moveit_planner/SetVelocity.h"
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include "geometry_msgs/Pose.h"
geometry_msgs::PointStamped robot_point;
void wait_for_service(int32_t timeout){
    ros::service::waitForService("move_to_pose",timeout);
    ros::service::waitForService("gripPosServer", timeout);
    ros::service::waitForService("move_to_joint_space", timeout);
    ros::service::waitForService("set_velocity_scaling", timeout);
    ROS_INFO("SERVICES READY");
}

void transformPoint(const tf::TransformListener& listener){
  //we'll create a point in the base_laser frame that we'd like to transform to the base_link frame
  geometry_msgs::PointStamped obj_point;
  obj_point.header.frame_id = "object_frame";

  //we'll just use the most recent transform available for our simple example
  obj_point.header.stamp = ros::Time();

  //just an arbitrary point in space
  obj_point.point.x = 0.0610282234;
  obj_point.point.y = 0.1253089268;
  obj_point.point.z = -0.0507819287;

  try{

    listener.transformPoint("panda_link0", obj_point, robot_point);

    ROS_INFO("object_frame: (%.2f, %.2f. %.2f) -----> robot_frame: (%.2f, %.2f, %.2f) at time %.2f",
        obj_point.point.x, obj_point.point.y, obj_point.point.z,
        robot_point.point.x, robot_point.point.y, robot_point.point.z, robot_point.header.stamp.toSec());
  }
  catch(tf::TransformException& ex){
    ROS_ERROR("Received an exception trying to transform a point from \"object_frame\" to \"panda_link0\": %s", ex.what());
  }
}

int main(int argc, char **argv){

    ros::init(argc, argv, "control");
    ros::NodeHandle n;

    //Waiting for services to be available
    int32_t timeout = 1000;
    wait_for_service(timeout);

    // Transform Listener

    tf::TransformListener listener(ros::Duration(10));

    ros::Timer timer = n.createTimer(ros::Duration(1.0), boost::bind(&transformPoint, boost::ref(listener)));

    //Declaring Clients

    ros::ServiceClient movePoseClient = n.serviceClient<moveit_planner::MovePose>("move_to_pose");
    ros::ServiceClient moveCartClient = n.serviceClient<moveit_planner::MoveCart>("cartesian_move");
    ros::ServiceClient gripperClient = n.serviceClient<franka_gripper_gazebo::GripMsg>("gazebo_franka_grip");
    ros::ServiceClient gripperPosClient = n.serviceClient<franka_pos_grasping_gazebo::GripPos>("gripPosServer");
    ros::ServiceClient jointSpaceClient = n.serviceClient<moveit_planner::MoveJoint>("move_to_joint_space");
    ros::ServiceClient velScalingClient = n.serviceClient<moveit_planner::SetVelocity>("set_velocity_scaling");

    // Client Objects

    moveit_planner::MovePose pose;
    moveit_planner::MoveCart cart;
    moveit_planner::MoveCart cart2;
    moveit_planner::MoveCart cart3;
    franka_gripper_gazebo::GripMsg grip;
    franka_pos_grasping_gazebo::GripPos grasp;
    moveit_planner::MoveJoint jpos;
    moveit_planner::SetVelocity velscale;

  std::cout<<robot_point.point.x;

    grip.request.force = 10;
    gripperClient.call(grip);

    pose.request.val.position.x =  0.65; //robot_point.point.x; //0.65;
    pose.request.val.position.y = 0.0;// robot_point.point.y; //0.0;
    pose.request.val.position.z = 0.444;//robot_point.point.z; //0.444;
    pose.request.val.orientation.x = 0.9491722;//0;//0.0582049;
    pose.request.val.orientation.y = 0.2966163;//1;//0.3604694;
    pose.request.val.orientation.z = 0;//-0.8861846;
    pose.request.val.orientation.w = -0.1053135;// 0.2852208;
    pose.request.execute = true;
    movePoseClient.call(pose);

    ros::Duration(0.5).sleep();
    geometry_msgs::Pose p;
    p.position = pose.request.val.position;
    p.orientation = pose.request.val.orientation;
    p.position.z -= 0.085;
    cart.request.execute =true;
    cart.request.val.push_back(p);
    // cart.request.val.push_back(p);
    // cart.request.val.push_back(p);
    // cart.request.val.push_back()
    moveCartClient.call(cart);
    ros::Duration(0.7).sleep();

    grip.request.force = -50;
    gripperClient.call(grip);

    ros::Duration(0.5).sleep();

    p.position = pose.request.val.position;
    p.position.z -= 0.085;
    p.orientation = pose.request.val.orientation;
    p.position.z += 0.25;
    cart.request.execute =true;
    cart.request.val.push_back(p);
    moveCartClient.call(cart);


    ros::spin();

    return 0;
}
