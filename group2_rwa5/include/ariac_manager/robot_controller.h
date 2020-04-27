
#ifndef SRC_ROBOT_CONTROLLER_H
#define SRC_ROBOT_CONTROLLER_H


#include "sensor.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <ros/ros.h>
#include <stdarg.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <string>
#include <initializer_list>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <osrf_gear/VacuumGripperControl.h>
#include <osrf_gear/VacuumGripperState.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

class RobotController{
public:
    RobotController(std::string arm_id_1);
    ~RobotController();
    bool Planner();
    bool Planner2();
    void Execute();
    void Execute2();
    void GoToTarget(std::initializer_list<geometry_msgs::Pose> list, int f);
    void GoToTarget(const geometry_msgs::Pose& pose,int f);
    void SendRobotPosition(std::vector<double> pose);
    void SendRobotPosition2(std::vector<double> pose);
    void SendRobotHome();
    void SendRobotHome2();
    void sendRobotToConveyor();
    bool DropPart(geometry_msgs::Pose pose, int agv_id);
    bool DropPart(geometry_msgs::Pose part_pose);
    void GripperToggle(const bool& state);
    void GripperToggle2(const bool& state);
    void GripperCallback(const osrf_gear::VacuumGripperState::ConstPtr& grip);
    void GripperCallback2(const osrf_gear::VacuumGripperState::ConstPtr& grip);
    void qualityControlSensor1Callback(const osrf_gear::LogicalCameraImage::ConstPtr &);
    void qualityControlSensor2Callback(const osrf_gear::LogicalCameraImage::ConstPtr &);
    void GripperStateCheck(geometry_msgs::Pose pose);
    void GripperStateCheck2(geometry_msgs::Pose pose);
    bool PickPart(geometry_msgs::Pose& part_pose, int agv_id);
    bool PickPart(geometry_msgs::Pose& part_pose);
    bool PickPartconveyor(std::string);


private:
    ros::NodeHandle robot_controller_nh_;
    moveit::planning_interface::MoveGroupInterface::Options robot_controller_options;
    ros::ServiceClient gripper_client_;
    ros::NodeHandle gripper_nh_;
    ros::Subscriber gripper_subscriber_;

    ros::NodeHandle robot_controller_nh_2;
    moveit::planning_interface::MoveGroupInterface::Options robot_controller_options_2;
    ros::ServiceClient gripper_client_2;
    ros::NodeHandle gripper_nh_2;
    ros::Subscriber gripper_subscriber_2;

    AriacSensorManager beam;
    tf::TransformListener robot_tf_listener_;
    tf::StampedTransform robot_tf_transform_;
    tf::TransformListener agv_tf_listener_;
    tf::StampedTransform agv_tf_transform_;
    geometry_msgs::Pose target_pose_;
    geometry_msgs::Pose conveyor_part;

    moveit::planning_interface::MoveGroupInterface robot_move_group_;
    moveit::planning_interface::MoveGroupInterface::Plan robot_planner_;
    osrf_gear::VacuumGripperControl gripper_service_;
    osrf_gear::VacuumGripperState gripper_status_;

    moveit::planning_interface::MoveGroupInterface robot_move_group_2;
    moveit::planning_interface::MoveGroupInterface::Plan robot_planner_2;
    osrf_gear::VacuumGripperControl gripper_service_2;
    osrf_gear::VacuumGripperState gripper_status_2;

    std::string object;
    bool plan_success_;
    bool plan_success_2;
    std::vector<double> home_joint_pose_;
    std::vector<double> home_joint_pose_1;
    std::vector<double> bin_drop_pose_;
    std::vector<double> kit_drop_pose_;
    std::vector<double> belt_drop_pose_;
    std::vector<double> drop_part;
    std::vector<double> conveyor;
    float time=0.6;

    std::vector<double> home_joint_pose_2;
    std::vector<double> bin_drop_pose_2;
    std::vector<double> kit_drop_pose_2;
    std::vector<double> belt_drop_pose_2;
    std::vector<double> conveyor2;

    geometry_msgs::Pose home_cart_pose_;
    geometry_msgs::Quaternion fixed_orientation_;
    geometry_msgs::Quaternion fixed_end_orientation_;
    geometry_msgs::Quaternion conveyor_fixed_orientation_;
    geometry_msgs::Quaternion temp_orientation_;
    geometry_msgs::Pose agv_position_;
    geometry_msgs::Pose robot_pose_;
    std::vector<double> end_position_;
    double offset_;
    double roll_def_,pitch_def_,yaw_def_;
    tf::Quaternion q;
    int counter_;
    bool gripper_state_, drop_flag_,drop,pick,is_faulty_, gripper_state_2;
    ros::Subscriber quality_control_camera_subscriber_, quality_control_camera_2_subscriber_;
    geometry_msgs::Pose final_;
    geometry_msgs::Pose grab_pose_;
    geometry_msgs::Pose place_pose_;
    std::map<std::string, double> temp_pose_;

};
#endif //SRC_ROBOT_CONTROLLER_H
