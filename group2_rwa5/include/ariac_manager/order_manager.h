#pragma once

#include <list>
#include <map>
#include <string>
#include <iostream>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <osrf_gear/LogicalCameraImage.h>
#include <osrf_gear/Order.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <osrf_gear/LogicalCameraImage.h>
#include "sensor.h"
#include "robot_controller.h"

class AriacOrderManager {
public:
    AriacOrderManager();
    ~AriacOrderManager();
    void OrderCallback(const osrf_gear::Order::ConstPtr& order_msg);
    void ExecuteOrder();
    std::string GetProductFrame(std::string product_type);
    std::map<std::string, std::list<std::pair<std::string,geometry_msgs::Pose>>> GetOrder();
    bool PickAndPlace(std::pair<std::string,geometry_msgs::Pose> product_type_pose,int agv_id);
    bool PickAndPlace(std::pair<std::string,geometry_msgs::Pose> product_type_pose, std::string empty_bin, int agv_id);
    void SubmitAGV(int num);
    bool checkOrderUpdate(int,int,std::string, int agv_id);
    void dropallparts(std::vector<std::pair<std::string,geometry_msgs::Pose>>, int agv_id);
    void OutOfReach(std::string arm, std::string num, std::pair<std::string,geometry_msgs::Pose> product, int agv_id);
    std::vector<std::string> GetProductType();
    std::vector<geometry_msgs::Pose> GetProductPose();
    std::vector<std::string> productlist_type;
    std::vector<geometry_msgs::Pose> productlist_pose;
    std::string bin1_part;
    std::string bin2_part;
    std::string bin3_part;
    std::string bin4_part;
    std::string bin5_part;
    std::string bin6_part;
    std::vector<std::string> bin_parts;
    std::string empty_bin;
    std::vector<double> flipped_drop_pose_, flipped_arm1_pose_1,
                        flipped_arm2_pose_1, flipped_arm1_pose_2,
                        flipped_arm2_pose_2, flipped_arm1_pose_3,
                        flipped_arm1_pose_4, flipped_arm1_pose_5,
                        flipped_arm2_pose_3, flipped_arm1_pose_6,
                        flipped_arm2_pose_4, kit_drop_pose_;
    std::vector<double> out_arm2_pose1, out_arm2_pose2, out_arm2_pose3,
                        out_arm1_pose1, out_arm1_pose2;
    void FlippedPart(int agv_id, auto pose);
    double roll, pitch, yaw;
    tf::Quaternion q;
    bool isFlipped = false;
    bool isReachable = true;

private:
    ros::NodeHandle order_manager_nh_;
    ros::Subscriber order_subscriber_;
    bool conveyor_part_found ;

    std::vector<osrf_gear::Order> received_orders_;
    AriacSensorManager camera_;
    geometry_msgs::Pose bin_pose;
    RobotController arm1_, arm2_;
    // RobotController arm2_;
    tf::TransformListener part_tf_listener_;
    std::vector<double> drop_pose_;
    std::pair<std::string,geometry_msgs::Pose> product_type_pose_;
    std::string object;
    std::map<std::string, std::vector<std::string>> product_frame_list_;
    osrf_gear::Order order_;
    std::vector<std::pair<std::string,geometry_msgs::Pose>> placed_parts;
    std::vector<double> home_joint_pose_1;
    std::vector<double> home_joint_pose_2;

    int piston_count_ = 0;
    int gear_count_ = 0;
    int pulley_count_ = 0;

};
