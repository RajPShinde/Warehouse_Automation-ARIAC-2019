//
// Created by zeid on 2/27/20.
//

#pragma once


#include <list>
#include <map>
#include <string>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <osrf_gear/Proximity.h>
#include <tf/transform_listener.h>
#include <osrf_gear/LogicalCameraImage.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


#include "ariac_part_manager.h"

class GetObject{
public:
    int object = 0;
    bool arm1_engage = false;
    bool arm2_engage = false;
};

class AriacSensorManager {
public:
    AriacSensorManager();
    ~AriacSensorManager();
    void LogicalCamera1Callback(const osrf_gear::LogicalCameraImage::ConstPtr&);
    void LogicalCamera2Callback(const osrf_gear::LogicalCameraImage::ConstPtr&);
    void LogicalCamera3Callback(const osrf_gear::LogicalCameraImage::ConstPtr&);
    void LogicalCamera4Callback(const osrf_gear::LogicalCameraImage::ConstPtr&);
    void LogicalCamera5Callback(const osrf_gear::LogicalCameraImage::ConstPtr&);
    void LogicalCamera6Callback(const osrf_gear::LogicalCameraImage::ConstPtr&);
    void LogicalCamera7Callback(const osrf_gear::LogicalCameraImage::ConstPtr&);
    void break_beam_callback_(const osrf_gear::Proximity::ConstPtr &);
    void break_beam_callback_2(const osrf_gear::Proximity::ConstPtr &);
    bool getBeam();
    bool init1, init2, init3 , init4, init5, init6, init7;
    std::vector<std::string> order_parts;
    std::vector<std::string> belt_parts_lcam;
    int counter{0};
    
    bool gripper_state_1;
    int break_beam_counter = 0;
    bool grab_now_1 = false;

    int object_derived = 0;
    bool arm1_engage_derived = false;
    bool arm2_engage_derived = false;
    
    float roll_grab, pitch_grab, yaw_grab;
    float x_grab, y_grab, z_grab;
//    bool IsPartFaulty() const;
//    void qualityControlSensor1Callback(const osrf_gear::LogicalCameraImage::ConstPtr &image_msg);

    // std::map<std::string, std::vector<std::string>> product_frame_list_;


    geometry_msgs::Pose GetPartPose(const std::string& src_frame,
                                    const std::string& target_frame);
    std::map<std::string, std::vector<std::string>> get_product_frame_list(){
        return product_frame_list_;
    }
    //void ScanParts(int cam_number);
    void BuildProductFrames(int camera_id);
    std::string getpart();
    std::vector<std::string> belt_parts;
    std::string LogicalCamera1PartType();
    std::string LogicalCamera2PartType();
    std::string LogicalCamera3PartType();
    std::string LogicalCamera4PartType();
    std::string LogicalCamera5PartType();
    std::string LogicalCamera6PartType();
    geometry_msgs::Pose BinGetPartPose(const std::string& src_frame,
                                                           std::string product_type, std::string num);

    std::string cam1_part_type;
    std::string cam2_part_type;
    std::string cam3_part_type;
    std::string cam4_part_type;
    std::string cam5_part_type;
    std::string cam6_part_type;
    std::string cam7_part_type;

private:
    ros::NodeHandle sensor_nh_;
    ros::Subscriber camera_1_subscriber_;
    ros::Subscriber camera_2_subscriber_;
    ros::Subscriber camera_3_subscriber_;
    ros::Subscriber camera_4_subscriber_;
    ros::Subscriber camera_5_subscriber_;
    ros::Subscriber camera_6_subscriber_;
    ros::Subscriber camera_7_subscriber_;
    ros::Subscriber break_beam_subscriber_2_;
    ros::Subscriber break_beam_subscriber_;

//    ros::Subscriber quality_control_camera_subscriber_;


    tf::TransformListener camera_tf_listener_;
    tf::StampedTransform camera_tf_transform_;

    osrf_gear::LogicalCameraImage current_parts_1_;
    osrf_gear::LogicalCameraImage current_parts_2_;
    osrf_gear::LogicalCameraImage current_parts_3_;
    osrf_gear::LogicalCameraImage current_parts_4_;
    osrf_gear::LogicalCameraImage current_parts_5_;
    osrf_gear::LogicalCameraImage current_parts_6_;
    osrf_gear::LogicalCameraImage current_parts_7_;

    std::map<std::string, std::vector<geometry_msgs::Pose>> part_list_;
    std::vector<AriacPartManager> camera1_part_list,camera2_part_list,camera3_part_list,camera4_part_list,camera5_part_list,camera6_part_list,camera7_part_list;

    //std::map<std::string, std::list<std::string>> parts_list_;
    std::map<std::string, std::vector<std::string>> product_frame_list_;

    bool init1_,init2_,init3_,init4_,init5_,init6_,init7_, cam_1_, cam_2_,cam_3_,cam_4_,cam_5_,cam_6_,cam_7_,beam_, is_faulty_;
    std::string logical1_,logical2_,logical3_,logical4_,logical5_,logical6_,logical7_;
    int camera1_frame_counter_, camera2_frame_counter_, camera3_frame_counter_, camera4_frame_counter_,camera5_frame_counter_,camera6_frame_counter_,camera7_frame_counter_,break_beam_counter_,prev1,prev2,prev3,prev4,prev5,prev6,prev7,f;
};
