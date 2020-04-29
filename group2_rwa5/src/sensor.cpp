//
// Created by zeid on 2/27/20.
//
#include "sensor.h"

GetObject ObjectOnBelt;

bool grab_now_1 = false;

AriacSensorManager::AriacSensorManager():
camera1_part_list{},
camera2_part_list{},
camera3_part_list{}{
    ROS_INFO_STREAM(">>>>> Subscribing to logical sensors");
    prev1=0;
    prev2=0;
    prev3=0;
    prev4=0;
    prev5=0;
    prev6=0;
    camera_1_subscriber_ = sensor_nh_.subscribe("/ariac/logical_camera_1", 10,
                                                &AriacSensorManager::LogicalCamera1Callback, this);
    camera_2_subscriber_ = sensor_nh_.subscribe("/ariac/logical_camera_2", 10,
                                                &AriacSensorManager::LogicalCamera2Callback, this);
    camera_3_subscriber_ = sensor_nh_.subscribe("/ariac/logical_camera_3", 10,
                                                &AriacSensorManager::LogicalCamera3Callback, this);
    camera_4_subscriber_ = sensor_nh_.subscribe("/ariac/logical_camera_4", 10,
                                                &AriacSensorManager::LogicalCamera4Callback, this);
    camera_5_subscriber_ = sensor_nh_.subscribe("/ariac/logical_camera_5", 10,
                                                &AriacSensorManager::LogicalCamera5Callback, this);
    camera_6_subscriber_ = sensor_nh_.subscribe("/ariac/logical_camera_6", 10,
                                                &AriacSensorManager::LogicalCamera6Callback, this);
    camera_7_subscriber_ = sensor_nh_.subscribe("/ariac/logical_camera_7", 10,
                                                &AriacSensorManager::LogicalCamera7Callback, this);
    camera_8_subscriber_ = sensor_nh_.subscribe("/ariac/logical_camera_8", 10,
                                                &AriacSensorManager::LogicalCamera8Callback, this);
    break_beam_subscriber_1_ = sensor_nh_.subscribe("/ariac/break_beam_1_change", 10, 
                                                &AriacSensorManager::break_beam_callback_1, this);
    break_beam_subscriber_2_ = sensor_nh_.subscribe("/ariac/break_beam_2_change", 10, 
                                                &AriacSensorManager::break_beam_callback_2, this);

//    quality_control_camera_subscriber_ = sensor_nh_.subscribe("/ariac/quality_control_sensor_1", 10,
//    		&AriacSensorManager::qualityControlSensor1Callback, this);

    camera1_frame_counter_ = 1;
    camera2_frame_counter_ = 1;
    camera3_frame_counter_ = 1;
    camera4_frame_counter_ = 1;
    camera5_frame_counter_ = 1;
    camera6_frame_counter_ = 1;
    camera7_frame_counter_ = 1;

    init1_ = false;
    init2_ = false;
    init3_ = false;
    init4_ = false;
    init5_ = false;
    init6_ = false;
    init7_ = false;

    init1 = false;
    init2 = false;
    init3 = false;
    init4 = false;
    init5 = false;
    init6 = false;
    init7 = false;

    cam_1_ = false;
    cam_2_ = false;
    cam_3_ = false;
    cam_4_ = false;
    cam_5_ = false;
    cam_6_ = false;
    cam_7_ = false;


    is_faulty_ = false;
}

AriacSensorManager::~AriacSensorManager() {}

//void AriacSensorManager::qualityControlSensor1Callback(const osrf_gear::LogicalCameraImage::ConstPtr &image_msg) {
//	is_faulty_ = !image_msg->models.empty(); // if not empty then part is faulty
//}

//bool AriacSensorManager::IsPartFaulty() const{
//	return is_faulty_;
//}

void AriacSensorManager::LogicalCamera1Callback(const osrf_gear::LogicalCameraImage::ConstPtr& image_msg){
    if (init1) return;
    count_bin1 = image_msg->models.size();
    if (image_msg->models.size() == 0) {
        // ROS_ERROR_STREAM("Logical Camera 1 does not see anything");
        init1 = true;
    }
    else{
        if(image_msg->models.size() !=prev1){
            prev1=image_msg->models.size();
            ROS_INFO_STREAM_THROTTLE(10,
                             "Logical camera 1: '" << image_msg->models.size() << "' objects.");
            current_parts_1_ = *image_msg;
            this->BuildProductFrames(1);
            init1 = true;
        }
    }
}


void AriacSensorManager::LogicalCamera2Callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
    if (init2) return;
    count_bin2 = image_msg->models.size();
    if (image_msg->models.size() == 0){
        // ROS_ERROR_STREAM("Logical Camera 2 does not see anything");
        init2 = true;
    }
   else{
        if(image_msg->models.size() !=prev2){
            prev2=image_msg->models.size();
            ROS_INFO_STREAM_THROTTLE(10,
                             "Logical camera 2: '" << image_msg->models.size() << "' objects.");
            current_parts_2_ = *image_msg;
            this->BuildProductFrames(2);
            init2 = true;
        }
    }
}

void AriacSensorManager::LogicalCamera3Callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
    if (init3) return;
    count_bin3 = image_msg->models.size();
    if (image_msg->models.size() == 0){
        // ROS_ERROR_STREAM("Logical Camera 3 does not see anything");
        init3 = true;
    }
    else {
        if(image_msg->models.size() !=prev3){
            prev3=image_msg->models.size();
            ROS_INFO_STREAM_THROTTLE(10,
                             "Logical camera 3: '" << image_msg->models.size() << "' objects.");
            current_parts_3_ = *image_msg;
            this->BuildProductFrames(3);
            init3 = true;
        }
    }
}


void AriacSensorManager::LogicalCamera4Callback(const osrf_gear::LogicalCameraImage::ConstPtr& image_msg){
    if (init4) return;
    count_bin4 = image_msg->models.size();
    ROS_INFO_STREAM_THROTTLE(2,
                     "Logical camera 4: '" << image_msg->models.size() << "' objects.");

    if (image_msg->models.size() == 0) {
        // ROS_ERROR_STREAM("Logical Camera 4 does not see anything");
        init4 = true;
    }
    else {
        if(image_msg->models.size() !=prev4){
            prev4=image_msg->models.size();
            ROS_INFO_STREAM_THROTTLE(2,
                             "Logical camera 4: '" << image_msg->models.size() << "' objects.");
            current_parts_4_ = *image_msg;
            this->BuildProductFrames(4);
            init4 = true;
        }
    }
    // std::string type = image_msg->models.type;
    // ROS_INFO_STREAM_THROTTLE(4,
    //                  "Logical camera 4: '" << type << "' objects.");
}

void AriacSensorManager::LogicalCamera5Callback(const osrf_gear::LogicalCameraImage::ConstPtr& image_msg){
    if (init5) return;
     count_bin5 = image_msg->models.size(); 
    if (image_msg->models.size() == 0) {
        // ROS_ERROR_STREAM("Logical Camera 5 does not see anything");
        init5 = true;
    }
    else {
        if(image_msg->models.size() !=prev5){
            prev5=image_msg->models.size();
            ROS_INFO_STREAM_THROTTLE(10,
                             "Logical camera 5: '" << image_msg->models.size() << "' objects.");
            current_parts_5_ = *image_msg;
            this->BuildProductFrames(5);
            init5 = true;
        }
    }
}

void AriacSensorManager::LogicalCamera6Callback(const osrf_gear::LogicalCameraImage::ConstPtr& image_msg){
    if (init6) return;
    count_bin6 = image_msg->models.size();
    if (image_msg->models.size() == 0) {
        // ROS_ERROR_STREAM("Logical Camera 6 does not see anything");
        init6 = true;
    }
    else {
        if(image_msg->models.size() !=prev6){
            prev6=image_msg->models.size();
            ROS_INFO_STREAM_THROTTLE(10,
                             "Logical camera 6: '" << image_msg->models.size() << "' objects.");
            current_parts_6_ = *image_msg;
            this->BuildProductFrames(6);
            init6 = true;
        }
    }
}

void AriacSensorManager::LogicalCamera7Callback(const osrf_gear::LogicalCameraImage::ConstPtr& image_msg){
    // if (init6) return;
    // count_bin6 = image_msg->models.size();
    auto imageMessage = *image_msg ;
    if (image_msg->models.size() == 0) {
        // ROS_INFO_STREAM_THROTTLE(3, "Logical Camera 7 does not see anything");
        init7 = true;
    }
    else if(image_msg->models.size() != 0){
        logicalcam_7 = imageMessage.models[0].type;
    }    
}

void AriacSensorManager::LogicalCamera8Callback(const osrf_gear::LogicalCameraImage::ConstPtr& image_msg){
    // if (init6) return;
    // count_bin6 = image_msg->models.size();
    auto imageMessage = *image_msg ;
    if (image_msg->models.size() == 0) {
        // ROS_INFO_STREAM_THROTTLE(3, "Logical Camera 8 does not see anything");
        init8 = true;
    }
    else if(image_msg->models.size() != 0){
        logicalcam_8 = imageMessage.models[0].type;
    }    
}

void AriacSensorManager::break_beam_callback_1(const osrf_gear::Proximity::ConstPtr &msg) {
    if (msg->object_detected) {
        ROS_INFO("Break beam 1 triggered.");
        break_beam_counter_1_+= 1;
        beam_1_=true;
     }
     else{
        beam_1_=false;
     }
}

void AriacSensorManager::break_beam_callback_2(const osrf_gear::Proximity::ConstPtr & msg) {
    if (msg->object_detected) {
        ROS_INFO("Break beam 2 triggered.");
        break_beam_counter_2_ += 1;
        grab_now_2 = true;
        beam_2_ = true;
     }
     else{
        beam_2_ = false;
     }
}

void AriacSensorManager::BuildProductFrames(int camera_id){
    if (camera_id == 1 and current_parts_1_.models.size() != 0) {
        for (auto& msg : current_parts_1_.models) {
            //--build the frame for each product
            std::string product_frame = "logical_camera_1_" + msg.type + "_" +
                                        std::to_string(camera1_frame_counter_) + "_frame";
            logical1_=msg.type;
            product_frame_list_[msg.type].emplace_back(product_frame);
            camera1_frame_counter_++;
            // prev1=camera1_frame_counter_;
        }
        cam_1_ = true;
        init1_=true;
    }
    else if (camera_id == 2 and current_parts_2_.models.size() != 0) {
        for (auto& msg : current_parts_2_.models) {
            //--build the frame for each product
            std::string product_frame = "logical_camera_2_" + msg.type + "_" +
                                        std::to_string(camera2_frame_counter_) + "_frame";
            logical2_=msg.type;
            product_frame_list_[msg.type].emplace_back(product_frame);
            camera2_frame_counter_++;
            // prev2=camera2_frame_counter_;

        }
        cam_2_ = true;
        init2_=true;
    }
    else if (camera_id == 3 and current_parts_3_.models.size() != 0) {
        for (auto& msg : current_parts_3_.models) {
            //--build the frame for each product
            std::string product_frame = "logical_camera_3_" + msg.type + "_" +
                                        std::to_string(camera3_frame_counter_) + "_frame";
            logical3_=msg.type;
            product_frame_list_[msg.type].emplace_back(product_frame);
            camera3_frame_counter_++;
            // prev3=camera3_frame_counter_;

        }
        cam_3_ = true;
        init3_=true;
    }
    else if (camera_id == 4 and current_parts_4_.models.size() != 0) {
    for (auto& msg : current_parts_4_.models) {
            //--build the frame for each product
            std::string product_frame = "logical_camera_4_" + msg.type + "_" +
                                        std::to_string(camera4_frame_counter_) + "_frame";
            logical4_=msg.type;
            product_frame_list_[msg.type].emplace_back(product_frame);
            camera4_frame_counter_++;
            // prev3=camera3_frame_counter_;

        }
        cam_4_ = true;
        init4_=true;
    }
    else if (camera_id == 5 and current_parts_5_.models.size() != 0) {
    for (auto& msg : current_parts_5_.models) {
            //--build the frame for each product
            std::string product_frame = "logical_camera_5_" + msg.type + "_" +
                                        std::to_string(camera5_frame_counter_) + "_frame";
            logical5_=msg.type;
            product_frame_list_[msg.type].emplace_back(product_frame);
            camera5_frame_counter_++;
            // prev3=camera3_frame_counter_;

        }
        cam_5_ = true;
        init5_=true;
    }
    else if (camera_id == 6 and current_parts_6_.models.size() != 0) {
    for (auto& msg : current_parts_6_.models) {
            //--build the frame for each product
            std::string product_frame = "logical_camera_6_" + msg.type + "_" +
                                        std::to_string(camera6_frame_counter_) + "_frame";
            logical6_=msg.type;
            product_frame_list_[msg.type].emplace_back(product_frame);
            camera6_frame_counter_++;
            // prev3=camera3_frame_counter_;

        }
        cam_6_ = true;
        init6_=true;
    }
    // else if (camera_id == 7 and current_parts_7_.models.size() != 0) {
    // for (auto& msg : current_parts_7_.models) {
    //         //--build the frame for each product
    //         std::string product_frame = "logical_camera_7_" + msg.type + "_" +
    //                                     std::to_string(camera7_frame_counter_) + "_frame";
    //         logical7_=msg.type;
    //         product_frame_list_[msg.type].emplace_back(product_frame);
    //         camera7_frame_counter_++;
    //         // prev3=camera3_frame_counter_;

    //     }
    //     cam_7_ = true;
    //     init7_=true;
    // }

}


geometry_msgs::Pose AriacSensorManager::GetPartPose(const std::string& src_frame,
                                        const std::string& target_frame) {
    geometry_msgs::Pose part_pose;

    ROS_INFO_STREAM("Getting part pose...");

    if (true) {
        camera_tf_listener_.waitForTransform(src_frame, target_frame, ros::Time(0),
                                             ros::Duration(3));
        camera_tf_listener_.lookupTransform(src_frame, target_frame, ros::Time(0),
                                            camera_tf_transform_);

        part_pose.position.x = camera_tf_transform_.getOrigin().x();
        part_pose.position.y = camera_tf_transform_.getOrigin().y();
        part_pose.position.z = camera_tf_transform_.getOrigin().z();

    } else {
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        // this->BuildProductFrames(1);
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        // this->BuildProductFrames(6);

        part_pose = this->GetPartPose(src_frame, target_frame);
    }

    return part_pose;
}

// void AriacSensorManager::break_beam_callback_(const osrf_gear::Proximity::ConstPtr &msg) {
//     if (msg->object_detected) {
//         ROS_INFO("Break beam 2 triggered.");
// break_beam_counter_ += 1;
//         beam_=true;
//      }
//      else{
//         beam_=false;
//      }
// }

bool AriacSensorManager::getBeam() {
 return beam_1_,beam_2_;
 }

 std::string AriacSensorManager::getpart() {
 return logical3_;
 }

std::string AriacSensorManager::LogicalCamera1PartType(){
    // if (init1_) return;
    if(logical1_ == ""){
        cam1_part_type = 'n';
    }
    else{
        cam1_part_type = logical1_;
    }
    
    return cam1_part_type;
 }

std::string AriacSensorManager::LogicalCamera2PartType(){
    // if (init1_) return;
    if(logical2_ == ""){
        cam2_part_type = 'n';
    }
    else{
        cam2_part_type = logical2_;
    }
    
    return cam2_part_type;
 }
std::string AriacSensorManager::LogicalCamera3PartType(){
    // if (init1_) return;
    if(logical3_ == ""){
        cam3_part_type = 'n';
    }
    else{
        cam3_part_type = logical3_;
    }
    
    return cam3_part_type;
 }

std::string AriacSensorManager::LogicalCamera4PartType(){
    // if (init1_) return;
    if(logical4_ == ""){
        cam4_part_type = 'n';
    }
    else{
        cam4_part_type = logical4_;
    }
    
    return cam4_part_type;
 }

std::string AriacSensorManager::LogicalCamera5PartType(){
    // if (init1_) return;
    if(logical5_ == ""){
        cam5_part_type = 'n';
    }
    else{
        cam5_part_type = logical5_;
    }
    
    return cam5_part_type;
 }

std::string AriacSensorManager::LogicalCamera6PartType(){
    // if (init1_) return;
    if(logical6_ == ""){
        cam6_part_type = 'n';
    }
    else{
        cam6_part_type = logical6_;
    }
    
    return cam6_part_type;
 }

 std::string AriacSensorManager::BeltCamera1Part(){
    // ROS_INFO_STREAM_THROTTLE(1, "LOGICAL7_ = " << logical7_);
    if(!logical7_.empty()){
        cam7_part_type = logical7_;
        // ROS_INFO_STREAM("Conveyor camera sees: " << cam7_part_type);
        all_parts_belt.push_back(cam7_part_type);
    }
    else{
        cam7_part_type = 'n';
        // ROS_INFO_STREAM_THROTTLE(2,"Logical Camera 7 does not see anything");
    }
    
    return cam7_part_type;
 }

 std::string AriacSensorManager::BeltCamera2Part(){
    // ROS_INFO_STREAM_THROTTLE(1, "LOGICAL7_ = " << logical7_);
    if(!logical8_.empty()){
        cam8_part_type = logical8_;
        // ROS_INFO_STREAM("Conveyor camera sees: " << cam7_part_type);
        // all_parts_belt.push_back(cam7_part_type);
    }
    else{
        cam7_part_type = 'n';
        // ROS_INFO_STREAM_THROTTLE(2,"Logical Camera 7 does not see anything");
    }
    
    return cam8_part_type;
 }

 geometry_msgs::Pose AriacSensorManager::BinGetPartPose(const std::string& src_frame,
                                                        std::string product_type, std::string num) {
     geometry_msgs::Pose part_pose;
     int counter{1};
     int limit;
     std::string type;
     std::map<std::string, std::vector<std::string>> empty_bin_list;
     std::string target_frame;
     std::string frame;
     if (product_type == LogicalCamera6PartType()) {
       limit = prev6+1;
     }
     if (product_type == LogicalCamera1PartType()) {
       limit = prev1+1;
     }
     for (int i = 1; i <= limit; i++) {
       frame = "logical_camera_" + num + "_" + product_type + "_"
                        + std::to_string(counter) + "_frame";
       empty_bin_list[product_type].emplace_back(frame);
       counter++;
     }
     target_frame = empty_bin_list[product_type].back();
     int j = 0;
     for (int j = 1; j <= limit; j++) {
       target_frame = empty_bin_list[product_type].back();
       try {
         camera_tf_listener_.waitForTransform(src_frame, target_frame, ros::Time(0),
                                              ros::Duration(3));
         camera_tf_listener_.lookupTransform(src_frame, target_frame, ros::Time(0),
                                             camera_tf_transform_);

         part_pose.position.x = camera_tf_transform_.getOrigin().x();
         part_pose.position.y = camera_tf_transform_.getOrigin().y();
         part_pose.position.z = camera_tf_transform_.getOrigin().z();
      } catch (tf2::TransformException &ex) {
         ROS_WARN("%s",ex.what());
         // ros::Duration(1.0).sleep();
         empty_bin_list[product_type].pop_back();
         continue;
        }
     }
     return part_pose;
 }
