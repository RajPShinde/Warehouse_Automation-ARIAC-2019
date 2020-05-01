
#include "order_manager.h"
#include <osrf_gear/AGVControl.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <iomanip>

AriacOrderManager::AriacOrderManager(): arm1_{"arm1"}, arm2_{"arm2"}
{
    order_subscriber_ = order_manager_nh_.subscribe(
            "/ariac/orders", 10,
            &AriacOrderManager::OrderCallback, this);
    conveyor_part_found  = false;
    /* These are joint positions used for the home position
     * [0] = linear_arm_actuator
     * [1] = shoulder_pan_joint
     * [2] = shoulder_lift_joint
     * [3] = elbow_joint
     * [4] = wrist_1_joint
     * [5] = wrist_2_joint3
     * [6] = wrist_3_joint
     */
    flipped_arm1_pose_1 = {1.18, 4.59, -0.50, 0.99, 4.31, -1.53, 0};
    flipped_arm1_pose_2 = {1.18, 4.59, -1.50, 1.32, 4.31, -1.53, 0};
    flipped_arm1_pose_3 = {1.11, 4.56, -1.00, 1.34, 4.31, -1.53, 0};
    flipped_arm1_pose_4 = {1.11, 4.59, -1.00, 1.30, 4.31, -1.53, 0};

    // for agv_id = 2
    flipped_arm1_pose_5 = {0.50, 4.59, -0.50, 0.99, 4.32, 1.55, 0};
    flipped_arm1_pose_6 = {-0.50, 4.59, -0.50, 0.99, 4.32, 1.55, 0};
    flipped_arm2_pose_3 = {-0.59, 1.30, -1.37, 1.84, 4.10, -1.53, 0};
    flipped_arm2_pose_4 = {-0.59, 1.30, -1.37, 1.97, 4.10, -1.53, 0};
    flipped_arm2_pose_5 = {0.72, 1.40, -0.47, 1.00, 4.35, 1.51, 0};

    flipped_arm2_pose_1 = {0.69, 1.40, -0.47, 0.95, 4.31, -1.53, 0};
    flipped_arm2_pose_2 = {0.72, 1.40, -0.47, 0.85, 4.35, 1.51, 0};
    kit_drop_pose_ = {2.65, 1.57, -1.60, 2.0, 4.30, -1.53, 0};

    // for out of reach agv_id = 2 and arm1 out of reach
    out_arm2_pose1 = {-1.18, 1.43, -0.50, 0.95, 4.25, -1.54, 0};
    out_arm2_pose2 = {-1.18, 1.43, -1.13, 1.22, 4.25, -1.53, 0};
    out_arm2_pose3 = {-1.11, 1.43, -0.88, 1.09, 4.57, -1.54, 0};

    out_arm1_pose1 = {-0.59, 4.58, -0.30, 0.64, 4.43, -1.53, 0};
    out_arm1_pose2 = {-0.59, 4.58, -0.30, 0.49, 4.43, 1.61, 0};
}

AriacOrderManager::~AriacOrderManager(){}


void AriacOrderManager::OrderCallback(const osrf_gear::Order::ConstPtr& order_msg) {
    ROS_WARN_STREAM(">>>>> OrderCallback");
    ROS_INFO_STREAM("Order function called");
    received_orders_.push_back(*order_msg);
    bin1_part = camera_.LogicalCamera1PartType();
    bin2_part = camera_.LogicalCamera2PartType();
    bin3_part = camera_.LogicalCamera3PartType();
    bin4_part = camera_.LogicalCamera4PartType();
    bin5_part = camera_.LogicalCamera5PartType();
    bin6_part = camera_.LogicalCamera6PartType();
    // ROS_INFO_STREAM("Bin 6 part extreme FIRST: "<<bin6_part);
    belt_part_1 = camera_.BeltCamera1Part();
    bin_parts = {bin1_part, bin2_part, bin3_part,
              bin4_part, bin5_part, bin6_part};
    for (const auto &order:received_orders_){

        // ROS_INFO_STREAM(received_orders_);
        auto order_id = order.order_id;
        ROS_INFO_STREAM(order_id);
        auto shipments = order.shipments;

        for (const auto &shipment: shipments){
            auto shipment_type = shipment.shipment_type;
            auto agv = shipment.agv_id.back();//--this returns a char
            ROS_INFO_STREAM(agv);
            int agv_id = (shipment.agv_id == "any") ? 1 : agv - '0';

            auto products = shipment.products;
            ROS_INFO_STREAM("Order ID: " << order_id);
            ROS_INFO_STREAM("Shipment Type: " << shipment_type);
            ROS_INFO_STREAM("AGV ID: " << agv_id);
            //  if(agv_id==2)
            // {
            // 	arm1_.SendRobotPosition({2.6, 3.11, -1.60, 2.0, 4.30, -1.53, 0});
            // }
            // else
            // {
            // 	arm2_.SendRobotPosition({-2.6, 3.11, -1.60, 2.0, 4.30, -1.53, 0});
            // }
            for (const auto &product: products){
                ROS_INFO_STREAM("Product"<<product);
                // ros::spinOnce();
                // product_frame_list_ = camera_.get_product_frame_list();
                product_type_pose_.first = product.type;
                //ROS_INFO_STREAM("Product type: " << product_type_pose_.first);
                product_type_pose_.second = product.pose;
                // product_list_type_pose_[product.type] = product.pose;
                productlist_type.push_back(product.type);
                productlist_pose.push_back(product.pose);
                if (product.type == bin1_part and agv_id == 1) {
                  ROS_INFO_STREAM_THROTTLE(5, "Out of range for Arm1");
                }
                if (product.type == bin6_part and agv_id == 2) {
                  ROS_INFO_STREAM_THROTTLE(5, "Out of range for Arm2");
                }
                // ROS_INFO_STREAM("Product pose: " << product_type_pose_.second.position.x);
            }
            for (auto it = productlist_type.cbegin(); it != productlist_type.cend(); it++){
                std::cout << *it << ',' << std::endl;;
                if(*it == "piston_rod_part"){
                    piston_count_++;
                }
                else if(*it == "gear_part"){
                    gear_count_++;
                }
                else if(*it == "gasket_part"){
                    gasket_count_++;
                }
                else if(*it == "pulley_part"){
                    pulley_count_++;
                }
                else if(*it == "disk_part"){
                    disk_count_++;
                }
            }
            
            ROS_INFO_STREAM("Part read from bin 1: " << bin1_part);
            bin_part.push_back(bin1_part);
            ROS_INFO_STREAM("Part read from bin 2: " << bin2_part);
            bin_part.push_back(bin2_part);
            ROS_INFO_STREAM("Part read from bin 3: " << bin3_part);
            bin_part.push_back(bin3_part);
            ROS_INFO_STREAM("Part read from bin 4: " << bin4_part);
            bin_part.push_back(bin4_part);
            ROS_INFO_STREAM("Part read from bin 5: " << bin5_part);
            bin_part.push_back(bin5_part);
            ROS_INFO_STREAM("Part read from bin 6: " << bin6_part);
            bin_part.push_back(bin6_part);

            count1 = camera_.count_bin1;
            count2 = camera_.count_bin2;
            count3 = camera_.count_bin3;
            count4 = camera_.count_bin4;
            count5 = camera_.count_bin5;
            count6 = camera_.count_bin6;

            if(bin1_part == "n"){
                empty_bins.push_back("bin1");
                bin1_poses = FillBin(1, "piston_rod_part");
                ROS_INFO_STREAM("B1 poses: " << bin1_poses.size());
                bins_arm2.push_back("bin1");
                // all_empty_bins_pose.push_back(empty_bin_poses);
            }
            if(bin2_part == "n"){
                empty_bins.push_back("bin2");
                bin2_poses = FillBin(2, "piston_rod_part");
                ROS_INFO_STREAM("B2 poses: " << bin2_poses.size());
                bins_arm2.push_back("bin2");
                // all_empty_bins_pose.push_back(empty_bin_poses);
            }
            if(bin3_part == "n"){
                empty_bins.push_back("bin3");
                bin3_poses = FillBin(3, "piston_rod_part");
                ROS_INFO_STREAM("B3 poses: " << bin3_poses.size());
                bins_arm2.push_back("bin3");
                // all_empty_bins_pose.push_back(empty_bin_poses);
            }
            if(bin4_part == "n"){
                empty_bins.push_back("bin4");
                bin4_poses = FillBin(4, "piston_rod_part");
                ROS_INFO_STREAM("B4 poses: " << bin4_poses.size());
                bins_arm1.push_back("bin4");
                // all_empty_bins_pose.push_back(empty_bin_poses);
            }
            if(bin5_part == "n"){
                empty_bins.push_back("bin5");
                bin5_poses = FillBin(5, "piston_rod_part");
                ROS_INFO_STREAM("B5 poses: " << bin5_poses.size());
                bins_arm1.push_back("bin5");
                // all_empty_bins_pose.push_back(empty_bin_poses);
            }
            if(bin6_part == "n"){
                empty_bins.push_back("bin6");
                bin6_poses = FillBin(6, "piston_rod_part");
                ROS_INFO_STREAM("B6 poses: " << bin6_poses.size());
                bins_arm1.push_back("bin6");
                // all_empty_bins_pose.push_back(empty_bin_poses);
            }
            
            if(std::find(empty_bins.begin(), empty_bins.end(), "bin1") != empty_bins.end()){
                ROS_INFO_STREAM("Bin 1 positions already created");
            }
            else{
                ROS_INFO_STREAM("Checking available points, as follows: ");
                if(count1 >= threshold){
                    bin1_poses = FillBin(1, bin1_part);
                    ROS_INFO_STREAM("B1x poses: " << bin1_poses.size());
                    parts_already_in_bin.push_back(bin1_part);
                    // ROS_INFO_STREAM(bin1_poses[0]);
                }
                
            }
            if(std::find(empty_bins.begin(), empty_bins.end(), "bin2") != empty_bins.end()){
                ROS_INFO_STREAM("Bin 2 positions already created");
            }
            else{
                ROS_INFO_STREAM("Checking available points, as follows: ");
                if(count2 < threshold){
                    bin2_poses = FillBin(2, bin2_part);
                    ROS_INFO_STREAM("B2x poses: " << bin2_poses.size());
                    parts_already_in_bin.push_back(bin2_part);
                    // ROS_INFO_STREAM(bin1_poses[0]);
                }
                
            }
            if(std::find(empty_bins.begin(), empty_bins.end(), "bin3") != empty_bins.end()){
                ROS_INFO_STREAM("Bin 3 positions already created");
            }
            else{
                ROS_INFO_STREAM("Checking available points, as follows: ");
                if(count3 < threshold){
                    bin3_poses = FillBin(3, bin3_part);
                    ROS_INFO_STREAM("B3x poses: " << bin3_poses.size());
                    parts_already_in_bin.push_back(bin3_part);
                    // ROS_INFO_STREAM(bin1_poses[0]);
                }
                
            }
            if(std::find(empty_bins.begin(), empty_bins.end(), "bin4") != empty_bins.end()){
                ROS_INFO_STREAM("Bin 4 positions already created");
            }
            else{
                ROS_INFO_STREAM("Checking available points, as follows: ");
                if(count4 < threshold){
                    bin4_poses = FillBin(4, bin4_part);
                    ROS_INFO_STREAM("B4x poses: " << bin4_poses.size());
                    parts_already_in_bin.push_back(bin4_part);
                    // ROS_INFO_STREAM(bin1_poses[0]);
                    // ROS_INFO_STREAM("Bin4 poses trial: " << bin4_poses[0]);
                }
                
            }
            if(std::find(empty_bins.begin(), empty_bins.end(), "bin5") != empty_bins.end()){
                ROS_INFO_STREAM("Bin 5 positions already created");
            }
            else{
                ROS_INFO_STREAM("Checking available points, as follows: ");
                if(count5 < threshold){
                    bin5_poses = FillBin(5, bin5_part);
                    ROS_INFO_STREAM("B5x poses: " << bin5_poses.size());
                    parts_already_in_bin.push_back(bin5_part);
                    // ROS_INFO_STREAM(bin1_poses[0]);
                }
                
            }
            if(std::find(empty_bins.begin(), empty_bins.end(), "bin6") != empty_bins.end()){
                ROS_INFO_STREAM("Bin 6 positions already created");
            }
            else{
                ROS_INFO_STREAM("Checking available points, as follows: ");
                if(count6 < threshold){
                    bin6_poses = FillBin(6, bin6_part);
                    ROS_INFO_STREAM("B6x poses: " << bin6_poses.size());
                    parts_already_in_bin.push_back(bin6_part);
                    ROS_INFO_STREAM("Orig bin6 poses size: "<<bin6_poses.size());
                    // ROS_INFO_STREAM(bin1_poses[0]);
                }                
            }
            if(count1 >= threshold and count2 >= threshold and count3 >= threshold and count4 >= threshold and 
                count5 >= threshold and count6 >= threshold){
                ROS_INFO_STREAM("All bins full");
                StopConveyorPick = true;
            }

        }
    }
}

void AriacOrderManager::UpdateBin(){
    belt_part_1 = camera_.BeltCamera1Part();
    //update parts present in the bins
    bin1_part = camera_.LogicalCamera1PartType();
    bin2_part = camera_.LogicalCamera2PartType();
    bin3_part = camera_.LogicalCamera3PartType();
    bin4_part = camera_.LogicalCamera4PartType();
    bin5_part = camera_.LogicalCamera5PartType();
    bin6_part = camera_.LogicalCamera6PartType();

    //Store all part types present
    bin_part.push_back(bin1_part);
    bin_part.push_back(bin2_part);
    bin_part.push_back(bin3_part);
    bin_part.push_back(bin4_part);
    bin_part.push_back(bin5_part);
    bin_part.push_back(bin6_part);

    //update parts already present in bins
    parts_already_in_bin.clear();
    if(bin1_part == "n")
        empty_bins.push_back("bin1");
    else
        parts_already_in_bin.push_back(bin1_part);
    if(bin2_part == "n")
        empty_bins.push_back("bin2");
    else
        parts_already_in_bin.push_back(bin2_part);
    if(bin3_part == "n")
        empty_bins.push_back("bin3");
    else
        parts_already_in_bin.push_back(bin3_part);
    if(bin4_part == "n")
        empty_bins.push_back("bin4");
    else
        parts_already_in_bin.push_back(bin4_part);
    if(bin5_part == "n")
        empty_bins.push_back("bin5");
    else
        parts_already_in_bin.push_back(bin5_part);
    if(bin6_part == "n")
        empty_bins.push_back("bin6");
    else
        parts_already_in_bin.push_back(bin6_part);

    //update part count
    count1 = camera_.count_bin1;
    count2 = camera_.count_bin2;
    count3 = camera_.count_bin3;
    count4 = camera_.count_bin4;
    count5 = camera_.count_bin5;
    count6 = camera_.count_bin6;
}

int AriacOrderManager::DecideBinArm(std::string conveyor_part){
    //call update bin before this function always
    ROS_INFO_STREAM("Inside Decide arm bin function");  
    int result;
    int part_count = 0;
    int bin_number = 0;
    int b;
    int a;
    std::string arm;
    std::string bin;
    std::vector<std::string> bin_and_arm;
    // ROS_INFO_STREAM("Bin1 part: "<<bin1_part);
    if(bin1_part == conveyor_part){
        // ROS_INFO_STREAM("Condition met");
        part_count = count1;
        bin_number = 1;
        bin = "bin1";
    }
    if(bin2_part == conveyor_part){
        part_count = count2;
        bin_number = 2;
        bin = "bin2";
    }
    if(bin3_part == conveyor_part){
        part_count = count3;
        bin_number = 3;
        bin = "bin3";
    }
    if(bin4_part == conveyor_part){
        part_count = count4;
        bin_number = 4;
        bin = "bin4";
    }
    if(bin5_part == conveyor_part){
        part_count = count5;
        bin_number = 5;
        bin = "bin5";
    }
    if(bin6_part == conveyor_part){
        part_count = count6;
        bin_number = 6;
        bin = "bin6";
    }
    // if(bin1_part != conveyor_part && bin2_part != conveyor_part && bin3_part != conveyor_part && bin4_part != conveyor_part && bin5_part != conveyor_part && bin6_part != conveyor_part){
    //     bin = empty_bins[0];
    //     bin_number = std::stoi(bin);
    // }

    if(part_count >= threshold){
        ROS_INFO_STREAM("Sufficient parts found in bin");
        bin = "x";
        arm = "x";
        // break;
    }
    ROS_INFO_STREAM("Part count from decide func: " << part_count);

    if(part_count < threshold){
        if(part_count == 0){
            if(std::find(empty_bins.begin(), empty_bins.end(), "bin4") != empty_bins.end()){
                ROS_INFO_STREAM("Bin 4 is empty for placing using arm1");
                bin = "bin4";
                arm = "arm1";
            }
            else if(std::find(empty_bins.begin(), empty_bins.end(), "bin5") != empty_bins.end()){
                ROS_INFO_STREAM("Bin 5 is empty for placing using arm1");
                bin = "bin5";
                arm = "arm1";
            }
            else if(std::find(empty_bins.begin(), empty_bins.end(), "bin6") != empty_bins.end()){
                ROS_INFO_STREAM("Bin 6 is empty for placing using arm1");
                bin = "bin6";
                arm = "arm1";
            }
            else if(std::find(empty_bins.begin(), empty_bins.end(), "bin3") != empty_bins.end()){
                ROS_INFO_STREAM("Bin 3 is empty for placing using arm2");
                bin = "bin3";
                arm = "arm2";
            }
            else if(std::find(empty_bins.begin(), empty_bins.end(), "bin2") != empty_bins.end()){
                ROS_INFO_STREAM("Bin 2 is empty for placing using arm2");
                bin = "bin2";
                arm = "arm2";
            }
            else if(std::find(empty_bins.begin(), empty_bins.end(), "bin1") != empty_bins.end() ){
                ROS_INFO_STREAM("Bin 1 is empty for placing using arm2");
                bin = "bin1";
                arm = "arm2";
            }

        }
        if(part_count != 0){
            if(bin_number == 1 || bin_number == 2 || bin_number == 3)
                arm = "arm2";
            if(bin_number == 4 || bin_number == 5 || bin_number == 6)
                arm = "arm1";
        } 
    }
    ROS_INFO_STREAM("Bin, arm: "<<bin<<" "<<arm);
    bin_and_arm.push_back(bin);
    bin_and_arm.push_back(arm);
    if(bin=="bin1")
    	b=1;
    else if(bin=="bin2")
        b=2;    	
    else if(bin=="bin3")
    	b=3;
    else if(bin=="bin4")
        b=4;
    else if(bin=="bin5")
        b=5;
    else if(bin=="bin6")
        b=6;
    
    if(arm=="arm1")
    	a=1;
    else if(arm=="arm2")
        a=2;

    if(bin!="x" && arm!="x")
        return 10*b+a;
    if(bin.empty() && arm.empty())
        return 0;

    return 0;
}


std::string AriacOrderManager::GetProductFrame(std::string product_type) {
    //--Grab the last one from the list then remove itle()
    ROS_WARN_STREAM("Came Here");

    if (!product_frame_list_.empty() && product_frame_list_.count(product_type)!=0) {
        std::string frame = product_frame_list_[product_type].back();
        // ROS_INFO_STREAM("Frame >>>> " << frame);
        product_frame_list_[product_type].pop_back();
        return frame;
    } else 
    {
          ROS_ERROR_STREAM("No product frame found for " << product_type);
          return "x";
  }

}

bool AriacOrderManager::PickAndPlace(const std::pair<std::string,geometry_msgs::Pose> product_type_pose, int agv_id) {
    ros::spinOnce();
    std::string product_type = product_type_pose.first;

    std::vector<std::string> bin_and_arm;
    UpdateBin();


    // if(bin_and_arm[0] != "x" and bin_and_arm[0] != "x" and StopConveyorPick == false){
    //     ROS_INFO_STREAM("Pick the part");
    // }
    
    std::string product_frame = GetProductFrame(product_type);

    if(product_frame!="x")
    {

    auto part_pose = camera_.GetPartPose("/world", product_frame);


    if(product_type == "pulley_part")
        part_pose.position.z += 0.037;
    else if(product_type == "piston_rod_part")
        part_pose.position.z -= 0.0159;
    else if(product_type == "gear_part")
        part_pose.position.z -= 0.012;
    else if(product_type == "disk_part")
        part_pose.position.z = part_pose.position.z;
    else if(product_type == "gasket_part")
        part_pose.position.z -= 0.01;

    //--task the robot to pick up this part again from the bin
    if (isFlipped) {
      FlippedPart(agv_id, part_pose);
    }
    else if (agv_id == 1 and isFlipped == false) {
      bool failed_pick = arm1_.PickPart(part_pose, agv_id);
    } else {
      home_joint_pose_1 = {1.18, 3.11, -1.60, 2.0, 4.30, -1.53, 0};
      arm1_.SendRobotPosition(home_joint_pose_1);
      bool failed_pick = arm2_.PickPart(part_pose, agv_id);
    }

    //--get the pose of the object in the tray from the order
    geometry_msgs::Pose drop_pose = product_type_pose.second;

    geometry_msgs::PoseStamped StampedPose_in, StampedPose_out;

    if(agv_id==1){
        StampedPose_in.header.frame_id = "/kit_tray_1";
        StampedPose_in.pose = drop_pose;
        part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
    }
    else if(agv_id==2){
        StampedPose_in.header.frame_id = "/kit_tray_2";
        StampedPose_in.pose = drop_pose;
        part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
    }

    // This is checking if part is faulty ior not
    bool success = false;
    if( not success and agv_id == 1) {
      success = arm1_.DropPart(StampedPose_out.pose, agv_id); //robot_controller
    } else if(not success and agv_id == 2){
      success = arm2_.DropPart(StampedPose_out.pose, agv_id);
    }
    if(placed_parts.size()!=0){
        if(placed_parts[placed_parts.size()-1].first==product_type && placed_parts[placed_parts.size()-1].second==StampedPose_out.pose){}
        else{
          placed_parts.push_back(std::make_pair(product_type,StampedPose_out.pose));
        }
    }
    else {
      placed_parts.push_back(std::make_pair(product_type,StampedPose_out.pose));
    }

    return success;
}
return true;

}

void AriacOrderManager::ExecuteOrderNew(){
    ROS_WARN(">>>>>> Executing order new....");
    
}


void AriacOrderManager::FlippedPart(int agv_id, auto pose) {
  if (agv_id == 1 and isReachable == true) {
    home_joint_pose_1 = {1.18, 3.11, -1.60, 2.0, 4.30, -1.53, 0};
    arm1_.SendRobotPosition(home_joint_pose_1);
    bool failed_pick = arm2_.PickPart(pose);
    arm2_.SendRobotPosition2(flipped_arm2_pose_2);
    this->arm1_.GripperToggle(true);
    arm1_.SendRobotPosition(flipped_arm1_pose_4);
    arm1_.SendRobotPosition(flipped_arm1_pose_3);
    this->arm2_.GripperToggle2(false);
    arm2_.SendRobotPosition2(flipped_arm2_pose_5);
    // arm2_.SendRobot2();
    arm2_.SendRobotPosition({-2.6, 3.11, -1.60, 2.0, 4.30, -1.53, 0});
  }
  if (agv_id == 2 and isReachable == true) {
    bool failed_pick = arm1_.PickPart(pose, 1);
    arm1_.SendRobotPosition(flipped_arm1_pose_5);
    arm1_.SendRobotPosition(flipped_arm1_pose_6);
    arm2_.SendRobotPosition2(flipped_arm2_pose_3);
    arm2_.SendRobotPosition2(flipped_arm2_pose_4);
    this->arm2_.GripperToggle2(true);
    this->arm1_.GripperToggle(false);
    // arm1_.SendRobot1();
    arm1_.SendRobotPosition({2.6, 3.11, -1.60, 2.0, 4.30, -1.53, 0});
  }
  if (agv_id == 1 and isReachable == false) {
    bool failed_pick = arm1_.PickPart(pose, 1);
    arm1_.SendRobotPosition(flipped_arm1_pose_1);
    this->arm1_.GripperToggle(false);
    arm1_.SendRobotPosition(flipped_arm1_pose_2);
    this->arm2_.GripperToggle2(true);
    arm2_.SendRobotPosition2(flipped_arm2_pose_1);
    arm2_.SendRobotPosition2(flipped_arm2_pose_2);
    this->arm1_.GripperToggle(true);
    arm1_.SendRobotPosition(flipped_arm1_pose_3);
    this->arm2_.GripperToggle2(false);
    // arm2_.SendRobot2();
    arm2_.SendRobotPosition({-2.6, 3.11, -1.60, 2.0, 4.30, -1.53, 0});
  }
  if (agv_id == 2 and isReachable == false) {
    bool failed_pick = arm2_.PickPart(pose);
    arm2_.SendRobotPosition2(out_arm2_pose1);
    this->arm2_.GripperToggle2(false);
    this->arm1_.GripperToggle(true);
    arm2_.SendRobotPosition2(out_arm2_pose2);
    arm1_.SendRobotPosition(out_arm1_pose1);
    arm1_.SendRobotPosition(out_arm1_pose2);
    this->arm2_.GripperToggle2(true);
    arm2_.SendRobotPosition2(out_arm2_pose3);
    this->arm1_.GripperToggle(false);
    // arm1_.SendRobot1();
    arm1_.SendRobotPosition({2.6, 3.11, -1.60, 2.0, 4.30, -1.53, 0});
  }
  isFlipped=false;
}

void AriacOrderManager::ExecuteOrder() {
    ROS_WARN(">>>>>> Executing order...");

    PickPartconveyor();
    ROS_WARN(">>>>>> Executing order return");
    bool pick_n_place_success{false};

    std::list<std::pair<std::string,geometry_msgs::Pose>> failed_parts;
    ros::spinOnce();
    ros::Duration(0.3).sleep();
    product_frame_list_ = camera_.get_product_frame_list();
    int i=0;
    bool update=false;
    int flag=0;
    arm1_.SendRobotPosition({0, 3.11, -1.60, 2.0, 4.30, -1.53, 0});
    arm2_.SendRobotPosition2({0, 3.11, -1.60, 2.0, 4.30, -1.53, 0});
    geometry_msgs::PoseStamped StampedPose_in, StampedPose_out;
    UpdateBin();
    while(true) {
        ROS_INFO_STREAM("in");

        const auto &order=received_orders_[0];

        int size=received_orders_.size();
        auto order_id = order.order_id;
        auto shipments = order.shipments;


        for (const auto &shipment: shipments){
            auto shipment_type = shipment.shipment_type;
            ROS_INFO_STREAM(shipment.agv_id);
            auto agv = shipment.agv_id.back();
            ROS_INFO_STREAM(agv);
            int agv_id = (shipment.agv_id == "any") ? 1 : agv - '0';
            auto products = shipment.products;
            ROS_INFO_STREAM("Order ID: " << order_id);
            ROS_INFO_STREAM("Shipment Type: " << shipment_type);
            ROS_INFO_STREAM("AGV ID: " << agv_id);
            if(agv_id==2)
            {
            	arm1_.SendRobotPosition({2.6, 3.11, -1.60, 2.0, 4.30, -1.53, 0});
            }
            else
            {
            	arm2_.SendRobotPosition({-2.6, 3.11, -1.60, 2.0, 4.30, -1.53, 0});
            }


            for (const auto &product: products){

                // Check for Updated Order
                ros::spinOnce();
                int new_size=received_orders_.size();

                if(size!=new_size) {
                    int diff=new_size-size;
                    update=checkOrderUpdate(i,diff,order_id, agv_id);
                }
                if(update==true)
                {
                    break;
                }

                ROS_INFO_STREAM("Product"<<product);
                // ros::spinOnce();
                // product_frame_list_ = camera_.get_product_frame_list();
                product_type_pose_.first = product.type;
                //ROS_INFO_STREAM("Product type: " << product_type_pose_.first);
                product_type_pose_.second = product.pose;
                tf::quaternionMsgToTF(product.pose.orientation,q);
                tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
                ROS_INFO_STREAM("Roll: " << roll);
                // if (roll == -3.14159 || roll == 3.14159) {
                if(roll!=0){
                  ROS_INFO_STREAM("Part is to be Flipped");
                  isFlipped = true;
                }
                // isFlipped = true;Running SendRobotPosition2
                if (product.type == bin1_part and agv_id == 2 and isFlipped == true) {
                  isReachable = false;
                }
                if (product.type == bin6_part and agv_id == 1 and isFlipped == true) {
                  isReachable = false;
                }
                if(agv_id==1)
                {
                StampedPose_in.header.frame_id = "/kit_tray_1";
                }
                else if(agv_id==2)
                {
                StampedPose_in.header.frame_id = "/kit_tray_2";
                }
                StampedPose_in.pose = product.pose;
                
                ROS_WARN_STREAM("HURRAY");
                part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
                auto f = std::make_pair(product.type, StampedPose_out.pose);
                ROS_WARN_STREAM("HURRAY");
                if(std::count(placed_parts.begin(),placed_parts.end(),f)==0)
                {
                  ROS_WARN_STREAM("HURRAY");
                 if(std::count(bin_parts.begin(),bin_parts.end(),product.type)!=0)
                 {
                 ROS_WARN_STREAM(bin1_part);
                 ROS_WARN_STREAM(bin2_part);
                 ROS_WARN_STREAM(bin3_part);
                 ROS_WARN_STREAM(bin4_part);
                 ROS_WARN_STREAM(bin5_part);
                 ROS_WARN_STREAM(bin6_part);

                if (product.type == bin1_part and agv_id == 1 and isFlipped == false) {
                	ROS_WARN_STREAM("1");
                  // bool pick_n_place_success = false;
                  // while (not pick_n_place_success){
                  std::string num;
                  for (int i = 0; i < bin_parts.size(); i++) {
                  	ROS_WARN_STREAM("11");
                    if (bin_parts[i]=="n" and i != 0) {
                      empty_bin = "bin" + std::to_string(i+1);
                      num = std::to_string(i+1);
                      break;
                    }
                  }

                  ROS_WARN_STREAM("12");
                   while(pick_n_place_success==false){
                  pick_n_place_success =  PickAndPlace(product_type_pose_, empty_bin, agv_id);
              }
               OutOfReach("arm1", num, product_type_pose_, agv_id);
             } 


             else if(product.type == bin6_part and agv_id == 2 and isFlipped == false) {
             	ROS_WARN_STREAM("2");
                  std::string num;
                  for (int i = 0; i < bin_parts.size(); i++) {
                  	ROS_WARN_STREAM("21");
                    if (bin_parts[i]=="n" and i != 5) {
                      empty_bin = "bin" + std::to_string(i+1);
                      num = std::to_string(i+1);
                      break;
                    }
                  }
                  ROS_WARN_STREAM("22");
                   while(pick_n_place_success==false){
                  pick_n_place_success =  PickAndPlace(product_type_pose_, empty_bin, agv_id);
              }
                  OutOfReach("arm2", num, product_type_pose_, agv_id);
                } 



                else {
                	ROS_WARN_STREAM("3");
                while(pick_n_place_success==false){
                	ROS_WARN_STREAM("31");
                  pick_n_place_success =  PickAndPlace(product_type_pose_, agv_id);
                }
                }
                // ROS_INFO_STREAM(placed_parts.size());
                // ROS_INFO_STREAM(placed_parts[placed_parts.size()-1].first);
                ROS_INFO_STREAM("Part is to be Flipped");

                pick_n_place_success=false;
            }
        }
                //--todo: What do we do if pick and place fails?
            }
            //wait 5 sec
            if(update==false)
            {
            ros::Time a=ros::Time::now();
            ros::Duration(2.5).sleep();
            ros::Time b=ros::Time::now();
            ros::Duration diffi1=b-a;


            ros::Time lasttime=ros::Time::now();
            ros::Time currtime=ros::Time::now();
            ros::Duration diffi=currtime-lasttime;
            while(diffi<diffi1)
            {
                ros::Time currtime=ros::Time::now();
                diffi=currtime-lasttime;
                ros::spinOnce();
            }
            int new_size=received_orders_.size();
            if(size!=new_size) {
                int diff=new_size-size;
                update=checkOrderUpdate(i,diff,order_id, agv_id);
            }
            }


            if(update==false) {
                SubmitAGV(agv_id);
                ROS_INFO_STREAM("Submitting AGV");
                placed_parts.clear();
                int finish=1;
            }
            else {
                break;
            }
        }
        received_orders_.erase(received_orders_.begin());
        // ROS_INFO_STREAM(received_orders_.size());
        update=false;

        // Again check for update

        if (received_orders_.size()==0)
        {
            break;
        }

    }
}

void AriacOrderManager::SubmitAGV(int num) {
    std::string s = std::to_string(num);
    ros::ServiceClient start_client =
            order_manager_nh_.serviceClient<osrf_gear::AGVControl>("/ariac/agv"+s);
    if (!start_client.exists()) {
        // ROS_INFO("Waiting for the client to be ready...");
        start_client.waitForExistence();
        // ROS_INFO("Service started.");
    }

    osrf_gear::AGVControl srv;
    // srv.request.kit_type = "order_0_kit_0";
    start_client.call(srv);

    if (!srv.response.success) {
        // ROS_ERROR_STREAM("Service failed!");
    } else{
        // ROS_INFO("Service succeeded.");
    }
}

void AriacOrderManager::dropallparts(std::vector<std::pair<std::string,geometry_msgs::Pose>> placed_parts, int agv_id)
{
  home_joint_pose_1 = {0.0, 3.11, -1.60, 2.0, 4.30, -1.53, 0};
  home_joint_pose_2 = {-1.18, 3.11, -1.60, 2.0, 4.30, -1.53, 0};
  int i=0;
  do {
    // ROS_INFO_STREAM(i);

    if (agv_id == 1) {
      drop_pose_ = {2.4, 1.57, -1.60, 2.0, 4.30, -1.53, 0};
      if(placed_parts[i].first=="pulley_part")
      {
        placed_parts[i].second.position.z +=0.03;
      }
      arm1_.PickPart(placed_parts[i].second, agv_id);
      arm1_.SendRobotPosition(drop_pose_);
      arm1_.GripperToggle(false);
    } else {
      drop_pose_ = {-2.4, -1.57, -1.60, 2.0, 4.30, -1.53, 0};
      if(placed_parts[i].first=="pulley_part")
      {
        placed_parts[i].second.position.z +=0.03;
      }
      arm2_.PickPart(placed_parts[i].second, agv_id);
      arm2_.SendRobotPosition(drop_pose_);
      arm2_.GripperToggle2(false);
    }
    i++;
  }while(i<placed_parts.size());
  if (agv_id == 1) arm1_.SendRobotPosition(home_joint_pose_1);

  else arm2_.SendRobotPosition2(home_joint_pose_2);
}

std::vector<std::string> AriacOrderManager::GetProductType(){
    return productlist_type;
}

std::vector<geometry_msgs::Pose> AriacOrderManager::GetProductPose(){
    return productlist_pose;
}

bool AriacOrderManager::checkOrderUpdate(int i,int diff, std::string order_id, int agv_id){
    int update_no=0;
            ROS_INFO_STREAM(placed_parts[0].first);
            ROS_INFO_STREAM(placed_parts[1].first);
            ROS_INFO_STREAM(placed_parts[2].first);
            ROS_INFO_STREAM(placed_parts[3].first);

    for(int k=i+1;k<diff+1;k++) {
        const auto &var = received_orders_[k];
        ROS_INFO_STREAM("Order"<<var.order_id);
        auto new_order=var.order_id;
        if(order_id+"_update_"+std::to_string(update_no)==new_order) {
            ROS_INFO_STREAM("Found a New Order");
            auto new_shipments = var.shipments;
            const auto &var1 = new_shipments[0];
            auto new_agv=var1.agv_id.back();
            int new_agv_id = (var1.agv_id == "any") ? 2 : new_agv - '0';
            auto new_products = var1.products;

            
            std::vector<std::pair<std::string,geometry_msgs::Pose>> new_parts;
            std::vector<std::pair<std::string,geometry_msgs::Pose>> present;
            std::vector<std::pair<std::string,geometry_msgs::Pose>> new_placed_parts;
            std::vector<std::pair<std::string,geometry_msgs::Pose>> remaining;
            std::vector<std::pair<std::string,geometry_msgs::Pose>> shuffle_from;
            std::vector<std::pair<std::string,geometry_msgs::Pose>> shuffle_to;
            std::vector<std::pair<std::string,geometry_msgs::Pose>> parts_to_remove;
            std::vector<std::pair<std::string,geometry_msgs::Pose>> temp_shuffle_to;
            geometry_msgs::Pose temp,temp1,vacant,slider,slider2;
            int pulley,gasket,piston,gear,disk,n_pulley,n_gasket,n_piston,n_gear,n_disk=0;
            int space=0;
            int interchange=0;

            // Find which parts exist at same location in new order
            geometry_msgs::PoseStamped StampedPose_in, StampedPose_out;
            geometry_msgs::PoseStamped Pose_in, Pose_out;
        if(new_agv_id==agv_id)
        {
              for (const auto &p: new_products)
              {
              if(agv_id==1)
              {
              StampedPose_in.header.frame_id = "/kit_tray_1";
              }
              else
              {
              StampedPose_in.header.frame_id = "/kit_tray_2";              
              }
              StampedPose_in.pose = p.pose;
              part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
              new_parts.push_back(std::make_pair(p.type,StampedPose_out.pose));
              auto h=std::make_pair(p.type,StampedPose_out.pose);
    

              for(int q=0;q<placed_parts.size();q++)
              {
                if(placed_parts[q].first==p.type && placed_parts[q].second==StampedPose_out.pose)
                {
                    ROS_INFO_STREAM(q);
                    ROS_INFO_STREAM("Found");
                    present.push_back(std::make_pair(p.type,StampedPose_out.pose));
                }
              }
            }


            // Find Remaining parts
            for(int m=0;m<placed_parts.size();m++)
            {
                if(std::find(present.begin(),present.end(),placed_parts[m])==present.end())
                {
                remaining.push_back(std::make_pair(placed_parts[m].first,placed_parts[m].second));
                }
            }
            ROS_INFO_STREAM("YY"<<remaining.size());

            // Find which parts can be suffled 
            for(int b=0;b<new_parts.size();b++)
            {
                if(std::find(present.begin(),present.end(),new_parts[b])==present.end())
                {
                    for(int n=0;n<remaining.size();n++)
                    {
                        if(remaining[n].first==new_parts[b].first)
                        {
                            shuffle_from.push_back(std::make_pair(remaining[n].first,remaining[n].second));
                            shuffle_to.push_back(std::make_pair(remaining[n].first,new_parts[b].second));
                            remaining.erase(remaining.begin()+n);
                            new_parts.erase(new_parts.begin()+b);
                            b=0;
                            break;
                        }
                    }

                }  
            }
            ROS_INFO_STREAM("ZZ"<<shuffle_from.size());
            ROS_INFO_STREAM("YY"<<remaining.size());
            // int f=shuffle_from.size();
            // for(int x=0;x<f;x++)
            // {
            //     shuffle_from.push_back(std::make_pair(shuffle_from[x].first,shuffle_to[x].second));
            //     shuffle_to.push_back(std::make_pair(shuffle_from[x].first,shuffle_from[x].second));
            //     for(int r=0;r<remaining.size();r++)
            //     {
            //         if(remaining[r]==std::make_pair(shuffle_from[x].first,shuffle_to[x].second))
            //         {
            //             remaining.erase(remaining.begin()+r);
            //             break;
            //         }
            //     }
            // }
            // ROS_INFO_STREAM("AAAAAAAAA"<<shuffle_from.size());
            // ROS_INFO_STREAM("AAAAAAAAA"<<shuffle_to.size());
            ROS_INFO_STREAM("LLLLLLLLLLL"<<placed_parts.size());

            // Find Remaining parts
            remaining.clear();
            for(int v=0;v<placed_parts.size();v++)
            {
                if(std::find(shuffle_from.begin(),shuffle_from.end(),placed_parts[v])==shuffle_from.end() && std::find(present.begin(),present.end(),placed_parts[v])==present.end())
                    remaining.push_back(std::make_pair(placed_parts[v].first,placed_parts[v].second));  
            }
            ROS_INFO_STREAM("YY"<<remaining.size());


            // Find Parts which can be removed
            if(remaining.size()>0)
            {
            dropallparts(remaining, agv_id);
            // space=1;
            }
            
            temp_shuffle_to=shuffle_to;
            // Shuffle parts
            if(shuffle_from.size()>0)
            {
                  for(int e=0;e<shuffle_to.size();e++)
                  {
                    for(int d=0;d<shuffle_from.size();d++)
                    {
                        if(shuffle_to[e].second==shuffle_from[d].second && e!=d)
                        {
                          temp=shuffle_from[e].second;
                          temp1=shuffle_from[d].second;
                          if(agv_id==1)
                          {
                            arm1_.PickPart(shuffle_from[d].second,1);
                          }
                          else
                          {
                            arm2_.PickPart(shuffle_from[d].second,2);
                          }
                          interchange=1;
                          shuffle_from.erase(shuffle_from.begin()+d);
                          break;
                        }
                    }
                    // To swap
                    if(interchange==1)
                    {
                        if(space==0)
                        {
                          slider=temp1;
                          slider2=temp1;
                          if(agv_id==1)
                          {
                            slider.position.x=0.31;
                            slider.position.y=2.2;
                            slider.position.z=0.94;
                            arm1_.DropPart(slider,1);
                            arm1_.PickPart(shuffle_from[e].second,1);
                            arm1_.DropPart(shuffle_to[e].second,1);
                            arm1_.PickPart(slider,1);
                            arm1_.DropPart(shuffle_from[e].second,1);
                          }
                        else
                          {
                            slider2.position.x=0.31;
                            slider2.position.y=-2.2;
                            slider2.position.z=0.94;
                            arm2_.DropPart(slider2,2);
                            arm2_.PickPart(shuffle_from[e].second,2);
                            arm2_.DropPart(shuffle_to[e].second,2);
                            arm2_.PickPart(slider,2);
                            arm2_.DropPart(shuffle_from[e].second,2);
                          }
                         
                        }
                        else
                        {
                          vacant=shuffle_from[e].second;
                          vacant.position.x=remaining[0].second.position.x;
                          vacant.position.y=remaining[0].second.position.y;
                          vacant.position.z=remaining[0].second.position.z;

                          if(agv_id==1)
                          {
                            arm1_.DropPart(vacant,1);
                            arm1_.PickPart(shuffle_from[e].second,1);
                            arm1_.DropPart(shuffle_to[e].second,1);
                            arm1_.PickPart(vacant,1);
                            arm1_.DropPart(shuffle_from[e].second,1);
                          }
                        else
                          {
                            arm2_.DropPart(vacant,2);
                            arm2_.PickPart(shuffle_from[e].second,2);
                            arm2_.DropPart(shuffle_to[e].second,2);
                            arm2_.PickPart(vacant,2);
                            arm2_.DropPart(shuffle_from[e].second,2);
                          }
                        }
                        interchange=0;
                    }
                    // To just keep
                    else
                    {
                        if(agv_id==1)
                          {
                            arm1_.PickPart(shuffle_from[e].second,1);
                            arm1_.DropPart(shuffle_to[e].second,1);
                          }
                        else
                          {
                            arm2_.PickPart(shuffle_from[e].second,2);
                            arm2_.DropPart(shuffle_to[e].second,2);
                          }
                    }
                    shuffle_to.erase(shuffle_to.begin());
                    e=0;
                  }

                
            }
            
            // Update PLaced Parts List
            if(present.size()>0 || temp_shuffle_to.size()>0)
            {
                placed_parts.clear();
                placed_parts=present;
                for(int y=0;y<temp_shuffle_to.size();y++)
                {   
                    ROS_INFO_STREAM(temp_shuffle_to[y].first);
                    ROS_INFO_STREAM(temp_shuffle_to[y].second);
                    placed_parts.push_back(std::make_pair(temp_shuffle_to[y].first,temp_shuffle_to[y].second));
                }

            }
            ROS_INFO_STREAM("LLLLLLLLLLL"<<placed_parts.size());
            ROS_WARN_STREAM("Update Changes Complete");
            return true;
        }

            // drop all parts if updated order on different AGV
            // drop all parts if all parts are different
            if(agv_id!=new_agv_id || (present.size()==0 || shuffle_from.size()==0) )
            {
            ROS_INFO_STREAM("All parts in new order are different or on Other AGV");
            ROS_INFO_STREAM("Dropping all parts in the kit");
            dropallparts(placed_parts, agv_id);
            placed_parts.clear();
            return true;
            }
        }
        update_no++;
    }
    return false;
}

bool AriacOrderManager::PickAndPlace(std::pair<std::string,geometry_msgs::Pose> product_type_pose, std::string empty_bin, int agv_id) {
    std::string product_type = product_type_pose.first;
    std::vector<std::string> bin_and_arm; 
    UpdateBin();
    // if(bin_and_arm[0] != "x" and bin_and_arm[1] != "x" and StopConveyorPick == false){
    //     ROS_INFO_STREAM("Pick the part");
    // }

    std::string product_frame = GetProductFrame(product_type);


    if(product_frame!="x")
    {
    auto part_pose = camera_.GetPartPose("/world", product_frame);
    ROS_INFO_STREAM("PARRT"<<product_type);


    if(product_type == "pulley_part")
        part_pose.position.z += 0.037;
    if(product_type == "piston_rod_part")
        part_pose.position.z -= 0.0157;
    if(product_type == "gear_part")
        part_pose.position.z -= 0.012;
    if(product_type == "disk_part")
        part_pose.position.z = part_pose.position.z;
    if(product_type == "gasket_part")
        part_pose.position.z = part_pose.position.z;
    home_joint_pose_1 = {0.0, 3.11, -1.60, 2.0, 4.30, -1.53, 0};
    home_joint_pose_2 = {-1.18, 3.11, -1.60, 2.0, 4.30, -1.53, 0};

    //--task the robot to pick up this part again from the bin
    if (agv_id == 2) {
      bool failed_pick = arm1_.PickPart(part_pose);
    } else {
      arm1_.SendRobotPosition(home_joint_pose_1);
      bool failed_pick = arm2_.PickPart(part_pose);
    }
    //--get the pose of the object in the tray from the order
    geometry_msgs::Pose drop_pose = product_type_pose.second;

    geometry_msgs::PoseStamped StampedPose_in, StampedPose_out;

    StampedPose_in.header.frame_id = "/" + empty_bin + "_frame";
    StampedPose_in.pose = drop_pose;
        // ROS_INFO_STREAM("StampedPose_int (" << StampedPose_in.pose.position.x <<","<< StampedPose_in.pose.position.y << "," << StampedPose_in.pose.position.z<<")");
    part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);

    bool success = false;
    if (agv_id == 2 and !success) {
      success = arm1_.DropPart(StampedPose_out.pose);
    } else {
      success = arm2_.DropPart(StampedPose_out.pose);
    }

    return true;
	    }
	return true;

}

void AriacOrderManager::OutOfReach(std::string arm, std::string num, std::pair<std::string,geometry_msgs::Pose> product, int agv_id) {
  // geometry_msgs::Pose part_pose;
  geometry_msgs::Pose part_pose = camera_.BinGetPartPose("/world", product.first, num);
  home_joint_pose_1 = {0.0, 3.11, -1.60, 2.0, 4.30, -1.53, 0};

  // bool pick_n_place_success_1 =  arm1_.PickPart(product_type_pose_, agv_id);
  if(product.first == "pulley_part")
      part_pose.position.z += 0.037;
  if(product.first == "piston_rod_part")
      part_pose.position.z -= 0.0157;
  if(product.first == "gear_part")
      part_pose.position.z -= 0.012;
  if(product.first == "disk_part")
      part_pose.position.z = part_pose.position.z;
  if(product.first == "gasket_part")
      part_pose.position.z = part_pose.position.z;

  //--task the robot to pick up this part again from the bin
  if (arm == "arm1") {
    // arm1_.SendRobotHome();
    bool failed_pick = arm1_.PickPart(part_pose, agv_id);
  } else {
    arm1_.SendRobotPosition(home_joint_pose_1);
    bool failed_pick = arm2_.PickPart(part_pose, agv_id);
  }

  //--get the pose of the object in the tray from the order
  geometry_msgs::Pose drop_pose = product.second;

  geometry_msgs::PoseStamped StampedPose_in, StampedPose_out;

  if(arm == "arm1"){
      StampedPose_in.header.frame_id = "/kit_tray_1";
      StampedPose_in.pose = drop_pose;
      // ROS_INFO_STREAM("StampedPose_int (" << StampedPose_in.pose.position.x <<","<< StampedPose_in.pose.position.y << "," << StampedPose_in.pose.position.z<<")");
      part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
      // StampedPose_out.pose.position.x += 0.2;
      // StampedPose_out.pose.position.y += 0.2;
      // ROS_INFO_STREAM("StampedPose_out (" << StampedPose_out.pose.position.x <<","<< StampedPose_out.pose.position.y << "," << StampedPose_out.pose.position.z<<")");
  }
  else{
      StampedPose_in.header.frame_id = "/kit_tray_2";
      StampedPose_in.pose = drop_pose;
      //ROS_INFO_STREAM("StampedPose_in " << StampedPose_in.pose.position.x);
      part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
      // StampedPose_out.pose.position.z += 0.1;
      // StampedPose_out.pose.position.y += 0.2;
      //ROS_INFO_STREAM("StampedPose_out " << StampedPose_out.pose.position.x);
  }

  // This is checking if part is faulty ior not
  bool success = false;
  if( not success and arm == "arm1") {
    // arm1_.SendRobotHome1();
    success = arm1_.DropPart(StampedPose_out.pose, agv_id);
    // bool failed_pick = arm2_.PickPart(part_pose, agv_id); //robot_controller
  } else {
    success = arm2_.DropPart(StampedPose_out.pose, agv_id);
    // bool failed_pick = arm1_.PickPart(part_pose, agv_id);
  }
  placed_parts.push_back(std::make_pair(product.first,StampedPose_out.pose));
}

double AriacOrderManager::round_up(double value, int decimal_places) {
    const double multiplier = std::pow(10.0, decimal_places);
    return std::ceil(value * multiplier) / multiplier;
}

std::vector<geometry_msgs::Pose> AriacOrderManager::FillBin(int bin_number, std::string conveyor_part_type)
{
    ROS_INFO_STREAM("FillBin called");
    // std::vector<geometry_msgs::Pose> new_place_pose;
    std::vector<geometry_msgs::Pose> position_available_bin1;
    std::vector<geometry_msgs::Pose> position_available_bin2;
    std::vector<geometry_msgs::Pose> position_available_bin3;
    std::vector<geometry_msgs::Pose> position_available_bin4;
    std::vector<geometry_msgs::Pose> position_available_bin5;
    std::vector<geometry_msgs::Pose> position_available_bin6;
    std::string part_in_bin;
    geometry_msgs::Pose new_place_pose;
    int threshold = 4;
    bool horizontal_flag_single = false;
    bool vertical_flag_single = false;
    bool single_part_flag = false;
    int count1 = camera_.count_bin1;
    int count2 = camera_.count_bin2;
    int count3 = camera_.count_bin3;
    int count4 = camera_.count_bin4;
    int count5 = camera_.count_bin5;
    int count6 = camera_.count_bin6;
    // std::vector<geometry_msgs::Pose> part_pose_wrt_bin1;
    geometry_msgs::Pose part_pose_wrt_bin1;

    int iter_count = 0;

    std::vector<float> position_x;
    std::vector<float> position_y;

    std::vector<int> count_set = {count1, count2, count3, count4, count5, count6};

    switch(bin_number)
    {
        case 1: 
            iter_count = count1;
            part_in_bin = bin1_part;
            if(bin1_part == bin2_part)
                iter_count = count1 + count2;
            if(bin1_part == bin3_part)
                iter_count = count1 + count3;
            if(bin1_part == bin4_part)
                iter_count = count1 + count4;
            if(bin1_part == bin5_part)
                iter_count = count1 + count5;
            if(bin1_part == bin6_part)
                iter_count = count1 + count6;
            break;
        case 2:
            iter_count = count2;
            part_in_bin = bin2_part;
            if(bin2_part == bin1_part)
                iter_count = count2 + count1;
            if(bin2_part == bin3_part)
                iter_count = count2 + count3;
            if(bin2_part == bin4_part)
                iter_count = count2 + count4;
            if(bin2_part == bin5_part)
                iter_count = count2 + count5;
            if(bin2_part == bin6_part)
                iter_count = count2 + count6;;
            break;
        case 3: 
            iter_count = count3;
            part_in_bin = bin3_part;
            if(bin3_part == bin1_part)
                iter_count = count3 + count1;
            if(bin3_part == bin2_part)
                iter_count = count3 + count2;
            if(bin3_part == bin4_part)
                iter_count = count3 + count4;
            if(bin3_part == bin5_part)
                iter_count = count3 + count5;
            if(bin3_part == bin6_part)
                iter_count = count3 + count6;
            break;
        case 4:
            iter_count = count4;
            part_in_bin = bin4_part;
            if(bin4_part == bin1_part)
                iter_count = count4 + count1;
            if(bin4_part == bin2_part)
                iter_count = count4 + count2;
            if(bin4_part == bin3_part)
                iter_count = count4 + count3;
            if(bin4_part == bin5_part)
                iter_count = count4 + count5;
            if(bin4_part == bin6_part)
                iter_count = count4 + count6;
            break;
        case 5: 
            iter_count = count5;
            part_in_bin = bin5_part;
            if(bin5_part == bin1_part)
                iter_count = count5 + count1;
            if(bin5_part == bin2_part)
                iter_count = count5 + count2;
            if(bin5_part == bin3_part)
                iter_count = count5 + count3;
            if(bin5_part == bin4_part)
                iter_count = count5 + count4;
            if(bin5_part == bin6_part)
                iter_count = count5 + count6;
            break;
        case 6:
            iter_count = count6;
            part_in_bin = bin6_part;
            if(bin6_part == bin1_part)
                iter_count = count6 + count1;
            if(bin6_part == bin2_part)
                iter_count = count6 + count2;
            if(bin6_part == bin3_part)
                iter_count = count6 + count3;
            if(bin6_part == bin4_part)
                iter_count = count6 + count4;
            if(bin6_part == bin5_part)
                iter_count = count6 + count5;
            break;
        default:
            break;
    }
    //for bin1:
    //if bin is not empty
    position_x.clear();
    position_y.clear();
    if(iter_count != 0 /*&& iter_count < threshold*/ && part_in_bin == conveyor_part_type){
        ROS_INFO_STREAM("Part in bin: "<< part_in_bin << " " << "and count: " << iter_count);
        for(int i = 1; i <= iter_count; i++){
            try{
            std::string frame_product = "logical_camera_" + std::to_string(bin_number)+ "_" + part_in_bin + "_" + std::to_string(i) + "_frame";
            // auto part_pose = camera_.GetPartPose("/world", frame_product);
            std::string bin_frame = "/bin" + std::to_string(bin_number) + "_frame";
            part_pose_wrt_bin1 = camera_.GetPartPose(bin_frame, frame_product);
            ROS_INFO_STREAM("From fillbin pose: "<<part_pose_wrt_bin1);            
            position_x.push_back(part_pose_wrt_bin1.position.x);
            position_y.push_back(part_pose_wrt_bin1.position.y);
        }
        catch(tf2::TransformException &ex){
            ROS_WARN("%s",ex.what());
            continue;
        }
        }
        auto max_x = round_up(*std::max_element(position_x.begin(), position_x.end()), 2);
        auto max_y = round_up(*std::max_element(position_y.begin(), position_y.end()), 2);
        auto min_x = round_up(*std::min_element(position_x.begin(), position_x.end()), 2);
        auto min_y = round_up(*std::min_element(position_y.begin(), position_y.end()), 2);
        
        ROS_INFO_STREAM("X: "<<round_up(max_x*min_x, 2));
        ROS_INFO_STREAM("Y: "<<round_up(max_y*min_y, 2));

        //pattern of storage
        if(round_up(max_x*min_x, 2) <= 0 && iter_count < threshold && iter_count != 1){
            vertical_flag_single = true;
            ROS_INFO_STREAM("parts are vertical, single line");

        }
        if(round_up(max_y*min_y, 2) <= 0 && iter_count < threshold && iter_count != 1){
            horizontal_flag_single = true;
            ROS_INFO_STREAM("parts are horizontal, single line");
        }
        if(iter_count >= threshold){
            ROS_INFO_STREAM("Enough parts present in bin");
        }
        if(iter_count == 1){
            ROS_INFO_STREAM("Single part found..");
            single_part_flag = true;
        }
        

        //vertical, single line
        if(vertical_flag_single){
            // ROS_INFO_STREAM("Max Y: " << max_y);
            if(max_y >= 0){
                ROS_INFO_STREAM("Parts in single file RHS");
                position_available_bin1.erase(position_available_bin1.begin(), position_available_bin1.end());
                ROS_INFO_STREAM("Parts to be added in bin 1: " << threshold - iter_count);
                for(int j = 1; j <= (threshold - iter_count); j++){
                    //todo - diff thresholds for different part types
                    new_place_pose.position.x = max_x;
                    new_place_pose.position.y = max_y - 0.2;
                    new_place_pose.position.z = 0.005;
                    new_place_pose.orientation = part_pose_wrt_bin1.orientation;
                    position_available_bin1.push_back(new_place_pose);
                    new_place_pose.position.x = min_x;
                    new_place_pose.position.y = max_y - 0.2;
                    new_place_pose.position.z = 0.005;
                    new_place_pose.orientation = part_pose_wrt_bin1.orientation;
                    position_available_bin1.push_back(new_place_pose);
                    max_y = max_y - 0.2;
                    min_y = min_y - 0.2;
                }
                max_x = round_up(*std::max_element(position_x.begin(), position_x.end()), 2);
                max_y = round_up(*std::max_element(position_y.begin(), position_y.end()), 2);
                min_x = round_up(*std::min_element(position_x.begin(), position_x.end()), 2);
                min_y = round_up(*std::min_element(position_y.begin(), position_y.end()), 2);

            }
            else if(max_y < 0){
                ROS_INFO_STREAM("Parts in single file LHS");
                position_available_bin1.erase(position_available_bin1.begin(), position_available_bin1.end());
                ROS_INFO_STREAM("Parts to be added in bin 1: " << threshold - iter_count);
                for(int j = 1; j <= (threshold - iter_count); j++){
                    //todo - diff thresholds for different part types
                    new_place_pose.position.x = max_x;
                    new_place_pose.position.y = max_y + 0.2;
                    new_place_pose.position.z = 0.005;
                    new_place_pose.orientation = part_pose_wrt_bin1.orientation;
                    position_available_bin1.push_back(new_place_pose);
                    new_place_pose.position.x = min_x;
                    new_place_pose.position.y = max_y + 0.2;
                    new_place_pose.position.z = 0.005;
                    new_place_pose.orientation = part_pose_wrt_bin1.orientation;
                    position_available_bin1.push_back(new_place_pose);
                    max_y = max_y + 0.2;
                    min_y = min_y + 0.2;
                }
                max_x = round_up(*std::max_element(position_x.begin(), position_x.end()), 2);
                max_y = round_up(*std::max_element(position_y.begin(), position_y.end()), 2);
                min_x = round_up(*std::min_element(position_x.begin(), position_x.end()), 2);
                min_y = round_up(*std::min_element(position_y.begin(), position_y.end()), 2);

            }            

        }
        if(horizontal_flag_single){
            if(max_x >= 0){
                ROS_INFO_STREAM("Parts in single file up");
                position_available_bin1.erase(position_available_bin1.begin(), position_available_bin1.end());
                ROS_INFO_STREAM("Parts to be added in bin 1: " << threshold - iter_count);
                for(int j = 1; j <= (threshold - iter_count); j++){
                    //todo - diff thresholds for different part types
                    new_place_pose.position.x = max_x - 0.2;
                    new_place_pose.position.y = max_y;
                    new_place_pose.position.z = 0.005;
                    new_place_pose.orientation = part_pose_wrt_bin1.orientation;
                    position_available_bin1.push_back(new_place_pose);
                    new_place_pose.position.x = max_x - 0.2;
                    new_place_pose.position.y = min_y;
                    new_place_pose.position.z = 0.005;
                    new_place_pose.orientation = part_pose_wrt_bin1.orientation;
                    position_available_bin1.push_back(new_place_pose);
                    max_x = max_x - 0.2;
                    min_x = min_x - 0.2;
                }
                max_x = round_up(*std::max_element(position_x.begin(), position_x.end()), 2);
                max_y = round_up(*std::max_element(position_y.begin(), position_y.end()), 2);
                min_x = round_up(*std::min_element(position_x.begin(), position_x.end()), 2);
                min_y = round_up(*std::min_element(position_y.begin(), position_y.end()), 2);

            }
            if(max_x < 0){
                ROS_INFO_STREAM("Parts in single file down");
                position_available_bin1.erase(position_available_bin1.begin(), position_available_bin1.end());
                ROS_INFO_STREAM("Parts to be added in bin 1: " << threshold - iter_count);
                for(int j = 1; j <= (threshold - iter_count); j++){
                    //todo - diff thresholds for different part types
                    new_place_pose.position.x = max_x + 0.2;
                    new_place_pose.position.y = max_y;
                    new_place_pose.position.z = 0.005;
                    new_place_pose.orientation = part_pose_wrt_bin1.orientation;
                    position_available_bin1.push_back(new_place_pose);
                    new_place_pose.position.x = max_x + 0.2;
                    new_place_pose.position.y = min_y;
                    new_place_pose.position.z = 0.005;
                    new_place_pose.orientation = part_pose_wrt_bin1.orientation;
                    position_available_bin1.push_back(new_place_pose);
                    max_x = max_x + 0.2;
                    min_x = min_x + 0.2;
                }
                max_x = round_up(*std::max_element(position_x.begin(), position_x.end()), 2);
                max_y = round_up(*std::max_element(position_y.begin(), position_y.end()), 2);
                min_x = round_up(*std::min_element(position_x.begin(), position_x.end()), 2);
                min_y = round_up(*std::min_element(position_y.begin(), position_y.end()), 2);

            }
            
        }
        if(single_part_flag){
            if(max_x > 0 and max_y > 0){
                new_place_pose.position.x = max_x;
                new_place_pose.position.y = -max_y;
                new_place_pose.position.z = 0.005;
                new_place_pose.orientation = part_pose_wrt_bin1.orientation;
                position_available_bin1.push_back(new_place_pose);
                new_place_pose.position.x = -max_x;
                new_place_pose.position.y = -max_y;
                new_place_pose.position.z = 0.005;
                new_place_pose.orientation = part_pose_wrt_bin1.orientation;
                position_available_bin1.push_back(new_place_pose);
                new_place_pose.position.x = -max_x;
                new_place_pose.position.y = max_y;
                new_place_pose.position.z = 0.005;
                new_place_pose.orientation = part_pose_wrt_bin1.orientation;
                position_available_bin1.push_back(new_place_pose);
            }
            if(max_x > 0 and max_y < 0){
                new_place_pose.position.x = max_x;
                new_place_pose.position.y = abs(max_y);
                new_place_pose.position.z = 0.005;
                new_place_pose.orientation = part_pose_wrt_bin1.orientation;
                position_available_bin1.push_back(new_place_pose);
                new_place_pose.position.x = -max_x;
                new_place_pose.position.y = max_y;
                new_place_pose.position.z = 0.005;
                new_place_pose.orientation = part_pose_wrt_bin1.orientation;
                position_available_bin1.push_back(new_place_pose);
                new_place_pose.position.x = -max_x;
                new_place_pose.position.y = abs(max_y);
                new_place_pose.position.z = 0.005;
                new_place_pose.orientation = part_pose_wrt_bin1.orientation;
                position_available_bin1.push_back(new_place_pose);
            }
            if(max_x < 0 and max_y < 0){
                new_place_pose.position.x = abs(max_x);
                new_place_pose.position.y = abs(max_y);
                new_place_pose.position.z = 0.005;
                new_place_pose.orientation = part_pose_wrt_bin1.orientation;
                position_available_bin1.push_back(new_place_pose);
                new_place_pose.position.x = abs(max_x);
                new_place_pose.position.y = max_y;
                new_place_pose.position.z = 0.005;
                new_place_pose.orientation = part_pose_wrt_bin1.orientation;
                position_available_bin1.push_back(new_place_pose);
                new_place_pose.position.x = max_x;
                new_place_pose.position.y = abs(max_y);
                new_place_pose.position.z = 0.005;
                new_place_pose.orientation = part_pose_wrt_bin1.orientation;
                position_available_bin1.push_back(new_place_pose);
            }
            if(max_x < 0 and max_y > 0){
                new_place_pose.position.x = abs(max_x);
                new_place_pose.position.y = abs(max_y);
                new_place_pose.position.z = 0.005;
                new_place_pose.orientation = part_pose_wrt_bin1.orientation;
                position_available_bin1.push_back(new_place_pose);
                new_place_pose.position.x = abs(max_x);
                new_place_pose.position.y = max_y;
                new_place_pose.position.z = 0.005;
                new_place_pose.orientation = part_pose_wrt_bin1.orientation;
                position_available_bin1.push_back(new_place_pose);
                new_place_pose.position.x = max_x;
                new_place_pose.position.y = -max_y;
                new_place_pose.position.z = 0.005;
                new_place_pose.orientation = part_pose_wrt_bin1.orientation;
                position_available_bin1.push_back(new_place_pose);
            }

        }
    }
    else if(iter_count == 0){
        //4 parts at 4 corners
        ROS_INFO_STREAM("Empty bin found, bin number: " << bin_number);
        int i = 1;       
        new_place_pose.orientation.x = 0.0;
        new_place_pose.orientation.y = 0.0;
        new_place_pose.orientation.z = 0.0;
        new_place_pose.orientation.w = 1.0;
        new_place_pose.position.x = 0.2;
        new_place_pose.position.y = 0.2;
        new_place_pose.position.z = 0.003;
        position_available_bin1.push_back(new_place_pose);
        new_place_pose.position.x = 0.2;
        new_place_pose.position.y = -0.2;
        new_place_pose.position.z = 0.003;
        position_available_bin1.push_back(new_place_pose);
        new_place_pose.position.x = -0.2;
        new_place_pose.position.y = -0.2;
        new_place_pose.position.z = 0.003;
        position_available_bin1.push_back(new_place_pose);
        new_place_pose.position.x = -0.2;
        new_place_pose.position.y = 0.2;
        new_place_pose.position.z = 0.003;
        position_available_bin1.push_back(new_place_pose);        
    }
    else if(iter_count >= threshold){
        ROS_INFO_STREAM("Bin full");
    }
    return position_available_bin1;
    }
    
bool AriacOrderManager::PickPartconveyor()
{
	ROS_WARN_STREAM("cAME IN PPC");
	arm1_.SendRobotPosition({0.0,-3.11,-2.39,-1.63,-0.70,1.57,0});
	// arm1_.SendRobotPosition({0.0,-3.11,-2.39,-1.63,-0.70,1.57,0});
	arm2_.SendRobotPosition2({0.0,-3.11,-2.39,-1.63,-0.70,1.57,0});

    int part_count=1;
    std::string part_under7;
    std::string part_under8;
    int FinalDecision;
    int Decision7;
    int Decision8;
    bool state;
    geometry_msgs::Pose keep;
    geometry_msgs::PoseStamped StampedPose_in, StampedPose_out;
    ros::Time start, end;
    // ROS_INFO_STREAM("Time start");
    start = ros::Time::now();
    
    while((end - start).toSec()*10 < 2000)
    {
    	// ROS_WARN_STREAM("Diff: "<<(end - start).toSec());
        ros::spinOnce();
    	FinalDecision=0;
    	int c=0;
    	// store parts to avoid in between change
    	part_under7=camera_.logicalcam_7;
    	part_under8=camera_.logicalcam_8;

        // Take decision whether to pick part or not
        // If Yes, then which Arm to use and which bin to store 

        // Both Logical sensors see a part
        if(part_under7!="" && part_under8!="")
        {
        	// Store Decisions
        	Decision7=DecideBinArm(camera_.logicalcam_7);
        	Decision8=DecideBinArm(camera_.logicalcam_8);

        	// Both Parts Not Required
        	if(Decision7==0 && Decision8==0)
        	{
        		// Do Nothing
        	}
        	// Both Parts Required
        	else if(Decision7!=0 && Decision8!=0)
        	{
                FinalDecision=Decision7;
                c=FinalDecision/10;
                state=grasp(FinalDecision-c*10,part_under7);
        	}
        	// Part under Logical 7 required
        	else if(Decision7!=0 && Decision8==0)
        	{
        		FinalDecision=Decision7;
        		c=FinalDecision/10;
        		 state=grasp(FinalDecision-c*10,part_under7);
        	}
        	// Part under Logical 8 required
        	else if(Decision7==0 && Decision8!=0)
        	{
        		FinalDecision=Decision8;
        		c=FinalDecision/10;
        		 state=grasp(FinalDecision-c*10,part_under8);
        	}
        }

        // Logical sensors 7 sees a part
        else if(part_under7!="" && part_under8=="")
        {
            Decision7 = DecideBinArm(camera_.logicalcam_7);
           if(Decision7!=0)
            {
            	FinalDecision=Decision7;  
            	 c=FinalDecision/10;
            	 state=grasp(FinalDecision-c*10,part_under7);          }
            }

        // Logical sensors 8 sees a part
        else if(part_under7=="" && part_under8!="")
        {
            Decision8 = DecideBinArm(camera_.logicalcam_8);
            if(Decision8!=0)
            {
            	FinalDecision=Decision8;
            	c=FinalDecision/10;
            	 state=grasp(FinalDecision-c*10,part_under8);
            }
        }
        if(FinalDecision!=0)
        {
        	int bin=FinalDecision/10;
        	int arm=FinalDecision-bin*10;

        	if(bin==1)
        	{
        		ROS_INFO_STREAM(bin1_poses[0]);
        		StampedPose_in.header.frame_id = "/bin1_frame";
                StampedPose_in.pose = bin1_poses[0];
                part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
        		keep=StampedPose_out.pose;
        		bin1_poses.erase(bin1_poses.begin());
        	}
        	if(bin==2)
        	{
        		StampedPose_in.header.frame_id = "/bin2_frame";
				StampedPose_in.pose = bin2_poses[0];
                part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
        		keep=StampedPose_out.pose;
        		bin2_poses.erase(bin2_poses.begin());
        	}
        	if(bin==3)
        	{
        		StampedPose_in.header.frame_id = "/bin3_frame";
                StampedPose_in.pose = bin3_poses[0];
                part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
        		keep=StampedPose_out.pose;
        		bin3_poses.erase(bin3_poses.begin());
        	}
        	if(bin==4)
        	{
        		StampedPose_in.header.frame_id = "/bin4_frame";
                StampedPose_in.pose = bin4_poses[0];
                part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
        		keep=StampedPose_out.pose;
        		bin4_poses.erase(bin4_poses.begin());
        	}
        	if(bin==5)
        	{
        		StampedPose_in.header.frame_id = "/bin5_frame";
                StampedPose_in.pose = bin5_poses[0];
                part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
        		keep=StampedPose_out.pose;
        		bin5_poses.erase(bin5_poses.begin());
        	}
        	if(bin==6)
        	{
        		StampedPose_in.header.frame_id = "/bin6_frame";
        		StampedPose_in.pose = bin6_poses[0];
                part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
        		keep=StampedPose_out.pose;
        		bin6_poses.erase(bin6_poses.begin());
        	}
            
            // Place the Part in Bin
        	if(arm==1 && state==true)
        	{
        		arm1_.DropPart(keep,10);

        	}
        	if(arm==2 && state ==true)
        	{
        		arm2_.DropPart(keep,20);
        	}
        	part_count +=1;
        	UpdateBin();
        }
        end = ros::Time::now();
        ros::spinOnce();
    }
    ROS_WARN_STREAM("Time up");
}

bool AriacOrderManager::grasp(int arm, std::string part)
{
	ROS_WARN_STREAM("ARM"<<arm);
	geometry_msgs::Pose pick;
	pick.orientation.x=0.001324;
	pick.orientation.y=0.002297;
	pick.orientation.z=0.002667;
	pick.orientation.w=1.0;
	pick.position.z=0.0;
    if(part=="gear_part")
    {
        if(arm==1)
        {
            pick.position.y=0.87;
            pick.position.z  =0.935;
	    }
	    else
	    {
             // ROS_INFO_STREAM("bABURAO");
             pick.position.y=-1.06; //-1.24
             pick.position.z  =0.933;
	    }
    }
    else if(part=="piston_rod_part")
    {
        if(arm==1)
        {
            pick.position.y=0.82;
            pick.position.z  =0.922;
	    }
	    else
	    {
	    	// ROS_INFO_STREAM("bABURAO");
             pick.position.y=-1.24;
             pick.position.z  =0.922;
	    }
    }
    else if(part=="disk_part")
    {
        if(arm==1)
        {
            ROS_INFO_STREAM("bABURAO");
            pick.position.y=0.85;
            pick.position.z  =0.944;
	    }
	    else
	    {    
             pick.position.y=-1.00;
             pick.position.z  =0.944;
	    }
	// pick.position.z  =0.90;
    }
    else if(part=="pulley_part")
    {
        if(arm==1)
        {
            pick.position.y=1.36;
	    }
	    else
	    {
             pick.position.y=-0.97;
	    }
	pick.position.z =0.9999;
    }
    else if(part=="gasket_part")
    {
        if(arm==1)
        {
            pick.position.y=0.75;
            pick.position.z  =0.93;
	    }
	    else
	    {
             pick.position.y=-1.24;
             pick.position.z  =0.93;
	    }
    }
    if(arm==1)
    {   while(camera_.getBeam1()!=true)
    	{
    		 ros::spinOnce();
    	}
    	pick.position.x=1.22;
        arm1_.GripperToggle(true);
        bool s=arm1_.go(pick);
        ros::Duration(0.3).sleep();
        arm1_.SendRobotPosition({0.0,-3.11,-2.39,-1.63,-0.70,1.57,0});
        return s;
    }
    if(arm==2)
    {
    	 while(camera_.getBeam2()!=true)
    	{
    		 ros::spinOnce();
    	}
    	pick.position.x=1.22;
        arm2_.GripperToggle(true);
        bool s=arm2_.go(pick);
        ros::Duration(0.3).sleep();
        arm2_.SendRobotPosition2({0.0,-3.11,-2.39,-1.63,-0.70,1.57,0});
        return s;
    }
}

