
#include "order_manager.h"
#include <osrf_gear/AGVControl.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>

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
     * [5] = wrist_2_joint
     * [6] = wrist_3_joint
     */
    flipped_arm1_pose_1 = {1.18, 4.59, -0.50, 0.99, 4.31, -1.53, 0};
    flipped_arm1_pose_2 = {1.18, 4.59, -0.50, -1.0, 4.31, -1.53, 0};
    flipped_arm1_pose_3 = {1.11, 4.59, -1.00, 1.32, 4.31, -1.53, 0};
    flipped_arm1_pose_4 = {1.11, 4.59, -1.00, 1.35, 4.31, -1.53, 0};

    // for agv_id = 2
    flipped_arm1_pose_5 = {0.50, 4.59, -0.50, 0.99, 4.32, 1.55, 0};
    flipped_arm1_pose_6 = {-0.50, 4.59, -0.50, 0.99, 4.32, 1.55, 0};
    flipped_arm2_pose_3 = {-0.59, 1.30, -1.37, 1.84, 4.10, -1.53, 0};
    flipped_arm2_pose_4 = {-0.59, 1.30, -1.37, 1.97, 4.10, -1.53, 0};

    flipped_arm2_pose_1 = {0.68, 1.40, -0.47, 0.95, 4.31, -1.53, 0};
    flipped_arm2_pose_2 = {0.72, 1.40, -0.47, 0.85, 4.35, 1.51, 0};
    kit_drop_pose_ = {2.65, 1.57, -1.60, 2.0, 4.30, -1.53, 0};
}

AriacOrderManager::~AriacOrderManager(){}


void AriacOrderManager::OrderCallback(const osrf_gear::Order::ConstPtr& order_msg) {
    ROS_WARN_STREAM(">>>>> OrderCallback");
    received_orders_.push_back(*order_msg);
    bin1_part = camera_.LogicalCamera1PartType();
    bin2_part = camera_.LogicalCamera2PartType();
    bin3_part = camera_.LogicalCamera3PartType();
    bin4_part = camera_.LogicalCamera4PartType();
    bin5_part = camera_.LogicalCamera5PartType();
    bin6_part = camera_.LogicalCamera6PartType();
    bin_parts = {bin1_part, bin2_part, bin3_part,bin4_part, bin5_part, bin6_part};
    // arm1_.SendRobot1();
    // arm2_.SendRobot2();

    for (const auto &order:received_orders_){

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

            for (const auto &product: products){

                ROS_INFO_STREAM("Product"<<product);
                product_type_pose_.first = product.type;
                product_type_pose_.second = product.pose;
                productlist_type.push_back(product.type);
                productlist_pose.push_back(product.pose);
                if (product.type == bin1_part and agv_id == 1) {
                  ROS_INFO_STREAM_THROTTLE(5, "Out of range for Arm1");
                }
                if (product.type == bin6_part and agv_id == 2) {
                  ROS_INFO_STREAM_THROTTLE(10, "Out of range for Arm2");
                }
            }

            for (auto it = productlist_type.cbegin(); it != productlist_type.cend(); it++){
                std::cout << *it << ',' << std::endl;;
                if(*it == "piston_rod_part"){
                    piston_count_++;
                }
                else if(*it == "gear_part"){
                    gear_count_++;
                }

            }
            // bin1_part = camera_.LogicalCamera1PartType();
            // bin2_part = camera_.LogicalCamera2PartType();
            // bin3_part = camera_.LogicalCamera3PartType();
            // bin4_part = camera_.LogicalCamera4PartType();
            // bin5_part = camera_.LogicalCamera5PartType();
            // bin6_part = camera_.LogicalCamera6PartType();
            // ROS_INFO_STREAM("Piston count in order: " << piston_count_);
            // ROS_INFO_STREAM("Gear count in order " << gear_count_);
            ROS_INFO_STREAM("Part read from bin 1: " << bin1_part);
            ROS_INFO_STREAM("Part read from bin 2: " << bin2_part);
            ROS_INFO_STREAM("Part read from bin 3: " << bin3_part);
            ROS_INFO_STREAM("Part read from bin 4: " << bin4_part);
            ROS_INFO_STREAM("Part read from bin 5: " << bin5_part);
            ROS_INFO_STREAM("Part read from bin 6: " << bin6_part);
        }
    }
}



/**
 * @brief Get the product frame for a product type
 * @param product_type
 * @return
 */
std::string AriacOrderManager::GetProductFrame(std::string product_type) {
    //--Grab the last one from the list then remove itle()
    ROS_WARN_STREAM("Came Here");

    if (!product_frame_list_.empty() && product_frame_list_.count(product_type)!=0) {
        std::string frame = product_frame_list_[product_type].back();
        // ROS_INFO_STREAM("Frame >>>> " << frame);
        product_frame_list_[product_type].pop_back();
        return frame;
    } else {
         ROS_ERROR_STREAM("No product frame found for " << product_type);
         ROS_INFO_STREAM("Getting " << product_type << " from the conveyor Belt");
//		else return "NOO";

         bool failed_pick = arm1_.PickPartconveyor(product_type);
		 while(!failed_pick){
			 failed_pick = arm1_.PickPartconveyor(product_type);  //robot_controller
		 }
            bin_pose.position.x=-0.2;
            bin_pose.position.y=-0.2;
            bin_pose.position.z=0.1;

            bin_pose.orientation.x=-0.703527;
            bin_pose.orientation.y=0.0254473;
            bin_pose.orientation.z=-0.710205;
            bin_pose.orientation.w=-0.0032328;

            geometry_msgs::PoseStamped StampedPose_in,StampedPose_out;
            StampedPose_in.header.frame_id = "/bin6_frame";
            StampedPose_in.pose = bin_pose;
            part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);

            // Drop in the adjacent bin
            // arm1_.DropPart(StampedPose_out.pose);

            ros::spinOnce();
            product_frame_list_ = camera_.get_product_frame_list();

            ROS_INFO_STREAM("Length"<<product_frame_list_.size());
            std::string frame = product_frame_list_[product_type].back();

            product_frame_list_[product_type].pop_back();
            return frame;


  }

}

bool AriacOrderManager::PickAndPlace(const std::pair<std::string,geometry_msgs::Pose> product_type_pose, int agv_id) {
    ros::spinOnce();
    std::string product_type = product_type_pose.first;

    std::string product_frame = GetProductFrame(product_type);

    auto part_pose = camera_.GetPartPose("/world", product_frame);


    if(product_type == "pulley_part")
        part_pose.position.z += 0.05;
    if(product_type == "piston_rod_part")
        part_pose.position.z -= 0.0155;
    if(product_type == "gear_part")
        part_pose.position.z -= 0.011;
    if(product_type == "disk_part")
        part_pose.position.z = part_pose.position.z;
    if(product_type == "gasket_part")
        part_pose.position.z = part_pose.position.z;

    //--task the robot to pick up this part again from the bin
    if (isFlipped) {
      FlippedPart(agv_id, part_pose);
    }
    else if (agv_id == 1) {
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
    if( not success and agv_id == 1) {
      success = arm1_.DropPart(StampedPose_out.pose, agv_id); //robot_controller
    } else {
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

// flipped_arm1_pose_1 = {1.18, 4.59, -0.50, 0.99, 4.31, -1.53, 0};
// flipped_arm1_pose_2 = {1.18, 4.59, -0.50, -1.0, 4.31, -1.53, 0};
// flipped_arm1_pose_3 = {1.11, 4.59, -1.00, 1.32, 4.31, -1.53, 0};
//
// flipped_arm2_pose_1 = {0.68, 1.40, -0.47, 0.95, 4.31, -1.53, 0};
// flipped_arm2_pose_2 = {0.68, 1.40, -0.47, 0.82, 4.35, 1.51, 0};
// kit_drop_pose_ = {2.65, 1.57, -1.60, 2.0, 4.30, -1.53, 0};
//
// arm1_.SendRobotPosition(flipped_arm1_pose_1);
// this->arm1_.GripperToggle(false);
// // SendRobotPosition(flipped_arm1_pose_2);
// arm1_.SendRobotPosition(flipped_arm1_pose_2);
// this->arm2_.GripperToggle2(true);
// arm2_.SendRobotPosition2(flipped_arm2_pose_1);
// arm2_.SendRobotPosition2(flipped_arm2_pose_2);
// this->arm1_.GripperToggle(true);
// this->arm2_.GripperToggle2(false);
// arm1_.SendRobotPosition(flipped_arm1_pose_3);

void AriacOrderManager::FlippedPart(int agv_id, auto pose) {
  if (agv_id == 1) {
    home_joint_pose_1 = {1.18, 3.11, -1.60, 2.0, 4.30, -1.53, 0};
    arm1_.SendRobotPosition(home_joint_pose_1);
    bool failed_pick = arm2_.PickPart(pose);
    arm2_.SendRobotPosition2(flipped_arm2_pose_2);
    this->arm1_.GripperToggle(true);
    arm1_.SendRobotPosition(flipped_arm1_pose_3);
    arm1_.SendRobotPosition(flipped_arm1_pose_4);
    this->arm2_.GripperToggle2(false);
  }
  if (agv_id == 2) {
    bool failed_pick = arm1_.PickPart(pose, 1);
    arm1_.SendRobotPosition(flipped_arm1_pose_5);
    arm1_.SendRobotPosition(flipped_arm1_pose_6);
    arm2_.SendRobotPosition2(flipped_arm2_pose_3);
    arm2_.SendRobotPosition2(flipped_arm2_pose_4);
    this->arm2_.GripperToggle2(true);
    this->arm1_.GripperToggle(false);
  }
}

void AriacOrderManager::ExecuteOrder() {
    ROS_WARN(">>>>>> Executing order...");
    //scanned_objects_ = camera_.GetParts();

    //-- used to check if pick and place was successful
    bool pick_n_place_success{false};

    std::list<std::pair<std::string,geometry_msgs::Pose>> failed_parts;
    ros::spinOnce();
    ros::Duration(1.0).sleep();
    product_frame_list_ = camera_.get_product_frame_list();
    int i=0;
    bool update=false;
    int flag=0;
    geometry_msgs::PoseStamped StampedPose_in, StampedPose_out;

    // for (const auto &order:received_orders_){
    while(true) {
        ROS_INFO_STREAM("in");

        const auto &order=received_orders_[0];

        int size=received_orders_.size();
        auto order_id = order.order_id;
        auto shipments = order.shipments;


        for (const auto &shipment: shipments){
            auto shipment_type = shipment.shipment_type;
            auto agv = shipment.agv_id.back();//--this returns a char
            //-- if agv is any then we use AGV1, else we convert agv id to int
            //--agv-'0' converts '1' to 1 and '2' to 2
            ROS_INFO_STREAM(agv);
            int agv_id = (shipment.agv_id == "any") ? 2 : agv - '0';
            auto products = shipment.products;
            ROS_INFO_STREAM("Order ID: " << order_id);
            ROS_INFO_STREAM("Shipment Type: " << shipment_type);
            ROS_INFO_STREAM("AGV ID: " << agv_id);


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
                ROS_INFO_STREAM("Product pose: " << product_type_pose_.second.position.x);

                StampedPose_in.header.frame_id = "/kit_tray_1";
                StampedPose_in.pose = product.pose;
                part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);

                auto f = std::make_pair(product.type, StampedPose_out.pose);

                if(std::find(placed_parts.begin(),placed_parts.end(),f)==placed_parts.end())
                {


                    // ros::spinOnce();
                    // product_frame_list_ = camera_.get_product_frame_list();
//                bool pick_n_place_success = false;
//                while (not pick_n_place_success){
                if (product.type == bin1_part and agv_id == 1) {
                  // bool pick_n_place_success = false;
                  // while (not pick_n_place_success){
                  std::string num;
                  for (int i = 0; i < bin_parts.size(); i++) {
                    if (bin_parts[i].empty() and i != 5) {
                      empty_bin = "bin" + std::to_string(i+1);
                      num = std::to_string(i+1);
                      break;
                    }
                  }
                   while(pick_n_place_success==false){
                  pick_n_place_success =  PickAndPlace(product_type_pose_, empty_bin);
              }
               OutOfReach("arm1", num, product_type_pose_, agv_id);
                } else if(product.type == bin6_part and agv_id == 2) {
                  std::string num;
                  for (int i = 0; i < bin_parts.size(); i++) {
                    if (bin_parts[i].empty() and i != 5) {
                      empty_bin = "bin" + std::to_string(i+1);
                      num = std::to_string(i+1);
                      break;
                    }
                  }
                   while(pick_n_place_success==false){
                  pick_n_place_success =  PickAndPlace(product_type_pose_, empty_bin);
              }
                  OutOfReach("arm2", num, product_type_pose_, agv_id);
                } else {
                while(pick_n_place_success==false){
                  pick_n_place_success =  PickAndPlace(product_type_pose_, agv_id);
                }
                }
                ROS_INFO_STREAM(placed_parts.size());
                ROS_INFO_STREAM(placed_parts[placed_parts.size()-1].first);
                pick_n_place_success=false;
            }
                //--todo: What do we do if pick and place fails?
            }
            //wait 5 sec
            if(update==false)
            {
            ros::Time a=ros::Time::now();
            ros::Duration(5).sleep();
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
        ROS_INFO_STREAM(received_orders_.size());
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
    ROS_INFO_STREAM(i);

    if (agv_id == 1) {
      drop_pose_ = {2.4, 1.57, -1.60, 2.0, 4.30, -1.53, 0};
      arm1_.PickPart(placed_parts[i].second, agv_id);
      arm1_.SendRobotPosition(drop_pose_);
      arm1_.GripperToggle(false);
    } else {
      drop_pose_ = {-0.89, -2.11, -1.60, 2.0, 4.30, -1.53, 0};
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
    for(int k=i+1;k<diff+1;k++) {
        const auto &var = received_orders_[k];
        ROS_INFO_STREAM("Order"<<var.order_id);
        auto new_order=var.order_id;
        if(order_id+"_update_"+std::to_string(update_no)==new_order) {
            ROS_INFO_STREAM("Found a New Order");
            auto new_shipments = var.shipments;
            const auto &var1 = new_shipments[0];
            auto new_agv=var1.agv_id.back();
            int new_agv_id = (var1.agv_id == "any") ? 1 : new_agv - '0';
            auto new_products = var1.products;

            int parts_in_order=0;
            int parts_already_placed=0;
            bool all_different=false;
            std::vector<int> present;
            std::vector<int> present_wrong_location;

            // Check which parts exist at same location in new order
            // Check which parts are at diffrent locations (Can be shuffled)
            geometry_msgs::PoseStamped StampedPose_in, StampedPose_out;
            if(new_agv_id==agv_id)
            {
            for (const auto &p: new_products)
            {
              parts_in_order++;
              ROS_INFO_STREAM("new"<<p.type);
              StampedPose_in.header.frame_id = "/kit_tray_1";
              StampedPose_in.pose = p.pose;
              part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
              ROS_INFO_STREAM("new"<<StampedPose_out.pose);
              for(int q=0;q<placed_parts.size();q++)
              {
                ROS_INFO_STREAM("old"<<placed_parts[q].first);
                ROS_INFO_STREAM("old"<<placed_parts[q].second);
                if(p.type==placed_parts[q].first && StampedPose_out.pose==placed_parts[q].second)
                {
                    present.push_back(q);
                }
                else if(p.type == placed_parts[q].first)
                {
                    present_wrong_location.push_back(q);
                }
              }
            }
            ROS_INFO_STREAM("Length"<<present.size());
            ROS_INFO_STREAM("Length"<<placed_parts.size());
            std::vector<std::pair<std::string,geometry_msgs::Pose>> parts_to_remove;
            parts_to_remove=placed_parts;

            // Keep parts which are at right pose or can be shuffled
            if(present.size()>0 )
            {
                ROS_INFO_STREAM("Dropping unrequired parts");
                for(int l=0;l<present.size();l++){
                    parts_to_remove.erase(parts_to_remove.begin() + present[l]);
                }
                for(int h=0;h<placed_parts.size();h++)
                {
                    if(std::find(present.begin(),present.end(),h)==present.end())
                    {
                      placed_parts.erase(placed_parts.begin() + h);
                    }
                }
                ROS_INFO_STREAM("Length"<<placed_parts.size());
                dropallparts(parts_to_remove, agv_id);
                return true;
            }
        }




            // drop all parts if updated order on different AGV
            // drop all parts if all parts are different
            if(agv_id!=new_agv_id || (present.size()==0 && present_wrong_location.size()==0) )
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

bool AriacOrderManager::PickAndPlace(std::pair<std::string,geometry_msgs::Pose> product_type_pose, std::string empty_bin) {
    std::string product_type = product_type_pose.first;

    std::string product_frame = GetProductFrame(product_type);

    auto part_pose = camera_.GetPartPose("/world", product_frame);


    if(product_type == "pulley_part")
        part_pose.position.z += 0.08;
    if(product_type == "piston_rod_part")
        part_pose.position.z -= 0.0155;
    if(product_type == "gear_part")
        part_pose.position.z -= 0.011;
    if(product_type == "disk_part")
        part_pose.position.z = part_pose.position.z;
    if(product_type == "gasket_part")
        part_pose.position.z = part_pose.position.z;
    home_joint_pose_1 = {0.0, 3.11, -1.60, 2.0, 4.30, -1.53, 0};
    home_joint_pose_2 = {-1.18, 3.11, -1.60, 2.0, 4.30, -1.53, 0};

    //--task the robot to pick up this part again from the bin
      // bool failed_pick = arm1_.PickPart(part_pose);
    arm1_.SendRobotPosition(home_joint_pose_1);
    bool failed_pick = arm2_.PickPart(part_pose);

    //--get the pose of the object in the tray from the order
    geometry_msgs::Pose drop_pose = product_type_pose.second;

    geometry_msgs::PoseStamped StampedPose_in, StampedPose_out;

    StampedPose_in.header.frame_id = "/" + empty_bin + "_frame";
    StampedPose_in.pose = drop_pose;
        // ROS_INFO_STREAM("StampedPose_int (" << StampedPose_in.pose.position.x <<","<< StampedPose_in.pose.position.y << "," << StampedPose_in.pose.position.z<<")");
    part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
        // StampedPose_out.pose.position.x += 0.2;
        // StampedPose_out.pose.position.y += 0.2;
        // ROS_INFO_STREAM("StampedPose_out (" << StampedPose_out.pose.position.x <<","<< StampedPose_out.pose.position.y << "," << StampedPose_out.pose.position.z<<")");

        // StampedPose_in.header.frame_id = "/kit_tray_2";
        // StampedPose_in.pose = drop_pose;
        //ROS_INFO_STREAM("StampedPose_in " << StampedPose_in.pose.position.x);
        // part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
        // StampedPose_out.pose.position.z += 0.1;
        // StampedPose_out.pose.position.y += 0.2;
        //ROS_INFO_STREAM("StampedPose_out " << StampedPose_out.pose.position.x);

    // This is checking if part is faulty ior not
    bool success = false;
    if( not success ) {
    	success = arm2_.DropPart(StampedPose_out.pose); //robot_controller
    }
    //   success = arm2_.DropPart(StampedPose_out.pose);
    // }
    // placed_parts.push_back(std::make_pair(product_type,StampedPose_out.pose));


    return true;

}

void AriacOrderManager::OutOfReach(std::string arm, std::string num, std::pair<std::string,geometry_msgs::Pose> product, int agv_id) {
  // geometry_msgs::Pose part_pose;
  geometry_msgs::Pose part_pose = camera_.BinGetPartPose("/world", product.first, num);
  home_joint_pose_1 = {0.0, 3.11, -1.60, 2.0, 4.30, -1.53, 0};

  // bool pick_n_place_success_1 =  arm1_.PickPart(product_type_pose_, agv_id);
  if(product.first == "pulley_part")
      part_pose.position.z += 0.08;
  if(product.first == "piston_rod_part")
      part_pose.position.z -= 0.0155;
  if(product.first == "gear_part")
      part_pose.position.z -= 0.011;
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
