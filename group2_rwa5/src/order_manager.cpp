
#include "order_manager.h"
#include <osrf_gear/AGVControl.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>



//AriacOrderManager::AriacOrderManager(): arm1_{"arm1"}, arm2_{"arm2"}
AriacOrderManager::AriacOrderManager(): arm1_{"arm1"}, arm2_{"arm2"}

{
    order_subscriber_ = order_manager_nh_.subscribe(
            "/ariac/orders", 10,
            &AriacOrderManager::OrderCallback, this);
    conveyor_part_found  = false;

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
                  ROS_INFO_STREAM_THROTTLE(10, "Out of range for Arm2");
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
            }
            
            ROS_INFO_STREAM("Part read from bin 1: " << bin1_part);
            ROS_INFO_STREAM("Part read from bin 2: " << bin2_part);
            ROS_INFO_STREAM("Part read from bin 3: " << bin3_part);
            ROS_INFO_STREAM("Part read from bin 4: " << bin4_part);
            ROS_INFO_STREAM("Part read from bin 5: " << bin5_part);
            ROS_INFO_STREAM("Part read from bin 6: " << bin6_part);
            if(bin1_part == "n"){
                empty_bins.push_back("bin1");
                bin1_poses = FillBin(1, "piston_rod_part");
                // all_empty_bins_pose.push_back(empty_bin_poses);
            }
            if(bin2_part == "n"){
                empty_bins.push_back("bin2");
                bin2_poses = FillBin(2, "piston_rod_part");
                // all_empty_bins_pose.push_back(empty_bin_poses);
            }
            if(bin3_part == "n"){
                empty_bins.push_back("bin3");
                bin3_poses = FillBin(3, "piston_rod_part");
                // all_empty_bins_pose.push_back(empty_bin_poses);
            }
            if(bin4_part == "n"){
                empty_bins.push_back("bin4");
                bin4_poses = FillBin(4, "piston_rod_part");
                // all_empty_bins_pose.push_back(empty_bin_poses);
            }
            if(bin5_part == "n"){
                empty_bins.push_back("bin5");
                bin5_poses = FillBin(5, "piston_rod_part");
                // all_empty_bins_pose.push_back(empty_bin_poses);
            }
            if(bin6_part == "n"){
                empty_bins.push_back("bin6");
                bin6_poses = FillBin(6, "piston_rod_part");
                // all_empty_bins_pose.push_back(empty_bin_poses);
            }
            
            if(std::find(empty_bins.begin(), empty_bins.end(), "bin1") != empty_bins.end()){
                ROS_INFO_STREAM("Bin 1 positions already created");
            }
            else{
                ROS_INFO_STREAM("Checking available points, as follows: ");
                bin1_poses = FillBin(1, bin1_part);
                // ROS_INFO_STREAM(bin1_poses[0]);
            }
            if(std::find(empty_bins.begin(), empty_bins.end(), "bin2") != empty_bins.end()){
                ROS_INFO_STREAM("Bin 2 positions already created");
            }
            else{
                ROS_INFO_STREAM("Checking available points, as follows: ");
                bin2_poses = FillBin(2, bin2_part);
                // ROS_INFO_STREAM(bin1_poses[0]);
            }
            if(std::find(empty_bins.begin(), empty_bins.end(), "bin3") != empty_bins.end()){
                ROS_INFO_STREAM("Bin 3 positions already created");
            }
            else{
                ROS_INFO_STREAM("Checking available points, as follows: ");
                bin3_poses = FillBin(3, bin3_part);
                // ROS_INFO_STREAM(bin1_poses[0]);
            }
            if(std::find(empty_bins.begin(), empty_bins.end(), "bin4") != empty_bins.end()){
                ROS_INFO_STREAM("Bin 4 positions already created");
            }
            else{
                ROS_INFO_STREAM("Checking available points, as follows: ");
                bin4_poses = FillBin(4, bin4_part);
                // ROS_INFO_STREAM(bin1_poses[0]);
                // ROS_INFO_STREAM("Bin4 poses trial: " << bin4_poses[0]);
            }
            if(std::find(empty_bins.begin(), empty_bins.end(), "bin5") != empty_bins.end()){
                ROS_INFO_STREAM("Bin 5 positions already created");
            }
            else{
                ROS_INFO_STREAM("Checking available points, as follows: ");
                bin5_poses = FillBin(5, bin5_part);
                // ROS_INFO_STREAM(bin1_poses[0]);
            }
            if(std::find(empty_bins.begin(), empty_bins.end(), "bin6") != empty_bins.end()){
                ROS_INFO_STREAM("Bin 6 positions already created");
            }
            else{
                ROS_INFO_STREAM("Checking available points, as follows: ");
                bin6_poses = FillBin(6, bin6_part);
                // ROS_INFO_STREAM(bin1_poses[0]);
            }

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
    ROS_INFO_STREAM("Came Here");
    int threshold_new = 4;
    ROS_INFO_STREAM("Product count: " << product_frame_list_.count(product_type));
    if (!product_frame_list_.empty() && product_frame_list_.count(product_type)!=0 && 
        product_frame_list_.count(product_type) >= threshold_new) {
        std::string frame = product_frame_list_[product_type].back();
        // ROS_INFO_STREAM("Frame >>>> " << frame);
        product_frame_list_[product_type].pop_back();
        return frame;
    } else if(product_frame_list_.count(product_type) < threshold_new /*or product_frame_list_.count(product_type) == 0*/){
         ROS_ERROR_STREAM("No product frame found for or less products for " << product_type);
         ROS_INFO_STREAM("Getting " << product_type << " from the conveyor Belt");
//		else return "NOO";

         bool failed_pick = arm1_.PickPartconveyor(product_type);
		 while(!failed_pick){
			 failed_pick = arm1_.PickPartconveyor(product_type);  //robot_controller
             // FillBin("/world", product_type);
		 }
            bin_pose.position.x=-0.2;
            bin_pose.position.y=-0.2;
            bin_pose.position.z=0.1;
            bin_pose.orientation.x =-0.703527;
            bin_pose.orientation.y =0.0254473;
            bin_pose.orientation.z =-0.710205;
            bin_pose.orientation.w =-0.0032328;

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

    //--task the robot to pick up this part again from the bin
    if (agv_id == 1)
      bool failed_pick = arm1_.PickPart(part_pose, agv_id);
    else {
      arm1_.SendRobotHome1();
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
    placed_parts.push_back(std::make_pair(product_type,StampedPose_out.pose));


    return true;

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
            int agv_id = (shipment.agv_id == "any") ? 1 : agv - '0';
            auto products = shipment.products;
            ROS_INFO_STREAM("Order ID: " << order_id);
            ROS_INFO_STREAM("Shipment Type: " << shipment_type);
            ROS_INFO_STREAM("AGV ID: " << agv_id);


            for (const auto &product: products){

                // Check for Updated Order
                ros::spinOnce();
                int new_size=received_orders_.size();

                if(size!=new_size) {
                    ROS_INFO_STREAM("Phir");
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
                  pick_n_place_success =  PickAndPlace(product_type_pose_, empty_bin);
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
                  pick_n_place_success =  PickAndPlace(product_type_pose_, empty_bin);
                  OutOfReach("arm2", num, product_type_pose_, agv_id);
                } else {
                  pick_n_place_success =  PickAndPlace(product_type_pose_, agv_id);
                }
                ROS_INFO_STREAM(placed_parts.size());
                ROS_INFO_STREAM(placed_parts[placed_parts.size()-1].first);

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
  int i=0;
  ROS_INFO_STREAM("Dropping all Parts");
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
  if (agv_id == 1) arm1_.SendRobotHome();

  else arm2_.SendRobotHome2();
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
            // TO-DO Compare current and new order
            // drop all parts (Remove parts from tray )
            dropallparts(placed_parts, agv_id);
            placed_parts.clear();
            return true;
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

    //--task the robot to pick up this part again from the bin
      // bool failed_pick = arm1_.PickPart(part_pose);
    arm1_.SendRobotHome1();
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
    arm1_.SendRobotHome1();
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

    int iter_count = 0;

    std::vector<float> position_x;
    std::vector<float> position_y;

    std::vector<int> count_set = {count1, count2, count3, count4, count5, count6};

    switch(bin_number)
    {
        case 1: 
            iter_count = count1;
            break;
        case 2:
            iter_count = count2;
            break;
        case 3: 
            iter_count = count3;
            break;
        case 4:
            iter_count = count4;
            break;
        case 5: 
            iter_count = count5;
            break;
        case 6:
            iter_count = count6;
            break;
        default:
            break;
    }
    //for bin1:
    //if bin is not empty
    if(iter_count != 0 && bin1_part == conveyor_part_type){
        ROS_INFO_STREAM("Part in bin1: "<< bin1_part << " " << "and count: " << iter_count);
        for(int i = 1; i <= iter_count; i++){
            std::string frame_product = "logical_camera_" + std::to_string(bin_number)+ "_" + bin1_part + "_" + std::to_string(i) + "_frame";
            auto part_pose = camera_.GetPartPose("/world", frame_product);
            std::string bin_frame = "/bin" + std::to_string(bin_number) + "_frame";
            auto part_pose_wrt_bin1 = camera_.GetPartPose(bin_frame, frame_product);
            ROS_INFO_STREAM("From fillbin pose: "<<part_pose_wrt_bin1);            
            position_x.push_back(part_pose_wrt_bin1.position.x);
            position_y.push_back(part_pose_wrt_bin1.position.y);
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
        else if(iter_count >= threshold){
            ROS_INFO_STREAM("Enough parts present in bin 1");
        }
        else if(iter_count == 1){
            ROS_INFO_STREAM("Single part found..");
            single_part_flag = true;
        }
        

        //vertical, single line
        if(vertical_flag_single){
            ROS_INFO_STREAM("Max Y: " << max_y);
            if(max_y >= 0){
                ROS_INFO_STREAM("Parts in single file RHS");
                position_available_bin1.erase(position_available_bin1.begin(), position_available_bin1.end());
                ROS_INFO_STREAM("Parts to be added in bin 1: " << threshold - iter_count);
                for(int j = 1; j <= (threshold - iter_count); j++){
                    //todo - diff thresholds for different part types
                    new_place_pose.position.x = max_x;
                    new_place_pose.position.y = max_y - 0.2;
                    new_place_pose.position.z = 0.005;
                    position_available_bin1.push_back(new_place_pose);
                    new_place_pose.position.x = min_x;
                    new_place_pose.position.y = max_y - 0.2;
                    new_place_pose.position.z = 0.005;
                    position_available_bin1.push_back(new_place_pose);
                    max_y = max_y - 0.2;
                    min_y = min_y - 0.2;
                }

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
                    position_available_bin1.push_back(new_place_pose);
                    new_place_pose.position.x = min_x;
                    new_place_pose.position.y = max_y + 0.2;
                    new_place_pose.position.z = 0.005;
                    position_available_bin1.push_back(new_place_pose);
                    max_y = max_y + 0.2;
                    min_y = min_y + 0.2;
                }

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
                    position_available_bin1.push_back(new_place_pose);
                    new_place_pose.position.x = max_x - 0.2;
                    new_place_pose.position.y = min_y;
                    position_available_bin1.push_back(new_place_pose);
                    max_x = max_x - 0.2;
                    min_x = min_x - 0.2;
                }

            }
            if(max_x < 0){
                ROS_INFO_STREAM("Parts in single file down");
                position_available_bin1.erase(position_available_bin1.begin(), position_available_bin1.end());
                ROS_INFO_STREAM("Parts to be added in bin 1: " << threshold - iter_count);
                for(int j = 1; j <= (threshold - iter_count); j++){
                    //todo - diff thresholds for different part types
                    new_place_pose.position.x = max_x + 0.2;
                    new_place_pose.position.y = max_y;
                    position_available_bin1.push_back(new_place_pose);
                    new_place_pose.position.x = max_x + 0.2;
                    new_place_pose.position.y = min_y;
                    position_available_bin1.push_back(new_place_pose);
                    max_x = max_x + 0.2;
                    min_x = min_x + 0.2;
                }

            }
            
        }
        if(single_part_flag){
            if(max_x > 0 and max_y > 0){
                new_place_pose.position.x = max_x;
                new_place_pose.position.y = -max_y;
                position_available_bin1.push_back(new_place_pose);
                new_place_pose.position.x = -max_x;
                new_place_pose.position.y = -max_y;
                position_available_bin1.push_back(new_place_pose);
                new_place_pose.position.x = -max_x;
                new_place_pose.position.y = max_y;
                position_available_bin1.push_back(new_place_pose);
            }
            if(max_x > 0 and max_y < 0){
                new_place_pose.position.x = max_x;
                new_place_pose.position.y = abs(max_y);
                position_available_bin1.push_back(new_place_pose);
                new_place_pose.position.x = -max_x;
                new_place_pose.position.y = max_y;
                position_available_bin1.push_back(new_place_pose);
                new_place_pose.position.x = -max_x;
                new_place_pose.position.y = abs(max_y);
                position_available_bin1.push_back(new_place_pose);
            }
            if(max_x < 0 and max_y < 0){
                new_place_pose.position.x = abs(max_x);
                new_place_pose.position.y = abs(max_y);
                position_available_bin1.push_back(new_place_pose);
                new_place_pose.position.x = abs(max_x);
                new_place_pose.position.y = max_y;
                position_available_bin1.push_back(new_place_pose);
                new_place_pose.position.x = max_x;
                new_place_pose.position.y = abs(max_y);
                position_available_bin1.push_back(new_place_pose);
            }
            if(max_x < 0 and max_y > 0){
                new_place_pose.position.x = abs(max_x);
                new_place_pose.position.y = abs(max_y);
                position_available_bin1.push_back(new_place_pose);
                new_place_pose.position.x = abs(max_x);
                new_place_pose.position.y = max_y;
                position_available_bin1.push_back(new_place_pose);
                new_place_pose.position.x = max_x;
                new_place_pose.position.y = -max_y;
                position_available_bin1.push_back(new_place_pose);
            }

        }
    }
    else if(iter_count == 0){
        //4 parts at 4 corners
        ROS_INFO_STREAM("Empty bin found, bin number: " << bin_number);
        int i = 1;       
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

