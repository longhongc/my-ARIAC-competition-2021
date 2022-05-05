#include "kitting_arm.h"

#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <eigen_conversions/eigen_msg.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/convert.h>
#include <Eigen/Geometry>
#include <math.h>
#include <random>

#include <nist_gear/AGVToAssemblyStation.h>
#include <ariac_group1/IsFaulty.h>
#include <ariac_group1/PartsInCamera.h>
#include <ariac_group1/IsPartPicked.h>
#include <ariac_group1/GetPartPosition.h>
#include <ariac_group1/CheckQualitySensor.h>
#include <ariac_group1/GetCompetitionTime.h>
#include <std_srvs/Trigger.h>
#include <ariac_group1/GetBeltPart.h>
#include <ariac_group1/GetBeltProximitySensor.h>
#include <ariac_group1/GetVacancyPose.h>
#include <ariac_group1/PartsUnderCamera.h>

using AGVToAssem = nist_gear::AGVToAssemblyStation; 

KittingArm::KittingArm():
  m_nh{"/ariac/kitting"},
  m_planning_group{"/ariac/kitting/robot_description"},
  m_arm_options{"kitting_arm", m_planning_group, m_nh},
  m_arm_group{m_arm_options}, 
  m_shipments{&m_nh}
{
  // publishers to directly control the joints without moveit
  m_arm_joint_trajectory_publisher =
      m_nh.advertise<trajectory_msgs::JointTrajectory>("/ariac/kitting/kitting_arm_controller/command", 10);

  // subscribers
  m_arm_joint_states_subscriber =
      m_nh.subscribe("/ariac/kitting/joint_states", 10, &KittingArm::arm_joint_states_callback, this);
  
  m_arm_controller_state_subscriber =
      m_nh.subscribe("/ariac/kitting/kitting_arm_controller/state", 10, &KittingArm::arm_controller_state_callback, this);

  m_gripper_state_subscriber = 
      m_nh.subscribe("/ariac/kitting/arm/gripper/state", 10, &KittingArm::gripper_state_callback, this);

  // server
  m_get_working_station_service = m_nh.advertiseService("/kitting_arm/get_working_station", &KittingArm::get_working_station, this); 

  // clients
  m_gripper_control_client =
      m_nh.serviceClient<nist_gear::VacuumGripperControl>("/ariac/kitting/arm/gripper/control");
  m_gripper_control_client.waitForExistence();

  m_get_parts_client = 
      m_nh.serviceClient<ariac_group1::GetParts>("/sensor_manager/get_parts"); 
  m_get_parts_client.waitForExistence();

  m_is_faulty_client = 
      m_nh.serviceClient<ariac_group1::IsFaulty>("/sensor_manager/is_faulty"); 
  m_is_faulty_client.waitForExistence();

  m_parts_in_camera_client = 
      m_nh.serviceClient<ariac_group1::PartsInCamera>("/sensor_manager/parts_in_camera"); 
  m_parts_in_camera_client.waitForExistence();

  m_is_part_picked_client = 
      m_nh.serviceClient<ariac_group1::IsPartPicked>("/sensor_manager/is_part_picked"); 
  m_is_part_picked_client.waitForExistence();

  m_get_part_position_client = 
      m_nh.serviceClient<ariac_group1::GetPartPosition>("/sensor_manager/get_part_position"); 
  m_get_part_position_client.waitForExistence();

  m_check_quality_sensor_client = 
      m_nh.serviceClient<ariac_group1::CheckQualitySensor>("/sensor_manager/check_quality_sensor"); 
  m_check_quality_sensor_client.waitForExistence();

  m_get_competition_time_client = 
      m_nh.serviceClient<ariac_group1::GetCompetitionTime>("/factory_manager/get_competition_time"); 
  m_get_competition_time_client.waitForExistence();

  m_is_belt_sensor_triggered_client = 
      m_nh.serviceClient<std_srvs::Trigger>("/sensor_manager/is_belt_sensor_triggered"); 
  m_is_belt_sensor_triggered_client.waitForExistence();

  m_get_belt_part_client = 
      m_nh.serviceClient<ariac_group1::GetBeltPart>("/sensor_manager/get_belt_part"); 
  m_get_belt_part_client.waitForExistence();

  m_get_belt_proximity_sensor_client = 
      m_nh.serviceClient<ariac_group1::GetBeltProximitySensor>("/sensor_manager/get_belt_proximity_sensor"); 
  m_get_belt_proximity_sensor_client.waitForExistence();

  m_get_vacancy_pose_client = 
      m_nh.serviceClient<ariac_group1::GetVacancyPose>("/sensor_manager/get_vacancy_pose"); 
  m_get_vacancy_pose_client.waitForExistence();

  m_parts_under_camera_client = 
      m_nh.serviceClient<ariac_group1::PartsUnderCamera>("/sensor_manager/parts_under_camera"); 
  m_parts_under_camera_client.waitForExistence();

  for (auto& id: m_agvs_id) {
      m_agvs_dict[id] = std::make_unique<AGV>(&m_nh, id); 
  }

  // Preset locations
  // ^^^^^^^^^^^^^^^^
  // Joints for the arm are in this order:
  // - linear_arm_actuator_joint
  // - shoulder_pan_joint
  // - shoulder_lift_joint
  // - elbow_joint
  // - wrist_1_joint
  // - wrist_2_joint
  // - wrist_3_joint

  double linear_arm_actuator_joint{ 0 };
  double shoulder_pan_joint{ 0 };
  double shoulder_lift_joint{ -1.25 };
  double elbow_joint{ 1.74 };
  double wrist_1_joint{ -2.04 };
  double wrist_2_joint{ -1.57 };
  double wrist_3_joint{ 0 };

  std::vector<double> default_pos = {linear_arm_actuator_joint,
                                     shoulder_pan_joint,
                                     shoulder_lift_joint,
                                     elbow_joint,
                                     wrist_1_joint,
                                     wrist_2_joint,
                                     wrist_3_joint}; 
  //home position
  home_face_belt.joints_position = default_pos;
  home_face_belt.name = "home_face_belt";

  home_face_bins.joints_position = default_pos;
  // shoulder_pan_joint
  home_face_bins.joints_position.at(1) = -M_PI;
  home_face_bins.name = "home_face_bins";

  location_belt_part.joints_position = default_pos;
  location_belt_part.joints_position.at(0) = -3;
  location_belt_part.name = "belt_part";

  location_belt_intercept.joints_position = default_pos;
  location_belt_intercept.joints_position.at(1) = -M_PI;
  location_belt_intercept.joints_position.at(2) = -2;
  location_belt_intercept.joints_position.at(3) = -2;
  location_belt_intercept.joints_position.at(5) = 3.05;
  location_belt_intercept.name = "belt_intercept";


  location_agv1.joints_position = home_face_bins.joints_position; 
  // linear actuator at y axis 
  location_agv1.joints_position.at(0) = 3.83; 
  location_agv1.name = "agv1"; 

  location_agv2.joints_position = home_face_bins.joints_position; 
  // linear actuator at y axis 
  location_agv2.joints_position.at(0) = 0.83; 
  location_agv2.name = "agv2"; 

  location_agv3.joints_position = home_face_bins.joints_position; 
  // linear actuator at y axis 
  location_agv3.joints_position.at(0) = -1.63; 
  location_agv3.name = "agv3"; 

  location_agv4.joints_position = home_face_bins.joints_position; 
  // linear actuator at y axis 
  location_agv4.joints_position.at(0) = -4.1; 
  location_agv4.name = "agv4"; 

  location_bins0.joints_position = home_face_bins.joints_position; 
  // linear actuator at y axis 
  location_bins0.joints_position.at(0) = 3; 
  location_bins0.name = "bins0"; 

  location_bins1.joints_position = home_face_bins.joints_position; 
  // linear actuator at y axis 
  location_bins1.joints_position.at(0) = -4; 
  location_bins1.name = "bins1"; 

  m_arm_group.setPlanningTime(10.0); 


  // initialize home position
  this->go_to_preset_location("home_face_belt"); 
  this->go_to_preset_location("home_face_bins"); 
}

//==========public functions
// flow control
void KittingArm::wait_for_belt(int wait_time)
{
  ariac_group1::GetCompetitionTime get_comp_time_srv; 
  double competition_time; 
  do {
    ROS_INFO("Waiting for belt"); 
    if (this->check_emergency_interrupt()) {
        return; 
    }
    m_get_competition_time_client.call(get_comp_time_srv); 
    competition_time = get_comp_time_srv.response.competition_time;
    ros::Duration(1.0).sleep(); 
  } while(competition_time < wait_time);
}

bool KittingArm::get_order()
{
  ros::Duration(1.0).sleep();
  ros::Rate wait_rate(20); 
  this->check_emergency_interrupt();  
  // check if there are tasks and make sure ROS is running 
  while (ros::ok()) {
    ROS_INFO_THROTTLE(3, "Waiting for part task.");
    this->check_emergency_interrupt();  

    m_shipments.update_part_task_queue(m_part_task_queue); 

    if (not m_part_task_queue.empty()) {
        break; 
    }
    
    wait_rate.sleep(); 
  }

  ROS_INFO("Received part task"); 
  return true; 
}

void KittingArm::plan()
{
  const std::lock_guard<std::mutex> lock(*m_mutex_ptr); 
  std::sort(m_part_task_queue.begin(), m_part_task_queue.end()); 
}

void KittingArm::execute()
{
  const std::lock_guard<std::mutex> lock(*m_mutex_ptr); 

  ROS_INFO("==============================================="); 
  ROS_INFO("Task queue size: %d", (int)m_part_task_queue.size()); 
  auto& part_task_info = m_part_task_queue.back(); 
  auto& priority = std::get<0>(part_task_info); 
  auto& part_task = *std::get<1>(part_task_info); 

  ROS_INFO("unfinished part task %d", m_shipments.shipments_record[part_task.shipment_type]->unfinished_part_tasks); 
  // check if part_task has already been done, if so reduce unfinished_part_tasks
  if (m_shipments.shipments_record[part_task.shipment_type]->unfinished_part_tasks > 0) {
      // is_part_task_done will also decrease unfinished part task if done
      if (m_shipments.is_part_task_done(part_task)) {
        ROS_INFO("Part task %s at %s is already done !!", part_task.part.type.c_str(), part_task.agv_id.c_str()); 
      }
  }

  nist_gear::Model wrong_part; 
  auto shipment_state = this->check_shipment_state(part_task, wrong_part); 
  m_shipments.shipments_record[part_task.shipment_type]->state = shipment_state; 
  this->process_shipment_state(shipment_state, part_task, priority, wrong_part); 
  if (shipment_state != ShipmentState::NOT_READY) {
    return; 
  }

  ROS_INFO("Shipment: %s", part_task.shipment_type.c_str()); 
  ROS_INFO("Priority: %d", priority); 
  ROS_INFO("AGV: %s", part_task.agv_id.c_str()); 
  ROS_INFO("Station id: %s", part_task.station_id.c_str()); 
  ROS_INFO("Part type: %s", part_task.part.type.c_str()); 
  // ros::Duration(30.0).sleep();
  //
  ariac_group1::GetParts get_parts_srv; 
  get_parts_srv.request.type = part_task.part.type; 
  get_parts_srv.request.client = "kitting_arm"; 

  m_get_parts_client.call(get_parts_srv); 

  auto& parts_info = get_parts_srv.response.parts_info; 

  if (not parts_info.empty()) {
    auto part_init_info = parts_info[0]; 
    int idx = 0; 

    // don't use part on ks unless necessary
    while (part_init_info.camera_id.find("ks") != std::string::npos) {
      if (idx >= (parts_info.size() - 1)) {
        if (priority >= Constants::PriorityWeight::Level::HIGH) {
          ROS_INFO("High priority gets part from agv"); 
          break; 
        }
        ROS_INFO("No enough part for %s", part_task.part.type.c_str()); 
        priority += Constants::PriorityWeight::Penalty::NO_PART;  
        if (this->check_insufficient_shipment(priority)) {
          ROS_INFO("Part task %s %s infeasible, discard part task", part_task.shipment_type.c_str(), part_task.part.type.c_str()); 
          this->clear_part_task(part_task, priority); 
        }
        return; 
      }
      idx++; 
      part_init_info = parts_info[idx]; 
    }

    ROS_INFO("Found %s upder %s", part_task.part.type.c_str(), part_init_info.camera_id.c_str()); 
    ROS_INFO("Part location: "); 
    m_working_station = part_task.agv_id; 
    Utility::print_part_pose(part_init_info.part); 

    auto part_bin = Utility::location::get_pose_location(part_init_info.part.pose); 
    bool from_back_row_bins = false; 
    if (Utility::location::is_back_row_bins(part_bin)) {
        from_back_row_bins = true; 
    }

    bool success = this->move_part(part_init_info, part_task, from_back_row_bins); 

    if (success) {
      ROS_INFO("Move part success"); 
      this->clear_part_task(part_task, priority); 
      return; 

    }
    else {
      ROS_INFO("Move part fails"); 
      priority += Constants::PriorityWeight::Penalty::MOVE_FAILS; 
      return; 
    }
  }
  else {
    ROS_INFO("No valid %s, back to task queue", part_task.part.type.c_str()); 
    priority += Constants::PriorityWeight::Penalty::NO_PART;  
    part_task.priority = priority; 
    ROS_INFO("Priority decrease to %d", priority); 
    
    if (this->check_insufficient_shipment(priority)) {
      ROS_INFO("Part task %s %s infeasible, discard part task", part_task.shipment_type.c_str(), part_task.part.type.c_str()); 
      this->clear_part_task(part_task, priority); 
    }
     
    return; 
  }

}

// basic actions
void KittingArm::go_to_preset_location(std::string location_name)
{

    this->copy_current_joints_position(); 

    ArmPresetLocation location;
    if (location_name.compare("home_face_belt") == 0) {
        ROS_INFO("Move to home_face_belt"); 
        location = home_face_belt;
    }
    else if (location_name.compare("home_face_bins") == 0) {
        ROS_INFO("Move to home_face_bins"); 
        location = home_face_bins;
    }
    else if (location_name.compare("agv1") == 0 or
             location_name.find("ks1") != std::string::npos) {
        ROS_INFO("Move to agv1"); 
        location = location_agv1;
    }
    else if (location_name.compare("agv2") == 0 or 
             location_name.find("ks2") != std::string::npos) {
        ROS_INFO("Move to agv2"); 
        location = location_agv2;
    }
    else if (location_name.compare("agv3") == 0 or
             location_name.find("ks3") != std::string::npos) {
        ROS_INFO("Move to agv3"); 
        location = location_agv3;
    }
    else if (location_name.compare("agv4") == 0 or 
             location_name.find("ks4") != std::string::npos) {
        ROS_INFO("Move to agv4"); 
        location = location_agv4;
    }
    else if (location_name.find("bins0") != std::string::npos) {
        ROS_INFO("Move to bins0"); 
        location = location_bins0;
    }
    else if (location_name.find("bins1") != std::string::npos) {
        ROS_INFO("Move to bins1"); 
        location = location_bins1;

    }
    else if (location_name.find("belt_part") != std::string::npos) {
        ROS_INFO("Move to belt part"); 
        location = location_belt_part;
    }
    else if (location_name.find("belt_intercept") != std::string::npos) {
        ROS_INFO("Move to belt intercept"); 
        location = location_belt_intercept;
    }

    m_joint_group_positions.at(0) = location.joints_position.at(0);
    m_joint_group_positions.at(1) = location.joints_position.at(1);
    m_joint_group_positions.at(2) = location.joints_position.at(2);
    m_joint_group_positions.at(3) = location.joints_position.at(3);
    m_joint_group_positions.at(4) = location.joints_position.at(4);
    m_joint_group_positions.at(5) = location.joints_position.at(5);
    m_joint_group_positions.at(6) = location.joints_position.at(6);

    m_arm_group.setMaxVelocityScalingFactor(1.0);
    m_arm_group.setJointValueTarget(m_joint_group_positions);
    this->move_arm_group(); 
}

void KittingArm::move_base_to(double linear_arm_actuator_joint_position)
{

    this->copy_current_joints_position(); 
    
    // next, assign a value to only the linear_arm_actuator_joint
    m_joint_group_positions.at(0) = linear_arm_actuator_joint_position;

    // move the arm
    m_arm_group.setJointValueTarget(m_joint_group_positions);
    this->move_arm_group(); 
}

void KittingArm::reset_arm()
{
    this->copy_current_joints_position(); 
    m_joint_group_positions.at(1) = -M_PI;
    m_joint_group_positions.at(2) = -1.25;
    m_joint_group_positions.at(3) = 1.74;
    m_joint_group_positions.at(4) = -2.04;
    m_joint_group_positions.at(5) = -1.57;
    m_joint_group_positions.at(6) = 0;
    m_arm_group.setJointValueTarget(m_joint_group_positions);
    this->move_arm_group(); 
}

bool KittingArm::move_part(const ariac_group1::PartInfo& part_init_info, const ariac_group1::PartTask& part_task, bool from_back_row_bins) {
    auto part_init_pose_in_world = part_init_info.part.pose;
    auto camera_id = part_init_info.camera_id; 
    auto part_type = part_init_info.part.type; 
    auto target_pose_in_frame = part_task.part.pose; 
    auto target_agv = part_task.agv_id; 

    if (this->check_emergency_interrupt()) {
        return false; 
    }

    auto target_rpy = Utility::motioncontrol::eulerFromQuaternion(target_pose_in_frame); 

    bool flip = false; 
    if (abs(target_rpy.at(0)) > 0.1) {
        ROS_INFO("Flipped part!"); 
        flip = true; 
        auto part_rpy = Utility::motioncontrol::eulerFromQuaternion(part_init_pose_in_world);
        auto rectified_orientation = Utility::motioncontrol::quaternionFromEuler(part_rpy.at(0), part_rpy.at(1), 0);
        target_pose_in_frame.orientation.x = rectified_orientation.getX();
        target_pose_in_frame.orientation.y = rectified_orientation.getY();
        target_pose_in_frame.orientation.z = rectified_orientation.getZ();
        target_pose_in_frame.orientation.w = rectified_orientation.getW();
    }

    // use -0.3 to reduce awkward pick trajectory
    // go_to_preset_location(camera_id);
    if (Utility::location::get_pose_location(part_init_pose_in_world) == "bin6") {
        move_base_to(part_init_pose_in_world.position.y - 0.5);
    }
    else {
        move_base_to(part_init_pose_in_world.position.y - 0.3);
    }

    if (this->check_emergency_interrupt()) {
        return false; 
    }

    bool pick_from_bins_success = false;   
    if (from_back_row_bins) {
        pick_from_bins_success = pick_part_from_back_row(part_type, part_init_pose_in_world, camera_id); 
    }
    else {
        pick_from_bins_success = pick_part(part_type, part_init_pose_in_world, camera_id); 
    }

    if (pick_from_bins_success) {
        auto target_pose_in_world = place_part(part_type, part_init_pose_in_world, target_pose_in_frame, target_agv);
        ros::Duration(1).sleep(); 

        nist_gear::Model faulty_part; 
        faulty_part.type = part_type; 
        faulty_part.pose = target_pose_in_world; 

        ROS_INFO("Check faulty"); 
        if (this->check_faulty(faulty_part)) {
          if (this->check_emergency_interrupt()) {
              // do emergency first, discard later
              return true; 
          }

          ROS_INFO("Found faulty part:"); 
          Utility::print_part_pose(faulty_part); 
          bool discard_success = this->discard_faulty(faulty_part, target_agv); 
          this->check_emergency_interrupt(); 

          if (discard_success) {
            // if discard success, move part fails
            return false; 
          }
          else {
            // if discard fails, assume move part success
            // discard later
            return true; 
          }
        } 
        else {
          if (flip) {
            bool flip_success = this->flip_part(part_task);
            if (not flip_success) {
               ROS_INFO("Flip fails"); 
            }
          }
          ROS_INFO("Part not faulty"); 
          return true; 
        }
    }
    else {
      return false; 
    }
}

bool KittingArm::pick_part(std::string part_type, 
                          const geometry_msgs::Pose& part_init_pose,
                          std::string camera_id,
                          bool flip) 
{
    ROS_INFO("---Start pick part"); 
    ROS_INFO("Picking under camera %s", camera_id.c_str()); 
    m_arm_group.setMaxVelocityScalingFactor(1.0);

    geometry_msgs::Pose arm_ee_link_pose = m_arm_group.getCurrentPose().pose;
    auto flat_orientation = Utility::motioncontrol::quaternionFromEuler(0, 1.57, 0);
    arm_ee_link_pose.orientation.x = flat_orientation.getX();
    arm_ee_link_pose.orientation.y = flat_orientation.getY();
    arm_ee_link_pose.orientation.z = flat_orientation.getZ();
    arm_ee_link_pose.orientation.w = flat_orientation.getW();
    
    // preset z depending on the part type
    double z_pos{};
    if (part_type.find("pump") != std::string::npos) {
        z_pos = 0.075;
        if (camera_id.find("ks") != std::string::npos) {
          z_pos -= 0.0052;
        }
    }
    if (part_type.find("sensor") != std::string::npos) {
        z_pos = 0.05;
        if (camera_id.find("ks") != std::string::npos) {
          z_pos -= 0.005;
        }
    }
    if (part_type.find("battery") != std::string::npos) {
        z_pos = 0.052;
        if (camera_id.find("ks") != std::string::npos) {
          z_pos -= 0.005;
        }
    }
    if (part_type.find("regulator") != std::string::npos) {
        z_pos = 0.057;
        if (camera_id.find("ks") != std::string::npos) {
          z_pos -= 0.005;
        }
    }

    

    // post-grasp pose 
    // store the pose of the arm before it goes down to pick the part
    // we will bring the arm back to this pose after picking up the part
    auto postgrasp_pose = part_init_pose;
    postgrasp_pose.orientation = arm_ee_link_pose.orientation;
    postgrasp_pose.position.z = part_init_pose.position.z + z_pos + 0.05;

    if (not flip) {
        if (this->check_emergency_interrupt()) {
            return false; 
        }
    }
    

    // m_arm_group.setPoseTarget(postgrasp_pose);
    // m_arm_group.move();
    ROS_INFO("Move to postgrasp pose"); 
    if (not this->move_target_pose(postgrasp_pose)) {
      ROS_INFO("---End pick part: IK not found for postgrasp"); 
      return false; 
    }

    if (not flip) {
        if (this->check_emergency_interrupt()) {
            return false; 
        }
    }

    // pregrasp pose: right above the part
    auto pregrasp_pose = part_init_pose;
    pregrasp_pose.orientation = arm_ee_link_pose.orientation;
    pregrasp_pose.position.z = part_init_pose.position.z + z_pos;

    // activate gripper
    // sometimes it does not activate right away
    // so we are doing this in a loop
    while (!m_gripper_state.enabled) {
        activate_gripper();
    }

    ROS_INFO("Move to pregrasp pose"); 
    if (not this->move_target_pose(pregrasp_pose)) {
      ROS_INFO("IK not found for grasp"); 
      deactivate_gripper();
      this->lift(); 
      return false; 
    }

    if (not flip) {
        if (this->check_emergency_interrupt()) {
            return false; 
        }
    }

    ros::Duration(0.5).sleep();
    
    m_arm_group.setMaxVelocityScalingFactor(0.5);
    auto grasp_pose = pregrasp_pose; 
    // this->set_pick_constraints(); 
    ROS_INFO("Start grasping"); 
    // move the arm 1 mm down until the part is attached
    int count = 0; 
    int trial = 0; 
    double step = 0.001;
    while (!m_gripper_state.attached) {
        grasp_pose.position.z -= step;
        m_arm_group.setPoseTarget(grasp_pose);
        m_arm_group.move();
        count++; 
        ros::Duration(0.3).sleep();
        // if (step > 0.0008) {
          // step -= 0.0001; 
        // }
        geometry_msgs::Pose arm_ee_link_pose = m_arm_group.getCurrentPose().pose;
        if (arm_ee_link_pose.position.z < 0.76) {
          ROS_INFO("---End pick part: Arm moving lower then part, abort"); 
          this->reset_arm(); 
          deactivate_gripper();
          return false; 
        }

        if (trial > 3) {
          ROS_INFO("Hard to grasp. Abort"); 
          this->reset_arm(); 
          deactivate_gripper();
          return false; 
        }

        if (count > 2) {
          if (not flip) {
              if (this->check_emergency_interrupt()) {
                  return false; 
              }
          }
        }

        if (count > 7) {
          this->lift(); 
          ROS_INFO("Hard to grasp. Move back to pregrasp pose"); 
          if (not this->move_target_pose(pregrasp_pose)) {
            ROS_INFO("IK not found for grasp"); 
            deactivate_gripper();
            this->lift(); 
            return false; 
          }

          grasp_pose = pregrasp_pose; 
          count = 0; 
          trial++; 
        }
    }
    // m_arm_group.clearPathConstraints();
    
    m_arm_group.setMaxVelocityScalingFactor(1.0);
    m_arm_group.setMaxAccelerationScalingFactor(1.0);
    ROS_INFO_STREAM("[Gripper] = object attached");
    ros::Duration(0.5).sleep();

    ROS_INFO("Move back to postgrasp pose"); 
    if (not this->move_target_pose(postgrasp_pose)) {
      ROS_INFO("---End pick part: IK not found for postgrasp"); 
      this->lift(); 
      return false; 
    }
    // m_arm_group.setPoseTarget(postgrasp_pose);
    // m_arm_group.move();
    ros::Duration(0.5).sleep();

    this->lift(); 

    ariac_group1::IsPartPicked srv; 
    srv.request.camera_id = camera_id; 
    srv.request.part.type = part_type; 
    srv.request.part.pose = part_init_pose; 
    if (m_is_part_picked_client.call(srv)) {
      if (srv.response.picked) {
        ROS_INFO("---End pick part: pick success"); 
        return true; 
      }
      else {
        ROS_INFO("---End pick part: pick fails"); 
        deactivate_gripper();
        return false; 
      }
    } 
    else {
      ROS_INFO("---End pick part: no such camera: %s", camera_id.c_str()); 
      deactivate_gripper();
      return false; 
    }
}

bool KittingArm::pick_part_from_back_row(std::string part_type, const geometry_msgs::Pose& part_init_pose, std::string camera_id) {
    bool from_back_row_bin = true; 

    ROS_INFO("---Start pick part"); 
    ROS_INFO("Picking under camera %s", camera_id.c_str()); 
    m_arm_group.setMaxVelocityScalingFactor(1.0);

    geometry_msgs::Pose arm_ee_link_pose = m_arm_group.getCurrentPose().pose;
    auto flat_orientation = Utility::motioncontrol::quaternionFromEuler(0, 1.57, 0);
    arm_ee_link_pose.orientation.x = flat_orientation.getX();
    arm_ee_link_pose.orientation.y = flat_orientation.getY();
    arm_ee_link_pose.orientation.z = flat_orientation.getZ();
    arm_ee_link_pose.orientation.w = flat_orientation.getW();
    
    // preset z depending on the part type
    double z_pos{};
    if (part_type.find("pump") != std::string::npos) {
        z_pos = 0.075;
        if (camera_id.find("ks") != std::string::npos) {
          z_pos -= 0.0052;
        }
    }
    if (part_type.find("sensor") != std::string::npos) {
        z_pos = 0.05;
        if (camera_id.find("ks") != std::string::npos) {
          z_pos -= 0.005;
        }
    }
    if (part_type.find("battery") != std::string::npos) {
        z_pos = 0.052;
        if (camera_id.find("ks") != std::string::npos) {
          z_pos -= 0.005;
        }
    }
    if (part_type.find("regulator") != std::string::npos) {
        z_pos = 0.057;
        if (camera_id.find("ks") != std::string::npos) {
          z_pos -= 0.005;
        }
    }

    // post-grasp pose 
    // store the pose of the arm before it goes down to pick the part
    // we will bring the arm back to this pose after picking up the part
    auto postgrasp_pose = part_init_pose;
    postgrasp_pose.orientation = arm_ee_link_pose.orientation;
    postgrasp_pose.position.z = part_init_pose.position.z + z_pos + 0.05;

    if (this->check_emergency_interrupt()) {
        return false; 
    }
    

    // m_arm_group.setPoseTarget(postgrasp_pose);
    // m_arm_group.move();
    ROS_INFO("Move to postgrasp pose"); 
    if (not this->move_target_pose(postgrasp_pose, from_back_row_bin)) {
      ROS_INFO("---End pick part: IK not found for postgrasp"); 
      return false; 
    }

    if (this->check_emergency_interrupt()) {
        return false; 
    }

    // pregrasp pose: right above the part
    auto pregrasp_pose = part_init_pose;
    pregrasp_pose.orientation = arm_ee_link_pose.orientation;
    pregrasp_pose.position.z = part_init_pose.position.z + z_pos;

    // activate gripper
    // sometimes it does not activate right away
    // so we are doing this in a loop
    while (!m_gripper_state.enabled) {
        activate_gripper();
    }

    ROS_INFO("Move to pregrasp pose"); 
    if (not this->move_target_pose(pregrasp_pose, from_back_row_bin)) {
      ROS_INFO("IK not found for grasp"); 
      deactivate_gripper();
      this->lift(); 
      return false; 
    }

    if (this->check_emergency_interrupt()) {
        return false; 
    }

    ros::Duration(0.5).sleep();
    
    m_arm_group.setMaxVelocityScalingFactor(0.5);
    auto grasp_pose = pregrasp_pose; 
    // this->set_pick_constraints(); 
    ROS_INFO("Start grasping"); 
    // move the arm 1 mm down until the part is attached
    int count = 0; 
    int trial = 0; 
    double step = 0.001;
    while (!m_gripper_state.attached) {
        grasp_pose.position.z -= step;
        m_arm_group.setPoseTarget(grasp_pose);
        m_arm_group.move();
        count++; 
        ros::Duration(0.3).sleep();
        // if (step > 0.0008) {
          // step -= 0.0001; 
        // }
        geometry_msgs::Pose arm_ee_link_pose = m_arm_group.getCurrentPose().pose;
        if (arm_ee_link_pose.position.z < 0.76) {
          ROS_INFO("---End pick part: Arm moving lower then part, abort"); 
          this->reset_arm(); 
          deactivate_gripper();
          return false; 
        }

        if (trial > 3) {
          ROS_INFO("Hard to grasp. Abort"); 
          this->reset_arm(); 
          deactivate_gripper();
          return false; 
        }

        if (count > 2) {
          if (this->check_emergency_interrupt()) {
              return false; 
          }
        }

        if (count > 7) {
          this->lift(); 
          ROS_INFO("Hard to grasp. Move back to pregrasp pose"); 
          if (not this->move_target_pose(pregrasp_pose, from_back_row_bin)) {
            ROS_INFO("IK not found for grasp"); 
            deactivate_gripper();
            this->lift(); 
            return false; 
          }

          grasp_pose = pregrasp_pose; 
          count = 0; 
          trial++; 
        }
    }
    // m_arm_group.clearPathConstraints();
    
    m_arm_group.setMaxVelocityScalingFactor(1.0);
    m_arm_group.setMaxAccelerationScalingFactor(1.0);
    ROS_INFO_STREAM("[Gripper] = object attached");
    ros::Duration(0.5).sleep();

    ROS_INFO("Move back to postgrasp pose"); 
    if (not this->move_target_pose(postgrasp_pose, from_back_row_bin)) {
      ROS_INFO("---End pick part: IK not found for postgrasp"); 
      this->reset_arm(); 
      return false; 
    }
    // m_arm_group.setPoseTarget(postgrasp_pose);
    // m_arm_group.move();
    ros::Duration(0.5).sleep();

    // this->lift(); 
    this->reset_arm(); 

    ariac_group1::IsPartPicked srv; 
    srv.request.camera_id = camera_id; 
    srv.request.part.type = part_type; 
    srv.request.part.pose = part_init_pose; 
    if (m_is_part_picked_client.call(srv)) {
      if (srv.response.picked) {
        ROS_INFO("---End pick part: pick success"); 
        return true; 
      }
      else {
        ROS_INFO("---End pick part: pick fails"); 
        deactivate_gripper();
        return false; 
      }
    } 
    else {
      ROS_INFO("---End pick part: no such camera: %s", camera_id.c_str()); 
      deactivate_gripper();
      return false; 
    }
}

geometry_msgs::Pose KittingArm::place_part(std::string part_type, 
                                          geometry_msgs::Pose part_init_pose, 
                                          geometry_msgs::Pose part_pose_in_frame, 
                                          std::string agv, 
                                          bool flip)
{
    go_to_preset_location(agv);

    // get the target pose of the part in the world frame
    auto target_pose_in_world = Utility::motioncontrol::transformToWorldFrame(
         part_pose_in_frame,
         agv);

    ROS_INFO("---Start place part"); 
    Utility::print_pose(target_pose_in_world); 

   

    geometry_msgs::Pose arm_ee_link_pose = m_arm_group.getCurrentPose().pose;
    auto flat_orientation = Utility::motioncontrol::quaternionFromEuler(0, 1.57, 0);
    arm_ee_link_pose = m_arm_group.getCurrentPose().pose;
    arm_ee_link_pose.orientation.x = flat_orientation.getX();
    arm_ee_link_pose.orientation.y = flat_orientation.getY();
    arm_ee_link_pose.orientation.z = flat_orientation.getZ();
    arm_ee_link_pose.orientation.w = flat_orientation.getW();

    // store the current orientation of the end effector now
    // so we can reuse it later
    tf2::Quaternion q_current(
        arm_ee_link_pose.orientation.x,
        arm_ee_link_pose.orientation.y,
        arm_ee_link_pose.orientation.z,
        arm_ee_link_pose.orientation.w);
    
    // move the arm above the agv
    // gripper stays at the current z
    // only modify its x and y based on the part to grasp
    // In this case we do not need to use preset locations
    // everything is done dynamically
    arm_ee_link_pose.position.x = target_pose_in_world.position.x;
    arm_ee_link_pose.position.y = target_pose_in_world.position.y;
    // move the arm
    m_arm_group.setMaxVelocityScalingFactor(0.8);

    ROS_INFO("Move to place pose above"); 
    while (not this->move_target_pose(arm_ee_link_pose)) {
      ROS_INFO("IK not found for place pose above"); 
      go_to_preset_location(agv);

      std::random_device rd; 
      std::mt19937 gen(rd());
      std::normal_distribution<double> gauss_dist(-0.1, 0.1); 
      auto random_dist = gauss_dist(gen); 
      m_joint_group_positions.at(0) += random_dist; 
      this->move_base_to(m_joint_group_positions.at(0));
      ros::Duration(3.0).sleep();
    }

    // m_arm_group.setPoseTarget(arm_ee_link_pose);
    // m_arm_group.move();

  
    // orientation of the part in the bin, in world frame
    tf2::Quaternion q_init_part(
        part_init_pose.orientation.x,
        part_init_pose.orientation.y,
        part_init_pose.orientation.z,
        part_init_pose.orientation.w);

    // orientation of the part in the tray, in world frame
    tf2::Quaternion q_target_part(
        target_pose_in_world.orientation.x,
        target_pose_in_world.orientation.y,
        target_pose_in_world.orientation.z,
        target_pose_in_world.orientation.w);

    // relative rotation between init and target
    tf2::Quaternion q_rot = q_target_part * q_init_part.inverse();

    // apply this rotation to the current gripper rotation
    tf2::Quaternion q_rslt = q_rot * q_current;
    q_rslt.normalize();

    double z_margin{};
    if (part_type.find("pump") != std::string::npos) {
        z_margin = 0.16;
    }
    else {
      z_margin = 0.11; 
    }
    
    // orientation of the gripper when placing the part in the tray
    target_pose_in_world.orientation.x = q_rslt.x();
    target_pose_in_world.orientation.y = q_rslt.y();
    target_pose_in_world.orientation.z = q_rslt.z();
    target_pose_in_world.orientation.w = q_rslt.w();
    target_pose_in_world.position.z += z_margin;
    
    ROS_INFO("Move to place pose"); 
    if (flip) {
        ROS_INFO("flip part"); 
        target_pose_in_world.orientation.z += 5; 
        m_arm_group.setPoseTarget(target_pose_in_world); 
        m_arm_group.move(); 
    }
    else {
        while (not this->move_target_pose(target_pose_in_world)) {
          ROS_INFO("IK not found for place"); 
          go_to_preset_location(agv);

          std::random_device rd; 
          std::mt19937 gen(rd());
          std::normal_distribution<double> gauss_dist(-0.05, 0.05); 
          auto random_dist = gauss_dist(gen); 
          m_joint_group_positions.at(0) += random_dist; 
          this->move_base_to(m_joint_group_positions.at(0));
          ros::Duration(0.1).sleep();
        }
    }

    ros::Duration(1.0).sleep();
    deactivate_gripper();

    m_arm_group.setMaxVelocityScalingFactor(1.0);
    this->lift(); 
    ROS_INFO("---End place part"); 

    return target_pose_in_world;
}
bool KittingArm::discard_faulty(const nist_gear::Model& faulty_part, std::string target_agv)
{
      std::string camera_id; 
      if (target_agv.compare("agv1") == 0) {
        camera_id = "logical_camera_ks1"; 
      }
      else if (target_agv.compare("agv2") == 0) {
        camera_id = "logical_camera_ks2"; 
      }
      else if (target_agv.compare("agv3") == 0) {
        camera_id = "logical_camera_ks3"; 
      }
      else if (target_agv.compare("agv4") == 0) {
        camera_id = "logical_camera_ks4"; 
      } 

      ROS_INFO("---Start discard part"); 
      ariac_group1::GetPartPosition srv; 
      srv.request.camera_id = camera_id; 
      srv.request.part = faulty_part; 
      geometry_msgs::Pose faulty_part_pose; 
      if (m_get_part_position_client.call(srv)) {
        faulty_part_pose = srv.response.pose; 
      } else {
        ROS_INFO("No rectified pose"); 
      }
      this->copy_current_joints_position(); 

      std::random_device rd; 
      std::mt19937 gen(rd());

      int trial_count = 1; 
      ROS_INFO("  trial %d", trial_count); 
      while (not pick_part(faulty_part.type, faulty_part_pose, camera_id)) {
        
        ROS_INFO("Discard picking fails, try new joint config for discarding"); 
        ros::Duration(3.0).sleep(); 
        this->reset_arm(); 
        trial_count++; 
        ROS_INFO("trial %d", trial_count); 

        if (m_joint_group_positions.at(0) > 4 or 
            m_joint_group_positions.at(0) < -4 ) {
          return false; 
        }

        if (trial_count == 1) {
          std::normal_distribution<double> gauss_dist(-0.05, 0.05); 
          auto random_dist = gauss_dist(gen); 
          m_joint_group_positions.at(0) += random_dist; 
          m_arm_group.setJointValueTarget(m_joint_group_positions);
          this->move_arm_group(); 
        }

        if (trial_count > 1) {
          std::normal_distribution<double> gauss_dist(-0.3, 0.3); 
          auto random_dist = gauss_dist(gen); 
          move_base_to(m_joint_group_positions.at(0) + random_dist);
        }

        this->copy_current_joints_position(); 
        
        if (trial_count > 5) return false; 
      }
      ROS_INFO("Discard picking success"); 

      go_to_preset_location("home_face_bins");
      ros::Duration(0.5).sleep();
      deactivate_gripper();
      ROS_INFO("---End discard part"); 
      return true; 
}

bool KittingArm::flip_part(const ariac_group1::PartTask& part_task) 
{
    bool flip = true; 
    auto part_type = part_task.part.type; 
    auto target_agv = part_task.agv_id; 
    auto target_pose_in_frame = part_task.part.pose; 

    ariac_group1::PartsUnderCamera srv; 
    srv.request.camera_id = "ks"; 
    srv.request.camera_id += part_task.agv_id.back(); 

    bool srv_success = m_parts_under_camera_client.call(srv);
    if (not srv_success) {
        ROS_INFO("Sensor blackout don't flip'"); 
        return false; 
    }

    ros::Duration(0.5).sleep(); 

    std::string camera_id_flip = "logical_camera_ks"; 
    camera_id_flip += part_task.agv_id.back(); 

    nist_gear::Model target_part; 
    auto target_pose_in_world = Utility::motioncontrol::transformToWorldFrame(
         target_pose_in_frame,
         target_agv);

    target_part.type = part_type; 
    target_part.pose = target_pose_in_world;

    for (auto part: srv.response.parts) {
        if (Utility::is_same_part(part, target_part, 0.1) and
            part.type == target_part.type) {
            target_part = part; 
            break; 
        }
    }
    auto target_rpy = Utility::motioncontrol::eulerFromQuaternion(target_part.pose);
    // if target roll close to 0, then do two flip
    if (abs(target_rpy.at(0)) < 0.1) {
    
        move_base_to(target_part.pose.position.y - 0.3);

        // flip first 90 degree
        if (pick_part(part_type, target_part.pose, camera_id_flip)) {
            auto flip_pose_in_world_1 = place_part(part_type, target_part.pose, target_pose_in_frame, target_agv, flip);
        }
        else {
            return false; 
        }

        ros::Duration(0.5).sleep(); 
        srv_success = m_parts_under_camera_client.call(srv);
        if (not srv_success) {
            ROS_INFO("Sensor blackout don't flip'"); 
            return false; 
        }

        for (auto part: srv.response.parts) {
            auto part_rpy = Utility::motioncontrol::eulerFromQuaternion(part.pose);
            if (abs(part_rpy.at(0)) > 0.1) {
                target_part = part; 
                break; 
            }
        }
    }

    // flip second 90 degree
    if (pick_part(part_type, target_part.pose, camera_id_flip)) {
        auto flip_pose_in_world_2 = place_part(part_type, target_part.pose, target_pose_in_frame, target_agv, flip);
    }
    else {
        return false; 
    }

    srv_success = m_parts_under_camera_client.call(srv);
    if (not srv_success) {
        ROS_INFO("Sensor blackout don't flip'"); 
        return false; 
    }

    for (auto part: srv.response.parts) {
        auto part_rpy = Utility::motioncontrol::eulerFromQuaternion(part.pose);
        if (abs(part_rpy.at(0)) > 0.1) {
            target_part = part; 
            break; 
        }
    }

    // turn to correct orientation
    if (pick_part(part_type, target_part.pose, camera_id_flip)) {
        auto flip_pose_in_world_3 = place_part(part_type, target_part.pose, part_task.part.pose, target_agv);
    }
    else {
        return false; 
    }
}

void KittingArm::lift()
{
  this->copy_current_joints_position(); 
  m_joint_group_positions.at(2) -= 0.3; 
  m_joint_group_positions.at(3) -= 0.1; 
  m_arm_group.setJointValueTarget(m_joint_group_positions);
  this->move_arm_group(); 
}

void KittingArm::turn_to_bins()
{

  this->copy_current_joints_position(); 

  double epsilon = 0.01; 
  if (abs(m_joint_group_positions.at(1) - (-M_PI)) < epsilon) {
    return; 
  }

  m_joint_group_positions.at(1) = -M_PI; 
  m_arm_group.setJointValueTarget(m_joint_group_positions);
  this->move_arm_group(); 
}

void KittingArm::turn_to_belt()
{

  this->copy_current_joints_position(); 

  double epsilon = 0.01; 
  if (abs(m_joint_group_positions.at(1)) < epsilon) {
    return; 
  }

  m_joint_group_positions.at(1) = 0; 
  m_arm_group.setJointValueTarget(m_joint_group_positions);
  this->move_arm_group(); 
}

// belt actions
void KittingArm::move_to_belt_intercept_pose(const geometry_msgs::Pose& belt_part)
{
    this->go_to_preset_location("belt_intercept"); 
    geometry_msgs::Pose arm_ee_link_pose = m_arm_group.getCurrentPose().pose;
    auto flat_orientation = Utility::motioncontrol::quaternionFromEuler(2, 0, 1.57);
    arm_ee_link_pose.orientation.x = flat_orientation.getX();
    arm_ee_link_pose.orientation.y = flat_orientation.getY();
    arm_ee_link_pose.orientation.z = flat_orientation.getZ();
    arm_ee_link_pose.orientation.w = flat_orientation.getW();

    arm_ee_link_pose.position.x = belt_part.position.x; 
    arm_ee_link_pose.position.z = belt_part.position.z;

    m_arm_group.setPoseTarget(arm_ee_link_pose); 
    m_arm_group.move(); 
}

bool KittingArm::get_belt_part(double range)
{
    ROS_INFO("Get belt part"); 
        while (!m_gripper_state.enabled) {
        activate_gripper();
    }

    geometry_msgs::Pose belt_part; 
    belt_part.position.x = -0.65 + range; 
    // belt_part.position.z = 0.93; 
    belt_part.position.z = 0.945; 

    this->move_to_belt_intercept_pose(belt_part); 

    m_arm_group.setMaxVelocityScalingFactor(0.6);
    int count = 0; 
    while (!m_gripper_state.attached) {
        ROS_INFO("Not attached"); 
        move_base_to(m_current_joint_states.position.at(1) + 0.3); 
        ros::Duration(0.2).sleep(); 
        count++; 
        if (m_current_joint_states.position.at(1) > 3) {
            ROS_INFO("No belt part"); 
            deactivate_gripper();
            return false; 
        }
    }

    ROS_INFO("Attached!!!"); 

    m_arm_group.setMaxVelocityScalingFactor(1.0);
    this->reset_arm(); 
    ros::Duration(0.1).sleep(); 
    return true; 

}

void KittingArm::place_to_vacancy(const geometry_msgs::Pose& vacancy_pose, bool from_belt)
{
    ROS_INFO("Place to vacancy"); 
    geometry_msgs::Pose arm_ee_link_pose = m_arm_group.getCurrentPose().pose;

    tf2::Quaternion flat_orientation; 
    if (from_belt) {
      flat_orientation = Utility::motioncontrol::quaternionFromEuler(2, 0, 1.57);
    } 
    else {
      flat_orientation = Utility::motioncontrol::quaternionFromEuler(0, 1.57, 0);

    }
    arm_ee_link_pose.orientation.x = flat_orientation.getX();
    arm_ee_link_pose.orientation.y = flat_orientation.getY();
    arm_ee_link_pose.orientation.z = flat_orientation.getZ();
    arm_ee_link_pose.orientation.w = flat_orientation.getW();

    arm_ee_link_pose.position.x = vacancy_pose.position.x; 
    arm_ee_link_pose.position.y = vacancy_pose.position.y - 0.05; 
    arm_ee_link_pose.position.z = vacancy_pose.position.z + 0.2;

    m_arm_group.setPoseTarget(arm_ee_link_pose); 
    m_arm_group.move();
    ros::Duration(0.1).sleep(); 
    deactivate_gripper();
    ros::Duration(0.1).sleep(); 
    this->lift(); 
}

//==========private functions==========
// callbacks
void KittingArm::arm_joint_states_callback(const sensor_msgs::JointState::ConstPtr& joint_state_msg)
{
    if (joint_state_msg->position.size() == 0) {
        ROS_ERROR("[Arm][arm_joint_states_callback_] joint_state_msg->position.size() == 0!");
    }
    m_current_joint_states = *joint_state_msg;
}

void KittingArm::arm_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr& msg)
{
    m_arm_controller_state = *msg;
}


void KittingArm::gripper_state_callback(const nist_gear::VacuumGripperState::ConstPtr& gripper_state_msg)
{
    m_gripper_state = *gripper_state_msg;
}

nist_gear::VacuumGripperState KittingArm::get_gripper_state()
{
    return m_gripper_state;
}

// Shipments and part tasks
bool KittingArm::check_insufficient_shipment(int priority)
{
  if (priority < -6) {
    ariac_group1::GetCompetitionTime get_comp_time_srv; 
    m_get_competition_time_client.call(get_comp_time_srv); 
    auto competition_time = get_comp_time_srv.response.competition_time;
    if (competition_time > 25) {
      return true; 
    }
  }
  return false; 
}

void KittingArm::clear_part_task(ariac_group1::PartTask& part_task, int& priority)
{
  m_shipments.shipments_record[part_task.shipment_type]->unfinished_part_tasks--; 
  ROS_INFO("Part left in shipment %s: %d", part_task.shipment_type.c_str(),
                                           m_shipments.shipments_record[part_task.shipment_type]->unfinished_part_tasks); 

  //assume shipment has wrong_part
  nist_gear::Model wrong_part; 
  auto shipment_state = this->check_shipment_state(part_task, wrong_part); 
  this->process_shipment_state(shipment_state, part_task, priority, wrong_part); 
  if (shipment_state == ShipmentState::NOT_READY) {
    m_part_task_queue.pop_back();
  }
}

ShipmentState KittingArm::check_shipment_state(ariac_group1::PartTask& part_task, nist_gear::Model& wrong_part)
{
  if (m_shipments.check_redundant(part_task, wrong_part)) {
      ROS_INFO("Has redundant part in shipment %s", part_task.shipment_type.c_str()); 
      return ShipmentState::HAS_REDUNDANT; 
  }

  static bool missing_check = false;  

  // Check if shipment is ready to submit
  if (m_shipments.shipments_record[part_task.shipment_type]->unfinished_part_tasks == 0) {
    ariac_group1::CheckQualitySensor srv; 
    srv.request.agv_id = part_task.agv_id; 
    if (m_check_quality_sensor_client.call(srv)) {
      if (srv.response.faulty_parts.empty()) {
        ROS_INFO("No faulty part in shipment %s", part_task.shipment_type.c_str()); 
      }
      else { 
        ROS_INFO("Has faulty in shipment %s", part_task.shipment_type.c_str()); 
        //push faulty task back to queue
        nist_gear::Model faulty_part = srv.response.faulty_parts[0]; 
        part_task.part.type = faulty_part.type;  
        part_task.part.pose = faulty_part.pose;  
        return ShipmentState::HAS_FAULTY; 
      }

      std::string result = m_shipments.check_shipment_parts(part_task, wrong_part); 
      if (result == "wrong_type") {
        // put the wrong type part back to vacancy spot
        // possibily resolve insufficient shipment, therefore check missing
        missing_check = true; 
        ROS_INFO("Has wrong type in shipment %s", part_task.shipment_type.c_str()); 
        return ShipmentState::HAS_WRONG_TYPE; 
      }
      else if (result == "flip_part") {
        ROS_INFO("Has flip part in shipment %s", part_task.shipment_type.c_str()); 
        return ShipmentState::HAS_FLIP_PART; 
      }
      else if (result == "wrong_pose"){
        ROS_INFO("Has wrong pose in shipment %s", part_task.shipment_type.c_str()); 
        return ShipmentState::HAS_WRONG_POSE; 
      }
      
      else if (result == "missing_part") {
        ROS_INFO("missing priority: %d", part_task.priority); 
        if (not this->check_insufficient_shipment(part_task.priority)) {
            ROS_INFO("Has missing part in shipment %s", part_task.shipment_type.c_str()); 
            return ShipmentState::HAS_MISSING_PART; 
        }
      }

      ROS_INFO("Shipment %s correct", part_task.shipment_type.c_str()); 
      return ShipmentState::READY; 
      //todo
      //else if (result == "redundant_part") {
      //   return ShipmentState::HAS_REDUNDANT_PART; 
      // }
    } 
    else {
      ROS_INFO("Sensor blackout, postpone shipment %s", part_task.shipment_type.c_str()); 
      return ShipmentState::POSTPONE; 
    }
  }
  else {
    return ShipmentState::NOT_READY; 
  }
}

void KittingArm::process_shipment_state(ShipmentState shipment_state, ariac_group1::PartTask& part_task, int& priority, nist_gear::Model& wrong_part)
{
  switch(shipment_state) {
    case ShipmentState::READY: 
      {
        ros::Duration(1).sleep();

        // this->submit_shipment(part_task.agv_id, part_task.shipment_type, part_task.station_id); 
        m_agvs_dict[part_task.agv_id]->submit_shipment(part_task.shipment_type, part_task.station_id); 
        m_shipments.shipments_record[part_task.shipment_type]->state = ShipmentState::FINISH; 
        m_part_task_queue.pop_back();
        break; 
      }

    case ShipmentState::HAS_REDUNDANT: 
    case ShipmentState::HAS_WRONG_TYPE: 
      {
        ariac_group1::GetVacancyPose vacancy_pose_srv; 
        m_get_vacancy_pose_client.call(vacancy_pose_srv); 
        auto& vacancy_poses = vacancy_pose_srv.response.vacancy_poses;
        
        if (vacancy_poses.empty()) {
          bool discard_success = this->discard_faulty(wrong_part, part_task.agv_id); 
          if (discard_success) {
            if (shipment_state == ShipmentState::HAS_WRONG_TYPE) {
              m_shipments.shipments_record[part_task.shipment_type]->unfinished_part_tasks++; 
            }
          }
          return; 
        }
        else {
          std::string camera_id = "logical_camera_ks"; 
          camera_id += part_task.agv_id.back(); 
          if (pick_part(wrong_part.type, wrong_part.pose, camera_id)) {
            place_to_vacancy(vacancy_poses[0], false); 
            if (shipment_state == ShipmentState::HAS_WRONG_TYPE) {
              m_shipments.shipments_record[part_task.shipment_type]->unfinished_part_tasks++; 
            }
          }
        }
        break; 
      }

    case ShipmentState::HAS_WRONG_POSE: 
      {
        std::string camera_id = "logical_camera_ks"; 
        camera_id += part_task.agv_id.back(); 
        if (pick_part(wrong_part.type, wrong_part.pose, camera_id)) {
          auto target_pose_in_world = place_part(wrong_part.type, wrong_part.pose, part_task.part.pose, part_task.agv_id);
        }
        break; 
      }
    case ShipmentState::HAS_FLIP_PART: 
      {
        auto wrong_part_rpy = Utility::motioncontrol::eulerFromQuaternion(wrong_part.pose);
        // auto yaw = wrong_part_rpy.at(2); 
        // if (yaw < 0) {
        //     yaw = 2 * M_PI + yaw; 
        // }
        ariac_group1::PartInfo part_info; 
        part_info.part = wrong_part; 
        part_info.roll = wrong_part_rpy.at(0); 
        part_info.pitch = wrong_part_rpy.at(1); 
        part_info.yaw = wrong_part_rpy.at(2); 
        std::string camera_id = "logical_camera_ks"; 
        camera_id += part_task.agv_id.back(); 
        part_info.camera_id = camera_id; 

        this->move_part(part_info, part_task); 
        // }
        // else {
        //     this->flip_part(part_task); 
        // }

        break; 
      }

    case ShipmentState::HAS_MISSING_PART: 
      {
        m_shipments.shipments_record[part_task.shipment_type]->unfinished_part_tasks++; 
        break; 
      }

    case ShipmentState::HAS_FAULTY: 
      {
        go_to_preset_location(part_task.agv_id);
        nist_gear::Model faulty_part; 
        faulty_part.type = part_task.part.type; 
        faulty_part.pose = part_task.part.pose; 
        bool discard_success = this->discard_faulty(faulty_part, part_task.agv_id); 
        auto pose_in_tray_frame = Utility::motioncontrol::transformToTrayFrame(faulty_part.pose, part_task.agv_id); 
        part_task.part.pose = pose_in_tray_frame; 
        if (discard_success) {
          // ready to replace the discard part
          m_shipments.shipments_record[part_task.shipment_type]->unfinished_part_tasks++; 
        }
        break; 
      }

    case ShipmentState::POSTPONE: 
      {
        priority += Constants::PriorityWeight::Penalty::SHIPMENT_POSTPONE; 
        break; 
      }

    default: 
        break; 
  }
  
}

bool KittingArm::check_emergency_interrupt()
{
  // ROS_INFO("Checking for emergency interrupt"); 
  ariac_group1::GetBeltProximitySensor belt_proximity_sensor_srv; 

  bool interrupt = false; 

  bool get_belt_part_available = false; 

  int count = 0; 

  do {
    m_get_belt_proximity_sensor_client.call(belt_proximity_sensor_srv); 
    auto range = belt_proximity_sensor_srv.response.range; 

    ariac_group1::GetVacancyPose vacancy_pose_srv; 
    m_get_vacancy_pose_client.call(vacancy_pose_srv); 
    auto& vacancy_poses = vacancy_pose_srv.response.vacancy_poses;  

    get_belt_part_available = range > 0 and not vacancy_poses.empty(); 
    if (get_belt_part_available) {
      ROS_INFO("Belt sensor triggered"); 
      m_working_station = "belt"; 
      deactivate_gripper();
      ros::Duration(0.1).sleep(); 
      this->reset_arm(); 
      if (this->get_belt_part(range)) {
          this->place_to_vacancy(vacancy_poses.at(0)); 
      }
      this->reset_arm();
      interrupt = true; 
    }
    count++; 
  }
  while (get_belt_part_available or count < 3);  

  if (m_shipments.is_high_priority_alert()) {
      interrupt = true; 
  }
     
  return interrupt; 
}

// arm control
void KittingArm::copy_current_joints_position()
{
  moveit::core::RobotStatePtr current_state = m_arm_group.getCurrentState();
  const moveit::core::JointModelGroup* joint_model_group =
          current_state->getJointModelGroup("kitting_arm");

  // next get the current set of joint values for the group.
  current_state->copyJointGroupPositions(joint_model_group, m_joint_group_positions);
}

bool KittingArm::move_arm_group()
{
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    // check a plan is found first then execute the action
    bool success = (m_arm_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success)
        m_arm_group.move();
    else
        ROS_INFO("Path planning fails"); 

    return success; 
}


// gripper control
void KittingArm::activate_gripper()
{
    nist_gear::VacuumGripperControl srv;
    srv.request.enable = true;
    m_gripper_control_client.call(srv);

    ROS_INFO_STREAM("Activate gripper " << srv.response);
}

void KittingArm::deactivate_gripper()
{
    nist_gear::VacuumGripperControl srv;
    srv.request.enable = false;
    m_gripper_control_client.call(srv);

    ROS_INFO_STREAM("Deactivate gripper " << srv.response);
}

// auxiliary functions
bool KittingArm::move_target_pose(const geometry_msgs::Pose& pose, bool from_back_row_bins)
{
  this->set_pick_constraints(from_back_row_bins); 

  unsigned int max_attempts = 1; 
  int attempts = 0; 
  std::vector<double> current_joint_group_positions = m_joint_group_positions; 
  std::random_device rd; 
  std::mt19937 gen(rd()); 

  while(attempts < max_attempts) {
      attempts++; 

      if (attempts > 1) {
      // add random motion to try again
          std::normal_distribution<double> gauss_dist(-0.05, 0.05); 
          auto random_dist = gauss_dist(gen); 
          m_joint_group_positions.at(0) += random_dist; 
          this->move_base_to(m_joint_group_positions.at(0)); 
      }

      ROS_INFO("Set pose attempts: %d", attempts); 
      m_joint_group_positions = current_joint_group_positions; 
      moveit::core::RobotStatePtr current_state = m_arm_group.getCurrentState();
      const moveit::core::JointModelGroup* joint_model_group =
              current_state->getJointModelGroup("kitting_arm");

      double timeout = 5; 
      bool found_ik = current_state->setFromIK(joint_model_group, pose, timeout); 

      if (found_ik) {
        std::vector<double> target_joint_group_positions; 
        current_state->copyJointGroupPositions(joint_model_group, target_joint_group_positions);
        bool target_joints_infeasible = false; 

        if (not from_back_row_bins) {
            auto joint_1 = target_joint_group_positions.at(1);  
            auto joint_2 = target_joint_group_positions.at(2);  
            auto joint_3 = target_joint_group_positions.at(3);  
            auto joint_4 = target_joint_group_positions.at(4);  

            if (joint_2 > 0) {
                joint_2 = joint_2 - M_PI * 2; 
            }

            if (joint_3 < 0) {
                joint_3 = joint_3 + M_PI * 2; 
            }

            if (joint_4 > 0) {
                joint_4 = joint_4 - M_PI * 2; 
            }

            if (joint_2 > -0.17 or joint_2 < -1.74) { 
              ROS_INFO("Target joint 2 infisible: %f", target_joint_group_positions.at(2)); 
              target_joints_infeasible = true; 
            }

            if (joint_3 > 2.5 or joint_3 < 0.05) { 
              ROS_INFO("Target joint 3 infisible: %f", target_joint_group_positions.at(3)); 
              target_joints_infeasible = true; 
            }

            if (joint_4 > -0.7 or joint_4 < -3.45) { 
              ROS_INFO("Target joint 4 infisible: %f", target_joint_group_positions.at(4)); 
              target_joints_infeasible = true; 
            }
        }

        if (target_joints_infeasible) {

          ros::Duration(3.0).sleep(); 
          m_arm_group.setMaxVelocityScalingFactor(0.5);
          m_joint_group_positions = current_joint_group_positions; 
          m_arm_group.setJointValueTarget(m_joint_group_positions);
          this->move_arm_group(); 
          m_arm_group.setMaxVelocityScalingFactor(1.0);
          continue; 
        }

        m_joint_group_positions.at(0) = target_joint_group_positions.at(0); 
        m_joint_group_positions.at(1) = target_joint_group_positions.at(1); 
        m_arm_group.setJointValueTarget(m_joint_group_positions);
        this->move_arm_group(); 

        m_joint_group_positions = target_joint_group_positions; 
        m_arm_group.setJointValueTarget(m_joint_group_positions);
        this->move_arm_group(); 
        m_arm_group.clearPathConstraints();
        return true; 
      }
  }

  this->copy_current_joints_position(); 
  m_arm_group.clearPathConstraints();
  return false; 

}

bool KittingArm::check_faulty(const nist_gear::Model& faulty_part)
{
  ariac_group1::IsFaulty srv; 
  
  srv.request.part = faulty_part; 
  Utility::print_part_pose(faulty_part); 
  m_is_faulty_client.call(srv); 

  return srv.response.faulty; 
}

void KittingArm::set_pick_constraints(bool from_back_row_bins)
{
  // set constraint to shoulder lift and elbow
  moveit_msgs::Constraints constraints;

  moveit_msgs::JointConstraint joint_constraint; 

  joint_constraint.joint_name = "wrist_2_joint"; 
  joint_constraint.position = -1.57;  
  joint_constraint.tolerance_above = 0.1; 
  joint_constraint.tolerance_below = 0.1; 
  joint_constraint.weight = 1; 

  constraints.joint_constraints.push_back(joint_constraint); 


  if (from_back_row_bins) {
      m_arm_group.setPathConstraints(constraints);
      return; 
  }

  joint_constraint.joint_name = "shoulder_lift_joint"; 
  joint_constraint.position = -1.25;  
  joint_constraint.tolerance_above = 1.05; 
  joint_constraint.tolerance_below = 0.5; 
  joint_constraint.weight = 0.8; 

  constraints.joint_constraints.push_back(joint_constraint); 

  joint_constraint.joint_name = "elbow_joint"; 
  joint_constraint.position = 1.74;  
  joint_constraint.tolerance_above = 0.76; 
  joint_constraint.tolerance_below = 1.7; 
  joint_constraint.weight = 0.8; 

  constraints.joint_constraints.push_back(joint_constraint); 

  joint_constraint.joint_name = "wrist_1_joint"; 
  joint_constraint.position = -2.04;  
  joint_constraint.tolerance_above = 1.4; 
  joint_constraint.tolerance_below = 1.4; 
  joint_constraint.weight = 0.8; 

  constraints.joint_constraints.push_back(joint_constraint); 

  m_arm_group.setPathConstraints(constraints);

}

// utils
void KittingArm::print_joint_group_positions() {
  ROS_INFO("Target Joint positions: "); 
  for (int i; i < m_joint_group_positions.size(); ++i) {
    ROS_INFO("  Joint %d: %f", i, m_joint_group_positions.at(i)); 
  }
}

bool KittingArm::get_working_station(ariac_group1::GetWorkingStation::Request &req,
                                      ariac_group1::GetWorkingStation::Response &res)
{
    res.working_station = m_working_station; 
    return true; 
}
