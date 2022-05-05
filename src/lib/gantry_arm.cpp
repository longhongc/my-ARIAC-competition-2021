#include "gantry_arm.h"

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
#include <nist_gear/AssemblyStationSubmitShipment.h>
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


using AGVToAssem = nist_gear::AGVToAssemblyStation; 
using AssemSubmit = nist_gear::AssemblyStationSubmitShipment; 

GantryArm::GantryArm():
  m_nh{"/ariac/gantry"},
  m_planning_group{"/ariac/gantry/robot_description"},
  m_full_gantry_options{"gantry_full", m_planning_group, m_nh},
  m_gantry_options{"gantry_arm", m_planning_group, m_nh},
  m_torso_gantry_options{"gantry_torso", m_planning_group, m_nh},
  m_full_gantry_group{m_full_gantry_options},
  m_gantry_group{m_gantry_options},
  m_torso_gantry_group{m_torso_gantry_options}, 
  m_shipments{&m_nh}
{

  isNewShipment = true;
  atHome1 = false;
  atHome2 = false;
  gantry_cloc = "loc";
  
  // publishers to directly control the joints without moveit
  m_arm_joint_trajectory_publisher =
      m_nh.advertise<trajectory_msgs::JointTrajectory>("/ariac/gantry/gantry_arm_controller/command", 10);

  // subscribers
  m_arm_joint_states_subscriber =
      m_nh.subscribe("/ariac/gantry/joint_states", 10, &GantryArm::arm_joint_states_callback, this);
  
  m_arm_controller_state_subscriber =
      m_nh.subscribe("/ariac/gantry/gantry_arm_controller/state", 10, &GantryArm::arm_controller_state_callback, this);

  m_gripper_state_subscriber = 
      m_nh.subscribe("/ariac/gantry/arm/gripper/state", 10, &GantryArm::gripper_state_callback, this);

  // m_part_task_subscriber =
  //     m_nh.subscribe("/part_task", 10, &GantryArm::part_task_callback, this); 

  // clients
  m_gripper_control_client =
      m_nh.serviceClient<nist_gear::VacuumGripperControl>("/ariac/gantry/arm/gripper/control");
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

  m_submit_shipment_as1_client = 
      m_nh.serviceClient<AssemSubmit>("/ariac/as1/submit_shipment"); 
  m_submit_shipment_as1_client.waitForExistence();

  m_submit_shipment_as2_client = 
      m_nh.serviceClient<AssemSubmit>("/ariac/as2/submit_shipment"); 
  m_submit_shipment_as2_client.waitForExistence();

  m_submit_shipment_as3_client = 
      m_nh.serviceClient<AssemSubmit>("/ariac/as3/submit_shipment"); 
  m_submit_shipment_as3_client.waitForExistence();

  m_submit_shipment_as4_client = 
      m_nh.serviceClient<AssemSubmit>("/ariac/as4/submit_shipment"); 
  m_submit_shipment_as4_client.waitForExistence();

  for (auto& id: m_agvs_id) {
      m_agvs_dict[id] = std::make_unique<AGV>(&m_nh, id); 
  }

  // five
  five.gantry_torso ={ -2.0, 0, 1.57};
  five.gantry_arm = { 0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
  five.gantry_full = { -2.0, 1.57, 0 ,0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
  five.name = "five";
  // two
  two.gantry_torso ={ -1.50, 0, 0 };
  two.gantry_arm = { -3.25 , -2.43 , 1 ,1.43 ,-1.679 ,0 };
  two.gantry_full = { -1.50, 0, 0 ,0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
  two.name = "two";
  // eight
  eight.gantry_torso ={ -6.50, 0, 1.57 };
  eight.gantry_arm = { 0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
  eight.gantry_full = { -8.50, 0, 0 ,0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
  eight.name = "eight";
  // six
  six.gantry_torso ={ -3.50, -2.5, 0 };
  six.gantry_arm = { 0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
  six.gantry_full = { -3.50, -2.5, 0 ,0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
  six.name = "six";
  // three
  three.gantry_torso ={ -1.50, -2.5, 0 };
  three.gantry_arm = { 0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
  three.gantry_full = { -1.50, -2.5, 0 ,0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
  three.name = "three";
  // nine
  nine.gantry_torso ={ -8.50, -2.75, 0 };
  nine.gantry_arm = { 0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
  nine.gantry_full = { -8.50, -2.75, 0 ,0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
  nine.name = "nine";
  // four 
  four.gantry_torso ={ -3.50, 2.5, 0 };
  four.gantry_arm = { 0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
  four.gantry_full = { -3.50, 2.5, 0 ,0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
  four.name = "four";
  // one
  one.gantry_torso ={ -1.50, 2.5, 0 };
  one.gantry_arm = { -3.259 , -2.285 , 0.569 , 1.73 ,-1.7 ,-1.57 };
  one.gantry_full = { -1.50, 2.5, 0 ,0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
  one.name = "one";
  // seven
  seven.gantry_torso ={ -8.50, 2.75, 0 };
  seven.gantry_arm = { 0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
  seven.gantry_full = { -8.50, 2.75, 0 ,0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
  seven.name = "seven";
  
  // above bins3
  at_bins3.gantry_torso = { -0.56, -1.65, -0.02 };
  at_bins3.gantry_arm = { 0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
  at_bins3.gantry_full = { -0.56, -1.65, -0.02  ,0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
  at_bins3.name = "at_bins3";

  // above bins4
  at_bins4.gantry_torso = { -0.56, -2.5, -0.02 };
  at_bins4.gantry_arm = { 0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
  at_bins4.gantry_full = { -0.56, -2.5, -0.02  ,0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
  at_bins4.name = "at_bins4";

  // above bins7
  at_bins7.gantry_torso = { -0.8, -3.2, -0.02 };
  at_bins7.gantry_arm = { 0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
  at_bins7.gantry_full = { -0.8, -3.2, -0.02  ,0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
  at_bins7.name = "at_bins7";

  // above bins8
  at_bins8.gantry_torso = { -0.8, -4, -0.02 };
  at_bins8.gantry_arm = { 0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
  at_bins8.gantry_full = { -0.8, -4, -0.02  ,0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
  at_bins8.name = "at_bins8";

  // at_agv1
  at_agv1.gantry_torso = { 0.05, -3.7, -0.02 };
  at_agv1.gantry_arm = { 0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
  at_agv1.gantry_full = { 0.05, -3.7, -0.02  ,0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
  at_agv1.name = "at_agv1";

  // at_agv2
  at_agv2.gantry_torso = { 0.05, -0.7, -0.02 };
  at_agv2.gantry_arm = { 0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
  at_agv2.gantry_full = { 0.05, -0.7, -0.02  ,0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
  at_agv2.name = "at_agv2";
  
  // at_agv3
  at_agv3.gantry_torso = { 0.05, 2, -0.02 };
  at_agv3.gantry_arm = { 0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
  at_agv3.gantry_full = { 0.05, 2, -0.02  ,0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
  at_agv3.name = "at_agv3";

  // at_agv4
  at_agv4.gantry_torso = { 0.56, 4.27, 1.57 };
  at_agv4.gantry_arm = { 0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
  at_agv4.gantry_full = { 0.56, 4.27, 1.57  ,0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
  at_agv4.name = "at_agv4";

  // at AS3
  at_as3.gantry_torso = { -3.0 , -2.8 + 6.063705, 1.57 };
  at_as3.gantry_arm = { 0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
  at_as3.gantry_full = { -3.7 , -2.8, 1.57 ,0 , -1.13 , 1.88 ,-0.72 ,1.55 ,2.4 };
  at_as3.name = "at_as3";

  // at AS4
  at_as4.gantry_torso = { -8.0 , -2.8 + 6.063705, 1.57 };
  at_as4.gantry_arm = { 0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83  };
  at_as4.gantry_full = { -8.7 , -2.8, 1.57 ,0 , -1.13 , 1.88 ,-0.72 ,1.55 ,2.4 };
  at_as4.name = "at_as4";

  // at agv4 at as3
  at_agv4_at_as3.gantry_torso = { -2.72 , -1.26 + 6.063705, 1.57 };
  at_agv4_at_as3.gantry_arm = { 0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
  at_agv4_at_as3.gantry_full = { -2.72 , -1.26, 1.57 ,0 , -1.13 , 1.88 ,-0.72 ,1.55 ,2.4 };
  at_agv4_at_as3.name = "at_agv4_at_as3";

  // at agv3 at as3
  at_agv3_at_as3.gantry_torso = { -2.72 , -1.26 + 2.7, 1.15 };
  at_agv3_at_as3.gantry_arm = { 0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
  at_agv3_at_as3.gantry_full = { -2.72 , -1.26, 1.57 ,0 , -1.13 , 1.88 ,-0.72 ,1.55 ,2.4 };
  at_agv3_at_as3.name = "at_agv3_at_as3";

  // at agv4 at as4
  at_agv4_at_as4.gantry_torso = { -7.72 , -1.26 + 6.063705, 1.57 };
  at_agv4_at_as4.gantry_arm = { 0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
  at_agv4_at_as4.gantry_full = { -7.72 , -1.26, 1.57 ,0 , -1.13 , 1.88 ,-0.72 ,1.55 ,2.4 };
  at_agv4_at_as4.name = "at_agv4_at_as4";

  // at agv3 at as4
  at_agv3_at_as4.gantry_torso = { -7.70 , -1.26 + 2.75, 1.15 };
  at_agv3_at_as4.gantry_arm = { 0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
  at_agv3_at_as4.gantry_full = { -7.72 , -1.26, 1.57 ,0 , -1.13 , 1.88 ,-0.72 ,1.55 ,2.4 };
  at_agv3_at_as4.name = "at_agv3_at_as4";

  // at agv2 at as1
  at_agv2_at_as1.gantry_torso = { -2.72 , -1.26 , 1.15 };
  at_agv2_at_as1.gantry_arm = { 0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
  at_agv2_at_as1.gantry_full = { -2.72 , -1.26, 1.57 ,0 , -1.13 , 1.88 ,-0.72 ,1.55 ,2.4 };
  at_agv2_at_as1.name = "at_agv2_at_as1";
  
  // at agv2 at as2
  at_agv2_at_as2.gantry_torso = { -7.72 , -1.26 , 1.15 };
  at_agv2_at_as2.gantry_arm = { 0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
  at_agv2_at_as2.gantry_full = { -7.72 , -1.26, 1.57 ,0 , -1.13 , 1.88 ,-0.72 ,1.55 ,2.4 };
  at_agv2_at_as2.name = "at_agv2_at_as2";

  // at agv1 at as1
  at_agv1_at_as1.gantry_torso = { -2.72 , -1.26 - 3.36 , 1.15 };
  at_agv1_at_as1.gantry_arm = { 0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
  at_agv1_at_as1.gantry_full = { -2.72 , -1.26, 1.57 ,0 , -1.13 , 1.88 ,-0.72 ,1.55 ,2.4 };
  at_agv1_at_as1.name = "at_agv1_at_as1";

  // at agv1 at as2
  at_agv1_at_as2.gantry_torso = { -7.72 , -1.26 - 3.36, 1.15 };
  at_agv1_at_as2.gantry_arm = { 0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
  at_agv1_at_as2.gantry_full = { -7.72 , -1.26, 1.57 ,0 , -1.13 , 1.88 ,-0.72 ,1.55 ,2.4 };
  at_agv1_at_as2.name = "at_agv1_at_as2";



  // at as1
  at_as1.gantry_torso = { -3.0 , -2.8, 1.57 };
  at_as1.gantry_arm = { 0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83  };
  at_as1.gantry_full = { -3.7 , -2.8, 1.57 ,0 , -1.13 , 1.88 ,-0.72 ,1.55 ,2.4 };
  at_as1.name = "at_as1";

  // at as2
  at_as2.gantry_torso = { -8.0 , -2.8, 1.57 };
  at_as2.gantry_arm = { 0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83  };
  at_as2.gantry_full = { -8.7 , -2.8, 1.57 ,0 , -1.13 , 1.88 ,-0.72 ,1.55 ,2.4 };
  at_as2.name = "at_as2";


  // raw pointers are frequently used to refer to the planning group for improved performance.
  // to start, we will create a pointer that references the current robot’s state.
  const moveit::core::JointModelGroup* joint_model_group =
      m_full_gantry_group.getCurrentState()->getJointModelGroup("gantry_full");
  moveit::core::RobotStatePtr current_state = m_full_gantry_group.getCurrentState();
  // next get the current set of joint values for the group.
  current_state->copyJointGroupPositions(joint_model_group, m_joint_group_positions);
}

void GantryArm::copyCurrentJointsPosition()
{
  moveit::core::RobotStatePtr current_state = m_gantry_group.getCurrentState();
  const moveit::core::JointModelGroup* joint_model_group =
          current_state->getJointModelGroup("gantry_arm");

  // next get the current set of joint values for the group.
  current_state->copyJointGroupPositions(joint_model_group, m_joint_group_positions);
}

void GantryArm::print_joint_group_positions() {
  ROS_INFO("Target Joint positions: "); 
  for (int i; i < m_joint_group_positions.size(); ++i) {
    ROS_INFO("  Joint %d: %f", i, m_joint_group_positions.at(i)); 
  }
}

void GantryArm::arm_joint_states_callback(const sensor_msgs::JointState::ConstPtr& joint_state_msg)
{
    if (joint_state_msg->position.size() == 0) {
        ROS_ERROR("[Arm][arm_joint_states_callback_] joint_state_msg->position.size() == 0!");
    }
    m_current_joint_states = *joint_state_msg;
}

void GantryArm::arm_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr& msg)
{
    m_gantry_controller_state = *msg;
}
void GantryArm::gripper_state_callback(const nist_gear::VacuumGripperState::ConstPtr& gripper_state_msg)
{
    m_gripper_state = *gripper_state_msg;
}

// void GantryArm::part_task_callback(const ariac_group1::PartTask::ConstPtr& msg)
// {
//   const std::lock_guard<std::mutex> lock(*m_mutex_ptr); 
//   // add tasks to task vector
//   m_part_task_queue.emplace_back(std::make_tuple(msg->priority * Constants::PriorityWeight::Ratio::HIGH_PRIORITY,
//                                                  std::make_unique<ariac_group1::PartTask>(*msg))); 
//   if (not m_shipments_total_parts.count(msg->shipment_type)) {
//     m_shipments_total_parts[msg->shipment_type] = msg->total_parts; 
//     ROS_INFO("Receive shipment %s including %d parts", msg->shipment_type.c_str(), msg->total_parts); 
//   }
// }

// void GantryArm::print_shipments_total_parts() {
//   ROS_INFO("Shipment total parts: "); 
//   for (auto& part_count: m_shipments.shipment_record) {
//     ROS_INFO("  %s: %d", part_count.first.c_str(), part_count.second); 
//   }
//
// }

nist_gear::VacuumGripperState GantryArm::getGripperState()
{
    return m_gripper_state;
}

void GantryArm::activateGripper()
{
    nist_gear::VacuumGripperControl srv;
    srv.request.enable = true;
    m_gripper_control_client.call(srv);

    ROS_INFO_STREAM("Activate gripper " << srv.response);
}

void GantryArm::deactivateGripper()
{
    nist_gear::VacuumGripperControl srv;
    srv.request.enable = false;
    m_gripper_control_client.call(srv);

    ROS_INFO_STREAM("Deactivate gripper " << srv.response);
}

void GantryArm::goToPresetLocation(std::string location_name)
{

    ArmPresetLocation location;
    gantry_cloc = location_name;
    if (location_name.compare("one") == 0) {
        ROS_INFO("Moving to one"); 
        location = one;
    }
    else if (location_name.compare("two") == 0) {
        ROS_INFO("Moving to two"); 
        location = two;
    }
    else if(location_name.compare("three")==0){
        ROS_INFO("Moving to three");
        location = three;
    }
    else if(location_name.compare("four")==0){
        ROS_INFO("Moving to four");
        location = four;
    }
    else if(location_name.compare("five")==0){
        ROS_INFO("Moving to five");
        location = five;
    }
    else if(location_name.compare("six")==0){
        ROS_INFO("Moving to six");
        location = six;
    }
    else if(location_name.compare("seven")==0){
        ROS_INFO("Moving to seven");
        location = seven;
    }
    else if (location_name.compare("eight") == 0) {
        ROS_INFO("Moving to eight"); 
        location = eight;
    }
    else if (location_name.compare("nine") == 0) {
        ROS_INFO("Moving to nine"); 
        location = nine;
    }
    else if (location_name.compare("at_bins3") == 0) {
        ROS_INFO("Moving to at_bins3"); 
        location = at_bins3;
    }
    else if (location_name.compare("at_bins4") == 0) {
        ROS_INFO("Moving to at_bins4"); 
        location = at_bins4;
    }
    else if (location_name.compare("at_bins7") == 0) {
        ROS_INFO("Moving to at_bins7"); 
        location = at_bins7;
    }
    else if (location_name.compare("at_bins8") == 0) {
        ROS_INFO("Moving to at_bins8"); 
        location = at_bins8;
    }
    else if (location_name.compare("at_agv1") == 0) {
        ROS_INFO("Moving to at_agv1"); 
        location = at_agv1;
    }
    else if (location_name.compare("at_agv2") == 0) {
        ROS_INFO("Moving to at_agv2"); 
        location = at_agv2;
    }
    else if (location_name.compare("at_agv3") == 0) {
        ROS_INFO("Moving to at_agv3"); 
        location = at_agv3;
    }
    else if (location_name.compare("at_agv4") == 0) {
        ROS_INFO("Moving to at_agv4"); 
        location = at_agv4;
    }

    else if (location_name.compare("at_as3") == 0) {
        ROS_INFO("Moving to at_as3"); 
        location = at_as3;
    }

    else if (location_name.compare("as3_4") == 0) {
        ROS_INFO("Moving to at_agv4_at_as3"); 
        location = at_agv4_at_as3;
    }

    else if (location_name.compare("at_as1") == 0) {
        ROS_INFO("Moving to at_as1"); 
        location = at_as1;
    }

    else if (location_name.compare("as1_2") == 0) {
        ROS_INFO("Moving to at_agv2_at_as1"); 
        location = at_agv2_at_as1;
    }

    else if (location_name.compare("at_as2") == 0) {
        ROS_INFO("Moving to at_as2"); 
        location = at_as2;
    }

    else if (location_name.compare("at_as4") == 0) {
        ROS_INFO("Moving to at_as4"); 
        location = at_as4;
    }
    
    else if (location_name.compare("as2_2") == 0) {
        ROS_INFO("Moving to agv2_at_as2"); 
        location = at_agv2_at_as2;
    }

    else if (location_name.compare("as4_4") == 0) {
        ROS_INFO("Moving to agv4_at_as4"); 
        location = at_agv4_at_as4;
    }

    else if (location_name.compare("as3_3") == 0) {
        ROS_INFO("Moving to agv3_at_as3"); 
        location = at_agv3_at_as3;
    }
    else if (location_name.compare("as4_3") == 0) {
        ROS_INFO("Moving to agv3_at_as4"); 
        location = at_agv3_at_as4;
    }
    else if (location_name.compare("as1_1") == 0) {
        ROS_INFO("Moving to agv1_at_as1"); 
        location = at_agv1_at_as1;
    }
    else if (location_name.compare("as2_1") == 0) {
        ROS_INFO("Moving to agv1_at_as2"); 
        location = at_agv1_at_as2;
    }
    else {
        ROS_ERROR("[Arm][goToPresetLocation] ERROR: location_name not found!");
    }
    

    // gantry torso
    m_joint_group_positions.at(0) = location.gantry_torso.at(0);
    m_joint_group_positions.at(1) = location.gantry_torso.at(1);
    m_joint_group_positions.at(2) = location.gantry_torso.at(2);
    // gantry arm
    m_joint_group_positions.at(3) = location.gantry_arm.at(0);
    m_joint_group_positions.at(4) = location.gantry_arm.at(1);
    m_joint_group_positions.at(5) = location.gantry_arm.at(2);
    m_joint_group_positions.at(6) = location.gantry_arm.at(3);
    m_joint_group_positions.at(7) = location.gantry_arm.at(4);
    m_joint_group_positions.at(8) = location.gantry_arm.at(5);


    m_full_gantry_group.setJointValueTarget(m_joint_group_positions);
    this->move_arm_group();
}

bool GantryArm::move_arm_group()
{
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    // check a plan is found first then execute the action
    bool success = (m_gantry_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success)
        m_full_gantry_group.move();
    else
        ROS_INFO("Path planning fails"); 

    return success; 
}

void GantryArm::moveBaseTo(double linear_arm_actuator_joint_position)
{

    this->copyCurrentJointsPosition(); 
    
    // next, assign a value to only the linear_arm_actuator_joint
    m_joint_group_positions.at(1) = linear_arm_actuator_joint_position;

    // move the arm
    m_full_gantry_group.setJointValueTarget(m_joint_group_positions);
    m_full_gantry_group.move();
}

void GantryArm::resetArm()
{
    this->copyCurrentJointsPosition(); 
    m_joint_group_positions.at(1) = -M_PI;
    m_joint_group_positions.at(2) = -1.13;
    m_joint_group_positions.at(3) = 1.88;
    m_joint_group_positions.at(4) = -0.72;
    m_joint_group_positions.at(5) = 1.55;
    m_joint_group_positions.at(6) = 0.83;
    m_full_gantry_group.setJointValueTarget(m_joint_group_positions);
    this->move_arm_group(); 
}


void GantryArm::lift()
{
  this->copyCurrentJointsPosition(); 
  m_joint_group_positions.at(2) -= 0.3; 
  m_joint_group_positions.at(3) -= 0.1; 
  m_gantry_group.setJointValueTarget(m_joint_group_positions);
  this->move_arm_group(); 
}

void GantryArm::setPickConstraints()
{
  // set constraint to shoulder lift and elbow
  moveit_msgs::Constraints constraints;

  moveit_msgs::JointConstraint joint_constraint; 
  // joint_constraint.joint_name = "shoulder_pan_joint"; 
  // joint_constraint.position = -M_PI;  
  // joint_constraint.tolerance_above = 1.2; 
  // joint_constraint.tolerance_below = 1.2; 
  // joint_constraint.weight = 1; 
  //
  // constraints.joint_constraints.push_back(joint_constraint); 
  //

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

  joint_constraint.joint_name = "wrist_2_joint"; 
  joint_constraint.position = -1.57;  
  joint_constraint.tolerance_above = 0.1; 
  joint_constraint.tolerance_below = 0.1; 
  joint_constraint.weight = 1; 

  constraints.joint_constraints.push_back(joint_constraint); 


  m_gantry_group.setPathConstraints(constraints);

}

bool GantryArm::moveTargetPose(const geometry_msgs::Pose& pose)
{
  this->setPickConstraints(); 

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
          this->moveBaseTo(m_joint_group_positions.at(0)); 
      }

      ROS_INFO("Set pose attempts: %d", attempts); 
      m_joint_group_positions = current_joint_group_positions; 
      moveit::core::RobotStatePtr current_state = m_gantry_group.getCurrentState();
      const moveit::core::JointModelGroup* joint_model_group =
              current_state->getJointModelGroup("gantry_arm");

      double timeout = 5; 
      bool found_ik = current_state->setFromIK(joint_model_group, pose, timeout); 

      if (found_ik) {
        std::vector<double> target_joint_group_positions; 
        current_state->copyJointGroupPositions(joint_model_group, target_joint_group_positions);
        bool target_joints_infeasible = false; 

        auto joint_1 = target_joint_group_positions.at(1);  
        auto joint_2 = target_joint_group_positions.at(2);  
        auto joint_3 = target_joint_group_positions.at(3);  
        auto joint_4 = target_joint_group_positions.at(4);  
        // if (joint_1 > 0) {
        //     joint_1 = joint_1 - M_PI * 2; 
        // }
        //
        if (joint_2 > 0) {
            joint_2 = joint_2 - M_PI * 2; 
        }

        if (joint_3 < 0) {
            joint_3 = joint_3 + M_PI * 2; 
        }

        if (joint_4 > 0) {
            joint_4 = joint_4 - M_PI * 2; 
        }

        // if (joint_1 > -1.94 or joint_1 < -4.34) {
        //   ROS_INFO("Target joint 1 infisible: %f", target_joint_group_positions.at(2)); 
        //   target_joints_infeasible = true; 
        // }
        //
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

        if (target_joints_infeasible) {

          ros::Duration(3.0).sleep(); 
          m_gantry_group.setMaxVelocityScalingFactor(0.5);
          m_joint_group_positions = current_joint_group_positions; 
          m_gantry_group.setJointValueTarget(m_joint_group_positions);
          this->move_arm_group(); 
          m_gantry_group.setMaxVelocityScalingFactor(1.0);
          continue; 
        }

        m_joint_group_positions.at(0) = target_joint_group_positions.at(0); 
        m_joint_group_positions.at(1) = target_joint_group_positions.at(1); 
        m_gantry_group.setJointValueTarget(m_joint_group_positions);
        this->move_arm_group(); 

        m_joint_group_positions = target_joint_group_positions; 
        m_gantry_group.setJointValueTarget(m_joint_group_positions);
        this->move_arm_group(); 
        m_gantry_group.clearPathConstraints();
        return true; 
      }
  }

  this->copyCurrentJointsPosition(); 
  m_gantry_group.clearPathConstraints();
  return false; 

}

bool GantryArm::pickPart(std::string part_type, 
                          const geometry_msgs::Pose& part_init_pose,
                          std::string camera_id) 
{
    ROS_INFO("---Start pick part"); 
    m_gantry_group.setMaxVelocityScalingFactor(1.0);

    geometry_msgs::Pose arm_ee_link_pose = m_gantry_group.getCurrentPose().pose;
    auto flat_orientation = Utility::motioncontrol::quaternionFromEuler(0, 1.57, 0);
    arm_ee_link_pose.orientation.x = flat_orientation.getX();
    arm_ee_link_pose.orientation.y = flat_orientation.getY();
    arm_ee_link_pose.orientation.z = flat_orientation.getZ();
    arm_ee_link_pose.orientation.w = flat_orientation.getW();

    arm_ee_link_pose.position.x = part_init_pose.position.x; 
    arm_ee_link_pose.position.y = part_init_pose.position.y; 

    // six.gantry_torso.at(0) = part_init_pose.position.x + 1.336 ; 
    // ROS_INFO_STREAM(six.gantry_torso.at(0));
    // six.gantry_torso.at(1) = part_init_pose.position.y + 4.45; 
    // ROS_INFO_STREAM(six.gantry_torso.at(1));
    // six.gantry_torso.at(2) = 1.15; 
    // m_torso_gantry_group.setJointValueTarget(six.gantry_torso); 
    // m_torso_gantry_group.move(); 
    



    // preset z depending on the part type
    double z_pos{};
    if (part_type.find("pump") != std::string::npos) {
        z_pos = 0.083;
    }
    if (part_type.find("battery") != std::string::npos) {
        z_pos = 0.052;
    }

    arm_ee_link_pose.position.z = part_init_pose.position.z + z_pos;

    while (!m_gripper_state.enabled) {
        activateGripper();
    }
    m_gantry_group.setPoseTarget(arm_ee_link_pose); 
    m_gantry_group.move();

    auto grasp_pose = arm_ee_link_pose; 
    double step = 0.001;
    while (!m_gripper_state.attached) {
        grasp_pose.position.z -= step;
        m_gantry_group.setPoseTarget(grasp_pose);
        m_gantry_group.move();
        geometry_msgs::Pose arm_ee_link_pose = m_gantry_group.getCurrentPose().pose;
        if (arm_ee_link_pose.position.z < 0.76) {
          ROS_INFO("---End pick part: Arm moving lower then part, abort"); 
          return true; 
        }
        ros::Duration(0.3).sleep();
    }
    ROS_INFO("grasp success"); 
    
}

geometry_msgs::Pose GantryArm::placePart(std::string part_type, 
                                          geometry_msgs::Pose part_init_pose, 
                                          geometry_msgs::Pose part_pose_in_frame, 
                                          std::string station)
{

    // get the target pose of the part in the world frame
    auto target_pose_in_world = Utility::motioncontrol::transformBriefcaseToWorldFrame(
         part_pose_in_frame,
         station);

    ROS_INFO("---Start place part"); 
    Utility::print_pose(target_pose_in_world); 

    geometry_msgs::Pose arm_ee_link_pose = m_gantry_group.getCurrentPose().pose;
    auto flat_orientation = Utility::motioncontrol::quaternionFromEuler(0, 1.57, 0);
    arm_ee_link_pose = m_gantry_group.getCurrentPose().pose;
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
    m_gantry_group.setMaxVelocityScalingFactor(0.8);
  
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
    double x_margin{};
    double y_margin{};
    double z_margin{};
    if (part_type.find("battery") != std::string::npos) {
        y_margin = -0.25;
        z_margin = 0.1;
        x_margin = 0.8;
    }
    else{
      z_margin = 0.1;
      x_margin = 0.7;
    }
    // else if (part_type.find("regulator") != std::string::npos) {
      // z_margin = 0.11; 
    // }
    //
    // orientation of the gripper when placing the part in the tray
    target_pose_in_world.orientation.x = q_rslt.x();
    target_pose_in_world.orientation.y = q_rslt.y();
    target_pose_in_world.orientation.z = q_rslt.z();
    target_pose_in_world.orientation.w = q_rslt.w();
    target_pose_in_world.position.x += x_margin;
    target_pose_in_world.position.y += y_margin;
    target_pose_in_world.position.z += z_margin;

    m_gantry_group.setPoseTarget(target_pose_in_world); 
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    ROS_INFO("Start place planning"); 
    bool success = (m_gantry_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success){
      m_gantry_group.move();
    }
    else{
      ROS_INFO("Path planning Failed, moving manually  ");
      if(part_type.find("battery") != std::string::npos){
        m_gantry_group.setJointValueTarget(one.gantry_arm);
      }
      else{
        m_gantry_group.setJointValueTarget(two.gantry_arm);
      }
      
      m_gantry_group.move();
    }

    if (part_type.find("battery") != std::string::npos) {
        ROS_INFO("move for battery"); 
        six.gantry_torso.at(0) = m_current_joint_states.position.at(7) - 0.8; 
        six.gantry_torso.at(1) = m_current_joint_states.position.at(10) - 0.25; 
        six.gantry_torso.at(2) = m_current_joint_states.position.at(8); 
        m_torso_gantry_group.setJointValueTarget(six.gantry_torso); 
        m_torso_gantry_group.move(); 
    }
    else{
      six.gantry_torso.at(0) = m_current_joint_states.position.at(7) - 0.7; 
      six.gantry_torso.at(1) = m_current_joint_states.position.at(10) ; 
      six.gantry_torso.at(2) = m_current_joint_states.position.at(8); 
      m_torso_gantry_group.setJointValueTarget(six.gantry_torso); 
      m_torso_gantry_group.move(); 

    }

    ros::Duration(1.0).sleep();
    deactivateGripper();

    if (part_type.find("battery") != std::string::npos) {
        ROS_INFO("move for battery"); 
        six.gantry_torso.at(0) = m_current_joint_states.position.at(7) + 0.8; 
        six.gantry_torso.at(1) = m_current_joint_states.position.at(10) + 0.25; 
        six.gantry_torso.at(2) = m_current_joint_states.position.at(8); 
        m_torso_gantry_group.setJointValueTarget(six.gantry_torso); 
        m_torso_gantry_group.move(); 
        m_gantry_group.setJointValueTarget(six.gantry_arm); 
        m_gantry_group.move(); 
    }
    else{
      six.gantry_torso.at(0) = m_current_joint_states.position.at(7) + 0.7; 
      six.gantry_torso.at(1) = m_current_joint_states.position.at(10) ; 
      six.gantry_torso.at(2) = m_current_joint_states.position.at(8); 
      m_torso_gantry_group.setJointValueTarget(six.gantry_torso); 
      m_torso_gantry_group.move(); 
      m_gantry_group.setJointValueTarget(six.gantry_arm); 
      m_gantry_group.move();
    }

    m_gantry_group.setMaxVelocityScalingFactor(1.0);
    ROS_INFO("---End place part"); 

    return target_pose_in_world;
}

bool GantryArm::discard_faulty(const nist_gear::Model& faulty_part, std::string target_agv)
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
      this->copyCurrentJointsPosition(); 

      std::random_device rd; 
      std::mt19937 gen(rd());

      int trial_count = 1; 
      ROS_INFO("  trial %d", trial_count); 
      while (not pickPart(faulty_part.type, faulty_part_pose, camera_id)) {
        
        ROS_INFO("Discard picking fails, try new joint config for discarding"); 
        ros::Duration(3.0).sleep(); 
        this->resetArm(); 
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
          m_gantry_group.setJointValueTarget(m_joint_group_positions);
          this->move_arm_group(); 
        }

        if (trial_count > 1) {
          std::normal_distribution<double> gauss_dist(-0.3, 0.3); 
          auto random_dist = gauss_dist(gen); 
          moveBaseTo(m_joint_group_positions.at(0) + random_dist);
        }

        this->copyCurrentJointsPosition(); 
        
        if (trial_count > 5) return false; 
      }
      ROS_INFO("Discard picking success"); 

      goToPresetLocation("home_face_bins");
      ros::Duration(0.5).sleep();
      deactivateGripper();
      ROS_INFO("---End discard part"); 
      return true; 
}

bool GantryArm::check_faulty(const nist_gear::Model& faulty_part)
{
  ariac_group1::IsFaulty srv; 
  
  srv.request.part = faulty_part; 
  Utility::print_part_pose(faulty_part); 
  m_is_faulty_client.call(srv); 

  return srv.response.faulty; 
}

bool GantryArm::movePart(const ariac_group1::PartInfo& part_init_info, const ariac_group1::PartTask& part_task) {
    auto part_init_pose_in_world = part_init_info.part.pose;
    auto camera_id = part_init_info.camera_id; 
    auto part_type = part_init_info.part.type; 
    auto target_pose_in_frame = part_task.part.pose; 
    auto target_agv = part_task.agv_id; 

    if (this->check_emergency_interrupt()) {
        return false; 
    }

    // use -0.3 to reduce awkward pick trajectory
    // goToPresetLocation(camera_id);
    moveBaseTo(part_init_pose_in_world.position.y - 0.3);

    if (this->check_emergency_interrupt()) {
        return false; 
    }

    if (pickPart(part_type, part_init_pose_in_world, camera_id)) {
        auto target_pose_in_world = placePart(part_type, part_init_pose_in_world, target_pose_in_frame, target_agv);
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
        ROS_INFO("Part not faulty"); 
        return true; 
    }
    else {
      return false; 
    }
}

bool GantryArm::check_emergency_interrupt()
{
  ROS_INFO("Checking for emergency interrupt"); 
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
      deactivateGripper();
      ros::Duration(0.1).sleep(); 
      this->resetArm(); 
      if (this->get_belt_part(range)) {
          this->place_to_vacancy(vacancy_poses.at(0)); 
      }
      this->resetArm();
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

void GantryArm::move_to_belt_intercept_pose(const geometry_msgs::Pose& belt_part)
{
    this->goToPresetLocation("belt_intercept"); 
    geometry_msgs::Pose arm_ee_link_pose = m_gantry_group.getCurrentPose().pose;
    auto flat_orientation = Utility::motioncontrol::quaternionFromEuler(2, 0, 1.57);
    arm_ee_link_pose.orientation.x = flat_orientation.getX();
    arm_ee_link_pose.orientation.y = flat_orientation.getY();
    arm_ee_link_pose.orientation.z = flat_orientation.getZ();
    arm_ee_link_pose.orientation.w = flat_orientation.getW();

    arm_ee_link_pose.position.x = belt_part.position.x; 
    arm_ee_link_pose.position.z = belt_part.position.z;

    m_gantry_group.setPoseTarget(arm_ee_link_pose); 
    m_gantry_group.move(); 
}

bool GantryArm::get_belt_part(double range)
{
    // ariac_group1::GetBeltPart srv; 
    // do {
    //     m_get_belt_part_client.call(srv); 
    //     ros::Duration(0.2).sleep(); 
    //
    // } while(srv.response.part.type.empty()); 
    //
    ROS_INFO("Get belt part"); 
    // Utility::print_part_pose(srv.response.part); 
    //

    // geometry_msgs::Pose arm_ee_link_pose = m_gantry_group.getCurrentPose().pose;
    // auto flat_orientation = Utility::motioncontrol::quaternionFromEuler(0, 1.57, 0);
    // arm_ee_link_pose.orientation.x = flat_orientation.getX();
    // arm_ee_link_pose.orientation.y = flat_orientation.getY();
    // arm_ee_link_pose.orientation.z = flat_orientation.getZ();
    // arm_ee_link_pose.orientation.w = flat_orientation.getW();
    //
    // // preset z depending on the part type
    // auto part_type = srv.response.part.type; 
    // double z_pos{};
    // if (part_type.find("pump") != std::string::npos) {
    //     z_pos = 0.07;
    // }
    // if (part_type.find("sensor") != std::string::npos) {
    //     z_pos = 0.05;
    // }
    // if (part_type.find("battery") != std::string::npos) {
    //     z_pos = 0.052;
    // }
    // if (part_type.find("regulator") != std::string::npos) {
    //     z_pos = 0.057;
    // }

    // pregrasp pose: right above the part
    // auto pregrasp_pose = srv.response.part.pose;
    // pregrasp_pose.orientation = arm_ee_link_pose.orientation;
    // pregrasp_pose.position.y = srv.response.part.pose.position.y - 0.5;
    // pregrasp_pose.position.z = srv.response.part.pose.position.z + z_pos;
    //
    // activate gripper
    // sometimes it does not activate right away
    // so we are doing this in a loop
    while (!m_gripper_state.enabled) {
        activateGripper();
    }

    geometry_msgs::Pose belt_part; 
    belt_part.position.x = -0.65 + range; 
    // belt_part.position.z = 0.93; 
    belt_part.position.z = 0.945; 

    this->move_to_belt_intercept_pose(belt_part); 

    m_gantry_group.setMaxVelocityScalingFactor(0.6);
    int count = 0; 
    while (!m_gripper_state.attached) {
        ROS_INFO("Not attached"); 
        moveBaseTo(m_current_joint_states.position.at(1) + 0.3); 
        ros::Duration(0.2).sleep(); 
        count++; 
        if (m_current_joint_states.position.at(1) > 3) {
            ROS_INFO("No belt part"); 
            deactivateGripper();
            return false; 
        }
    }

    ROS_INFO("Attached!!!"); 

    m_gantry_group.setMaxVelocityScalingFactor(1.0);
    this->resetArm(); 
    ros::Duration(0.1).sleep(); 
    return true; 

}

void GantryArm::place_to_vacancy(const geometry_msgs::Pose& vacancy_pose, bool from_belt)
{
    ROS_INFO("Place to vacancy"); 
    geometry_msgs::Pose arm_ee_link_pose = m_gantry_group.getCurrentPose().pose;

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

    m_gantry_group.setPoseTarget(arm_ee_link_pose); 
    m_gantry_group.move();
    ros::Duration(0.1).sleep(); 
    deactivateGripper();
    ros::Duration(0.1).sleep(); 
    this->lift(); 
}

bool GantryArm::get_order()
{
  ros::Duration(1.0).sleep();
  ros::Rate wait_rate(20); 
  // this->check_emergency_interrupt();  
  // check if there are tasks and make sure ROS is running 
  while (ros::ok()) {
    ROS_INFO_THROTTLE(3, "Waiting for gantry part task.");
    // this->check_emergency_interrupt();  

    m_shipments.update_part_task_queue(m_part_task_queue); 

    if (not m_part_task_queue.empty()) {
        break; 
    }
    
    wait_rate.sleep(); 
  }

  ROS_INFO("Received part task"); 
  return true; 
}

void GantryArm::plan()
{
  const std::lock_guard<std::mutex> lock(*m_mutex_ptr); 
  std::sort(m_part_task_queue.begin(), m_part_task_queue.end()); 
}

void GantryArm::goToHome(std::string station)
{
  if((station == "as1" || station == "as3") && gantry_cloc != "five")
  {
    this->goToPresetLocation("five");

  }
  if((station == "as2" || station == "as4" ) && gantry_cloc != "eight")
  {
    this->goToPresetLocation("eight");
  }
}

std::string GantryArm::getAgvAtStation(std::string station)
{
  std::string st = "NULL";
  if(m_agvs_dict["agv1"]->get_station() == station)
  {
    st = "agv1";
  }
  if(m_agvs_dict["agv2"]->get_station() == station)
  {
    st = "agv2";
  }
  if(m_agvs_dict["agv3"]->get_station() == station)
  {
    st = "agv3";
  }
  if(m_agvs_dict["agv4"]->get_station() == station)
  {
    st = "agv4";
  }
  
  return st;
  
}

void GantryArm::execute()
{
  const std::lock_guard<std::mutex> lock(*m_mutex_ptr); 

  ROS_INFO("==============================================="); 
  ROS_INFO("Task queue size: %d", (int)m_part_task_queue.size()); 
  auto& part_task_info = m_part_task_queue.back(); 
  auto& priority = std::get<0>(part_task_info); 
  auto& part_task = *std::get<1>(part_task_info); 


  part_task.agv_id = getAgvAtStation(part_task.station_id.c_str());
  ROS_INFO("Shipment: %s", part_task.shipment_type.c_str()); 
  ROS_INFO("Priority: %d", priority); 
  ROS_INFO("AGV: %s", part_task.agv_id.c_str()); 
  ROS_INFO("Station id: %s", part_task.station_id.c_str()); 
  ROS_INFO("Part type: %s", part_task.part.type.c_str()); 
  // // ros::Duration(30.0).sleep();
  // //
  std::string camera_id = part_task.station_id; 
  camera_id += "_"; 
  camera_id += part_task.agv_id.back(); 

  ROS_INFO("camera_id: %s", camera_id.c_str()); 

  ariac_group1::PartsUnderCamera parts_under_camera_srv; 
  parts_under_camera_srv.request.camera_id = camera_id; 
  ros::Duration(1.0).sleep(); 
  m_parts_under_camera_client.call(parts_under_camera_srv); 

  auto& parts = parts_under_camera_srv.response.parts; 
  ROS_INFO("Parts: %d", (int)parts.size()); 
  if (parts.size() == 0) {
    this->submit_shipment(part_task.shipment_type, part_task.station_id); 
    ros::Duration(0.5).sleep();
    goToHome(part_task.station_id);
    isNewShipment =true;
    m_part_task_queue.pop_back();
    return; 
  }

  if (m_agvs_dict[part_task.agv_id]->get_station() != part_task.station_id) {
      return; 
  }

  nist_gear::Model target_part; 
  bool found_part = false; 
  for (auto& part: parts) {
      if (part_task.part.type == part.type) {
        target_part = part;  
        found_part = true; 
        break; 
      }
  }

  if (not found_part) {
      m_shipments.shipments_record[part_task.shipment_type]->unfinished_part_tasks--; 
      if (m_shipments.shipments_record[part_task.shipment_type]->unfinished_part_tasks == 0) {
        this->submit_shipment(part_task.shipment_type, part_task.station_id); 
        ros::Duration(0.5).sleep();
        goToHome(part_task.station_id);
        isNewShipment =true;
      }
      m_part_task_queue.pop_back();
      return; 
  }

  Utility::print_part_pose(target_part); 

  if(isNewShipment == true)
  {
    goToHome(part_task.station_id.c_str());
    isNewShipment = false;
  }
  this->goToPresetLocation(camera_id); 

  ros::Duration(0.5).sleep(); 

  this->pickPart(target_part.type, target_part.pose, camera_id); 

  ros::Duration(0.5).sleep(); 

  this->goToPresetLocation(camera_id); 

  ros::Duration(0.5).sleep(); 

  this->goToPresetLocation("at_" + part_task.station_id); 

  this->placePart(target_part.type, target_part.pose, part_task.part.pose, part_task.station_id); 
  ros::Duration(0.5).sleep(); 

  this->goToPresetLocation("at_" + part_task.station_id); 
  ros::Duration(0.5).sleep(); 
  m_shipments.shipments_record[part_task.shipment_type]->unfinished_part_tasks--; 

  if (m_shipments.shipments_record[part_task.shipment_type]->unfinished_part_tasks == 0) {
    this->submit_shipment(part_task.shipment_type, part_task.station_id); 
    ros::Duration(0.5).sleep();
    goToHome(part_task.station_id);
    isNewShipment =true;
  }

  m_part_task_queue.pop_back();
}

bool GantryArm::check_insufficient_shipment(int priority)
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

void GantryArm::clear_part_task(ariac_group1::PartTask& part_task, int& priority)
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

ShipmentState GantryArm::check_shipment_state(ariac_group1::PartTask& part_task, nist_gear::Model& wrong_part)
{
  static bool missing_check = false;  

  // Check if shipment is ready to submit
  if (m_shipments.shipments_record[part_task.shipment_type]->unfinished_part_tasks == 0) {
    ariac_group1::CheckQualitySensor srv; 
    srv.request.agv_id = part_task.agv_id; 
    if (m_check_quality_sensor_client.call(srv)) {

      std::string result = m_shipments.check_shipment_parts(part_task, wrong_part); 
      if (result == "wrong_type") {
        // put the wrong type part back to vacancy spot
        // possibily resolve insufficient shipment, therefore check missing
        missing_check = true; 
        ROS_INFO("Has wrong type in shipment %s", part_task.shipment_type.c_str()); 
        return ShipmentState::HAS_WRONG_TYPE; 
      }
      else if (result == "wrong_pose"){
        ROS_INFO("Has wrong pose in shipment %s", part_task.shipment_type.c_str()); 
        return ShipmentState::HAS_WRONG_POSE; 
      }
      else if (result == "missing_part" and missing_check) {
        // only check missing part if wrong type happens before
        missing_check = false; 
        ROS_INFO("Has missing part in shipment %s", part_task.shipment_type.c_str()); 
        return ShipmentState::HAS_MISSING_PART; 
      }
      // else if (result == "redundant_part") {
      //   return ShipmentState::HAS_REDUNDANT_PART; 
      // }
      
      if (srv.response.faulty_parts.empty()) {
        ROS_INFO("No faulty part in shipment %s", part_task.shipment_type.c_str()); 
        return ShipmentState::READY; 
      }
      else { 
        ROS_INFO("Has faulty in shipment %s", part_task.shipment_type.c_str()); 
        //push faulty task back to queue
        nist_gear::Model faulty_part = srv.response.faulty_parts[0]; 
        part_task.part.type = faulty_part.type;  
        part_task.part.pose = faulty_part.pose;  
        return ShipmentState::HAS_FAULTY; 
      }
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

void GantryArm::process_shipment_state(ShipmentState shipment_state, ariac_group1::PartTask& part_task, int& priority, nist_gear::Model& wrong_part)
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

    case ShipmentState::HAS_WRONG_TYPE: 
      {
        ariac_group1::GetVacancyPose vacancy_pose_srv; 
        m_get_vacancy_pose_client.call(vacancy_pose_srv); 
        auto& vacancy_poses = vacancy_pose_srv.response.vacancy_poses;
        
        if (vacancy_poses.empty()) {
          bool discard_success = this->discard_faulty(wrong_part, part_task.agv_id); 
          if (discard_success) {
            m_shipments.shipments_record[part_task.shipment_type]->unfinished_part_tasks++; 
          }
          return; 
        }
        else {
          std::string camera_id = "logical_camera_ks"; 
          camera_id += part_task.agv_id.back(); 
          if (pickPart(wrong_part.type, wrong_part.pose, camera_id)) {
            place_to_vacancy(vacancy_poses[0], false); 
            m_shipments.shipments_record[part_task.shipment_type]->unfinished_part_tasks++; 
          }
        }
        break; 
      }

    case ShipmentState::HAS_WRONG_POSE: 
      {
        std::string camera_id = "logical_camera_ks"; 
        camera_id += part_task.agv_id.back(); 
        if (pickPart(wrong_part.type, wrong_part.pose, camera_id)) {
          auto target_pose_in_world = placePart(wrong_part.type, wrong_part.pose, part_task.part.pose, part_task.agv_id);
        }
        break; 
      }

    case ShipmentState::HAS_MISSING_PART: 
      {
        m_shipments.shipments_record[part_task.shipment_type]->unfinished_part_tasks++; 
        break; 
      }

    case ShipmentState::HAS_FAULTY: 
      {
        goToPresetLocation(part_task.agv_id);
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

void GantryArm::submit_shipment(const std::string& shipment_type, const std::string& station_id)
{  
  AssemSubmit srv; 
  srv.request.shipment_type = shipment_type; 

  if (station_id == "as1") {
    if (m_submit_shipment_as1_client.call(srv)) {
      ROS_INFO("Submitting shipment %s", shipment_type.c_str()); 
      ROS_INFO_STREAM("inspection result: " << srv.response.inspection_result); 
    }
    else{
      ROS_ERROR("Failed to submit shipment %s", shipment_type.c_str()); 
    }
  }
  else if (station_id == "as2") {
    if (m_submit_shipment_as2_client.call(srv)) {
      ROS_INFO("Submitting shipment %s", shipment_type.c_str()); 
      ROS_INFO_STREAM("inspection result: " << srv.response.inspection_result); 
    }
    else{
      ROS_ERROR("Failed to submit shipment %s", shipment_type.c_str()); 
    }
  }
  else if (station_id == "as3") {
    if (m_submit_shipment_as3_client.call(srv)) {
      ROS_INFO("Submitting shipment %s", shipment_type.c_str()); 
      ROS_INFO_STREAM("inspection result: " << srv.response.inspection_result); 
    }
    else{
      ROS_ERROR("Failed to submit shipment %s", shipment_type.c_str()); 
    }
  }
  else if (station_id == "as4") {
    if (m_submit_shipment_as4_client.call(srv)) {
      ROS_INFO("Submitting shipment %s", shipment_type.c_str()); 
      ROS_INFO_STREAM("inspection result: " << srv.response.inspection_result); 
    }
    else{
      ROS_ERROR("Failed to submit shipment %s", shipment_type.c_str()); 
    }
  }

}

void GantryArm::waypoints_planning(){
  std::vector<std::vector<double>> waypoints;
  this->goToPresetLocation("one");
  waypoints.push_back(two.gantry_torso);
  waypoints.push_back(eight.gantry_torso);
}
