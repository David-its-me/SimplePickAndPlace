#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "ur_msgs/srv/set_io.hpp"

using namespace std::chrono_literals;


static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

void open_gripper(std::shared_ptr<rclcpp::Node> ur_io_publisher_node, rclcpp::Client<ur_msgs::srv::SetIO>::SharedPtr ur_msgs_client){
  auto closing_revert_request = std::make_shared<ur_msgs::srv::SetIO::Request>();
  closing_revert_request->fun = 1;
  closing_revert_request->pin = 0;
  closing_revert_request->state = 0;

  auto result = ur_msgs_client->async_send_request(closing_revert_request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(ur_io_publisher_node, result) == rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(LOGGER, "Successfully reset io port");
  } else {
    RCLCPP_ERROR(LOGGER, "Failed to reset io port");
  }

  auto opening_request = std::make_shared<ur_msgs::srv::SetIO::Request>();
  opening_request->fun = 1;
  opening_request->pin = 1;
  opening_request->state = 1;

  result = ur_msgs_client->async_send_request(opening_request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(ur_io_publisher_node, result) == rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(LOGGER, "Success opening gripper");
  } else {
    RCLCPP_ERROR(LOGGER, "Failed to open gripper");
  }
}

void close_gripper(std::shared_ptr<rclcpp::Node> ur_io_publisher_node, rclcpp::Client<ur_msgs::srv::SetIO>::SharedPtr ur_msgs_client){
  auto opening_revert_request = std::make_shared<ur_msgs::srv::SetIO::Request>();
  opening_revert_request->fun = 1;
  opening_revert_request->pin = 1;
  opening_revert_request->state = 0;

  auto result = ur_msgs_client->async_send_request(opening_revert_request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(ur_io_publisher_node, result) == rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(LOGGER, "Successfully reset io port");
  } else {
    RCLCPP_ERROR(LOGGER, "Failed to reset io port");
  }
  
  auto closing_request = std::make_shared<ur_msgs::srv::SetIO::Request>();
  closing_request->fun = 1;
  closing_request->pin = 0;
  closing_request->state = 1;

  result = ur_msgs_client->async_send_request(closing_request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(ur_io_publisher_node, result) == rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(LOGGER, "Success closing gripper");
  } else {
    RCLCPP_ERROR(LOGGER, "Failed to close gripper");
  }

}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("simple_pick_and_place_applictation", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  static const std::string PLANNING_GROUP_ARM = "arm";
  //static const std::string PLANNING_GROUP_GRIPPER = "hand";

  RCLCPP_INFO(LOGGER, "Get arm interface");
  moveit::planning_interface::MoveGroupInterface move_group_arm(
      move_group_node, PLANNING_GROUP_ARM);
  /*RCLCPP_INFO(LOGGER, "Get hand interface");
  moveit::planning_interface::MoveGroupInterface move_group_gripper(
      move_group_node, PLANNING_GROUP_GRIPPER);*/
  

  const moveit::core::JointModelGroup *joint_model_group_arm =
      move_group_arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);
  /*const moveit::core::JointModelGroup *joint_model_group_gripper =
      move_group_gripper.getCurrentState()->getJointModelGroup(
          PLANNING_GROUP_GRIPPER);*/

  // Get Current State
  RCLCPP_INFO(LOGGER, "Get current arm state");
  moveit::core::RobotStatePtr current_state_arm =
      move_group_arm.getCurrentState(10);
  /*RCLCPP_INFO(LOGGER, "Get current hand state");
  moveit::core::RobotStatePtr current_state_gripper =
      move_group_gripper.getCurrentState(10);*/

  
  std::vector<double> arm_position;

  RCLCPP_INFO(LOGGER, "Get arm joint group position");
  current_state_arm->copyJointGroupPositions(joint_model_group_arm,
                                             arm_position);
  /*RCLCPP_INFO(LOGGER, "Get hand joint group position");
  std::vector<double> joint_group_positions_gripper;
  current_state_gripper->copyJointGroupPositions(joint_model_group_gripper,
                                               joint_group_positions_gripper);*/

  RCLCPP_INFO(LOGGER, "Set start state to current state");
  move_group_arm.setStartStateToCurrentState();
  //move_group_gripper.setStartStateToCurrentState();

  std::shared_ptr<rclcpp::Node> ur_io_publisher_node = rclcpp::Node::make_shared("ur_io_msgs_publisher_node");

  rclcpp::Client<ur_msgs::srv::SetIO>::SharedPtr ur_msgs_client = ur_io_publisher_node->create_client<ur_msgs::srv::SetIO>("io_and_status_controller/set_io");

  open_gripper(ur_io_publisher_node, ur_msgs_client);

  // Start Position
  std::vector<double> start_position_arm(6);
  start_position_arm[0] = -1.39626;  // Shoulder Pan
  start_position_arm[1] = -2.00713; // Shoulder Lift
  start_position_arm[2] = -1.67552;  // Elbow
  start_position_arm[3] = -1.01229; // Wrist 1
  start_position_arm[4] = 1.5708; // Wrist 2
  start_position_arm[5] = 0.174533;  // Wrist 3

  // Pregrasp green Position
  std::vector<double> pregrasp_green_position_arm(6);
  pregrasp_green_position_arm[0] = -1.53589;  // Shoulder Pan
  pregrasp_green_position_arm[1] = -2.0944; // Shoulder Lift
  pregrasp_green_position_arm[2] = -1.6844173;  // Elbow
  pregrasp_green_position_arm[3] = -0.91769412; // Wrist 1
  pregrasp_green_position_arm[4] = 1.5708; // Wrist 2
  pregrasp_green_position_arm[5] = 0.0523599;  // Wrist 3

  // Pick green Position
  std::vector<double> pick_green_position_arm(6);
  pick_green_position_arm[0] = -1.53589;  // Shoulder Pan
  pick_green_position_arm[1] = -2.12546196; // Shoulder Lift
  pick_green_position_arm[2] = -1.6940166;  // Elbow
  pick_green_position_arm[3] = -0.88889619; // Wrist 1
  pick_green_position_arm[4] = 1.5708; // Wrist 2
  pick_green_position_arm[5] = 0.0523599;  // Wrist 3

  // Pick blue Position
  std::vector<double> pick_blue_position_arm(6);
  pick_blue_position_arm[0] = -1.2665854;  // Shoulder Pan
  pick_blue_position_arm[1] = -2.10783414; // Shoulder Lift
  pick_blue_position_arm[2] = -1.729621;  // Elbow
  pick_blue_position_arm[3] = -0.85800386; // Wrist 1
  pick_blue_position_arm[4] = 1.5708; // Wrist 2
  pick_blue_position_arm[5] = 0.32148965;  // Wrist 3

  // Pregrasp blue Position
  std::vector<double> pregrasp_blue_position_arm(6);
  pregrasp_blue_position_arm[0] = -1.2665854;  // Shoulder Pan
  pregrasp_blue_position_arm[1] = -2.0804325; // Shoulder Lift
  pregrasp_blue_position_arm[2] = -1.7158332;  // Elbow
  pregrasp_blue_position_arm[3] = -0.907571; // Wrist 1
  pregrasp_blue_position_arm[4] = 1.5708; // Wrist 2
  pregrasp_blue_position_arm[5] = 0.32148965;  // Wrist 3

  // Pick red Position
  std::vector<double> pick_red_position_arm(6);
  pick_red_position_arm[0] = -1.3725269;  // Shoulder Pan
  pick_red_position_arm[1] = -1.93330121; // Shoulder Lift
  pick_red_position_arm[2] = -2.02213847;  // Elbow
  pick_red_position_arm[3] = -0.73932147; // Wrist 1
  pick_red_position_arm[4] = 1.5708; // Wrist 2
  pick_red_position_arm[5] = 0.1982694;  // Wrist 3

  // Pregrasp red Position
  std::vector<double> pregrasp_red_position_arm(6);
  pregrasp_red_position_arm[0] = -1.3725269;  // Shoulder Pan
  pregrasp_red_position_arm[1] = -1.89246051; // Shoulder Lift
  pregrasp_red_position_arm[2] = -2.00520878;  // Elbow
  pregrasp_red_position_arm[3] = -0.7972664; // Wrist 1
  pregrasp_red_position_arm[4] = 1.5708; // Wrist 2
  pregrasp_red_position_arm[5] = 0.1982694;  // Wrist 3

  // Pick gray Position
  std::vector<double> pick_gray_position_arm(6);
  pick_gray_position_arm[0] = -1.4229669;  // Shoulder Pan
  pick_gray_position_arm[1] = -2.30016942; // Shoulder Lift
  pick_gray_position_arm[2] = -1.381079;  // Elbow
  pick_gray_position_arm[3] = -1.0136872; // Wrist 1
  pick_gray_position_arm[4] = 1.5708; // Wrist 2
  pick_gray_position_arm[5] = 0.1476549;  // Wrist 3

  // Pregrasp gray Position
  std::vector<double> pregrasp_gray_position_arm(6);
  pregrasp_gray_position_arm[0] = -1.422443;  // Shoulder Pan
  pregrasp_gray_position_arm[1] = -2.28585772; // Shoulder Lift
  pregrasp_gray_position_arm[2] = -1.3723524;  // Elbow
  pregrasp_gray_position_arm[3] = -1.036726; // Wrist 1
  pregrasp_gray_position_arm[4] = 1.5708; // Wrist 2
  pregrasp_gray_position_arm[5] = 0.1476549;  // Wrist 3
  

  moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;

  RCLCPP_INFO(LOGGER, "Start Position");
  move_group_arm.setJointValueTarget(start_position_arm);
  bool success_arm = (move_group_arm.plan(my_plan_arm) == moveit::core::MoveItErrorCode::SUCCESS);
  move_group_arm.execute(my_plan_arm);

  for (int i = 0; i < 10; i++){
    RCLCPP_INFO(LOGGER, "Pregrasp green Position");
    move_group_arm.setJointValueTarget(pregrasp_green_position_arm);
    success_arm = (move_group_arm.plan(my_plan_arm) == moveit::core::MoveItErrorCode::SUCCESS);
    move_group_arm.execute(my_plan_arm);

    RCLCPP_INFO(LOGGER, "Pick green Position");
    move_group_arm.setJointValueTarget(pick_green_position_arm);
    success_arm = (move_group_arm.plan(my_plan_arm) == moveit::core::MoveItErrorCode::SUCCESS);
    move_group_arm.execute(my_plan_arm);
    close_gripper(ur_io_publisher_node, ur_msgs_client);

    RCLCPP_INFO(LOGGER, "Pregrasp green Position");
    move_group_arm.setJointValueTarget(pregrasp_green_position_arm);
    success_arm = (move_group_arm.plan(my_plan_arm) == moveit::core::MoveItErrorCode::SUCCESS);
    move_group_arm.execute(my_plan_arm);



    RCLCPP_INFO(LOGGER, "Pregrasp red Position");
    move_group_arm.setJointValueTarget(pregrasp_red_position_arm);
    success_arm = (move_group_arm.plan(my_plan_arm) == moveit::core::MoveItErrorCode::SUCCESS);
    move_group_arm.execute(my_plan_arm);

    RCLCPP_INFO(LOGGER, "Pick red Position");
    move_group_arm.setJointValueTarget(pick_red_position_arm);
    success_arm = (move_group_arm.plan(my_plan_arm) == moveit::core::MoveItErrorCode::SUCCESS);
    move_group_arm.execute(my_plan_arm);
    open_gripper(ur_io_publisher_node, ur_msgs_client);

    RCLCPP_INFO(LOGGER, "Pregrasp red Position");
    move_group_arm.setJointValueTarget(pregrasp_red_position_arm);
    success_arm = (move_group_arm.plan(my_plan_arm) == moveit::core::MoveItErrorCode::SUCCESS);
    move_group_arm.execute(my_plan_arm);



    RCLCPP_INFO(LOGGER, "Pregrasp gray Position");
    move_group_arm.setJointValueTarget(pregrasp_gray_position_arm);
    success_arm = (move_group_arm.plan(my_plan_arm) == moveit::core::MoveItErrorCode::SUCCESS);
    move_group_arm.execute(my_plan_arm);

    RCLCPP_INFO(LOGGER, "Pick gray Position");
    move_group_arm.setJointValueTarget(pick_gray_position_arm);
    success_arm = (move_group_arm.plan(my_plan_arm) == moveit::core::MoveItErrorCode::SUCCESS);
    move_group_arm.execute(my_plan_arm);
    close_gripper(ur_io_publisher_node, ur_msgs_client);

    RCLCPP_INFO(LOGGER, "Pregrasp gray Position");
    move_group_arm.setJointValueTarget(pregrasp_gray_position_arm);
    success_arm = (move_group_arm.plan(my_plan_arm) == moveit::core::MoveItErrorCode::SUCCESS);
    move_group_arm.execute(my_plan_arm);



    RCLCPP_INFO(LOGGER, "Pregrasp green Position");
    move_group_arm.setJointValueTarget(pregrasp_green_position_arm);
    success_arm = (move_group_arm.plan(my_plan_arm) == moveit::core::MoveItErrorCode::SUCCESS);
    move_group_arm.execute(my_plan_arm);

    RCLCPP_INFO(LOGGER, "Pick green Position");
    move_group_arm.setJointValueTarget(pick_green_position_arm);
    success_arm = (move_group_arm.plan(my_plan_arm) == moveit::core::MoveItErrorCode::SUCCESS);
    move_group_arm.execute(my_plan_arm);
    open_gripper(ur_io_publisher_node, ur_msgs_client);

    RCLCPP_INFO(LOGGER, "Pregrasp green Position");
    move_group_arm.setJointValueTarget(pregrasp_green_position_arm);
    success_arm = (move_group_arm.plan(my_plan_arm) == moveit::core::MoveItErrorCode::SUCCESS);
    move_group_arm.execute(my_plan_arm);



    RCLCPP_INFO(LOGGER, "Pregrasp blue Position");
    move_group_arm.setJointValueTarget(pregrasp_blue_position_arm);
    success_arm = (move_group_arm.plan(my_plan_arm) == moveit::core::MoveItErrorCode::SUCCESS);
    move_group_arm.execute(my_plan_arm);

    RCLCPP_INFO(LOGGER, "Pick blue Position");
    move_group_arm.setJointValueTarget(pick_blue_position_arm);
    success_arm = (move_group_arm.plan(my_plan_arm) == moveit::core::MoveItErrorCode::SUCCESS);
    move_group_arm.execute(my_plan_arm);
    close_gripper(ur_io_publisher_node, ur_msgs_client);

    RCLCPP_INFO(LOGGER, "Pregrasp blue Position");
    move_group_arm.setJointValueTarget(pregrasp_blue_position_arm);
    success_arm = (move_group_arm.plan(my_plan_arm) == moveit::core::MoveItErrorCode::SUCCESS);
    move_group_arm.execute(my_plan_arm);



    RCLCPP_INFO(LOGGER, "Pregrasp gray Position");
    move_group_arm.setJointValueTarget(pregrasp_gray_position_arm);
    success_arm = (move_group_arm.plan(my_plan_arm) == moveit::core::MoveItErrorCode::SUCCESS);
    move_group_arm.execute(my_plan_arm);

    RCLCPP_INFO(LOGGER, "Pick gray Position");
    move_group_arm.setJointValueTarget(pick_gray_position_arm);
    success_arm = (move_group_arm.plan(my_plan_arm) == moveit::core::MoveItErrorCode::SUCCESS);
    move_group_arm.execute(my_plan_arm);
    open_gripper(ur_io_publisher_node, ur_msgs_client);

    RCLCPP_INFO(LOGGER, "Pregrasp gray Position");
    move_group_arm.setJointValueTarget(pregrasp_gray_position_arm);
    success_arm = (move_group_arm.plan(my_plan_arm) == moveit::core::MoveItErrorCode::SUCCESS);
    move_group_arm.execute(my_plan_arm);



    RCLCPP_INFO(LOGGER, "Pregrasp red Position");
    move_group_arm.setJointValueTarget(pregrasp_red_position_arm);
    success_arm = (move_group_arm.plan(my_plan_arm) == moveit::core::MoveItErrorCode::SUCCESS);
    move_group_arm.execute(my_plan_arm);

    RCLCPP_INFO(LOGGER, "Pick red Position");
    move_group_arm.setJointValueTarget(pick_red_position_arm);
    success_arm = (move_group_arm.plan(my_plan_arm) == moveit::core::MoveItErrorCode::SUCCESS);
    move_group_arm.execute(my_plan_arm);
    close_gripper(ur_io_publisher_node, ur_msgs_client);

    RCLCPP_INFO(LOGGER, "Pregrasp red Position");
    move_group_arm.setJointValueTarget(pregrasp_red_position_arm);
    success_arm = (move_group_arm.plan(my_plan_arm) == moveit::core::MoveItErrorCode::SUCCESS);
    move_group_arm.execute(my_plan_arm);



    RCLCPP_INFO(LOGGER, "Pregrasp blue Position");
    move_group_arm.setJointValueTarget(pregrasp_blue_position_arm);
    success_arm = (move_group_arm.plan(my_plan_arm) == moveit::core::MoveItErrorCode::SUCCESS);
    move_group_arm.execute(my_plan_arm);

    RCLCPP_INFO(LOGGER, "Pick blue Position");
    move_group_arm.setJointValueTarget(pick_blue_position_arm);
    success_arm = (move_group_arm.plan(my_plan_arm) == moveit::core::MoveItErrorCode::SUCCESS);
    move_group_arm.execute(my_plan_arm);
    open_gripper(ur_io_publisher_node, ur_msgs_client);

    RCLCPP_INFO(LOGGER, "Pregrasp blue Position");
    move_group_arm.setJointValueTarget(pregrasp_blue_position_arm);
    success_arm = (move_group_arm.plan(my_plan_arm) == moveit::core::MoveItErrorCode::SUCCESS);
    move_group_arm.execute(my_plan_arm);

  }
  RCLCPP_INFO(LOGGER, "Start Position");
  move_group_arm.setJointValueTarget(start_position_arm);
  success_arm = (move_group_arm.plan(my_plan_arm) == moveit::core::MoveItErrorCode::SUCCESS);
  move_group_arm.execute(my_plan_arm);

/*
  // Pregrasp
  RCLCPP_INFO(LOGGER, "Pregrasp Position");

  geometry_msgs::msg::Pose target_pose1;
  target_pose1.orientation.x = -1.0;
  target_pose1.orientation.y = 0.00;
  target_pose1.orientation.z = 0.00;
  target_pose1.orientation.w = 0.00;
  target_pose1.position.x = 0.343;
  target_pose1.position.y = 0.132;
  target_pose1.position.z = 0.264;
  move_group_arm.setPoseTarget(target_pose1);

  success_arm = (move_group_arm.plan(my_plan_arm) ==
                 moveit::core::MoveItErrorCode::SUCCESS);

  move_group_arm.execute(my_plan_arm);

  // Open Gripper

  RCLCPP_INFO(LOGGER, "Open Gripper!");

  move_group_gripper.setNamedTarget("gripper_open");

  moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper;
  bool success_gripper = (move_group_gripper.plan(my_plan_gripper) ==
                          moveit::core::MoveItErrorCode::SUCCESS);

  move_group_gripper.execute(my_plan_gripper);

  // Approach
  RCLCPP_INFO(LOGGER, "Approach to object!");

  std::vector<geometry_msgs::msg::Pose> approach_waypoints;
  target_pose1.position.z -= 0.03;
  approach_waypoints.push_back(target_pose1);

  target_pose1.position.z -= 0.03;
  approach_waypoints.push_back(target_pose1);

  moveit_msgs::msg::RobotTrajectory trajectory_approach;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;

  double fraction = move_group_arm.computeCartesianPath(
      approach_waypoints, eef_step, jump_threshold, trajectory_approach);

  move_group_arm.execute(trajectory_approach);

  // Close Gripper

  RCLCPP_INFO(LOGGER, "Close Gripper!");

  move_group_gripper.setNamedTarget("gripper_close");

  success_gripper = (move_group_gripper.plan(my_plan_gripper) ==
                     moveit::core::MoveItErrorCode::SUCCESS);

  move_group_gripper.execute(my_plan_gripper);

  // Retreat

  RCLCPP_INFO(LOGGER, "Retreat from object!");

  std::vector<geometry_msgs::msg::Pose> retreat_waypoints;
  target_pose1.position.z += 0.03;
  retreat_waypoints.push_back(target_pose1);

  target_pose1.position.z += 0.03;
  retreat_waypoints.push_back(target_pose1);

  moveit_msgs::msg::RobotTrajectory trajectory_retreat;

  fraction = move_group_arm.computeCartesianPath(
      retreat_waypoints, eef_step, jump_threshold, trajectory_retreat);

  move_group_arm.execute(trajectory_retreat);

  // Place

  RCLCPP_INFO(LOGGER, "Rotating Arm");

  current_state_arm = move_group_arm.getCurrentState(10);
  current_state_arm->copyJointGroupPositions(joint_model_group_arm,
                                             start_position_arm);

  start_position_arm[0] = 1.57; // Shoulder Pan

  move_group_arm.setJointValueTarget(start_position_arm);

  success_arm = (move_group_arm.plan(my_plan_arm) ==
                 moveit::core::MoveItErrorCode::SUCCESS);

  move_group_arm.execute(my_plan_arm);

  // Open Gripper

  RCLCPP_INFO(LOGGER, "Release Object!");

  move_group_gripper.setNamedTarget("open");

  success_gripper = (move_group_gripper.plan(my_plan_gripper) ==
                     moveit::core::MoveItErrorCode::SUCCESS);

  move_group_gripper.execute(my_plan_gripper);
*/
  rclcpp::shutdown();
  return 0;
}