#include <chrono>
#include <cmath>
#include <memory>
#include <thread>
#include <vector>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <custom_msgs/msg/detected_objects.hpp>
#include <custom_msgs/msg/detected_surfaces.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_node");
static const std::string PLANNING_GROUP_ROBOT = "ur_manipulator";
static const std::string PLANNING_GROUP_GRIPPER = "gripper";

using namespace std::chrono_literals;

class PickAndPlace {
public:
  PickAndPlace(rclcpp::Node::SharedPtr base_node_) : base_node_(base_node_) {
    RCLCPP_INFO(LOGGER, "Initializing Class: Pick And Place...");

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    move_group_node_ =
        rclcpp::Node::make_shared("move_group_node", node_options);
    // start move_group node in a new executor thread and spin it
    executor_.add_node(move_group_node_);
    std::thread([this]() { this->executor_.spin(); }).detach();

    // initialize move_group interfaces
    move_group_robot_ = std::make_shared<MoveGroupInterface>(
        move_group_node_, PLANNING_GROUP_ROBOT);
    move_group_gripper_ = std::make_shared<MoveGroupInterface>(
        move_group_node_, PLANNING_GROUP_GRIPPER);

    // get initial state of robot and gripper
    joint_model_group_robot_ =
        move_group_robot_->getCurrentState()->getJointModelGroup(
            PLANNING_GROUP_ROBOT);
    joint_model_group_gripper_ =
        move_group_gripper_->getCurrentState()->getJointModelGroup(
            PLANNING_GROUP_GRIPPER);

    RCLCPP_INFO(LOGGER, "Planning Frame: %s",
                move_group_robot_->getPlanningFrame().c_str());
    RCLCPP_INFO(LOGGER, "End Effector Link: %s",
                move_group_robot_->getEndEffectorLink().c_str());
    RCLCPP_INFO(LOGGER, "Available Planning Groups:");
    std::vector<std::string> group_names =
        move_group_robot_->getJointModelGroupNames();
    // more efficient method than std::copy() method used in the docs
    for (long unsigned int i = 0; i < group_names.size(); i++) {
      RCLCPP_INFO(LOGGER, "Group %ld: %s", i, group_names[i].c_str());
    }

    // get current state of robot and gripper
    current_state_robot_ = move_group_robot_->getCurrentState(10);
    current_state_robot_->copyJointGroupPositions(joint_model_group_robot_,
                                                  joint_group_positions_robot_);
    current_state_gripper_ = move_group_gripper_->getCurrentState(10);
    current_state_gripper_->copyJointGroupPositions(
        joint_model_group_gripper_, joint_group_positions_gripper_);
    move_group_robot_->setStartStateToCurrentState();
    move_group_gripper_->setStartStateToCurrentState();

    objpose_sub_ = move_group_node_
                       ->create_subscription<custom_msgs::msg::DetectedObjects>(
                           "/object_detected", 10,
                           std::bind(&PickAndPlace::object_pose_cb, this,
                                     std::placeholders::_1));

    RCLCPP_INFO(LOGGER, "Class Initialized: Pick And Place");
  }

  ~PickAndPlace() { RCLCPP_INFO(LOGGER, "Class Terminated: Pick And Place"); }

  void object_pose_cb(const custom_msgs::msg::DetectedObjects::SharedPtr msg) {
    this->obj_pose_.position = msg->position;
    obj_width_ = msg->width;
    obj_thickness_ = msg->thickness;
    obj_pose_received_ = true;
  }

  void execute_trajectory_plan() {

    while (!obj_pose_received_) {
      RCLCPP_WARN(LOGGER, "Object Pose not received yet!");
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    float objX = this->obj_pose_.position.x + obj_thickness_ / 4.;
    float objY = this->obj_pose_.position.y - obj_width_ / 2.;
    // float objY = this->obj_pose_.position.y;
    float objZ = this->obj_pose_.position.z;

    RCLCPP_INFO(LOGGER, "Planning and Executing Pick And Place...");

    // Go to home position
    RCLCPP_INFO(LOGGER, "Going to Home Position...");
    RCLCPP_INFO(LOGGER, "Preparing Joint Value Trajectory...");
    setup_arm_named_pose("home");
    RCLCPP_INFO(LOGGER, "Planning Joint Value Trajectory...");
    plan_trajectory_kinematics();
    RCLCPP_INFO(LOGGER, "Executing Joint Value Trajectory...");
    execute_trajectory_kinematics();

    // Approach to pre-grasp position
    RCLCPP_INFO(LOGGER, "Preparing Pregrasp Position...");
    RCLCPP_INFO(LOGGER, "Setting Goal Pose: [%.3f, %.3f, 0.261]", objX, objY);
    setup_goal_pose_target(+objX, +objY, +0.261, -1.000, +0.000, +0.000,
                           +0.000);
    // setup_arm_named_pose("pregrasp");
    RCLCPP_INFO(LOGGER, "Planning Goal Pose Trajectory...");
    plan_trajectory_kinematics();
    RCLCPP_INFO(LOGGER, "Executing Goal Pose Trajectory...");
    execute_trajectory_kinematics();

    // Open the gripper
    RCLCPP_INFO(LOGGER, "Opening Gripper...");
    RCLCPP_INFO(LOGGER, "Preparing Gripper Value...");
    setup_gripper_named_pose("gripper_open");
    RCLCPP_INFO(LOGGER, "Planning Gripper Action...");
    plan_trajectory_gripper();
    RCLCPP_INFO(LOGGER, "Executing Gripper Action...");
    execute_trajectory_gripper();
    RCLCPP_INFO(LOGGER, "Gripper Opened");

    // Approach to grasping position
    RCLCPP_INFO(LOGGER, "Approaching to grasp...");
    RCLCPP_INFO(LOGGER, "Preparing Cartesian Trajectory...");
    setup_waypoints_target(+0.000, +0.000, -0.080);
    RCLCPP_INFO(LOGGER, "Planning Cartesian Trajectory...");
    plan_trajectory_cartesian();
    RCLCPP_INFO(LOGGER, "Executing Cartesian Trajectory...");
    execute_trajectory_cartesian();

    // Close the gripper and pick the box
    RCLCPP_INFO(LOGGER, "Closing Gripper...");
    RCLCPP_INFO(LOGGER, "Preparing Gripper Value...");
    setup_gripper_named_pose("gripper_close");
    RCLCPP_INFO(LOGGER, "Planning Gripper Action...");
    plan_trajectory_gripper();
    RCLCPP_INFO(LOGGER, "Executing Gripper Action...");
    execute_trajectory_gripper();
    RCLCPP_INFO(LOGGER, "Gripper Closed");

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // Retreat
    RCLCPP_INFO(LOGGER, "Retreating...");
    RCLCPP_INFO(LOGGER, "Preparing Cartesian Trajectory...");
    setup_waypoints_target(+0.000, +0.000, +0.080);
    RCLCPP_INFO(LOGGER, "Planning Cartesian Trajectory...");
    plan_trajectory_cartesian();
    RCLCPP_INFO(LOGGER, "Executing Cartesian Trajectory...");
    execute_trajectory_cartesian();

    // Go to dropping position
    RCLCPP_INFO(LOGGER, "Going to Place Position...");
    current_state_robot_ = move_group_robot_->getCurrentState(10);
    current_state_robot_->copyJointGroupPositions(joint_model_group_robot_,
                                                  joint_group_positions_robot_);
    RCLCPP_INFO(LOGGER, "Preparing Joint Value Trajectory...");
    setup_joint_value_target(
        M_PI, joint_group_positions_robot_[1], joint_group_positions_robot_[2],
        joint_group_positions_robot_[3], joint_group_positions_robot_[4],
        joint_group_positions_robot_[5]);
    RCLCPP_INFO(LOGGER, "Planning Joint Value Trajectory...");
    plan_trajectory_kinematics();
    RCLCPP_INFO(LOGGER, "Executing Joint Value Trajectory...");
    execute_trajectory_kinematics();

    // Open the gripper and drop the box
    RCLCPP_INFO(LOGGER, "Opening Gripper...");
    RCLCPP_INFO(LOGGER, "Preparing Gripper Value...");
    setup_gripper_named_pose("gripper_open");
    RCLCPP_INFO(LOGGER, "Planning Gripper Action...");
    plan_trajectory_gripper();
    RCLCPP_INFO(LOGGER, "Executing Gripper Action...");
    execute_trajectory_gripper();
    RCLCPP_INFO(LOGGER, "Gripper Opened");

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // Go back to home position
    RCLCPP_INFO(LOGGER, "Going to Home Position...");
    RCLCPP_INFO(LOGGER, "Preparing Joint Value Trajectory...");
    setup_arm_named_pose("home");
    RCLCPP_INFO(LOGGER, "Planning Joint Value Trajectory...");
    plan_trajectory_kinematics();
    RCLCPP_INFO(LOGGER, "Executing Joint Value Trajectory...");
    execute_trajectory_kinematics();

    // Close the gripper
    RCLCPP_INFO(LOGGER, "Closing Gripper...");
    RCLCPP_INFO(LOGGER, "Preparing Gripper Value...");
    setup_gripper_named_pose("gripper_close");
    RCLCPP_INFO(LOGGER, "Planning Gripper Action...");
    plan_trajectory_gripper();
    RCLCPP_INFO(LOGGER, "Executing Gripper Action...");
    execute_trajectory_gripper();
    RCLCPP_INFO(LOGGER, "Gripper Closed");

    RCLCPP_INFO(LOGGER, "Pick And Place Execution Complete");
  }

private:
  void setup_joint_value_target(float angle0, float angle1, float angle2,
                                float angle3, float angle4, float angle5) {
    // set the joint values for each joint of robot arm
    joint_group_positions_robot_[0] = angle0; // Shoulder Pan
    joint_group_positions_robot_[1] = angle1; // Shoulder Lift
    joint_group_positions_robot_[2] = angle2; // Elbow
    joint_group_positions_robot_[3] = angle3; // Wrist 1
    joint_group_positions_robot_[4] = angle4; // Wrist 2
    joint_group_positions_robot_[5] = angle5; // Wrist 3
    move_group_robot_->setJointValueTarget(joint_group_positions_robot_);
  }

  void setup_goal_pose_target(float pos_x, float pos_y, float pos_z,
                              float quat_x, float quat_y, float quat_z,
                              float quat_w) {
    // set the pose values for end effector of robot arm
    target_pose_robot_.position.x = pos_x;
    target_pose_robot_.position.y = pos_y;
    target_pose_robot_.position.z = pos_z;
    target_pose_robot_.orientation.x = quat_x;
    target_pose_robot_.orientation.y = quat_y;
    target_pose_robot_.orientation.z = quat_z;
    target_pose_robot_.orientation.w = quat_w;
    move_group_robot_->setPoseTarget(target_pose_robot_);
  }

  void plan_trajectory_kinematics() {
    plan_success_robot_ =
        (move_group_robot_->plan(kinematics_trajectory_plan_) ==
         moveit::core::MoveItErrorCode::SUCCESS);
  }

  void execute_trajectory_kinematics() {
    if (plan_success_robot_) {
      move_group_robot_->execute(kinematics_trajectory_plan_);
      RCLCPP_INFO(LOGGER, "Robot Kinematics Trajectory Success !");
    } else {
      RCLCPP_INFO(LOGGER, "Robot Kinematics Trajectory Failed !");
    }
  }

  void setup_waypoints_target(float x_delta, float y_delta, float z_delta) {
    // initially set target pose to current pose of the robot
    target_pose_robot_ = move_group_robot_->getCurrentPose().pose;
    // add the current pose to the target waypoints vector
    cartesian_waypoints_.push_back(target_pose_robot_);
    // calculate the desired pose from delta value for the axis
    target_pose_robot_.position.x += x_delta;
    target_pose_robot_.position.y += y_delta;
    target_pose_robot_.position.z += z_delta;
    // add the desired pose to the target waypoints vector
    cartesian_waypoints_.push_back(target_pose_robot_);
  }

  void plan_trajectory_cartesian() {
    plan_fraction_robot_ = move_group_robot_->computeCartesianPath(
        cartesian_waypoints_, end_effector_step_, jump_threshold_,
        cartesian_trajectory_plan_);
  }

  void execute_trajectory_cartesian() {
    if (plan_fraction_robot_ >= 0.0) {
      // 0.0 to 1.0 = success and -1.0 = failure
      move_group_robot_->execute(cartesian_trajectory_plan_);
      RCLCPP_INFO(LOGGER, "Robot Cartesian Trajectory Success !");
    } else {
      RCLCPP_INFO(LOGGER, "Robot Cartesian Trajectory Failed !");
    }
    // clear cartesian waypoints vector
    cartesian_waypoints_.clear();
  }

  void setup_joint_value_gripper(float angle) {
    // set the joint values for each joint of gripper
    // based on values provided
    joint_group_positions_gripper_[2] = angle;
    move_group_gripper_->setJointValueTarget(joint_group_positions_gripper_);
  }

  void setup_gripper_named_pose(std::string pose_name) {
    // set the joint values for each joint of gripper
    // based on predefined pose names
    move_group_gripper_->setNamedTarget(pose_name);
  }
  void setup_arm_named_pose(std::string pose_name) {
    // set the joint values for each joint of robot arm
    // based on predefined pose names
    move_group_robot_->setNamedTarget(pose_name);
  }

  void plan_trajectory_gripper() {
    plan_success_gripper_ =
        (move_group_gripper_->plan(gripper_trajectory_plan_) ==
         moveit::core::MoveItErrorCode::SUCCESS);
  }

  void execute_trajectory_gripper() {
    if (plan_success_gripper_) {
      move_group_gripper_->execute(gripper_trajectory_plan_);
      RCLCPP_INFO(LOGGER, "Gripper Action Command Success !");
    } else {
      RCLCPP_INFO(LOGGER, "Gripper Action Command Failed !");
    }
  }

  using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
  using JointModelGroup = moveit::core::JointModelGroup;
  using RobotStatePtr = moveit::core::RobotStatePtr;
  using Plan = MoveGroupInterface::Plan;
  using Pose = geometry_msgs::msg::Pose;
  using RobotTrajectory = moveit_msgs::msg::RobotTrajectory;

  rclcpp::Node::SharedPtr base_node_;
  rclcpp::Node::SharedPtr move_group_node_;
  rclcpp::executors::SingleThreadedExecutor executor_;

  std::shared_ptr<MoveGroupInterface> move_group_robot_;
  std::shared_ptr<MoveGroupInterface> move_group_gripper_;
  const JointModelGroup *joint_model_group_robot_;
  const JointModelGroup *joint_model_group_gripper_;
  rclcpp::Subscription<custom_msgs::msg::DetectedObjects>::SharedPtr
      objpose_sub_;
  geometry_msgs::msg::Pose obj_pose_;
  float obj_width_, obj_thickness_;
  bool obj_pose_received_ = false;

  // declare trajectory planning variables for robot and gripper
  std::vector<double> joint_group_positions_robot_;
  RobotStatePtr current_state_robot_;
  Plan kinematics_trajectory_plan_;
  Pose target_pose_robot_;
  bool plan_success_robot_ = false;
  std::vector<double> joint_group_positions_gripper_;
  RobotStatePtr current_state_gripper_;
  Plan gripper_trajectory_plan_;
  bool plan_success_gripper_ = false;

  // declare cartesian trajectory planning variables for robot
  std::vector<Pose> cartesian_waypoints_;
  RobotTrajectory cartesian_trajectory_plan_;
  const double jump_threshold_ = 0.0;
  const double end_effector_step_ = 0.01;
  double plan_fraction_robot_ = 0.0;
}; // class PickAndPlace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> base_node =
      std::make_shared<rclcpp::Node>("pick_and_place");

  PickAndPlace pick_and_place_trajectory_node(base_node);
  pick_and_place_trajectory_node.execute_trajectory_plan();

  rclcpp::shutdown();
  return 0;
}