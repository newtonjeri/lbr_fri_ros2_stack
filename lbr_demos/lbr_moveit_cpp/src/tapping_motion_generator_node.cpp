#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <math.h>

using namespace std::chrono_literals;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_arm");

const double eef_step = 0.005;
const double jump_threshold = 0.0;
const std::string cube = "box_peq";
const std::string support_surface = "world";

geometry_msgs::msg::Pose current_pose;

// Function to perform orientation constrained planning,
void planWithConstraints(moveit::planning_interface::MoveGroupInterface &move_group,
                         geometry_msgs::msg::Pose &target_pose,
                         const geometry_msgs::msg::Pose &orientation_pose)
{
    moveit_msgs::msg::OrientationConstraint orientation_constraint;
    orientation_constraint.header.frame_id = move_group.getPoseReferenceFrame();
    orientation_constraint.link_name = move_group.getEndEffectorLink();
    orientation_constraint.orientation = orientation_pose.orientation;
    orientation_constraint.absolute_x_axis_tolerance = 0.4;
    orientation_constraint.absolute_y_axis_tolerance = 0.4;
    orientation_constraint.absolute_z_axis_tolerance = 0.4;
    orientation_constraint.weight = 1.0;

    moveit_msgs::msg::Constraints orientation_constraints;
    orientation_constraints.orientation_constraints.emplace_back(orientation_constraint);

    move_group.setPathConstraints(orientation_constraints);
    move_group.setPoseTarget(target_pose);
    move_group.setPlanningTime(10.0);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(LOGGER, "Planning with constraints %s", success ? "SUCCEEDED" : "FAILED");

    if (success)
    {
        move_group.move();
        current_pose = target_pose;
    }
    // Clear path constraints after the move
    move_group.clearPathConstraints();
}



// Function to perform cartesian path planning, that is
void cartesianPathPlanning(
    moveit::planning_interface::MoveGroupInterface &move_group,
    const geometry_msgs::msg::Pose &start_pose,
    std::vector<geometry_msgs::msg::Pose> &waypoints,
    const double eef_step,
    const double jump_threshold,
    const int iterations,
    const std::vector<double> step)
{
    waypoints.push_back(start_pose);
    geometry_msgs::msg::Pose pose2 = start_pose;

    for (int i = 0; i < iterations; i++)
    {
        pose2.position.x += step[0];
        pose2.position.y += step[1];
        pose2.position.z += step[2];
        waypoints.push_back(pose2);
    }

    current_pose = pose2;

    // Define trajectory for the approach
    moveit_msgs::msg::RobotTrajectory trajectory_approach;
    // Compute the Cartesian for the approach
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory_approach);
    if (fraction < 0.90)
    {
        RCLCPP_WARN(rclcpp::get_logger("move_group"), "Cartesian path not fully planned, fraction: %f", fraction);
    }

    // Execute the approach trajectory
    move_group.execute(trajectory_approach);
}

// Function to plan to a target POSE, i.e. target specified in spacial coordinates x, y ,z and orientation w x y z
void planToTargetPose(
    moveit::planning_interface::MoveGroupInterface &move_group,
    geometry_msgs::msg::Pose &target_pose)
{
    move_group.setPoseTarget(target_pose);

    // Create a plan to that target pose
    auto const [success, plan] = [&move_group]
    {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group.plan(msg));
        return std::make_pair(ok, msg);
    }();

    // Execute the plan
    if (success)
    {
        move_group.execute(plan);
        current_pose = target_pose;
    }
    else
    {
        RCLCPP_ERROR(LOGGER, "Planning failed!");
    }
}

// Function to plan to a joint space target, to be used for openning and clossing the gripper
void planToJointSpaceTarget(
    moveit::planning_interface::MoveGroupInterface &move_group,
    std::vector<double> joint_target)
{
    move_group.setJointValueTarget(joint_target);

    // Create a plan to that joint value target
    auto const [success, plan] = [&move_group]
    {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group.plan(msg));
        return std::make_pair(ok, msg);
    }();

    // Execute the plan
    if (success)
    {
        move_group.execute(plan);
    }
    else
    {
        RCLCPP_ERROR(LOGGER, "Planning failed!");
    }
}

// Function to set a relative position to be used for orientation constrained planning
geometry_msgs::msg::Pose getRelativePose(
    geometry_msgs::msg::Pose pose,
    double pos_x,
    double pos_y,
    double pos_z)
{
    auto local_current_pose = pose;
    // Creates a pose at a given positional offset from the current pose
    auto get_relative_pose = [local_current_pose](double x, double y, double z)
    {
        auto target_pose = local_current_pose;
        target_pose.position.x += x;
        target_pose.position.y += y;
        target_pose.position.z += z;
        return target_pose;
    };

    auto relative_pose = get_relative_pose(pos_x, pos_y, pos_z);
    return relative_pose;
}

int main(int argc, char **argv){

  rclcpp::init(argc, argv);
  // Configure node
  auto node_ptr = rclcpp::Node::make_shared("tapping_motion_node");
  node_ptr->declare_parameter("robot_name", "lbr");
  auto robot_name = node_ptr->get_parameter("robot_name").as_string();


  // Create MoveGroupInterface (lives inside robot_name namespace)
  auto move_group_interface = moveit::planning_interface::MoveGroupInterface(
      node_ptr, moveit::planning_interface::MoveGroupInterface::Options("arm", "robot_description",
                                                                        robot_name));

  // Move to home position
  std::vector<double> home_joint_values = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  planToJointSpaceTarget(move_group_interface, home_joint_values);


  /*
    1. Setup and Initialization:
    The environment is loaded and the pose of the glass is read.
    The robot's end effector link is set, and any previous glass objects in the scene are removed.

    2. Move to the Glass Pickup Pose:
    The robot moves to the designated pose to pick up the glass.

    3.Move Under the Glass:
    The robot adjusts its position to be directly underneath the glass.

    4. Lift the Glass:

    The robot moves upward, lifting the glass from its original position.
    Move the Glass to the Tap:

    The robot moves the glass to the position of the beer tap.

    5. Tilt the Glass:
    The glass is tilted to prepare for the beer pouring.

    6. Start Tapping:
    A signal is sent to the Arduino to start the beer tapping process, which likely involves activating a dispensing mechanism.
    
    7. Wait During Tapping:
    The robot waits for a specified duration (9 seconds) to allow the beer to pour into the glass.
    Straighten the Glass:

    After the tapping process, the glass is returned to an upright position.
    
    8. Wait After Tapping:
    Another wait time (2 seconds) may be adjusted if necessary after straightening the glass.
    
    9. Move the Glass Above the Placement Area:
    The robot moves the glass to a position directly above where it will be placed down.
    
    10. Place the Glass Down:
    The robot lowers the glass to place it down.
    
    12. Move Away from the Glass:
    The robot moves away from the glass after placing it down.
    
    13. Return to Zero Pose:
    The robot returns to the Cartesian zero pose, which is typically a safe, home position.
    
    14. Ensure All Joints are at Zero:
    The robot performs a final motion to ensure that all joints are set to 0°, ensuring readiness for the next operation.
  */

  rclcpp::shutdown();

  return 0;
}