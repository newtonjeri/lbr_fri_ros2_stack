#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include "geometric_shapes/shape_operations.h"
#include "shape_msgs/msg/mesh.h"
#include <math.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("beer_tap_motion");

const double eef_step = 0.005;
const double jump_threshold = 0.0;
const std::string tap_object = "beer_tap";
const std::string support_surface = "bar_surface";

// Function to perform orientation constrained planning
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
    }
    // Clear path constraints after the move
    move_group.clearPathConstraints();
}

// Function to plan to a target pose
void planToTargetPose(moveit::planning_interface::MoveGroupInterface &move_group,
                      geometry_msgs::msg::Pose &target_pose)
{
    move_group.setPoseTarget(target_pose);

    auto const [success, plan] = [&move_group] {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group.plan(msg));
        return std::make_pair(ok, msg);
    }();

    if (success)
    {
        move_group.execute(plan);
    }
    else
    {
        RCLCPP_ERROR(LOGGER, "Planning failed!");
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Configure node
    auto node_ptr = rclcpp::Node::make_shared("beer_tap_motion_planner");
    node_ptr->declare_parameter("robot_name", "lbr");
    auto robot_name = node_ptr->get_parameter("robot_name").as_string();
    

    // Create MoveGroupInterface (lives inside robot_name namespace)
    auto move_group_interface = moveit::planning_interface::MoveGroupInterface(
      node_ptr, moveit::planning_interface::MoveGroupInterface::Options("arm", "robot_description",
                                                                        robot_name));

    // Set speed and acceleration factors
    move_group_interface.setMaxVelocityScalingFactor(0.5);
    move_group_interface.setMaxAccelerationScalingFactor(0.5);

    // Step 1: Move to the starting position (near the tap)
    geometry_msgs::msg::Pose start_pose;
    start_pose.orientation.w = 1.0; // Keep upright
    start_pose.position.x = 0.4;    // Adjust based on tap location
    start_pose.position.y = 0.0;    // Adjust Y-axis based on your setup
    start_pose.position.z = 0.6;    // Adjust height based on your setup
    RCLCPP_INFO(LOGGER, "1. Moving to starting position");
    planToTargetPose(move_group_interface, start_pose);

    // Step 2: Move to the beer pouring position (tilt glass under the tap)
    geometry_msgs::msg::Pose pouring_pose;
    pouring_pose.position.x = 0.5; // Adjust based on tap location
    pouring_pose.position.y = 0.2; // Adjust Y-axis for correct position
    pouring_pose.position.z = 0.4; // Position under the tap
    pouring_pose.orientation.x = 0.0;
    pouring_pose.orientation.y = 0.707; // 45 degree tilt for proper pouring
    pouring_pose.orientation.z = 0.0;
    pouring_pose.orientation.w = 0.707;
    
    // Use constraints to ensure the tilt is maintained
    RCLCPP_INFO(LOGGER, "2. Moving to pouring position with orientation constraints");
    planWithConstraints(move_group_interface, pouring_pose, pouring_pose);

    // Step 3: Move to the serving position (after pouring)
    geometry_msgs::msg::Pose serving_pose;
    serving_pose.position.x = 0.7;    // Adjust for serving area
    serving_pose.position.y = -0.2;   // Adjust Y-axis for serving area
    serving_pose.position.z = 0.4;    // Adjust height for placing the glass
    serving_pose.orientation.w = 1.0; // Keep upright after pouring
    RCLCPP_INFO(LOGGER, "3. Moving to serving position");
    planToTargetPose(move_group_interface, serving_pose);

    RCLCPP_INFO(LOGGER, "Beer tapping process completed.");
    rclcpp::shutdown();
    return 0;
}
