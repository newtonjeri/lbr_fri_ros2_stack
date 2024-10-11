#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include "geometric_shapes/shape_operations.h"
#include "shape_msgs/msg/mesh.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Geometry>

// Function to load the environment into the MoveIt scene
void loadEnvironment()
{
    // Variables 
    double height_tap_handle = 0.34;  // Height of the tap handle above the table
    double lateral_distance_tap_handle = 0.26;  // Lateral distance between tap handle & column center
    double height_handle = 0.05, radius_handle = 0.02;
    double lateral_shift_glass = 0.2;
    double height_tap_unit1 = 0.455, radius_tap_unit1 = (0.115 / 2) + 0.03;
    double width_tap_unit2 = 0.73, radius_tap_unit2 = (0.15 / 2) + 0.03;
    double height_tap_unit2 = 0.42;
    
    // Actuator
    double width_actuator1 = width_tap_unit2, depth_actuator1 = 0.3, height_actuator1 = 0.2;

    // Table of the tap unit
    double table_width = 2, table_depth = 0.665, table_height = 0.999; // Height without rollers is 0.87
    double shift_tap_unit_back = 0.1 + radius_tap_unit2 / 2;

    // Mobile Platform
    double height_platform = 0.703, width_platform = 1.19, depth_platform = 0.61;

    // Glass parameters
    double radius_glass_base = 0.04, height_glass_base = 0.005;
    double radius_glass_stem = 0.005, height_glass_stem = 0.075;
    double height_glass = 0.1, radius_glass = 0.15 / 2;
    double total_glass_height = height_glass_base + height_glass_stem + height_glass;  // Total glass height
    double grasp_height = 0.047;  // Height of the gripper over the table
    double glass_offset = 0.107;  // How far the glass sticks out in the z-direction from the end-effector

    // Walls
    double wall_thickness = 0.1, wall_height = 2, wall_width = 1.8, wall_width_2 = 1;
    double wall_distance_1 = 0.1, wall_distance_2 = 0.3;

    // Position of the LBR on the mobile platform
    double lbr_x = depth_platform / 2, lbr_y = 0.15;

    // Creating a collision object for the tap handle
    moveit_msgs::msg::CollisionObject tap_handle;
    tap_handle.id = "tap_handle";
    tap_handle.header.frame_id = "world";
    
    // Define the primitive shape (cylinder)
    shape_msgs::msg::SolidPrimitive tap_handle_primitive;
    tap_handle_primitive.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    tap_handle_primitive.dimensions = {height_handle, radius_handle};

    // Assign the primitive to the object
    tap_handle.primitives.push_back(tap_handle_primitive);

    // Assuming pose_ZH was previously read from calibration and available in the same function
    geometry_msgs::msg::Pose pose_ZH;
    pose_ZH.position.x = 0.7641;
    pose_ZH.position.y = -0.0346;
    pose_ZH.position.z = 0.5319;
    pose_ZH.orientation.x = 0.6909;
    pose_ZH.orientation.y = 0.723;
    pose_ZH.orientation.z = 0.0;
    pose_ZH.orientation.w = 0.0;


    // Set the pose of the tap handle
    geometry_msgs::msg::Pose tap_handle_pose;
    tap_handle_pose.position = pose_ZH.position;
    tap_handle_pose.position.z += height_handle / 2 + glass_offset;
    tap_handle_pose.orientation = pose_ZH.orientation;

    tap_handle.primitive_poses.push_back(tap_handle_pose);

    // Add subframes for "top" and "bottom" of the handle
    tap_handle.subframe_names.push_back("Top");
    tap_handle.subframe_poses.push_back(geometry_msgs::msg::Pose());
    tap_handle.subframe_poses[0].position.z = -height_handle / 2;

    tap_handle.subframe_names.push_back("Bottom");
    tap_handle.subframe_poses.push_back(geometry_msgs::msg::Pose());
    tap_handle.subframe_poses[1].position.z = height_handle / 2;

    // Add collision objects for tap unit
    moveit_msgs::msg::CollisionObject tap_unit1;
    tap_unit1.id = "tap_unit1";
    tap_unit1.header.frame_id = "tap_handle/Bottom";
    
    shape_msgs::msg::SolidPrimitive tap_unit1_primitive;
    tap_unit1_primitive.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    tap_unit1_primitive.dimensions = {height_tap_unit1, radius_tap_unit1};
    
    tap_unit1.primitives.push_back(tap_unit1_primitive);

    geometry_msgs::msg::Pose tap_unit1_pose;
    tap_unit1_pose.position.x = lateral_distance_tap_handle;
    tap_unit1_pose.position.y = shift_tap_unit_back;
    tap_unit1_pose.position.z = height_tap_handle - height_tap_unit1 / 2;
    tap_unit1_pose.orientation.w = 1;

    tap_unit1.primitive_poses.push_back(tap_unit1_pose);

    // Create the second tap unit object
    moveit_msgs::msg::CollisionObject tap_unit2;
    tap_unit2.id = "tap_unit2";
    tap_unit2.header.frame_id = "tap_handle/Bottom";
    
    shape_msgs::msg::SolidPrimitive tap_unit2_primitive;
    tap_unit2_primitive.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    tap_unit2_primitive.dimensions = {width_tap_unit2, radius_tap_unit2};

    tap_unit2.primitives.push_back(tap_unit2_primitive);

    geometry_msgs::msg::Pose tap_unit2_pose;
    tap_unit2_pose.position.x = lateral_distance_tap_handle;
    tap_unit2_pose.position.y = shift_tap_unit_back;
    tap_unit2_pose.position.z = height_tap_handle - height_tap_unit2;
    
    // Quaternion to rotate 45 degrees in the y-axis
    Eigen::Quaterniond rotation(Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitY()));
    tap_unit2_pose.orientation.w = rotation.w();
    tap_unit2_pose.orientation.y = rotation.y();

    tap_unit2.primitive_poses.push_back(tap_unit2_pose);

    // Add the rest of the components like actuator, table, glass (with base, stem, and body), and walls
    // Code for adding these objects is similar to above, setting up appropriate primitives, poses, and adding them to the scene

        // actuator
    moveit_msgs::msg::CollisionObject actuator1;
    actuator1.id = "actuator1";
    actuator1.header.frame_id = "tap_handle/Top";
    
    shape_msgs::msg::SolidPrimitive actuator1_primitive;
    actuator1_primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    actuator1_primitive.dimensions = {width_actuator1, depth_actuator1, height_actuator1}; // X, Y, Z
    actuator1.primitives.push_back(actuator1_primitive);
    
    geometry_msgs::msg::Pose actuator1_pose;
    actuator1_pose.position.x = lateral_distance_tap_handle;
    actuator1_pose.position.y = 0.075;
    actuator1_pose.position.z = -height_actuator1 / 2;
    actuator1_pose.orientation.w = 1;
    actuator1.primitive_poses.push_back(actuator1_pose);
    

    // Table of the tap unit
    moveit_msgs::msg::CollisionObject table;
    table.id = "table";
    table.header.frame_id = "world";
    
    shape_msgs::msg::SolidPrimitive table_primitive;
    table_primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    table_primitive.dimensions = {table_width, table_depth, table_height};
    table.primitives.push_back(table_primitive);
    
    geometry_msgs::msg::Pose table_pose;
    table_pose.position.y = -table_depth/2;
    // double z_floor = tap_handle_pose.position.z - height_tap_handle / 2 + height_platform; // Z of the floor in tap_handle/Bottom
    table_pose.position.z = table_height / 2;
    table.primitive_poses.push_back(table_pose);
    

    // // Glass Base
    // moveit_msgs::msg::CollisionObject glass_base;
    // glass_base.id = "glass_base";
    // glass_base.header.frame_id = "tap_handle/Bottom";
    
    // shape_msgs::msg::SolidPrimitive glass_base_primitive;
    // glass_base_primitive.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    // glass_base_primitive.dimensions = {height_glass_base, radius_glass_base};
    // glass_base.primitives.push_back(glass_base_primitive);
    
    // geometry_msgs::msg::Pose glass_base_pose;
    // glass_base_pose.position.x = -lateral_shift_glass;
    // glass_base_pose.position.z = z_floor - table_height - height_glass_base / 2;
    // glass_base.primitive_poses.push_back(glass_base_pose);
    
    // glass_base.operation = moveit_msgs::msg::CollisionObject::ADD;
    // glass_base.subframe_names.push_back("Top");
    // glass_base.subframe_poses.push_back(geometry_msgs::msg::Pose());
    // glass_base.subframe_poses[0].position.z = -height_glass_base / 2;
    
    // // Stem
    // moveit_msgs::msg::CollisionObject glass_stem;
    // glass_stem.id = "glass_stem";
    // glass_stem.header.frame_id = "glass_base/Top";
    
    // shape_msgs::msg::SolidPrimitive glass_stem_primitive;
    // glass_stem_primitive.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    // glass_stem_primitive.dimensions = {height_glass_stem, radius_glass_stem};
    // glass_stem.primitives.push_back(glass_stem_primitive);
    
    // geometry_msgs::msg::Pose glass_stem_pose;
    // glass_stem_pose.position.z = -height_glass_stem / 2;
    // glass_stem.primitive_poses.push_back(glass_stem_pose);
    
    // glass_stem.operation = moveit_msgs::msg::CollisionObject::ADD;
    // glass_stem.subframe_names.push_back("Top");
    // glass_stem.subframe_poses.push_back(geometry_msgs::msg::Pose());
    // glass_stem.subframe_poses[0].position.z = -height_glass_stem / 2;
    
    
    // // Glass
    // moveit_msgs::msg::CollisionObject glass;
    // glass.id = "glass";
    // glass.header.frame_id = "glass_stem/Top";
    
    // shape_msgs::msg::SolidPrimitive glass_primitive;
    // glass_primitive.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    // glass_primitive.dimensions = {height_glass, radius_glass};
    // glass.primitives.push_back(glass_primitive);
    
    // geometry_msgs::msg::Pose glass_pose;
    // glass_pose.position.z = -height_glass / 2;
    // glass.primitive_poses.push_back(glass_pose);
    
    // glass.operation = moveit_msgs::msg::CollisionObject::ADD;
    // glass.subframe_names.push_back("Top");
    // glass.subframe_poses.push_back(geometry_msgs::msg::Pose());
    // glass.subframe_poses[0].position.z = -height_glass / 2;

    // Glass from mesh
    // add glass mesh
    shapes::Mesh *glass_mesh = shapes::createMeshFromResource("package://lbr_moveit_cpp/scene_meshes/Bierglas.stl");
    shapes::ShapeMsg glass_shape_msg;
    shapes::constructMsgFromShape(glass_mesh, glass_shape_msg);
    shape_msgs::msg::Mesh glass_mesh_msg = boost::get<shape_msgs::msg::Mesh>(glass_shape_msg);


    std::vector<moveit_msgs::msg::CollisionObject> glasses;
    glasses.resize(1);

    glasses[0].meshes.resize(1);
    glasses[0].mesh_poses.resize(1);
    glasses[0].id = "glass_1";
    glasses[0].header.frame_id = "world";

    glasses[0].meshes[0] = glass_mesh_msg;
    glasses[0].mesh_poses[0].position.x = -lateral_shift_glass;
    glasses[0].mesh_poses[0].position.z = table_height;
    glasses[0].mesh_poses[0].orientation.w = 0.7071;
    glasses[0].mesh_poses[0].orientation.x = 0.7071;


    glasses[0].meshes.push_back(glass_mesh_msg);
    glasses[0].mesh_poses.push_back(glasses[0].mesh_poses[0]);
    glasses[0].operation = glasses[0].ADD;

    // Calculation of relevant poses
    geometry_msgs::msg::Pose pose_ZH_Bottom = pose_ZH;
    pose_ZH_Bottom.position.z = pose_ZH.position.z + glass_offset; // Pose to grip the glass (lower down)
    
    geometry_msgs::msg::Pose pose_grip_glass_bottom;
    pose_grip_glass_bottom.position.x = -lateral_shift_glass;
    // pose_grip_glass_bottom.position.z = z_floor - table_height - grasp_height;
    pose_grip_glass_bottom.orientation = pose_ZH.orientation;
    
    // Pose to lift the glass (raise up)
    geometry_msgs::msg::Pose pose_lift_bottom = pose_grip_glass_bottom;
    // pose_lift_bottom.position.z = z_floor - table_height - (total_glass_height - glass_offset);
    
    // Pose for tapping
    geometry_msgs::msg::Pose pose_tap_bottom;
    pose_tap_bottom.position.z = glass_offset + 0.005;

    // Mobile Platform
    moveit_msgs::msg::CollisionObject mobile_platform;
    mobile_platform.id = "mobile_platform";
    mobile_platform.header.frame_id = "world";
    
    shape_msgs::msg::SolidPrimitive mobile_platform_primitive;
    mobile_platform_primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    mobile_platform_primitive.dimensions = {depth_platform, width_platform, height_platform};
    mobile_platform.primitives.push_back(mobile_platform_primitive);
    
    geometry_msgs::msg::Pose mobile_platform_pose;
    mobile_platform_pose.position.z = -height_platform / 2;
    mobile_platform_pose.position.y = width_platform / 2 - lbr_y;
    mobile_platform_pose.position.x = 0; // Set x to 0 for centered placement
    mobile_platform_pose.orientation.w = 1;
    
    mobile_platform.operation = moveit_msgs::msg::CollisionObject::ADD;

    // Wall 1
    moveit_msgs::msg::CollisionObject wall1;
    wall1.id = "Wall_1";
    wall1.header.frame_id = "world";
    
    shape_msgs::msg::SolidPrimitive wall1_primitive;
    wall1_primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    wall1_primitive.dimensions = {wall_thickness, wall_width, wall_height};
    wall1.primitives.push_back(wall1_primitive);
    
    geometry_msgs::msg::Pose wall1_pose;
    wall1_pose.position.z = -height_platform + wall_height / 2;
    wall1_pose.position.y = width_platform / 2 - lbr_y;
    wall1_pose.position.x = -wall_thickness / 2 - lbr_x - wall_distance_1;
    wall1_pose.orientation.w = 1;
    
    wall1.primitive_poses.push_back(wall1_pose);
    wall1.operation = moveit_msgs::msg::CollisionObject::ADD;
    
    // Wall 2
    moveit_msgs::msg::CollisionObject wall2;
    wall2.id = "Wall_2";
    wall2.header.frame_id = "world";
    
    shape_msgs::msg::SolidPrimitive wall2_primitive;
    wall2_primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    wall2_primitive.dimensions = {wall_width_2, wall_thickness, wall_height};
    wall2.primitives.push_back(wall2_primitive);
    
    geometry_msgs::msg::Pose wall2_pose;
    wall2_pose.position.z = -height_platform + wall_height / 2;
    wall2_pose.position.y = -wall_thickness / 2 - lbr_y - wall_distance_2;
    wall2_pose.position.x = -lbr_x + depth_platform / 2;
    wall2_pose.orientation.w = 1;
    
    wall2.primitive_poses.push_back(wall2_pose);
    wall2.operation = moveit_msgs::msg::CollisionObject::ADD;

    moveit::planning_interface::PlanningSceneInterface psi;
    psi.applyCollisionObject(tap_handle);
    psi.applyCollisionObject(tap_unit1);
    psi.applyCollisionObject(tap_unit2);
    psi.applyCollisionObject(actuator1);
    psi.applyCollisionObject(table);
    // psi.applyCollisionObject(glass_base);
    // psi.applyCollisionObject(glass_stem);
    psi.applyCollisionObject(glasses[0]);
    psi.applyCollisionObject(mobile_platform);
    psi.applyCollisionObject(wall1);
    psi.applyCollisionObject(wall2);
}

int main(int argc, char** argv)
{
   rclcpp::init(argc, argv);

    rclcpp::NodeOptions node_options;
    
    node_options.automatically_declare_parameters_from_overrides(true);
    auto update_planning_scene_node = rclcpp::Node::make_shared("update_planning_scene_node", node_options);
      rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(update_planning_scene_node);
    std::thread([&executor]()
                { executor.spin(); })
        .detach();

  loadEnvironment();
  // Sleep for a while to let the planning scene update
  rclcpp::sleep_for(std::chrono::seconds(15));

  rclcpp::shutdown();
    return 0;
}
