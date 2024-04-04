#include <memory>

#include <rclcpp/rclcpp.hpp>
// #include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
// #include <moveit/moveit_commander.h>
#include <Eigen/Geometry>
#include "std_msgs/msg/string.hpp"
#include <string>


// rclcpp::Logger* logger;

// moveit::planning_interface::MoveGroupInterface move_group_interface;

// Declare a global pointer for MoveGroupInterface for ease of use in this example
std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface;
std::shared_ptr<rclcpp::Node> node;
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;


// cartesian drill function
int drill(const geometry_msgs::msg::PoseStamped msg){

  RCLCPP_ERROR(rclcpp::get_logger("static_obstacles"), "in drill function");
  // print the received message as a string
  move_group_interface->setStartStateToCurrentState();
  const geometry_msgs::msg::Pose pose = msg.pose;
  RCLCPP_INFO_STREAM(rclcpp::get_logger("static_obstacles"),"Position: [" << pose.position.x << ", " << pose.position.y << ", " << pose.position.z << "] "
                << "Orientation: [" << pose.orientation.x << ", " << pose.orientation.y << ", " << pose.orientation.z << ", " << pose.orientation.w << "]");
  
  //cartesian planning
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  moveit_msgs::msg::RobotTrajectory trajectory;
  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(pose);


  //calculate the final drill position
  // Convert the quaternion to a rotation matrix
  Eigen::Quaterniond quat(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  Eigen::Matrix3d rotation_matrix = quat.toRotationMatrix();

  // Create a vector representing the direction to move in
  float drill_length = 0.1;
  Eigen::Vector3d direction(0, 0, drill_length);

  // Multiply the rotation matrix by the direction vector
  Eigen::Vector3d result = rotation_matrix * direction;

  // Add the result to the original position to get the new position
  geometry_msgs::msg::Pose new_msg;
  new_msg.position.x = pose.position.x + result(0);
  new_msg.position.y = pose.position.y + result(1);
  new_msg.position.z = pose.position.z + result(2);

  // The orientation remains the same
  new_msg.orientation = pose.orientation;

  RCLCPP_INFO_STREAM(rclcpp::get_logger("static_obstacles"),"Position: [" << new_msg.position.x << ", " << new_msg.position.y << ", " << new_msg.position.z << "] "
                << "Orientation: [" << new_msg.orientation.x << ", " << new_msg.orientation.y << ", " << new_msg.orientation.z << ", " << new_msg.orientation.w << "]");
  waypoints.push_back(new_msg);

  move_group_interface->setMaxVelocityScalingFactor	(	0.1)	;
  double fraction = move_group_interface->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  
  move_group_interface->execute(trajectory);
  // move_group_interface->setMaxVelocityScalingFactor	(	1 )	;
  RCLCPP_ERROR(rclcpp::get_logger("static_obstacles"), "done");

  return 0;

}

void topic_callback(const geometry_msgs::msg::Pose msg)
{
  RCLCPP_ERROR(rclcpp::get_logger("static_obstacles"), "msg recd");
  if (!move_group_interface) {
    RCLCPP_ERROR(rclcpp::get_logger("static_obstacles"), "MoveGroupInterface is not initialized.");
    return;
  }
  //cartesian
  // const double jump_threshold = 0.0;
  // const double eef_step = 0.01;
  // moveit_msgs::msg::RobotTrajectory trajectory;
  // std::vector<geometry_msgs::msg::Pose> waypoints;
  // waypoints.push_back(msg);
  // double fraction = move_group_interface->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);  
  // move_group_interface->execute(trajectory);
  // joint space planning
  // move_group_interface->setStartStateToCurrentState();

  geometry_msgs::msg::PoseStamped gt_pose;
  gt_pose.header.frame_id = "world";
  gt_pose.header.stamp = node->get_clock()->now();
  gt_pose.pose = msg;

  move_group_interface->setPoseTarget(gt_pose);
  // float joints[] = {0.0, -30.0,0.5,0.0,0.0,-30,0.0};
  // std::vector<double> joint_group_positions_arm;
  // joint_group_positions_arm[0] = 0.00;  // Shoulder Pan
  // joint_group_positions_arm[1] = -30.0;  // Shoulder Lift
  // joint_group_positions_arm[2] = 0.0;  // Elbow
  // joint_group_positions_arm[3] = 0.0;  // Wrist 1
  // joint_group_positions_arm[4] = 0.0;  // Wrist 2
  // joint_group_positions_arm[5] = -30.0;  // Wrist 3
  // joint_group_positions_arm[6] = 0.0; 

  // move_group_interface->setJointValueTarget(msg, "link_tool");
  // move_group_interface->setMaxVelocityScalingFactor	(	0.1)	;


  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = static_cast<bool>(move_group_interface->plan(plan));

  if (success) {
    move_group_interface->execute(plan);
    RCLCPP_INFO(rclcpp::get_logger("static_obstacles"), "Plan was finally executed. Start Drill");
    auto message = std_msgs::msg::String();
      message.data = "drill" ;
    publisher_->publish(message);
    // publisher_->publish("drill");
    
    // publisher_->publish("drill");
    // publisher_->
    //put stop here after delay
    rclcpp::sleep_for(std::chrono::seconds(5));
    message.data = "stop" ;
    publisher_->publish(message);

    
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("static_obstacles"), "Planning failed!");
  }

  // drill(gt_pose);
}

int main(int argc, char * argv[])
{
  
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  node = std::make_shared<rclcpp::Node>(
    "static_obstacles",
    // "lbr",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  rclcpp::Logger logger = rclcpp::get_logger("static_obstacles");
  

  // Next step goes here
  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  RCLCPP_INFO(logger, "Creating MoveGroupInterface");
  const MoveGroupInterface::Options opt = MoveGroupInterface::Options("arm", "robot_description", "lbr");
  RCLCPP_INFO(logger, "The thingy returned by Options is %s", opt.move_group_namespace_.c_str());
  
  // move_group_interface = MoveGroupInterface(node, opt, std::shared_ptr<tf2_ros::Buffer>(), rclcpp::Duration(0, 0));
  move_group_interface = std::make_shared<MoveGroupInterface>(node, opt, std::shared_ptr<tf2_ros::Buffer>(), rclcpp::Duration(0, 0));

  // RCLCPP_INFO(logger, "The thingy returned by Options is %s", move_group_interface.);
  
  // auto move_group_interface = MoveGroupInterface(node, "arm");

  RCLCPP_INFO(logger, "Created");
  publisher_ = node->create_publisher<std_msgs::msg::String>("drill_commands", 10);


  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  // const moveit::core::JointModelGroup* joint_model_group = move_group_interface->getCurrentState()->getJointModelGroup("/lbr/arm");

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  RCLCPP_INFO(logger, "Planning frame: %s", move_group_interface->getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  RCLCPP_INFO(logger, "End effector link: %s", move_group_interface->getEndEffectorLink().c_str());

  // We can get a list of all the groups in the robot:
  RCLCPP_INFO(logger, "Available Planning Groups:");

  std::copy(move_group_interface->getJointModelGroupNames().begin(), move_group_interface->getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));




  std::string planner_plugin_name = "ompl_interface/OMPLPlanner";
  // move_group_interface->setPlannerParams("planning_plugin", planner_plugin_name);

  move_group_interface->setPlannerId("RRTstarkConfigDefault");



  // Assume move_group_interface is already defined and initialized
  // std::string frame_id = move_group_interface.getPlanningFrame();

  std::string frame_id = "world";


  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = frame_id;


  collision_object.id = "box1";

  shape_msgs::msg::SolidPrimitive primitive;
  // Define the size of the box in meters
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = 0.2;
  primitive.dimensions[primitive.BOX_Y] = 0.1;
  primitive.dimensions[primitive.BOX_Z] = 0.5;

  // Define the pose of the box (relative to the frame_id)
  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0;
  box_pose.position.y = 0.3;
  box_pose.position.z = 0.25;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;


  collision_object.id = "box2";

  // shape_msgs::msg::SolidPrimitive primitive;
  // Define the size of the box in meters
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = 0.05;
  primitive.dimensions[primitive.BOX_Y] = 1.1;
  primitive.dimensions[primitive.BOX_Z] = 1.5;

  // Define the pose of the box (relative to the frame_id)
  // geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.25;
  box_pose.position.y = 0;
  box_pose.position.z = 0.75;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

   collision_object.id = "table";

  // shape_msgs::msg::SolidPrimitive primitive;
  // Define the size of the box in meters
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = 1.1;
  primitive.dimensions[primitive.BOX_Y] = 1.1;
  primitive.dimensions[primitive.BOX_Z] = 0.02;

  // Define the pose of the box (relative to the frame_id)
  // geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = -0.25;
  box_pose.position.y = 0;
  box_pose.position.z = -0.02;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.applyCollisionObject(collision_object);

  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_ = node->create_subscription<geometry_msgs::msg::Pose>(
  "moveit_goal", 10, topic_callback);


  // RCLCPP_INFO(node->get_logger(), "Sleeping for 5 seconds...");
  // rclcpp::sleep_for(std::chrono::seconds(5));
  // RCLCPP_INFO(node->get_logger(), "Awake now!");
  // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
   

  // Shutdown ROS
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}