#include "../include/square.hpp"

int main(int argc, char **argv){
     // Setup ROS node
    ros::init(argc, argv, "square");
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::NodeHandle n;
    
    // Create PlanningOptions
    MoveitPlanning::PlanningOptions planning_options = MoveitPlanning::PlanningOptions();
    planning_options.num_attempts = 10;
    planning_options.allow_replanning = true;
    planning_options.set_planning_time = 30.0;
    planning_options.goal_position_tolerance = 0.01;
    planning_options.goal_orientation_tolerance = 0.01;
    planning_options.goal_joint_tolerance = 0.01;
    planning_options.velocity_scaling_factor = 0.1;
    planning_options.acceleration_scaling_factor = 0.4;

    // Create instance of MoveGroupInterface for given joint group
    moveit::planning_interface::MoveGroupInterface arm_move_group("manipulator");

    //Write your code for following the square trajectory here.
    moveit::planning_interface::MoveGroupInterface::Plan joint_plan;

    /*
    //vertical square
    std::map<std::string, double> joint_targets;

    joint_targets["elbow_joint"] = -31.0*3.14/180.0;
    joint_targets["shoulder_lift_joint"] = -23.0*3.14/180.0;
    joint_targets["shoulder_pan_joint"] = 38.0*3.14/180.0;
    joint_targets["wrist_1_joint"] = -126.0*3.14/180.0;
    joint_targets["wrist_2_joint"] = -39.0*3.14/180.0;
    joint_targets["wrist_3_joint"] = 0.0;


    bool joint_plan_success;
    joint_plan_success = ArmController::planToJointTargets(planning_options, arm_move_group, joint_plan, joint_targets);

    if(joint_plan_success){
        ROS_INFO("Moving to joint target");
        arm_move_group.execute(joint_plan);
    } 
    std::string reference_frame = "world";
    // Create instance of cartesian plan
    moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
    
    //vertical square

    // Get the start Pose
    geometry_msgs::Pose start_pose = arm_move_group.getCurrentPose().pose;

    geometry_msgs::Pose second_pose = start_pose;
    second_pose.position.x -= 0.18;
    //second_pose.position.y = end_pose.position.y;
    //second_pose.position.z += 0.1;

    geometry_msgs::Pose third_pose = second_pose;
    //third_pose.position.x -= 0.1;
    //third_pose.position.y -= 0.1;
    third_pose.position.z -= 0.18;

    geometry_msgs::Pose fourth_pose = third_pose;
    fourth_pose.position.x += 0.18;
    //third_pose.position.y += 0.2;
    //fourth_pose.position.z -= 0.1;


    // Define waypoints for the cartesian path
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(start_pose);
    waypoints.push_back(second_pose);

    waypoints.push_back(third_pose);

    
    waypoints.push_back(fourth_pose);

    waypoints.push_back(start_pose);

    moveit_msgs::RobotTrajectory trajectory;
    trajectory = ArmController::planCartesianPath(start_pose, waypoints, reference_frame, arm_move_group);

    n.setParam("/record_pose", true);
    arm_move_group.execute(trajectory);
    n.setParam("/record_pose", false);
    
    */
    
    //horizontal square

    std::map<std::string, double> joint_targets;

    joint_targets["elbow_joint"] = -0.93;
    joint_targets["shoulder_lift_joint"] = -2.67;
    joint_targets["shoulder_pan_joint"] = 0.0;
    joint_targets["wrist_1_joint"] = -1.12;
    joint_targets["wrist_2_joint"] = 4.71;
    joint_targets["wrist_3_joint"] = 0.0;


    bool joint_plan_success;
    joint_plan_success = ArmController::planToJointTargets(planning_options, arm_move_group, joint_plan, joint_targets);

    if(joint_plan_success){
        ROS_INFO("Moving to joint target");
        arm_move_group.execute(joint_plan);
    } 

    std::string reference_frame = arm_move_group.getPlanningFrame();
    // Create instance of cartesian plan
    moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;

    // Get the start Pose
    geometry_msgs::Pose start_pose = arm_move_group.getCurrentPose().pose;

    geometry_msgs::Pose second_pose = start_pose;
    second_pose.position.x = -start_pose.position.y;
    second_pose.position.y = start_pose.position.x;
    //second_pose.position.z += 0.1;

    geometry_msgs::Pose third_pose = start_pose;
    third_pose.position.x = -start_pose.position.x;
    third_pose.position.y = -start_pose.position.y;

    geometry_msgs::Pose fourth_pose = start_pose;
    fourth_pose.position.x = start_pose.position.y;
    fourth_pose.position.y = -start_pose.position.x;


    // Define waypoints for the cartesian path
    std::vector<geometry_msgs::Pose> waypoints;
    // waypoints.push_back(start_pose);
    
    waypoints.push_back(start_pose);
    
    waypoints.push_back(second_pose);

    waypoints.push_back(third_pose);

    
    waypoints.push_back(fourth_pose);

    waypoints.push_back(start_pose);


    moveit_msgs::RobotTrajectory trajectory;
    trajectory = ArmController::planCartesianPath(start_pose, waypoints, reference_frame, arm_move_group);

    n.setParam("/record_pose", true);
    arm_move_group.execute(trajectory);
    n.setParam("/record_pose", false);

    
}
