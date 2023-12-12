#include "../include/circle.hpp"
#include <math.h>

int main(int argc, char **argv)
{
     // Setup ROS node
    ros::init(argc, argv, "circle");
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::NodeHandle n;
    
    // Create PlanningOptions
    MoveitPlanning::PlanningOptions planning_options =
    MoveitPlanning::PlanningOptions();
    planning_options.num_attempts = 10;
    planning_options.allow_replanning = true;
    planning_options.set_planning_time = 30.0;
    planning_options.goal_position_tolerance = 0.01;
    planning_options.goal_orientation_tolerance = 0.01;
    planning_options.goal_joint_tolerance = 0.01;
    planning_options.velocity_scaling_factor = 0.1;
    planning_options.acceleration_scaling_factor = 0.1;

    // Create instance of MoveGroupInterface for given joint group
    moveit::planning_interface::MoveGroupInterface arm_move_group("manipulator");


    //Write your code for following the circle trajectory here.
    moveit::planning_interface::MoveGroupInterface::Plan joint_plan;
   
    // Vertical

    std::map<std::string, double> joint_targets;
    joint_targets["elbow_joint"] = 122.0*3.14/180.0;
    joint_targets["shoulder_lift_joint"] = -206.0*3.14/180.0;
    joint_targets["shoulder_pan_joint"] = -57.0*3.14/180.0;
    joint_targets["wrist_1_joint"] = -96.0*3.14/180.0;
    joint_targets["wrist_2_joint"] = 57.0*3.14/180.0;
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

    int int_sides = 20;
    double double_sides = 20.0;
    double r = 0.1;

    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose start_pose = arm_move_group.getCurrentPose().pose;

    geometry_msgs::Pose new_pose = start_pose;

    for(int i = 1;i<=int_sides;i++){
        
        new_pose.position.x = start_pose.position.x + r*(-sin((i/double_sides)*6.28));
        new_pose.position.z = start_pose.position.z + r*(1-cos((i/double_sides)*6.28));

        waypoints.push_back(new_pose);
    }
    
    moveit_msgs::RobotTrajectory trajectory;
    trajectory = ArmController::planCartesianPath(start_pose, waypoints, reference_frame, arm_move_group);

    n.setParam("/record_pose", true);
    arm_move_group.execute(trajectory);
    n.setParam("/record_pose", false);


    /*
    //horizontal    
    std::map<std::string, double> joint_targets;
    joint_targets["elbow_joint"] = 0.0;
    joint_targets["shoulder_lift_joint"] = -0.0;
    joint_targets["shoulder_pan_joint"] = 0.0;
    joint_targets["wrist_1_joint"] = -1.57;
    joint_targets["wrist_2_joint"] = -1.57;
    joint_targets["wrist_3_joint"] = 0.0;

    bool joint_plan_success;
    joint_plan_success = ArmController::planToJointTargets(planning_options, arm_move_group, joint_plan, joint_targets);

    if(joint_plan_success){
        ROS_INFO("Moving to joint target");
        arm_move_group.execute(joint_plan);
    } 

    joint_targets["shoulder_pan_joint"] += 4*1.57;

    joint_plan_success = ArmController::planToJointTargets(planning_options, arm_move_group, joint_plan, joint_targets);

    if(joint_plan_success){
        ROS_INFO("Moving to joint target");
        arm_move_group.execute(joint_plan);
    }

    
    
    for(int i = 0;i<9;i++){
        joint_targets["shoulder_pan_joint"] += 0.628;

        bool joint_plan_success;
        joint_plan_success = ArmController::planToJointTargets(planning_options, arm_move_group, joint_plan, joint_targets);

        if(joint_plan_success){
        ROS_INFO("Moving to joint target");
        arm_move_group.execute(joint_plan);
    } 
    }

    */
    
}
