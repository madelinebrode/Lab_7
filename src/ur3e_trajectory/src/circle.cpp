#include "../include/circle.hpp"

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

    
    /*
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
