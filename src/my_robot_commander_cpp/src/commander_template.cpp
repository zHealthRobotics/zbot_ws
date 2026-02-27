#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <example_interfaces/msg/float64_multi_array.hpp>
#include <my_robot_interfaces/msg/pose_command.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/bool.hpp>


using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using FloatArray = example_interfaces::msg::Float64MultiArray;
using PoseCmd = my_robot_interfaces::msg::PoseCommand;
using namespace std::placeholders;

class Commander
{
public:
    Commander(std::shared_ptr<rclcpp::Node> node)
    {
        node_ = node;
        arm_ = std::make_shared<MoveGroupInterface>(node_, "arm");
        arm_->setMaxVelocityScalingFactor(1.0);
        arm_->setMaxAccelerationScalingFactor(1.0);

        joint_cmd_sub_ = node_->create_subscription<FloatArray>(
            "joint_command", 10,
            std::bind(&Commander::jointCmdCallback, this, _1));

        pose_cmd_sub_ = node_->create_subscription<PoseCmd>(
            "pose_command", 10,
            std::bind(&Commander::poseCmdCallback, this, _1));

        // ✅ NEW: Position-only command subscriber
        position_cmd_sub_ = node_->create_subscription<geometry_msgs::msg::Point>(
            "position_command", 10,
            std::bind(&Commander::positionCmdCallback, this, _1));
            
        lock_orientation_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
            "/lock_orientation", 10,
            std::bind(&Commander::lockOrientationCallback, this, _1));

        clear_constraints_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
            "/clear_constraints", 10,
            std::bind(&Commander::clearConstraintsCallback, this, _1));
                    
                    
        execution_done_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/commander/execution_done", 10);
    
    }

    void goToNamedTarget(const std::string &name)
    {
        arm_->setStartStateToCurrentState();
        arm_->setNamedTarget(name);
        planAndExecute(arm_);
    }

    void goToJointTarget(const std::vector<double> &joints)
    {
        arm_->setStartStateToCurrentState();
        arm_->setJointValueTarget(joints);
        planAndExecute(arm_);
    }

    // ✅ Position-only motion
    void goToPositionTarget(double x, double y, double z)
    {
        arm_->setStartStateToCurrentState();
        arm_->clearPoseTargets(); 
        arm_->setPositionTarget(x, y, z);
        planAndExecute(arm_);
    }

    void goToPoseTarget(double x, double y, double z,
                        double roll, double pitch, double yaw,
                        bool cartesian_path = false)
    {
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        q.normalize();

        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.header.frame_id = "torso_link";
        target_pose.pose.position.x = x;
        target_pose.pose.position.y = y;
        target_pose.pose.position.z = z;
        target_pose.pose.orientation.x = q.x();
        target_pose.pose.orientation.y = q.y();
        target_pose.pose.orientation.z = q.z();
        target_pose.pose.orientation.w = q.w();

        arm_->setStartStateToCurrentState();
        
        arm_->clearPoseTargets(); 

        if (!cartesian_path)
        {
            arm_->setPoseTarget(target_pose);
            planAndExecute(arm_);
        }
        else
        {
            std::vector<geometry_msgs::msg::Pose> waypoints;
            waypoints.push_back(target_pose.pose);
            moveit_msgs::msg::RobotTrajectory trajectory;

            double eef_step = 0.01;
            double jump_threshold = 0.0;
            bool avoid_collisions = true;

            double fraction = arm_->computeCartesianPath(
                waypoints, eef_step, jump_threshold, trajectory, avoid_collisions);

            if (fraction > 0.4)
            {
                arm_->execute(trajectory);            
                std_msgs::msg::Bool msg;
                msg.data = true;
                execution_done_pub_->publish(msg);    
            }
        }
    }

    void lockEndEffectorOrientation(double tol = 1.2)
    {
        moveit_msgs::msg::OrientationConstraint ocm;

        ocm.link_name = arm_->getEndEffectorLink();
        ocm.header.frame_id = "torso_link";

        auto pose = arm_->getCurrentPose().pose;
        ocm.orientation = pose.orientation;

        ocm.absolute_x_axis_tolerance = tol;
        ocm.absolute_y_axis_tolerance = tol;
        ocm.absolute_z_axis_tolerance = M_PI;  // allow yaw

        ocm.weight = 1.0;

        moveit_msgs::msg::Constraints constraints;
        constraints.orientation_constraints.push_back(ocm);

        arm_->setPathConstraints(constraints);
    }


private:
    void planAndExecute(const std::shared_ptr<MoveGroupInterface> &interface)
    {
        MoveGroupInterface::Plan plan;
        bool success =
            (interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success)
        {
            interface->execute(plan);
            std_msgs::msg::Bool msg;
            msg.data = true;
            execution_done_pub_->publish(msg);
        }
    }

    void jointCmdCallback(const FloatArray &msg)
    {
        if (msg.data.size() == 7)
        {
            goToJointTarget(msg.data);
        }
    }

    void poseCmdCallback(const PoseCmd &msg)
    {
        goToPoseTarget(
            msg.x, msg.y, msg.z,
            msg.roll, msg.pitch, msg.yaw,
            msg.cartesian_path);
    }

    // ✅ NEW: Position command callback
    void positionCmdCallback(const geometry_msgs::msg::Point &msg)
    {
        goToPositionTarget(msg.x, msg.y, msg.z);
    }
    
    void lockOrientationCallback(const std_msgs::msg::Bool &msg)
    {
        if (!msg.data) return;

        lockEndEffectorOrientation();
        RCLCPP_INFO(node_->get_logger(), "End-effector orientation LOCKED");
    }

    void clearConstraintsCallback(const std_msgs::msg::Bool &msg)
    {
        if (!msg.data) return;

        arm_->clearPathConstraints();
        RCLCPP_INFO(node_->get_logger(), "Path constraints CLEARED");
    }
        

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<MoveGroupInterface> arm_;

    rclcpp::Subscription<FloatArray>::SharedPtr joint_cmd_sub_;
    rclcpp::Subscription<PoseCmd>::SharedPtr pose_cmd_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr position_cmd_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr execution_done_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr lock_orientation_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr clear_constraints_sub_;


};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("commander");
    Commander commander(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

