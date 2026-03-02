#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <example_interfaces/msg/float64_multi_array.hpp>
#include <my_robot_interfaces/msg/pose_command.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2/LinearMath/Quaternion.h>

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

        // 🔹 Two Move Groups
        left_arm_  = std::make_shared<MoveGroupInterface>(node_, "left_arm");
        right_arm_ = std::make_shared<MoveGroupInterface>(node_, "right_arm");

        left_arm_->setMaxVelocityScalingFactor(1.0);
        left_arm_->setMaxAccelerationScalingFactor(1.0);

        right_arm_->setMaxVelocityScalingFactor(1.0);
        right_arm_->setMaxAccelerationScalingFactor(1.0);

        joint_cmd_sub_ = node_->create_subscription<FloatArray>(
            "joint_command", 10,
            std::bind(&Commander::jointCmdCallback, this, _1));

        pose_cmd_sub_ = node_->create_subscription<PoseCmd>(
            "pose_command", 10,
            std::bind(&Commander::poseCmdCallback, this, _1));

        position_cmd_sub_ = node_->create_subscription<PoseCmd>(
            "position_command", 10,
            std::bind(&Commander::positionCmdCallback, this, _1));

        lock_orientation_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
            "/lock_orientation", 10,
            std::bind(&Commander::lockOrientationCallback, this, _1));

        clear_constraints_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
            "/clear_constraints", 10,
            std::bind(&Commander::clearConstraintsCallback, this, _1));

        execution_done_pub_ =
            node_->create_publisher<std_msgs::msg::Bool>("/commander/execution_done", 10);
    }

private:

    // 🔹 Helper to select arm
    std::shared_ptr<MoveGroupInterface> getArm(const std::string &arm_name)
    {
        if (arm_name == "left")
            return left_arm_;
        else
            return right_arm_;
    }

    void planAndExecute(const std::shared_ptr<MoveGroupInterface> &arm)
    {
        MoveGroupInterface::Plan plan;
        bool success =
            (arm->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success)
        {
            arm->execute(plan);
            std_msgs::msg::Bool msg;
            msg.data = true;
            execution_done_pub_->publish(msg);
        }
    }

    // 🔹 Joint Target
    void goToJointTarget(std::shared_ptr<MoveGroupInterface> arm,
                         const std::vector<double> &joints)
    {
        arm->setStartStateToCurrentState();
        arm->setJointValueTarget(joints);
        planAndExecute(arm);
    }

    // 🔹 Position Target
    void goToPositionTarget(std::shared_ptr<MoveGroupInterface> arm,
                            double x, double y, double z)
    {
        arm->setStartStateToCurrentState();
        arm->clearPoseTargets();
        arm->setPositionTarget(x, y, z);
        planAndExecute(arm);
    }

    // 🔹 Pose Target
    void goToPoseTarget(std::shared_ptr<MoveGroupInterface> arm,
                        double x, double y, double z,
                        double roll, double pitch, double yaw,
                        bool cartesian_path)
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

        arm->setStartStateToCurrentState();
        arm->clearPoseTargets();

        if (!cartesian_path)
        {
            arm->setPoseTarget(target_pose);
            planAndExecute(arm);
        }
        else
        {
            std::vector<geometry_msgs::msg::Pose> waypoints;
            waypoints.push_back(target_pose.pose);

            moveit_msgs::msg::RobotTrajectory trajectory;

            double fraction = arm->computeCartesianPath(
                waypoints, 0.01, 0.0, trajectory, true);

            if (fraction > 0.4)
            {
                arm->execute(trajectory);
                std_msgs::msg::Bool msg;
                msg.data = true;
                execution_done_pub_->publish(msg);
            }
        }
    }

    // 🔹 Callbacks

    void jointCmdCallback(const FloatArray &msg)
    {
        if (msg.data.size() != 8) return;

        std::string arm_name = (msg.data[0] == 0) ? "left" : "right";
        std::vector<double> joints(msg.data.begin() + 1, msg.data.end());

        auto arm = getArm(arm_name);
        goToJointTarget(arm, joints);
    }

    void poseCmdCallback(const PoseCmd &msg)
    {
        auto arm = getArm(msg.arm);

        goToPoseTarget(
            arm,
            msg.x, msg.y, msg.z,
            msg.roll, msg.pitch, msg.yaw,
            msg.cartesian_path);
    }

    void positionCmdCallback(const PoseCmd &msg)
    {
        auto arm = getArm(msg.arm);
        goToPositionTarget(arm, msg.x, msg.y, msg.z);
    }

    void lockOrientationCallback(const std_msgs::msg::Bool &msg)
    {
        if (!msg.data) return;

        auto arm = left_arm_; // example lock left by default

        moveit_msgs::msg::OrientationConstraint ocm;
        ocm.link_name = arm->getEndEffectorLink();
        ocm.header.frame_id = "torso_link";
        ocm.orientation = arm->getCurrentPose().pose.orientation;

        ocm.absolute_x_axis_tolerance = 1.2;
        ocm.absolute_y_axis_tolerance = 1.2;
        ocm.absolute_z_axis_tolerance = M_PI;
        ocm.weight = 1.0;

        moveit_msgs::msg::Constraints constraints;
        constraints.orientation_constraints.push_back(ocm);

        arm->setPathConstraints(constraints);

        RCLCPP_INFO(node_->get_logger(), "Orientation LOCKED");
    }

    void clearConstraintsCallback(const std_msgs::msg::Bool &msg)
    {
        if (!msg.data) return;

        left_arm_->clearPathConstraints();
        right_arm_->clearPathConstraints();

        RCLCPP_INFO(node_->get_logger(), "Constraints CLEARED");
    }

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<MoveGroupInterface> left_arm_;
    std::shared_ptr<MoveGroupInterface> right_arm_;

    rclcpp::Subscription<FloatArray>::SharedPtr joint_cmd_sub_;
    rclcpp::Subscription<PoseCmd>::SharedPtr pose_cmd_sub_;
    rclcpp::Subscription<PoseCmd>::SharedPtr position_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr lock_orientation_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr clear_constraints_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr execution_done_pub_;
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
