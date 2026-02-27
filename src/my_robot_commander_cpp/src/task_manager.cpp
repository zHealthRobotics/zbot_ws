#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <example_interfaces/msg/float64_multi_array.hpp>

#include <my_robot_interfaces/msg/pose_command.hpp>

using PoseCmd = my_robot_interfaces::msg::PoseCommand;
using JointCmd = example_interfaces::msg::Float64MultiArray;

enum class TaskState
{
    IDLE,
    REACH,
    APPROACH,
    REACH_POUR,
    POUR,
    RESET_ORIENTATION,
    PLACE,
    RETURN,
    DONE
};

class TaskManager : public rclcpp::Node
{
public:
    TaskManager()
    : Node("task_manager"), state_(TaskState::IDLE)
    {
        target_sub_ = this->create_subscription<PoseCmd>(
            "/detected_target_pose", 10,
            std::bind(&TaskManager::targetCallback, this, std::placeholders::_1));

        execution_done_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/commander/execution_done", 10,
            std::bind(&TaskManager::executionDoneCallback, this, std::placeholders::_1));

        command_pub_ = this->create_publisher<PoseCmd>(
            "/pose_command", 10);

        joint_command_pub_ = this->create_publisher<JointCmd>(
            "/joint_command", 10);
            
        lock_orientation_pub_ =
            this->create_publisher<std_msgs::msg::Bool>("/lock_orientation", 10);

        clear_constraints_pub_ =
            this->create_publisher<std_msgs::msg::Bool>("/clear_constraints", 10);
                    
            

        RCLCPP_INFO(
            this->get_logger(),
            "Task Manager (REACH → APPROACH → REACH_POUR → POUR → PLACE → RETURN → DONE) started");
    }

private:
    // ---------------- TARGET ----------------

    void targetCallback(const PoseCmd::SharedPtr msg)
    {
        if (state_ != TaskState::IDLE)
            return;

        target_pose_ = *msg;

        PoseCmd cmd = target_pose_;
        cmd.cartesian_path = false;
        command_pub_->publish(cmd);

        state_ = TaskState::REACH;
        RCLCPP_INFO(this->get_logger(), "State → REACH");
    }

    // ---------------- EXECUTION FEEDBACK ----------------

    void executionDoneCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (!msg->data)
            return;

        if (state_ == TaskState::REACH)
        {
            RCLCPP_INFO(this->get_logger(), "Reach execution DONE");
            sendApproachCommand();
            state_ = TaskState::APPROACH;
            RCLCPP_INFO(this->get_logger(), "State → APPROACH");
        }
        else if (state_ == TaskState::APPROACH)
        {
            RCLCPP_INFO(this->get_logger(), "Approach execution DONE");

//            std_msgs::msg::Bool lock;
//            lock.data = true;
//            lock_orientation_pub_->publish(lock);
//            rclcpp::sleep_for(std::chrono::milliseconds(200));
            sendReachPourCommand();
            state_ = TaskState::REACH_POUR;
            RCLCPP_INFO(this->get_logger(), "State → REACH_POUR");
        }

        else if (state_ == TaskState::REACH_POUR)
        {
            RCLCPP_INFO(this->get_logger(), "Reach pour execution DONE");
            sendPourCommand();
            state_ = TaskState::POUR;
            RCLCPP_INFO(this->get_logger(), "State → POUR");
        }

        else if (state_ == TaskState::POUR)
        {
            RCLCPP_INFO(this->get_logger(), "Pour execution DONE");
            sendResetOrientationCommand();
            state_ = TaskState::RESET_ORIENTATION;
            RCLCPP_INFO(this->get_logger(), "State → RESET_ORIENTATION");
        }
        else if (state_ == TaskState::RESET_ORIENTATION)
        {
            RCLCPP_INFO(this->get_logger(), "Reset orientation DONE");
            
//            std_msgs::msg::Bool clear;
//            clear.data = true;
//            clear_constraints_pub_->publish(clear);                 
            
            sendPlaceCommand();
            state_ = TaskState::PLACE;
            RCLCPP_INFO(this->get_logger(), "State → PLACE");
        }


        else if (state_ == TaskState::PLACE)
        {
            RCLCPP_INFO(this->get_logger(), "Place execution DONE");       
            
            sendReturnCommand();
            state_ = TaskState::RETURN;
            RCLCPP_INFO(this->get_logger(), "State → RETURN");
        }
        else if (state_ == TaskState::RETURN)
        {
            RCLCPP_INFO(this->get_logger(), "Return execution DONE");
            state_ = TaskState::DONE;
            RCLCPP_INFO(this->get_logger(), "State → DONE");
        }

    }

    // ---------------- COMMANDS ----------------

    void sendApproachCommand()
    {
        approach_pose_ = target_pose_;
        approach_pose_.x += 0.05;              // 5 cm approach
        approach_pose_.cartesian_path = true;
        command_pub_->publish(approach_pose_);
    }

    void sendReachPourCommand()
    {
        reach_pour_pose_ = approach_pose_;

        reach_pour_pose_.x -= 0.10;
        reach_pour_pose_.y += 0.03;
        reach_pour_pose_.z += 0.05;

        reach_pour_pose_.roll  = approach_pose_.roll;
        reach_pour_pose_.pitch = approach_pose_.pitch;
        reach_pour_pose_.yaw   = approach_pose_.yaw;

        reach_pour_pose_.cartesian_path = false;

        command_pub_->publish(reach_pour_pose_);

        RCLCPP_INFO(this->get_logger(),
            "Reach-pour pose sent (orientation locked)");
    }



    void sendPourCommand()
    {
        PoseCmd cmd = reach_pour_pose_;

        // Controlled pour
        cmd.roll -= 0.9;

        cmd.cartesian_path = true;

        command_pub_->publish(cmd);

        RCLCPP_INFO(this->get_logger(),
            "Pour command sent (cartesian orientation rotation)");
    }

    void sendResetOrientationCommand()
    {
        PoseCmd cmd = reach_pour_pose_;

        // Undo the pour
        cmd.roll = reach_pour_pose_.roll;   // back to original upright
        cmd.pitch = reach_pour_pose_.pitch;
        cmd.yaw   = reach_pour_pose_.yaw;

        cmd.cartesian_path = true;

        command_pub_->publish(cmd);

        RCLCPP_INFO(this->get_logger(),
            "Reset orientation command sent (upright again)");
    }


    void sendPlaceCommand()
    {
        PoseCmd cmd = approach_pose_;
        cmd.cartesian_path = false;
        command_pub_->publish(cmd);

        RCLCPP_INFO(this->get_logger(), "Place pose sent (same as approach)");
    }
    
    void sendReturnCommand()
    {
        JointCmd joints;
        joints.data = {
            0.0,  // shoulder_pan
            0.0,  // shoulder_yaw
            0.0,  // shoulder_roll
            0.0,  // elbow
            0.0,  // wrist_yaw
            0.0,  // wrist_pitch
            0.0   // wrist_roll
        };

        joint_command_pub_->publish(joints);
        RCLCPP_INFO(this->get_logger(), "Return (zero joints) command sent");
    }
        

        // ---------------- MEMBERS ----------------

        TaskState state_;
        PoseCmd target_pose_;
        PoseCmd approach_pose_;
        PoseCmd reach_pour_pose_;


        rclcpp::Subscription<PoseCmd>::SharedPtr target_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr execution_done_sub_;
        rclcpp::Publisher<PoseCmd>::SharedPtr command_pub_;
        rclcpp::Publisher<JointCmd>::SharedPtr joint_command_pub_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr lock_orientation_pub_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr clear_constraints_pub_;

    };

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TaskManager>());
    rclcpp::shutdown();
    return 0;
}

