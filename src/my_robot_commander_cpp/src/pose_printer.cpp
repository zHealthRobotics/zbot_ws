#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class EndEffectorTF : public rclcpp::Node
{
public:
  EndEffectorTF() : Node("ee_tf_node")
  {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&EndEffectorTF::printPose, this));
  }

private:
  void printPose()
  {
    try
    {
      auto tf = tf_buffer_->lookupTransform(
        "torso_link",        // reference
        "wrist_yaw_link",    // end effector
        tf2::TimePointZero  // latest
      );

      RCLCPP_INFO(this->get_logger(),
        "EE pose:\n"
        "Position: [%.3f %.3f %.3f]\n"
        "Orientation: [%.3f %.3f %.3f %.3f]",
        tf.transform.translation.x,
        tf.transform.translation.y,
        tf.transform.translation.z,
        tf.transform.rotation.x,
        tf.transform.rotation.y,
        tf.transform.rotation.z,
        tf.transform.rotation.w);
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_WARN(this->get_logger(), "%s", ex.what());
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EndEffectorTF>());
  rclcpp::shutdown();
  return 0;
}

