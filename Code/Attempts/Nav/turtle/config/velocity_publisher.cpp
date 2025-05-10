#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include <cmath>
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

class JointPublisher : public rclcpp::Node
{
public:
    JointPublisher()
    : Node("Joint_publisher"), count_(0) // Node name initialization and count initialization
    {
        publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/joint_trajectory_controller/joint_trajectory", 10); // Creating the publisher

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&JointPublisher::timer_callback, this)); // Timer set to 100 ms interval
    }

private:
    void timer_callback() // Timer callback that publishes joint trajectory
    {  
        auto message = trajectory_msgs::msg::JointTrajectory();
        // Names of joints to control
        message.joint_names.push_back("base_footprint_joint");
        message.joint_names.push_back("Front_right1_joint");
        message.joint_names.push_back("Front_right2_joint");
        message.joint_names.push_back("Back_right1_joint");
        message.joint_names.push_back("Back_right2_joint");
        message.joint_names.push_back("Front_left1_joint");
        message.joint_names.push_back("Front_left2_joint");
        message.joint_names.push_back("Back_left1_joint");
        message.joint_names.push_back("Back_left2_joint");

        auto point = trajectory_msgs::msg::JointTrajectoryPoint();
        double position1 = 1.0;
        double position2 = -position1;
        point.position.push_back(position1);
        point.position.push_back(position2);
        point.time_from_start = rclcpp::Duration::from_seconds(1.0);
        message.points.push_back(point);

        publisher_->publish(message);

        RCLCPP_INFO(this->get_logger(), "Publishing: '%f', '%f'", position1, position2);
        count_ += 1; // Increment count (for future use, e.g., debugging)
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
    size_t count_; // To count the number of messages published
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointPublisher>());
    rclcpp::shutdown();
    return 0;
}
