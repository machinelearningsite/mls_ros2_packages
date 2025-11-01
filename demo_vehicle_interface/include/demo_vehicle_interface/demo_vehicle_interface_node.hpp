#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <thread>
#include <chrono>

using namespace std::chrono;

class VehicleInterface : public rclcpp::Node
{
    public:
        explicit VehicleInterface(rclcpp::NodeOptions options = rclcpp::NodeOptions());

    private:
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
        std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster;
        rclcpp::Rate::SharedPtr loop_rate_;
        rclcpp::TimerBase::SharedPtr timer_;

        const double degree = M_PI/180.0;
        double tilt = 0.;
        double tinc = degree;
        double joint_front_left_wheel = 0.;
        double joint_front_right_wheel = 0.;
        double joint_rear_left_wheel = 0.;
        double joint_rear_right_wheel = 0.;
        double height = 0.;
        double hinc = 0.005;

        void publish();
};