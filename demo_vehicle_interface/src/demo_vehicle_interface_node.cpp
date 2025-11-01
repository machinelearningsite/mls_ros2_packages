#include "demo_vehicle_interface/demo_vehicle_interface_node.hpp"


VehicleInterface::VehicleInterface(rclcpp::NodeOptions options): Node("demo_vehicle_interface_pubisher",options)
{
    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    RCLCPP_INFO(this->get_logger(),"Starting state publisher");
    loop_rate_=std::make_shared<rclcpp::Rate>(33ms);
    timer_=this->create_wall_timer(33ms,std::bind(&VehicleInterface::publish,this));
}

void VehicleInterface::publish()
{
    // create the necessary messages
    geometry_msgs::msg::TransformStamped t;
    sensor_msgs::msg::JointState joint_state;

    // add time stamp
    joint_state.header.stamp=this->get_clock()->now();
    // Specify joints' name which are defined in the r2d2.urdf.xml and their content
    joint_state.name={"joint_rear_left_wheel","joint_rear_right_wheel"};
    joint_state.position={joint_rear_left_wheel, joint_rear_right_wheel};

    // add time stamp
    t.header.stamp=this->get_clock()->now();
    // specify the father and child frame

    // odom is the base coordinate system of tf2
    t.header.frame_id="base_link";
    // axis is defined in r2d2.urdf.xml file and it is the base coordinate of model
    t.child_frame_id="base_link";

    // add translation change
    t.transform.translation.x=cos(joint_rear_left_wheel)*2;
    t.transform.translation.y=sin(joint_rear_left_wheel)*2;
    t.transform.translation.z=0.7;
    tf2::Quaternion q;
    // euler angle into Quanternion and add rotation change
    q.setRPY(0,0,joint_rear_left_wheel+M_PI/2);
    t.transform.rotation.x=q.x();
    t.transform.rotation.y=q.y();
    t.transform.rotation.z=q.z();
    t.transform.rotation.w=q.w();

    joint_rear_left_wheel+=degree;  // Increment by 1 degree (in radians)
    joint_rear_right_wheel+=degree;    // Change angle at a slower pace

    // send message
    broadcaster->sendTransform(t);
    joint_pub_->publish(joint_state);

    RCLCPP_INFO(this->get_logger(),"Publishing joint state");
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VehicleInterface>());
    rclcpp::shutdown();
    return 0;
}