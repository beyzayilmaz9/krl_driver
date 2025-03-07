#ifndef KRL_DRIVER_H
#define KRL_DRIVER_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int64.hpp>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <std_msgs/msg/int64.hpp>
#include <iostream>
#include <geometry_msgs/msg/twist.hpp>
#include <serial/serial.h>
#include <yaml-cpp/yaml.h>

class krlDriver : public rclcpp::Node {
public:
    krlDriver();
    void startThread();
    void readSerialData();
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    int calculateRpm(int dcount, double dt);
    void writeSerialData();
    void readConfig();
private:
    std::string serial_port_;
    int baud_rate_;
    int num_slots_;
    double min_publish_time_;
    int buffer_size_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr publisher_right_wheel;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr publisher_left_wheel;
    
    // Serial object as a member variable
    std::unique_ptr<serial::Serial> my_serial_;  // Changed to std::unique_ptr for better management
};






#endif  // KRL_DRIVER_H