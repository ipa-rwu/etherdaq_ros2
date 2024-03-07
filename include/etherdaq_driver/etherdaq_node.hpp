#ifndef ETHERDAQ_NODE_HPP
#define ETHERDAQ_NODE_HPP

#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/empty.hpp>

#include "etherdaq_driver/etherdaq_driver.hpp"  // Adjust the include path based on your project setup

class EtherDAQNode : public rclcpp::Node {
 public:
  EtherDAQNode();
  void initializeEtherDAQDriver();

 private:
  void zeroFunction(const std_msgs::msg::Bool::SharedPtr msg);
  void tareCallback(const std_srvs::srv::Empty::Request::SharedPtr request,
                    std_srvs::srv::Empty::Response::SharedPtr response);
  void publishDiagnostics();

  std::unique_ptr<optoforce_etherdaq_ros2_driver::EtherDAQDriver>
      etherdaq_driver_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
      diag_pub_;  // Diagnostic publisher

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr zero_sub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr tare_service_;
  rclcpp::TimerBase::SharedPtr
      data_timer_;  // Timer for checking/publishing new data
  rclcpp::TimerBase::SharedPtr diag_timer_;  // Timer for publishing diagnostics

  std::string address_;
  int rate_;
  int filter_;
  std::string frame_id_;
  double T_lowpass_;
  bool tare_as_topic_;
};

#endif  // ETHERDAQ_NODE_HPP
