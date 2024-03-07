#include "etherdaq_driver/etherdaq_node.hpp"

EtherDAQNode::EtherDAQNode() : Node("etherdaq_node") {
  // Declare parameters
  address_ = this->declare_parameter<std::string>("address", "192.168.1.1");
  rate_ = this->declare_parameter<int>("rate", 1000);  // Hz
  filter_ = this->declare_parameter<int>("filter", 4);
  frame_id_ = this->declare_parameter<std::string>("frame_id", "etherdaq_link");
  T_lowpass_ = this->declare_parameter<double>("T_lowpass", 0.2);
  tare_as_topic_ = this->declare_parameter<bool>("tare_as_topic", true);

  // // Initialize EtherDAQ driver
  // etherdaq_driver_ =
  //     std::make_unique<optoforce_etherdaq_ros2_driver::EtherDAQDriver>(
  //         shared_from_this(), address_, rate_, filter_, T_lowpass_);

  // Initialize publisher
  wrench_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>(
      "etherdaq_data", 10);

  diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", 10);

  // Initialize subscriber if tare_as_topic is true
  if (tare_as_topic_) {
    zero_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "etherdaq_zero", 10,
        std::bind(&EtherDAQNode::zeroFunction, this, std::placeholders::_1));
  }

  // Initialize service for taring
  tare_service_ = this->create_service<std_srvs::srv::Empty>(
      "tare", std::bind(&EtherDAQNode::tareCallback, this,
                        std::placeholders::_1, std::placeholders::_2));

  // Timer for checking and publishing new data
  data_timer_ = this->create_wall_timer(
      1ms,  // Adjust according to your data rate needs
      [this]() {
        geometry_msgs::msg::WrenchStamped data;
        if (etherdaq_driver_->waitForNewData()) {
          etherdaq_driver_->getData(data);
          // packetCount++;
          data.header.frame_id =
              frame_id_;  // Ensure frame_id is a member variable
          wrench_pub_->publish(data);
        }
      });

  // Timer for publishing diagnostics
  diag_timer_ =
      this->create_wall_timer(1ms, [this]() { this->publishDiagnostics(); });
}

void EtherDAQNode::zeroFunction(const std_msgs::msg::Bool::SharedPtr msg) {
  if (msg->data) {
    etherdaq_driver_->doZero();
  } else {
    etherdaq_driver_->doUnzero();
  }
}

void EtherDAQNode::tareCallback(
    const std_srvs::srv::Empty::Request::SharedPtr request,
    std_srvs::srv::Empty::Response::SharedPtr response) {
  etherdaq_driver_->doZero();
}

void EtherDAQNode::initializeEtherDAQDriver() {
  // Ensure this method is called after the Node is fully constructed
  etherdaq_driver_ =
      std::make_unique<optoforce_etherdaq_ros2_driver::EtherDAQDriver>(
          shared_from_this(), address_, rate_, filter_, T_lowpass_);
  // Continue with the rest of the initialization
}

void EtherDAQNode::publishDiagnostics() {
  diagnostic_msgs::msg::DiagnosticArray diag_array;

  diagnostic_updater::DiagnosticStatusWrapper diag_status;
  etherdaq_driver_->diagnostics(diag_status);
  diag_array.status.push_back(diag_status);

  diag_array.header.stamp = this->now();
  diag_pub_->publish(diag_array);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EtherDAQNode>();
  node->initializeEtherDAQDriver();
  rclcpp::executors::MultiThreadedExecutor executor;
  // Add your node to the executor
  executor.add_node(node);

  // Spin the executor to process callbacks
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
