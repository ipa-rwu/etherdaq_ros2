/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, OptoForce, Ltd.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the OptoForce nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <chrono>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

class EtherDAQSubscriber : public rclcpp::Node {
 public:
  EtherDAQSubscriber() : Node("etherdaq_subscriber"), zeroing(true) {
    // Initialize subscribers
    sub_raw = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
        "ethdaq_data_raw", 10,
        std::bind(&EtherDAQSubscriber::chatterCallback, this, _1));
    sub_new = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
        "ethdaq_data", 10,
        std::bind(&EtherDAQSubscriber::chatterCallback, this, _1));

    // Initialize publisher
    zero_pub = this->create_publisher<std_msgs::msg::Bool>("ethdaq_zero", 10);

    // Initialize timer
    timer_ = this->create_wall_timer(
        10ms, std::bind(&EtherDAQSubscriber::timerCallback, this));

    // Initialize zeroing timer
    zeroing_timer_ = this->create_wall_timer(
        10s, std::bind(&EtherDAQSubscriber::zeroingCallback, this));
  }

 private:
  void chatterCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
    auto currentTime = this->now();

    if (lastTime.seconds() == 0) {  // Initial case
      lastTime = currentTime;
    }
    if (overallTime.seconds() == 0) {
      overallTime = currentTime;
    }

    auto duration = currentTime - lastTime;
    accDuration = accDuration + duration;
    double durationTime =
        duration.seconds() * 1000.0;  // Convert to milliseconds
    double frequency = 0.0;
    ++packetCount;
    if (accDuration > rclcpp::Duration(1.0s)) {
      accDuration = rclcpp::Duration(0.0s);
      packetCount = 0;
    }

    if (accDuration != rclcpp::Duration(0.0s)) {
      frequency = static_cast<double>(packetCount) / accDuration.seconds();
    }

    lastTime = currentTime;

    RCLCPP_INFO(this->get_logger(),
                "%u Fx:%.2f Fy:%.2f Fz:%.2f Tx:%.2f Ty:%.2f Tz:%.2f T:%.2f ms "
                "S: %.2f Hz\r\n",
                msg->header.stamp.sec, msg->wrench.force.x, msg->wrench.force.y,
                msg->wrench.force.z, msg->wrench.torque.x, msg->wrench.torque.y,
                msg->wrench.torque.z, durationTime, frequency);
  }

  void timerCallback() {
    // Example periodic callback if needed
  }

  void zeroingCallback() {
    std_msgs::msg::Bool z;
    z.data = zeroing;
    zero_pub->publish(z);
    zeroing = !zeroing;
  }

  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_raw,
      sub_new;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr zero_pub;
  rclcpp::TimerBase::SharedPtr timer_, zeroing_timer_;
  rclcpp::Time lastTime, overallTime;
  rclcpp::Duration accDuration = 0s;
  unsigned int packetCount = 0;
  bool zeroing;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EtherDAQSubscriber>());
  rclcpp::shutdown();
  return 0;
}
