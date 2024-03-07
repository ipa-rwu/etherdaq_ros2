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

#include "etherdaq_driver/etherdaq_driver.hpp"

#include <stdint.h>

#include <exception>

using boost::asio::ip::udp;

namespace optoforce_etherdaq_ros2_driver {

struct HSURecord  // High-speed UDP record
{
  uint32_t hs_sequence_;
  uint32_t ft_sequence_;
  uint32_t status_;
  int32_t fx_;
  int32_t fy_;
  int32_t fz_;
  int32_t tx_;
  int32_t ty_;
  int32_t tz_;

  enum { HSU_RECORD_SIZE = 36 };
  void unpack(const uint8_t *buffer);
  static uint32_t unpack32(const uint8_t *buffer);
};

uint32_t HSURecord::unpack32(const uint8_t *buffer) {
  return (uint32_t(buffer[0]) << 24) | (uint32_t(buffer[1]) << 16) |
         (uint32_t(buffer[2]) << 8) | (uint32_t(buffer[3]) << 0);
}

void HSURecord::unpack(const uint8_t *buffer) {
  hs_sequence_ = unpack32(buffer + 0);
  ft_sequence_ = unpack32(buffer + 4);
  status_ = unpack32(buffer + 8);
  fx_ = unpack32(buffer + 12);
  fy_ = unpack32(buffer + 16);
  fz_ = unpack32(buffer + 20);
  tx_ = unpack32(buffer + 24);
  ty_ = unpack32(buffer + 28);
  tz_ = unpack32(buffer + 32);
}

struct HSUCommand {
  uint16_t command_header_;
  uint16_t command_;
  uint32_t sample_count_;

  HSUCommand() : command_header_(HEADER) {
    // empty
  }

  enum { HEADER = 0x1234 };

  // Possible values for command_
  enum {
    CMD_STOP_STREAMING = 0,
    CMD_START_HIGH_SPEED_STREAMING = 2,
    CMD_SET_SPEED = 0x0082,
    CMD_SET_FILTER = 0x0081

  };

  // Special values for sample count
  enum { INFINITE_SAMPLES = 0 };

  enum { HSU_COMMAND_SIZE = 8 };

  //! Packet structure into buffer for network transport
  //  Buffer should be RDT_COMMAND_SIZE
  void pack(uint8_t *buffer) const;
};

void HSUCommand::pack(uint8_t *buffer) const {
  // Data is big-endian
  buffer[0] = (command_header_ >> 8) & 0xFF;
  buffer[1] = (command_header_ >> 0) & 0xFF;
  buffer[2] = (command_ >> 8) & 0xFF;
  buffer[3] = (command_ >> 0) & 0xFF;
  buffer[4] = (sample_count_ >> 8) & 0xFF;
  buffer[5] = (sample_count_ >> 0) & 0xFF;
  buffer[6] = (sample_count_ >> 8) & 0xFF;
  buffer[7] = (sample_count_ >> 0) & 0xFF;
}

size_t write_to_string(void *ptr, size_t size, size_t count, void *stream) {
  ((std::string *)stream)->append((char *)ptr, 0, size * count);
  return size * count;
}

EtherDAQDriver::EtherDAQDriver(rclcpp::Node::SharedPtr node,
                               const std::string &address, unsigned int uSpeed,
                               unsigned int filter, double T_lowpass)
    : address_(address),
      socket_(io_service_),
      stop_recv_thread_(false),
      recv_thread_running_(false),
      packet_count_(0),
      lost_packets_(0),
      out_of_order_count_(0),
      seq_counter_(0),
      diag_packet_count_(0),
      last_hs_sequence_(0),
      system_status_(0),
      speed_(uSpeed),
      filter_(filter),
      force_units_(0),
      torque_units_(0),
      T_lowpass_(T_lowpass),
      node_(node) {
  clock_ = node->get_clock();
  last_diag_pub_time_ = node_->now();
  // Construct UDP socket
  udp::endpoint etherdaq_endpoint(
      boost::asio::ip::address_v4::from_string(address), DAQ_PORT);
  socket_.open(udp::v4());
  socket_.connect(etherdaq_endpoint);

  // Get Force/Torque scale from device webserver
  double counts_per_force = 1.0;
  double counts_per_torque = 1.0;
  force_units_ = 0;
  torque_units_ = 0;

  doUnzero();

  CURL *curl;
  CURLcode result;
  std::string response;
  curl = curl_easy_init();
  if (curl) {
    std::string xml_url = "http://" + address_ + "/netftcalapi.xml";
    curl_easy_setopt(curl, CURLOPT_URL, xml_url.c_str());
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_to_string);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 60);

    result = curl_easy_perform(curl);
    if (result != CURLE_OK)
      RCLCPP_WARN(node_->get_logger(),
                  "Failed to connect to the EtherDAQ webserver: %s, error: %s",
                  xml_url.c_str(), curl_easy_strerror(result));
    else {
      tinyxml2::XMLDocument xml_doc;
      const tinyxml2::XMLError error = xml_doc.Parse(response.c_str());

      if (error != tinyxml2::XML_SUCCESS) {
        RCLCPP_WARN_STREAM(node_->get_logger(), error);
      } else {
        tinyxml2::XMLElement *cal_xml =
            xml_doc.FirstChildElement("netftCalibration");
        if (!cal_xml)
          RCLCPP_WARN(
              node_->get_logger(),
              "Could not find the 'netftCalibration' element in the xml file");
        else {
          // Counts per force
          const tinyxml2::XMLElement *cpf_xml =
              cal_xml->FirstChildElement("calcpf");
          if (cpf_xml && cpf_xml->GetText()) {
            try {
              counts_per_force =
                  boost::lexical_cast<double>(cpf_xml->GetText());
            } catch (boost::bad_lexical_cast & /*e*/) {
              RCLCPP_WARN_STREAM(node_->get_logger(),
                                 "DAQCalibration: calcpf ["
                                     << cpf_xml->GetText()
                                     << "] is not a number");
            }
          } else
            RCLCPP_WARN(node_->get_logger(),
                        "Could not find the 'calcpf' attribute");
          // Counts per torque
          const tinyxml2::XMLElement *cpt_xml =
              cal_xml->FirstChildElement("calcpt");
          if (cpt_xml && cpt_xml->GetText()) {
            try {
              counts_per_torque =
                  boost::lexical_cast<double>(cpt_xml->GetText());
            } catch (boost::bad_lexical_cast & /*e*/) {
              RCLCPP_WARN_STREAM(node_->get_logger(),
                                 "DAQCalibration: calcpt ["
                                     << cpt_xml->GetText()
                                     << "] is not a number");
            }
          } else
            RCLCPP_WARN(node_->get_logger(),
                        "Could not find the 'calcpt' attribute");
          // Force units
          const tinyxml2::XMLElement *fu_xml =
              cal_xml->FirstChildElement("calfu");
          if (fu_xml && fu_xml->GetText()) {
            try {
              force_units_ = boost::lexical_cast<uint32_t>(fu_xml->GetText());
            } catch (boost::bad_lexical_cast &) {
              RCLCPP_WARN_STREAM(node_->get_logger(),
                                 "DAQCalibration: calfu ["
                                     << fu_xml->GetText()
                                     << "] is not a number");
            }
          } else {
            RCLCPP_WARN(node_->get_logger(),
                        "Could not find the 'calfu' attribute");
          }
          // Torque units
          const tinyxml2::XMLElement *tu_xml =
              cal_xml->FirstChildElement("caltu");
          if (tu_xml && tu_xml->GetText()) {
            try {
              torque_units_ = boost::lexical_cast<uint32_t>(tu_xml->GetText());
            } catch (boost::bad_lexical_cast &) {
              RCLCPP_WARN_STREAM(node_->get_logger(),
                                 "DAQCalibration: caltu ["
                                     << tu_xml->GetText()
                                     << "] is not a number");
            }
          } else {
            RCLCPP_WARN(node_->get_logger(),
                        "Could not find the 'caltu' attribute");
          }
        }
      }
    }
    curl_easy_cleanup(curl);
  }

  force_scale_ = 1.0 / counts_per_force;
  torque_scale_ = 1.0 / counts_per_torque;

  // Start receive thread
  recv_thread_ = boost::thread(&EtherDAQDriver::recvThreadFunc, this);

  // Since start steaming command is sent with UDP packet,
  // the packet could be lost, retry startup 10 times before giving up
  for (int i = 0; i < 10; ++i) {
    startStreaming();
    if (waitForNewData()) break;
  }
  {
    boost::unique_lock<boost::mutex> lock(mutex_);
    if (packet_count_ == 0) {
      throw std::runtime_error("No data received from EthernetDAQ device");
    }
  }
}

EtherDAQDriver::~EtherDAQDriver() {
  stop_recv_thread_ = true;
  if (!recv_thread_.timed_join(boost::posix_time::time_duration(0, 0, 1, 0))) {
    RCLCPP_WARN(node_->get_logger(), "Interrupting recv thread");
    recv_thread_.interrupt();
    if (!recv_thread_.timed_join(
            boost::posix_time::time_duration(0, 0, 1, 0))) {
      RCLCPP_WARN(node_->get_logger(), "Failed second join to recv thread");
    }
  }
  socket_.close();
}

void EtherDAQDriver::doZero() {
  boost::unique_lock<boost::mutex> lock(mutex_);
  offset_data_.wrench.force.x = fx_lowpass_;
  offset_data_.wrench.force.y = fy_lowpass_;
  offset_data_.wrench.force.z = fz_lowpass_;

  offset_data_.wrench.torque.x = tx_lowpass_;
  offset_data_.wrench.torque.y = ty_lowpass_;
  offset_data_.wrench.torque.z = tz_lowpass_;
}

void EtherDAQDriver::doUnzero() {
  boost::unique_lock<boost::mutex> lock(mutex_);
  offset_data_.wrench.force.x = 0.0;
  offset_data_.wrench.force.y = 0.0;
  offset_data_.wrench.force.z = 0.0;
  offset_data_.wrench.torque.x = 0.0;
  offset_data_.wrench.torque.y = 0.0;
  offset_data_.wrench.torque.z = 0.0;
}

bool EtherDAQDriver::isRawData() const {
  if (force_units_ == 2 && torque_units_ == 3) {
    return false;
  }
  return true;
}

bool EtherDAQDriver::waitForNewData() {
  // Wait upto 100ms for new data
  bool got_new_data = false;
  {
    boost::mutex::scoped_lock lock(mutex_);
    unsigned current_packet_count = packet_count_;
    condition_.timed_wait(lock, boost::posix_time::milliseconds(100));
    got_new_data = packet_count_ != current_packet_count;
  }

  return got_new_data;
}

void EtherDAQDriver::startStreaming(void) {
  uint8_t buffer[HSUCommand::HSU_COMMAND_SIZE];
  // Command EtherDAQ to set up its speed
  HSUCommand speed_setup;
  speed_setup.command_ = HSUCommand::CMD_SET_SPEED;
  if (speed_ == 0) {
    speed_ = 1;
  }
  speed_setup.sample_count_ = 1000 / speed_;
  speed_setup.pack(buffer);
  socket_.send(boost::asio::buffer(buffer, HSUCommand::HSU_COMMAND_SIZE));
  // Command EtherDAQ to set up its filter
  HSUCommand filter_setup;
  filter_setup.command_ = HSUCommand::CMD_SET_FILTER;
  filter_setup.sample_count_ = filter_;
  filter_setup.pack(buffer);
  socket_.send(boost::asio::buffer(buffer, HSUCommand::HSU_COMMAND_SIZE));

  // Command EtherDAQ to start data transmission
  HSUCommand start_transmission;
  start_transmission.command_ = HSUCommand::CMD_START_HIGH_SPEED_STREAMING;
  start_transmission.sample_count_ = HSUCommand::INFINITE_SAMPLES;

  start_transmission.pack(buffer);
  socket_.send(boost::asio::buffer(buffer, HSUCommand::HSU_COMMAND_SIZE));
}

void EtherDAQDriver::recvThreadFunc() {
  try {
    recv_thread_running_ = true;
    HSURecord hsu_record;
    geometry_msgs::msg::WrenchStamped tmp_data;
    uint8_t buffer[HSURecord::HSU_RECORD_SIZE + 1];
    while (!stop_recv_thread_) {
      size_t len = socket_.receive(
          boost::asio::buffer(buffer, HSURecord::HSU_RECORD_SIZE + 1));
      if (len != HSURecord::HSU_RECORD_SIZE) {
        RCLCPP_WARN(
            node_->get_logger(),
            "Receive size of %d bytes does not match expected size of %d",
            int(len), int(HSURecord::HSU_RECORD_SIZE));
      } else {
        hsu_record.unpack(buffer);
        if (hsu_record.status_ != 0) {
          // Latch any system status error code
          boost::unique_lock<boost::mutex> lock(mutex_);
          system_status_ = hsu_record.status_;
        }
        int32_t seqdiff = int32_t(hsu_record.hs_sequence_ - last_hs_sequence_);
        last_hs_sequence_ = hsu_record.hs_sequence_;
        if (seqdiff < 1) {
          boost::unique_lock<boost::mutex> lock(mutex_);
          // Don't use data that is old
          ++out_of_order_count_;
        } else {
          tmp_data.header.stamp = clock_->now();
          tmp_data.header.frame_id = "base_link";
          tmp_data.wrench.force.x = double(hsu_record.fx_) * force_scale_;
          tmp_data.wrench.force.y = double(hsu_record.fy_) * force_scale_;
          tmp_data.wrench.force.z = double(hsu_record.fz_) * force_scale_;
          tmp_data.wrench.torque.x = double(hsu_record.tx_) * torque_scale_;
          tmp_data.wrench.torque.y = double(hsu_record.ty_) * torque_scale_;
          tmp_data.wrench.torque.z = double(hsu_record.tz_) * torque_scale_;
          {
            boost::unique_lock<boost::mutex> lock(mutex_);

            // update lowpass measurement signal for tare
            const double Ts = 1.0 / speed_;
            fx_lowpass_ =
                Ts / T_lowpass_ *
                (tmp_data.wrench.force.x + (T_lowpass_ / Ts - 1) * fx_lowpass_);
            fy_lowpass_ =
                Ts / T_lowpass_ *
                (tmp_data.wrench.force.y + (T_lowpass_ / Ts - 1) * fy_lowpass_);
            fz_lowpass_ =
                Ts / T_lowpass_ *
                (tmp_data.wrench.force.z + (T_lowpass_ / Ts - 1) * fz_lowpass_);
            tx_lowpass_ = Ts / T_lowpass_ *
                          (tmp_data.wrench.torque.x +
                           (T_lowpass_ / Ts - 1) * tx_lowpass_);
            ty_lowpass_ = Ts / T_lowpass_ *
                          (tmp_data.wrench.torque.y +
                           (T_lowpass_ / Ts - 1) * ty_lowpass_);
            tz_lowpass_ = Ts / T_lowpass_ *
                          (tmp_data.wrench.torque.z +
                           (T_lowpass_ / Ts - 1) * tz_lowpass_);

            new_data_ = tmp_data;
            new_data_.wrench.force.x -= offset_data_.wrench.force.x;
            new_data_.wrench.force.y -= offset_data_.wrench.force.y;
            new_data_.wrench.force.z -= offset_data_.wrench.force.z;
            new_data_.wrench.torque.x -= offset_data_.wrench.torque.x;
            new_data_.wrench.torque.y -= offset_data_.wrench.torque.y;
            new_data_.wrench.torque.z -= offset_data_.wrench.torque.z;
            lost_packets_ += (seqdiff - 1);
            ++packet_count_;
            condition_.notify_all();
          }
        }
      }
    }  // end while
  } catch (std::exception &e) {
    recv_thread_running_ = false;
    {
      boost::unique_lock<boost::mutex> lock(mutex_);
      recv_thread_error_msg_ = e.what();
      RCLCPP_ERROR(node_->get_logger(), "Receive thread exception: %s",
                   e.what());
    }
  }
}

void EtherDAQDriver::getData(geometry_msgs::msg::WrenchStamped &data) {
  {
    boost::unique_lock<boost::mutex> lock(mutex_);
    data = new_data_;
  }
}

void EtherDAQDriver::diagnostics(
    diagnostic_updater::DiagnosticStatusWrapper &d) {
  // Publish diagnostics
  d.name = "EthernetDAQ Driver : " + address_;

  d.summary(d.OK, "OK");
  d.hardware_id = "0";

  if (diag_packet_count_ == packet_count_) {
    d.mergeSummary(d.ERROR, "No new data in last second");
  }

  if (!recv_thread_running_) {
    d.mergeSummaryf(d.ERROR, "Receive thread has stopped : %s",
                    recv_thread_error_msg_.c_str());
  }

  if (system_status_ != 0) {
    d.mergeSummaryf(d.ERROR, "EtherDAQ reports error 0x%08x", system_status_);
  }

  auto current_time = clock_->now();
  double recv_rate = double(int32_t(packet_count_ - diag_packet_count_)) /
                     (current_time - last_diag_pub_time_).seconds();

  d.clear();
  d.addf("IP Address", "%s", address_.c_str());
  d.addf("System status", "0x%08x", system_status_);
  d.addf("Good packets", "%u", packet_count_);
  d.addf("Lost packets", "%u", lost_packets_);
  d.addf("Out-of-order packets", "%u", out_of_order_count_);
  d.addf("Recv rate (pkt/sec)", "%.2f", recv_rate);
  d.addf("Force scale", "%f", force_scale_);
  d.addf("Torque scale", "%f", torque_scale_);

  geometry_msgs::msg::WrenchStamped data;
  getData(data);
  d.addf("Force X ", "%f", data.wrench.force.x);
  d.addf("Force Y ", "%f", data.wrench.force.y);
  d.addf("Force Z ", "%f", data.wrench.force.z);
  d.addf("Torque X ", "%f", data.wrench.torque.x);
  d.addf("Torque Y ", "%f", data.wrench.torque.y);
  d.addf("Torque Z ", "%f", data.wrench.torque.z);

  last_diag_pub_time_ = current_time;
  diag_packet_count_ = packet_count_;
}

}  // namespace optoforce_etherdaq_ros2_driver
