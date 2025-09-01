#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

class TemperatureSubscriber : public rclcpp::Node {
public:
  TemperatureSubscriber() : rclcpp::Node("temp_sub") {
    refresh_publishers_label();  // look up current publishers once at startup

    // subscriber node to log temp
    auto cb = [this](std_msgs::msg::Float32::UniquePtr msg) {
      const char* from = publishers_label_.empty() ? "unknown" : publishers_label_.c_str();
      RCLCPP_INFO(this->get_logger(), "Reading %.2f °C from publisher node(s): %s",
                  msg->data, from);
    };

    sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "temperature", 10, cb);

    refresh_timer_ = this->create_wall_timer(
      std::chrono::seconds(5),
      std::bind(&TemperatureSubscriber::refresh_publishers_label, this));
  }

private:
  void refresh_publishers_label() {
    auto infos = this->get_publishers_info_by_topic("temperature");
    std::vector<std::string> names;
    names.reserve(infos.size());
    for (const auto & info : infos) {
      std::string ns = info.node_namespace();
      if (ns.empty() || ns == "/") {
        names.emplace_back("/" + info.node_name());
      } else {
        names.emplace_back(ns + "/" + info.node_name());
      }
    }
    publishers_label_.clear();
    for (size_t i = 0; i < names.size(); ++i) {
      if (i) publishers_label_ += ", ";
      publishers_label_ += names[i];
    }
  }

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr refresh_timer_;
  std::string publishers_label_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TemperatureSubscriber>());
  rclcpp::shutdown();
  return 0;
}

