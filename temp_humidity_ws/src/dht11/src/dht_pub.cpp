// dht_serial_publisher.cpp
#include <chrono>
#include <cstring>
#include <string>
#include <algorithm>
#include <cctype>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;

class DhtSerialPublisher : public rclcpp::Node {
public:
  DhtSerialPublisher() : rclcpp::Node("dht_pub"), fd_(-1) {
    //publishers for each topic
    temp_pub_  = this->create_publisher<std_msgs::msg::Float32>("temperature", 10);
    humid_pub_ = this->create_publisher<std_msgs::msg::Float32>("humidity", 10);

    if (!open_and_configure_port()) {
      RCLCPP_FATAL(get_logger(), "Could not open/configure %s", kSerialPort);
      throw std::runtime_error("serial open failed");
    }
    timer_ = this->create_wall_timer(50ms, std::bind(&DhtSerialPublisher::poll_serial, this));
    RCLCPP_INFO(get_logger(), "Reading DHT data on %s @ 9600 baud", kSerialPort);
  }

  ~DhtSerialPublisher() override {
    if (fd_ >= 0) close(fd_);
  }

private:
  static constexpr const char* kSerialPort = "/dev/ttyACM0";
  static constexpr int kPollBufSize = 256;

  static inline void trim(std::string &s) {
    auto not_space = [](unsigned char c){ return !std::isspace(c); };
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), not_space));
    s.erase(std::find_if(s.rbegin(), s.rend(), not_space).base(), s.end());
  }

  bool open_and_configure_port() {
    fd_ = open(kSerialPort, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      RCLCPP_ERROR(get_logger(), "open(%s) failed: %s", kSerialPort, std::strerror(errno));
      return false;
    }

    termios tty{};
    if (tcgetattr(fd_, &tty) != 0) {
      RCLCPP_ERROR(get_logger(), "tcgetattr failed: %s", std::strerror(errno));
      return false;
    }

    cfmakeraw(&tty);

    cfsetispeed(&tty, B9600);
    cfsetospeed(&tty, B9600);
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE; tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |=  CREAD | CLOCAL;

    tty.c_cc[VTIME] = 0; 
    tty.c_cc[VMIN]  = 0;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
      RCLCPP_ERROR(get_logger(), "tcsetattr failed: %s", std::strerror(errno));
      return false;
    }
    tcflush(fd_, TCIOFLUSH);
    rclcpp::sleep_for(1s);

    rclcpp::sleep_for(500ms);
    return true;
  }

  void handle_line(const std::string &line) {
    // "humidity, temperature"
    auto comma = line.find(',');
    if (comma == std::string::npos) {
      if (!line.empty()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "Ignoring non-data line: '%s'", line.c_str());
      }
      return;
    }

    std::string h = line.substr(0, comma);
    std::string t = line.substr(comma + 1);
    trim(h); trim(t);

    try {
        float humidity = std::stof(h);
        float temperature = std::stof(t);

        std_msgs::msg::Float32 hmsg; hmsg.data = humidity;
        std_msgs::msg::Float32 tmsg; tmsg.data = temperature;
        humid_pub_->publish(hmsg);
        temp_pub_->publish(tmsg);

        RCLCPP_INFO(get_logger(), "Humidity: %.2f %%   Temp: %.2f °C",
                    humidity, temperature);

    } catch (const std::exception &e) {
        RCLCPP_WARN(get_logger(), "Parse error on line '%s': %s", line.c_str(), e.what());
    }
  }

  void poll_serial() {
    if (fd_ < 0) return;
    char buf[kPollBufSize];
    ssize_t n = read(fd_, buf, sizeof(buf));
    RCLCPP_DEBUG(get_logger(), "read %zd bytes", n);

    if (n < 0) {
      if (errno != EAGAIN) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 3000,
                              "read() error: %s", std::strerror(errno));
      }
      return;
    }
    if (n == 0) return;

    buffer_.append(buf, static_cast<size_t>(n));

    size_t pos = 0;
    while (true) {
      size_t nl = buffer_.find('\n', pos);
      if (nl == std::string::npos) {
        buffer_ = buffer_.substr(pos);
        break;
      }
      std::string line = buffer_.substr(pos, nl - pos);
      if (!line.empty() && line.back() == '\r') line.pop_back(); // handle CRLF
      handle_line(line);
      pos = nl + 1;
    }
  }

  int fd_;
  std::string buffer_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr temp_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr humid_pub_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<DhtSerialPublisher>());
  } catch (const std::exception &e) {
    fprintf(stderr, "Fatal: %s\n", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
