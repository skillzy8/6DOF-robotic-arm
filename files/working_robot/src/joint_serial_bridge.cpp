#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

#include <cmath>
#include <string>
#include <sstream>
#include <iomanip>
#include <map>
#include <vector>

class PlannedPathSerialBridge : public rclcpp::Node
{
public:
  PlannedPathSerialBridge() : Node("planned_path_serial_bridge")
  {
    this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
    this->declare_parameter<bool>("fake_mode", false);

    port_name_ = this->get_parameter("serial_port").as_string();
    fake_mode_ = this->get_parameter("fake_mode").as_bool();

    baudrate_ = B115200;

    joint_order_ = {
      "bottom_holder_rotation",
      "shoulder_arm_rotation",
      "arm_holder_right_after_shoulder_rotation",
      "elbow_arm_rotation",
      "gripper_holder_rotation",
      "end_factor_rotation"
    };

    last_target_deg_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    if (!fake_mode_)
    {
      serial_fd_ = openSerialPort(port_name_);

      if (serial_fd_ < 0)
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", port_name_.c_str());
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "Serial port opened: %s", port_name_.c_str());
      }
    }
    else
    {
      serial_fd_ = -1;
      RCLCPP_WARN(this->get_logger(), "FAKE MODE ENABLED: no ESP32 needed");
    }

    planned_path_sub_ = this->create_subscription<moveit_msgs::msg::DisplayTrajectory>(
      "/display_planned_path",
      10,
      std::bind(&PlannedPathSerialBridge::trajectoryCallback, this, std::placeholders::_1)
    );

    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "/joint_states",
      10
    );

    serial_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(5),
      std::bind(&PlannedPathSerialBridge::readSerialData, this)
    );

    fake_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&PlannedPathSerialBridge::publishFakeJointStates, this)
    );

    RCLCPP_INFO(this->get_logger(), "Listening to /display_planned_path");
  }

  ~PlannedPathSerialBridge()
  {
    if (serial_fd_ >= 0)
    {
      close(serial_fd_);
    }
  }

private:
  int openSerialPort(const std::string & port)
  {
    int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);

    if (fd < 0)
    {
      return -1;
    }

    termios tty{};

    if (tcgetattr(fd, &tty) != 0)
    {
      close(fd);
      return -1;
    }

    cfsetospeed(&tty, baudrate_);
    cfsetispeed(&tty, baudrate_);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0)
    {
      close(fd);
      return -1;
    }

    return fd;
  }

  void trajectoryCallback(const moveit_msgs::msg::DisplayTrajectory::SharedPtr msg)
  {
    if (msg->trajectory.empty())
    {
      RCLCPP_WARN(this->get_logger(), "Received empty trajectory");
      return;
    }

    const auto & joint_traj = msg->trajectory[0].joint_trajectory;

    if (joint_traj.points.empty())
    {
      RCLCPP_WARN(this->get_logger(), "Trajectory has no points");
      return;
    }

    const auto & final_point = joint_traj.points.back();

    std::map<std::string, double> joint_map;

    for (size_t i = 0; i < joint_traj.joint_names.size(); i++)
    {
      if (i < final_point.positions.size())
      {
        joint_map[joint_traj.joint_names[i]] = final_point.positions[i];
      }
    }

    std::ostringstream line;
    line << "J";

    for (size_t i = 0; i < joint_order_.size(); i++)
    {
      const auto & joint_name = joint_order_[i];

      double angle_rad = 0.0;

      if (joint_map.find(joint_name) != joint_map.end())
      {
        angle_rad = joint_map[joint_name];
      }
      else
      {
        RCLCPP_WARN(
          this->get_logger(),
          "Joint not found in planned trajectory: %s",
          joint_name.c_str()
        );
      }

      double angle_deg = angle_rad * 180.0 / M_PI;
      last_target_deg_[i] = angle_deg;

      line << "," << std::fixed << std::setprecision(2) << angle_deg;
    }

    line << "\n";

    std::string data = line.str();

    if (!fake_mode_)
    {
      if (serial_fd_ < 0)
      {
        RCLCPP_ERROR(this->get_logger(), "Serial port is not open");
        return;
      }

      ssize_t bytes_written = write(serial_fd_, data.c_str(), data.size());

      if (bytes_written < 0)
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to write to serial");
        return;
      }
    }

    RCLCPP_INFO(this->get_logger(), "Target joints: %s", data.c_str());

    if (fake_mode_)
    {
      publishJointStatesFromDegrees(last_target_deg_);
    }
  }

  void readSerialData()
  {
    if (fake_mode_ || serial_fd_ < 0)
    {
      return;
    }

    char c;

    while (read(serial_fd_, &c, 1) > 0)
    {
      if (c == '\n')
      {
        parseSerialLine(serial_buffer_);
        serial_buffer_.clear();
      }
      else if (c != '\r')
      {
        serial_buffer_ += c;
      }
    }
  }

  void parseSerialLine(const std::string & line)
  {
    if (line.empty())
    {
      return;
    }

    // ESP32 encoder feedback format:
    // E,0.00,10.00,20.00,30.00,40.00,50.00
    if (line.rfind("E,", 0) != 0)
    {
      return;
    }

    std::string data = line.substr(2);
    std::stringstream ss(data);
    std::string value;

    std::vector<double> angles_deg;

    while (std::getline(ss, value, ','))
    {
      try
      {
        angles_deg.push_back(std::stod(value));
      }
      catch (...)
      {
        RCLCPP_WARN(this->get_logger(), "Bad encoder value: %s", value.c_str());
        return;
      }
    }

    if (angles_deg.size() != joint_order_.size())
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Expected 6 encoder values, got %zu",
        angles_deg.size()
      );
      return;
    }

    publishJointStatesFromDegrees(angles_deg);
  }

  void publishJointStatesFromDegrees(const std::vector<double> & angles_deg)
  {
    sensor_msgs::msg::JointState msg;

    msg.header.stamp = this->now();
    msg.name = joint_order_;

    for (double angle_deg : angles_deg)
    {
      msg.position.push_back(angle_deg * M_PI / 180.0);
    }

    joint_state_pub_->publish(msg);

    RCLCPP_INFO(
      this->get_logger(),
      "Published joint states: %.2f %.2f %.2f %.2f %.2f %.2f deg",
      angles_deg[0],
      angles_deg[1],
      angles_deg[2],
      angles_deg[3],
      angles_deg[4],
      angles_deg[5]
    );
  }

  void publishFakeJointStates()
  {
    if (!fake_mode_)
    {
      return;
    }

    publishJointStatesFromDegrees(last_target_deg_);
  }

  std::string port_name_;
  speed_t baudrate_;
  int serial_fd_;

  bool fake_mode_;

  std::string serial_buffer_;

  std::vector<std::string> joint_order_;
  std::vector<double> last_target_deg_;

  rclcpp::Subscription<moveit_msgs::msg::DisplayTrajectory>::SharedPtr planned_path_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

  rclcpp::TimerBase::SharedPtr serial_timer_;
  rclcpp::TimerBase::SharedPtr fake_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<PlannedPathSerialBridge>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
