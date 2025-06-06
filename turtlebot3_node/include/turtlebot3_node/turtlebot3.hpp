// Copyright 2019 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: Darby Lim

#ifndef TURTLEBOT3_NODE__TURTLEBOT3_HPP_
#define TURTLEBOT3_NODE__TURTLEBOT3_HPP_

#include <tf2_ros/transform_broadcaster.h>

#include <array>
#include <chrono>
#include <geometry_msgs/msg/twist.hpp>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <string>
#include <turtlebot3_msgs/msg/sensor_state.hpp>

#include "robot_msgs/srv/precise_control.hpp"
#include "turtlebot3_node/control_table.hpp"
#include "turtlebot3_node/devices/devices.hpp"
#include "turtlebot3_node/devices/motor_power.hpp"
#include "turtlebot3_node/devices/reset.hpp"
#include "turtlebot3_node/devices/sound.hpp"
#include "turtlebot3_node/dynamixel_sdk_wrapper.hpp"
#include "turtlebot3_node/odometry.hpp"
#include "turtlebot3_node/sensors/battery_state.hpp"
#include "turtlebot3_node/sensors/imu.hpp"
#include "turtlebot3_node/sensors/joint_state.hpp"
#include "turtlebot3_node/sensors/sensor_state.hpp"
#include "turtlebot3_node/sensors/sensors.hpp"
#include "turtlebot3_node/twist_subscriber.hpp"

namespace robotis {
namespace turtlebot3 {
extern const ControlTable extern_control_table;
class TurtleBot3 : public rclcpp::Node {
 public:
  typedef struct {
    float separation;
    float radius;
  } Wheels;

  typedef struct {
    float profile_acceleration_constant;
    float profile_acceleration;
  } Motors;

  explicit TurtleBot3(const std::string& usb_port);
  virtual ~TurtleBot3() {}

  Wheels* get_wheels();
  Motors* get_motors();

 private:
  void init_dynamixel_sdk_wrapper(const std::string& usb_port);
  void check_device_status();

  void add_sensors();
  void add_devices();
  void add_motors();
  void add_wheels();

  void run();

  void publish_timer(const std::chrono::milliseconds timeout);
  void heartbeat_timer(const std::chrono::milliseconds timeout);

  void cmd_vel_callback();
  void parameter_event_callback();
  void cmd_lift_callback();  // 리프트 모터 제어 콜백 추가

  void precise_control_service_callback();
  void handle_precise_control(const std::shared_ptr<robot_msgs::srv::PreciseControl::Request> request, std::shared_ptr<robot_msgs::srv::PreciseControl::Response> response);

  // ================ 정밀 제어 동작 메서드 ================
  bool perform_rotate_180(double& duration);
  bool perform_backward_20cm(double& duration);
  bool perform_pickup_sequence(double& duration);

  // ================ 헬퍼 메서드 ================
  void send_cmd_vel(double linear_x, double angular_z);
  void send_lift_cmd(double lift_z);
  void wait_for_duration(double seconds);
  void stop_all_motors();

  Wheels wheels_;
  Motors motors_;

  std::shared_ptr<DynamixelSDKWrapper> dxl_sdk_wrapper_;

  std::list<sensors::Sensors*> sensors_;
  std::map<std::string, devices::Devices*> devices_;

  std::unique_ptr<Odometry> odom_;

  rclcpp::Node::SharedPtr node_handle_;

  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;

  std::unique_ptr<TwistSubscriber> cmd_vel_sub_;
  std::unique_ptr<TwistSubscriber> lift_cmd_sub_;  // 리프트 명령 구독자 추가

  rclcpp::AsyncParametersClient::SharedPtr priv_parameters_client_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub_;

  // 정밀 제어 서비스 서버 추가
  rclcpp::Service<robot_msgs::srv::PreciseControl>::SharedPtr precise_control_service_;

  // ================ 정밀 제어 관련 상수 ================
  static constexpr double LINEAR_SPEED = 0.1;       // 선형 속도 (m/s)
  static constexpr double ROTATION_SPEED = 0.5;     // 회전 속도 (rad/s)
  static constexpr double LIFT_SPEED = 0.3;         // 리프트 속도
  static constexpr double ROTATION_ANGLE = M_PI;    // 180도 = π 라디안
  static constexpr double BACKWARD_DISTANCE = 0.2;  // 20cm 후진
};
}  // namespace turtlebot3
}  // namespace robotis
#endif  // TURTLEBOT3_NODE__TURTLEBOT3_HPP_
