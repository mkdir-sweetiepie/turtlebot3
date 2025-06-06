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

#include "turtlebot3_node/turtlebot3.hpp"

#include <memory>
#include <string>

using robotis::turtlebot3::TurtleBot3;
using namespace std::chrono_literals;

TurtleBot3::TurtleBot3(const std::string &usb_port) : Node("turtlebot3_node", rclcpp::NodeOptions().use_intra_process_comms(true)) {
  RCLCPP_INFO(get_logger(), "Init TurtleBot3 Node Main");
  node_handle_ = std::shared_ptr<::rclcpp::Node>(this, [](::rclcpp::Node *) {});

  init_dynamixel_sdk_wrapper(usb_port);
  check_device_status();

  add_motors();
  add_wheels();
  add_sensors();
  add_devices();

  run();
}

TurtleBot3::Wheels *TurtleBot3::get_wheels() { return &wheels_; }

TurtleBot3::Motors *TurtleBot3::get_motors() { return &motors_; }

void TurtleBot3::init_dynamixel_sdk_wrapper(const std::string &usb_port) {
  DynamixelSDKWrapper::Device opencr = {usb_port, 200, 1000000, 2.0f};

  this->declare_parameter<uint8_t>("opencr.id");
  this->declare_parameter<int>("opencr.baud_rate");
  this->declare_parameter<float>("opencr.protocol_version");
  this->declare_parameter<std::string>("namespace");

  this->get_parameter_or<uint8_t>("opencr.id", opencr.id, 200);
  this->get_parameter_or<int>("opencr.baud_rate", opencr.baud_rate, 1000000);
  this->get_parameter_or<float>("opencr.protocol_version", opencr.protocol_version, 2.0f);

  RCLCPP_INFO(this->get_logger(), "Init DynamixelSDKWrapper");

  dxl_sdk_wrapper_ = std::make_shared<DynamixelSDKWrapper>(opencr);

  dxl_sdk_wrapper_->init_read_memory(extern_control_table.millis.addr,
                                     (extern_control_table.profile_acceleration_right.addr - extern_control_table.millis.addr) + extern_control_table.profile_acceleration_right.length);
}

void TurtleBot3::check_device_status() {
  if (dxl_sdk_wrapper_->is_connected_to_device()) {
    std::string sdk_msg;
    uint8_t reset = 1;

    dxl_sdk_wrapper_->set_data_to_device(extern_control_table.imu_re_calibration.addr, extern_control_table.imu_re_calibration.length, &reset, &sdk_msg);

    RCLCPP_INFO(this->get_logger(), "Start Calibration of Gyro");
    rclcpp::sleep_for(std::chrono::seconds(5));
    RCLCPP_INFO(this->get_logger(), "Calibration End");
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed connection with Devices");
    rclcpp::shutdown();
    return;
  }

  const int8_t NOT_CONNECTED_MOTOR = -1;

  int8_t device_status = dxl_sdk_wrapper_->get_data_from_device<int8_t>(extern_control_table.device_status.addr, extern_control_table.device_status.length);

  switch (device_status) {
    case NOT_CONNECTED_MOTOR:
      RCLCPP_WARN(this->get_logger(), "Please double check your Dynamixels and Power");
      break;

    default:
      break;
  }
}

void TurtleBot3::add_motors() {
  RCLCPP_INFO(this->get_logger(), "Add Motors");

  this->declare_parameter<float>("motors.profile_acceleration_constant");
  this->declare_parameter<float>("motors.profile_acceleration");

  this->get_parameter_or<float>("motors.profile_acceleration_constant", motors_.profile_acceleration_constant, 214.577);

  this->get_parameter_or<float>("motors.profile_acceleration", motors_.profile_acceleration, 0.0);
}

void TurtleBot3::add_wheels() {
  RCLCPP_INFO(this->get_logger(), "Add Wheels");

  this->declare_parameter<float>("wheels.separation");
  this->declare_parameter<float>("wheels.radius");

  this->get_parameter_or<float>("wheels.separation", wheels_.separation, 0.160);
  this->get_parameter_or<float>("wheels.radius", wheels_.radius, 0.033);
}

void TurtleBot3::add_sensors() {
  RCLCPP_INFO(this->get_logger(), "Add Sensors");

  uint8_t is_connected_bumper_1 = 0;
  uint8_t is_connected_bumper_2 = 0;
  uint8_t is_connected_illumination = 0;
  uint8_t is_connected_ir = 0;
  uint8_t is_connected_sonar = 0;

  this->declare_parameter<uint8_t>("sensors.bumper_1");
  this->declare_parameter<uint8_t>("sensors.bumper_2");
  this->declare_parameter<uint8_t>("sensors.illumination");
  this->declare_parameter<uint8_t>("sensors.ir");
  this->declare_parameter<uint8_t>("sensors.sonar");

  this->get_parameter_or<uint8_t>("sensors.bumper_1", is_connected_bumper_1, 0);
  this->get_parameter_or<uint8_t>("sensors.bumper_2", is_connected_bumper_2, 0);
  this->get_parameter_or<uint8_t>("sensors.illumination", is_connected_illumination, 0);
  this->get_parameter_or<uint8_t>("sensors.ir", is_connected_ir, 0);
  this->get_parameter_or<uint8_t>("sensors.sonar", is_connected_sonar, 0);

  sensors_.push_back(new sensors::BatteryState(node_handle_, "battery_state"));

  sensors_.push_back(new sensors::Imu(node_handle_, "imu", "magnetic_field", "imu_link"));

  sensors_.push_back(new sensors::SensorState(node_handle_, "sensor_state", is_connected_bumper_1, is_connected_bumper_2, is_connected_illumination, is_connected_ir, is_connected_sonar));

  dxl_sdk_wrapper_->read_data_set();
  sensors_.push_back(new sensors::JointState(node_handle_, dxl_sdk_wrapper_, "joint_states", "base_link"));
}

void TurtleBot3::add_devices() {
  RCLCPP_INFO(this->get_logger(), "Add Devices");
  devices_["motor_power"] = new devices::MotorPower(node_handle_, dxl_sdk_wrapper_, "motor_power");
  devices_["reset"] = new devices::Reset(node_handle_, dxl_sdk_wrapper_, "reset");
  devices_["sound"] = new devices::Sound(node_handle_, dxl_sdk_wrapper_, "sound");
}

void TurtleBot3::run() {
  RCLCPP_INFO(this->get_logger(), "Run!");

  publish_timer(std::chrono::milliseconds(50));
  heartbeat_timer(std::chrono::milliseconds(100));

  parameter_event_callback();
  cmd_vel_callback();
  cmd_lift_callback();                 // 리프트 모터 콜백 추가
  precise_control_service_callback();  // 정밀 제어 서비스 추가
}

void TurtleBot3::publish_timer(const std::chrono::milliseconds timeout) {
  publish_timer_ = this->create_wall_timer(timeout, [this]() -> void {
    rclcpp::Time now = this->now();

    dxl_sdk_wrapper_->read_data_set();

    for (const auto &sensor : sensors_) {
      sensor->publish(now, dxl_sdk_wrapper_);
    }
  });
}

void TurtleBot3::heartbeat_timer(const std::chrono::milliseconds timeout) {
  heartbeat_timer_ = this->create_wall_timer(timeout, [this]() -> void {
    static uint8_t count = 0;
    std::string msg;

    dxl_sdk_wrapper_->set_data_to_device(extern_control_table.heartbeat.addr, extern_control_table.heartbeat.length, &count, &msg);

    RCLCPP_DEBUG(this->get_logger(), "hearbeat count : %d, msg : %s", count, msg.c_str());

    count++;
  });
}

void TurtleBot3::parameter_event_callback() {
  priv_parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this);
  while (!priv_parameters_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }

    RCLCPP_WARN(this->get_logger(), "service not available, waiting again...");
  }

  auto param_event_callback = [this](const rcl_interfaces::msg::ParameterEvent::SharedPtr event) -> void {
    for (const auto &changed_parameter : event->changed_parameters) {
      RCLCPP_DEBUG(this->get_logger(), "changed parameter name : %s", changed_parameter.name.c_str());

      if (changed_parameter.name == "motors.profile_acceleration") {
        std::string sdk_msg;

        motors_.profile_acceleration = rclcpp::Parameter::from_parameter_msg(changed_parameter).as_double();

        motors_.profile_acceleration = motors_.profile_acceleration / motors_.profile_acceleration_constant;

        union Data {
          int32_t dword[2];
          uint8_t byte[4 * 2];
        } data;

        data.dword[0] = static_cast<int32_t>(motors_.profile_acceleration);
        data.dword[1] = static_cast<int32_t>(motors_.profile_acceleration);

        uint16_t start_addr = extern_control_table.profile_acceleration_left.addr;
        uint16_t addr_length = (extern_control_table.profile_acceleration_right.addr - extern_control_table.profile_acceleration_left.addr) + extern_control_table.profile_acceleration_right.length;

        uint8_t *p_data = &data.byte[0];

        dxl_sdk_wrapper_->set_data_to_device(start_addr, addr_length, p_data, &sdk_msg);

        RCLCPP_INFO(this->get_logger(), "changed parameter value : %f [rev/min2] sdk_msg : %s", motors_.profile_acceleration, sdk_msg.c_str());
      }
    }
  };

  parameter_event_sub_ = priv_parameters_client_->on_parameter_event(param_event_callback);
}

void TurtleBot3::cmd_vel_callback() {
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  cmd_vel_sub_ = std::make_unique<TwistSubscriber>(
      node_handle_, "cmd_vel", qos, std::function<void(const geometry_msgs::msg::Twist::SharedPtr)>([this](const geometry_msgs::msg::Twist::SharedPtr msg) -> void {
        std::string sdk_msg;

        union Data {
          int32_t dword[6];
          uint8_t byte[4 * 6];
        } data;

        data.dword[0] = static_cast<int32_t>(msg->linear.x * 100);
        data.dword[1] = 0;
        data.dword[2] = 0;
        data.dword[3] = 0;
        data.dword[4] = 0;
        data.dword[5] = static_cast<int32_t>(msg->angular.z * 100);

        uint16_t start_addr = extern_control_table.cmd_velocity_linear_x.addr;
        uint16_t addr_length = (extern_control_table.cmd_velocity_angular_z.addr - extern_control_table.cmd_velocity_linear_x.addr) + extern_control_table.cmd_velocity_angular_z.length;

        uint8_t *p_data = &data.byte[0];

        dxl_sdk_wrapper_->set_data_to_device(start_addr, addr_length, p_data, &sdk_msg);

        RCLCPP_DEBUG(this->get_logger(), "lin_vel: %f ang_vel: %f msg : %s", msg->linear.x, msg->angular.z, sdk_msg.c_str());
      }),
      std::function<void(const geometry_msgs::msg::TwistStamped::SharedPtr)>([this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) -> void {
        std::string sdk_msg;

        union Data {
          int32_t dword[6];
          uint8_t byte[4 * 6];
        } data;

        data.dword[0] = static_cast<int32_t>(msg->twist.linear.x * 100);
        data.dword[1] = 0;
        data.dword[2] = 0;
        data.dword[3] = 0;
        data.dword[4] = 0;
        data.dword[5] = static_cast<int32_t>(msg->twist.angular.z * 100);

        uint16_t start_addr = extern_control_table.cmd_velocity_linear_x.addr;
        uint16_t addr_length = (extern_control_table.cmd_velocity_angular_z.addr - extern_control_table.cmd_velocity_linear_x.addr) + extern_control_table.cmd_velocity_angular_z.length;

        uint8_t *p_data = &data.byte[0];

        dxl_sdk_wrapper_->set_data_to_device(start_addr, addr_length, p_data, &sdk_msg);

        RCLCPP_DEBUG(this->get_logger(), "lin_vel: %f ang_vel: %f msg : %s", msg->twist.linear.x, msg->twist.angular.z, sdk_msg.c_str());
      }));
}

// 리프트 모터 제어 콜백 추가
void TurtleBot3::cmd_lift_callback() {
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  lift_cmd_sub_ = std::make_unique<TwistSubscriber>(
      node_handle_, "cmd_lift_vel", qos, std::function<void(const geometry_msgs::msg::Twist::SharedPtr)>([this](const geometry_msgs::msg::Twist::SharedPtr msg) -> void {
        std::string sdk_msg;
        int32_t lift_value = static_cast<int32_t>(msg->linear.z * 100);

        uint8_t lift_data[4] = {0};
        memcpy(lift_data, &lift_value, sizeof(lift_value));

        dxl_sdk_wrapper_->set_data_to_device(extern_control_table.cmd_velocity_lift.addr, extern_control_table.cmd_velocity_lift.length, lift_data, &sdk_msg);

        RCLCPP_DEBUG(this->get_logger(), "lift_vel: %f msg: %s", msg->linear.z, sdk_msg.c_str());
      }),
      std::function<void(const geometry_msgs::msg::TwistStamped::SharedPtr)>([this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) -> void {
        std::string sdk_msg;
        int32_t lift_value = static_cast<int32_t>(msg->twist.linear.z * 100);

        uint8_t lift_data[4] = {0};
        memcpy(lift_data, &lift_value, sizeof(lift_value));

        dxl_sdk_wrapper_->set_data_to_device(extern_control_table.cmd_velocity_lift.addr, extern_control_table.cmd_velocity_lift.length, lift_data, &sdk_msg);

        RCLCPP_DEBUG(this->get_logger(), "lift_vel: %f msg: %s", msg->twist.linear.z, sdk_msg.c_str());
      }));
}

// ================ 정밀 제어 서비스 구현 ================

void TurtleBot3::precise_control_service_callback() {
  precise_control_service_ =
      this->create_service<robot_msgs::srv::PreciseControl>("precise_control", std::bind(&TurtleBot3::handle_precise_control, this, std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "정밀 제어 서비스가 준비되었습니다");
}

void TurtleBot3::handle_precise_control(const std::shared_ptr<robot_msgs::srv::PreciseControl::Request> request, std::shared_ptr<robot_msgs::srv::PreciseControl::Response> response) {
  RCLCPP_INFO(this->get_logger(), "정밀 제어 요청 수신: %s", request->action.c_str());

  auto start_time = std::chrono::steady_clock::now();
  double duration = 0.0;
  bool success = false;

  try {
    if (request->action == "rotate_180") {
      success = perform_rotate_180(duration);
      response->message = "180도 회전 완료";

    } else if (request->action == "backward_20cm") {
      success = perform_backward_20cm(duration);
      response->message = "20cm 후진 완료";

    } else if (request->action == "pickup_sequence") {
      success = perform_pickup_sequence(duration);
      response->message = "픽업 시퀀스 완료";

    } else {
      success = false;
      response->message = "알 수 없는 동작: " + request->action;
      RCLCPP_WARN(this->get_logger(), "알 수 없는 동작 요청: %s", request->action.c_str());
    }

  } catch (const std::exception &e) {
    success = false;
    response->message = "실행 중 오류 발생: " + std::string(e.what());
    RCLCPP_ERROR(this->get_logger(), "정밀 제어 오류: %s", e.what());
  }

  // 최종적으로 모든 모터 정지
  stop_all_motors();

  response->success = success;
  response->duration = duration;

  RCLCPP_INFO(this->get_logger(), "정밀 제어 완료: %s (%.2f초)", success ? "성공" : "실패", duration);
}

bool TurtleBot3::perform_rotate_180(double &duration) {
  RCLCPP_INFO(this->get_logger(), "180도 회전 시작");

  auto start_time = std::chrono::steady_clock::now();

  // 회전 시간 계산 (180도 = π 라디안)
  double rotation_time = ROTATION_ANGLE / ROTATION_SPEED;

  // 회전 시작
  send_cmd_vel(0.0, ROTATION_SPEED);

  // 회전 시간만큼 대기
  wait_for_duration(rotation_time);

  // 정지
  stop_all_motors();

  // 안정화를 위한 짧은 대기
  wait_for_duration(0.5);

  auto end_time = std::chrono::steady_clock::now();
  duration = std::chrono::duration<double>(end_time - start_time).count();

  RCLCPP_INFO(this->get_logger(), "180도 회전 완료 (%.2f초)", duration);
  return true;
}

bool TurtleBot3::perform_backward_20cm(double &duration) {
  RCLCPP_INFO(this->get_logger(), "20cm 후진 시작");

  auto start_time = std::chrono::steady_clock::now();

  // 후진 시간 계산 (거리 / 속도)
  double backward_time = BACKWARD_DISTANCE / LINEAR_SPEED;

  // 후진 시작
  send_cmd_vel(-LINEAR_SPEED, 0.0);

  // 후진 시간만큼 대기
  wait_for_duration(backward_time);

  // 정지
  stop_all_motors();

  // 안정화를 위한 짧은 대기
  wait_for_duration(0.5);

  auto end_time = std::chrono::steady_clock::now();
  duration = std::chrono::duration<double>(end_time - start_time).count();

  RCLCPP_INFO(this->get_logger(), "20cm 후진 완료 (%.2f초)", duration);
  return true;
}

bool TurtleBot3::perform_pickup_sequence(double &duration) {
  RCLCPP_INFO(this->get_logger(), "픽업 시퀀스 시작");

  auto start_time = std::chrono::steady_clock::now();
  double step_duration = 0.0;

  try {
    // 1단계: 180도 회전
    RCLCPP_INFO(this->get_logger(), "1단계: 180도 회전");
    if (!perform_rotate_180(step_duration)) {
      RCLCPP_ERROR(this->get_logger(), "180도 회전 실패");
      return false;
    }

    // 2단계: 20cm 후진
    RCLCPP_INFO(this->get_logger(), "2단계: 20cm 후진");
    if (!perform_backward_20cm(step_duration)) {
      RCLCPP_ERROR(this->get_logger(), "20cm 후진 실패");
      return false;
    }

    // 3단계: 리프트 올리기
    RCLCPP_INFO(this->get_logger(), "3단계: 리프트 올리기");
    send_lift_cmd(LIFT_SPEED);
    wait_for_duration(1.0);  // 1초간 리프트 올리기

    // 4단계: 리프트 정지
    RCLCPP_INFO(this->get_logger(), "4단계: 리프트 정지");
    send_lift_cmd(0.0);
    wait_for_duration(0.5);  // 안정화 대기

    auto end_time = std::chrono::steady_clock::now();
    duration = std::chrono::duration<double>(end_time - start_time).count();

    RCLCPP_INFO(this->get_logger(), "픽업 시퀀스 완료 (%.2f초)", duration);
    return true;

  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "픽업 시퀀스 중 오류: %s", e.what());
    return false;
  }
}

// ================ 헬퍼 메서드들 ================

void TurtleBot3::send_cmd_vel(double linear_x, double angular_z) {
  std::string sdk_msg;

  union Data {
    int32_t dword[6];
    uint8_t byte[4 * 6];
  } data;

  data.dword[0] = static_cast<int32_t>(linear_x * 100);
  data.dword[1] = 0;
  data.dword[2] = 0;
  data.dword[3] = 0;
  data.dword[4] = 0;
  data.dword[5] = static_cast<int32_t>(angular_z * 100);

  uint16_t start_addr = extern_control_table.cmd_velocity_linear_x.addr;
  uint16_t addr_length = (extern_control_table.cmd_velocity_angular_z.addr - extern_control_table.cmd_velocity_linear_x.addr) + extern_control_table.cmd_velocity_angular_z.length;

  uint8_t *p_data = &data.byte[0];

  dxl_sdk_wrapper_->set_data_to_device(start_addr, addr_length, p_data, &sdk_msg);

  RCLCPP_DEBUG(this->get_logger(), "정밀 제어 - lin_vel: %f ang_vel: %f", linear_x, angular_z);
}

void TurtleBot3::send_lift_cmd(double lift_z) {
  std::string sdk_msg;
  int32_t lift_value = static_cast<int32_t>(lift_z * 100);

  uint8_t lift_data[4] = {0};
  memcpy(lift_data, &lift_value, sizeof(lift_value));

  dxl_sdk_wrapper_->set_data_to_device(extern_control_table.cmd_velocity_lift.addr, extern_control_table.cmd_velocity_lift.length, lift_data, &sdk_msg);

  RCLCPP_DEBUG(this->get_logger(), "정밀 제어 - lift_vel: %f", lift_z);
}

void TurtleBot3::wait_for_duration(double seconds) {
  rclcpp::WallRate rate(50);  // 50Hz로 스핀하면서 대기
  auto start_time = std::chrono::steady_clock::now();

  while (rclcpp::ok()) {
    auto current_time = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration<double>(current_time - start_time).count();

    if (elapsed >= seconds) {
      break;
    }

    // 센서 데이터 계속 읽기
    dxl_sdk_wrapper_->read_data_set();

    rate.sleep();
  }
}

void TurtleBot3::stop_all_motors() {
  send_cmd_vel(0.0, 0.0);
  send_lift_cmd(0.0);
}