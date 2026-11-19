#include "hand_control_service.hpp"

#include "sdk_adapter.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "CANFDMaster.h"
#include "EthercatMaster.h"
#include "RS485Master.h"

#include "hand_control_interfaces/msg/hand_joint_state.hpp"
#include "hand_control_interfaces/srv/control_motors.hpp"
#include "hand_control_interfaces/srv/get_angle.hpp"
#include "hand_control_interfaces/srv/get_angular_velocity.hpp"
#include "hand_control_interfaces/srv/get_can_node_id.hpp"
#include "hand_control_interfaces/srv/get_canfd_arb_baudrate.hpp"
#include "hand_control_interfaces/srv/get_canfd_data_baudrate.hpp"
#include "hand_control_interfaces/srv/get_control_mode.hpp"
#include "hand_control_interfaces/srv/get_enable.hpp"
#include "hand_control_interfaces/srv/get_firmware_version.hpp"
#include "hand_control_interfaces/srv/get_home_current.hpp"
#include "hand_control_interfaces/srv/get_max_current.hpp"
#include "hand_control_interfaces/srv/get_now_alarm.hpp"
#include "hand_control_interfaces/srv/get_now_angle.hpp"
#include "hand_control_interfaces/srv/get_now_angular_velocity.hpp"
#include "hand_control_interfaces/srv/get_now_current.hpp"
#include "hand_control_interfaces/srv/get_now_position.hpp"
#include "hand_control_interfaces/srv/get_now_position_velocity.hpp"
#include "hand_control_interfaces/srv/get_now_status.hpp"
#include "hand_control_interfaces/srv/get_position.hpp"
#include "hand_control_interfaces/srv/get_position_reached.hpp"
#include "hand_control_interfaces/srv/get_position_velocity.hpp"
#include "hand_control_interfaces/srv/get_rs485_baudrate.hpp"
#include "hand_control_interfaces/srv/get_rs485_node_id.hpp"
#include "hand_control_interfaces/srv/get_safe_current_enable.hpp"
#include "hand_control_interfaces/srv/get_serial_number.hpp"
#include "hand_control_interfaces/srv/get_torque_reached.hpp"
#include "hand_control_interfaces/srv/home_motors.hpp"
#include "hand_control_interfaces/srv/move_motors.hpp"
#include "hand_control_interfaces/srv/play_gesture.hpp"
#include "hand_control_interfaces/srv/set_angle.hpp"
#include "hand_control_interfaces/srv/set_angular_velocity.hpp"
#include "hand_control_interfaces/srv/set_can_node_id.hpp"
#include "hand_control_interfaces/srv/set_canfd_arb_baudrate.hpp"
#include "hand_control_interfaces/srv/set_canfd_data_baudrate.hpp"
#include "hand_control_interfaces/srv/set_clear_alarm.hpp"
#include "hand_control_interfaces/srv/set_control_mode.hpp"
#include "hand_control_interfaces/srv/set_enable.hpp"
#include "hand_control_interfaces/srv/set_home_current.hpp"
#include "hand_control_interfaces/srv/set_max_current.hpp"
#include "hand_control_interfaces/srv/set_move_no_home.hpp"
#include "hand_control_interfaces/srv/set_position.hpp"
#include "hand_control_interfaces/srv/set_position_velocity.hpp"
#include "hand_control_interfaces/srv/set_rs485_baudrate.hpp"
#include "hand_control_interfaces/srv/set_rs485_node_id.hpp"
#include "hand_control_interfaces/srv/set_safe_current_enable.hpp"
#include "hand_control_interfaces/srv/stop_motors.hpp"

#ifndef COMMUNICATION_MODE
#define COMMUNICATION_MODE 1
#endif
#ifndef CANFD_NODE_ID
#define CANFD_NODE_ID 1
#endif
#ifndef CANFD_ARB_BAUDRATE
#define CANFD_ARB_BAUDRATE 1000000
#endif
#ifndef CANFD_DATA_BAUDRATE
#define CANFD_DATA_BAUDRATE 5000000
#endif
#ifndef RS485_NODE_ID
#define RS485_NODE_ID 1
#endif
#ifndef RS485_BAUDRATE
#define RS485_BAUDRATE 500000
#endif

#ifndef HAND_TYPE
#define HAND_TYPE HC_SDK_NS::LAC_DOF_6
#endif

namespace {

constexpr const char* SRV_NAME_SET_ENABLE = "set_enable";
constexpr const char* SRV_NAME_GET_ENABLE = "get_enable";
constexpr const char* SRV_NAME_GET_NOW_ALARM = "get_now_alarm";
constexpr const char* SRV_NAME_SET_CLEAR_ALARM = "set_clear_alarm";
constexpr const char* SRV_NAME_SET_ANGLE = "set_angle";
constexpr const char* SRV_NAME_GET_ANGLE = "get_angle";
constexpr const char* SRV_NAME_SET_POSITION = "set_position";
constexpr const char* SRV_NAME_GET_POSITION = "get_position";
constexpr const char* SRV_NAME_SET_ANGULAR_VELOCITY =
    "set_angular_velocity";
constexpr const char* SRV_NAME_GET_ANGULAR_VELOCITY =
    "get_angular_velocity";
constexpr const char* SRV_NAME_GET_NOW_ANGLE = "get_now_angle";
constexpr const char* SRV_NAME_GET_NOW_ANGULAR_VELOCITY =
    "get_now_angular_velocity";
constexpr const char* SRV_NAME_GET_NOW_POSITION = "get_now_position";
constexpr const char* SRV_NAME_GET_NOW_POSITION_VELOCITY =
    "get_now_position_velocity";
constexpr const char* SRV_NAME_GET_NOW_CURRENT = "get_now_current";
constexpr const char* SRV_NAME_GET_NOW_STATUS = "get_now_status";
constexpr const char* SRV_NAME_GET_POSITION_VELOCITY =
    "get_position_velocity";
constexpr const char* SRV_NAME_SET_POSITION_VELOCITY =
    "set_position_velocity";
constexpr const char* SRV_NAME_GET_MAX_CURRENT = "get_max_current";
constexpr const char* SRV_NAME_SET_MAX_CURRENT = "set_max_current";
constexpr const char* SRV_NAME_GET_CONTROL_MODE = "get_control_mode";
constexpr const char* SRV_NAME_SET_CONTROL_MODE = "set_control_mode";
constexpr const char* SRV_NAME_GET_POSITION_REACHED =
    "get_position_reached";
constexpr const char* SRV_NAME_GET_TORQUE_REACHED = "get_torque_reached";
constexpr const char* SRV_NAME_SET_MOVE_NO_HOME = "set_move_no_home";
constexpr const char* SRV_NAME_GET_FIRMWARE_VERSION =
    "get_firmware_version";
constexpr const char* SRV_NAME_GET_SERIAL_NUMBER = "get_serial_number";
constexpr const char* SRV_NAME_SET_SAFE_CURRENT_ENABLE =
    "set_safe_current_enable";
constexpr const char* SRV_NAME_GET_SAFE_CURRENT_ENABLE =
    "get_safe_current_enable";
constexpr const char* SRV_NAME_SET_HOME_CURRENT = "set_home_current";
constexpr const char* SRV_NAME_GET_HOME_CURRENT = "get_home_current";
constexpr const char* SRV_NAME_SET_CAN_NODE_ID = "set_can_node_id";
constexpr const char* SRV_NAME_GET_CAN_NODE_ID = "get_can_node_id";
constexpr const char* SRV_NAME_SET_CANFD_ARB_BAUDRATE =
    "set_canfd_arb_baudrate";
constexpr const char* SRV_NAME_GET_CANFD_ARB_BAUDRATE =
    "get_canfd_arb_baudrate";
constexpr const char* SRV_NAME_SET_CANFD_DATA_BAUDRATE =
    "set_canfd_data_baudrate";
constexpr const char* SRV_NAME_GET_CANFD_DATA_BAUDRATE =
    "get_canfd_data_baudrate";
constexpr const char* SRV_NAME_SET_RS485_NODE_ID = "set_rs485_node_id";
constexpr const char* SRV_NAME_GET_RS485_NODE_ID = "get_rs485_node_id";
constexpr const char* SRV_NAME_SET_RS485_BAUDRATE = "set_rs485_baudrate";
constexpr const char* SRV_NAME_GET_RS485_BAUDRATE = "get_rs485_baudrate";
constexpr const char* SRV_NAME_HOME_MOTORS = "home_motors";
constexpr const char* SRV_NAME_MOVE_MOTORS = "move_motors";
constexpr const char* SRV_NAME_STOP_MOTORS = "stop_motors";
constexpr const char* SRV_NAME_PLAY_GESTURE = "play_gesture";
constexpr const char* SRV_NAME_CONTROL_MOTORS = "control_motors";
constexpr const char* TOPIC_NAME_REALTIME_STATE = "realtime_state";

std::string toLower(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(), [](char c) {
    return static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  });
  return value;
}

enum class BusType { Ethercat, Canfd, Rs485 };

std::string busTypeName(BusType type) {
  switch (type) {
    case BusType::Ethercat:
      return "ethercat";
    case BusType::Canfd:
      return "canfd";
    case BusType::Rs485:
      return "rs485";
  }
  return "unknown";
}

BusType parseBusType(const std::string& value) {
  const std::string type = toLower(value);
  if (type == "ethercat" || type == "ecat") return BusType::Ethercat;
  if (type == "canfd") return BusType::Canfd;
  if (type == "rs485") return BusType::Rs485;
  throw std::runtime_error("Unsupported bus type: " + value);
}

std::string defaultBusType() {
#if COMMUNICATION_MODE == 1
  return "canfd";
#elif COMMUNICATION_MODE == 2
  return "rs485";
#else
  return "ethercat";
#endif
}

int defaultAddressForType(BusType type) {
  switch (type) {
    case BusType::Canfd:
      return CANFD_NODE_ID;
    case BusType::Rs485:
      return RS485_NODE_ID;
    case BusType::Ethercat:
      return 1;
  }
  return 1;
}

int parseHandType(const std::string& value) {
  const std::string type = toLower(value);
  if (type == "lac_dof_6" || type == "dof_6") {
    return HC_SDK_NS::LAC_DOF_6;
  }
  if (type == "lac_dof_6_s" || type == "dof_6_s") {
    return HC_SDK_NS::LAC_DOF_6_S;
  }
  if (type == "lac_dof_16" || type == "dof_16") {
    return HC_SDK_NS::LAC_DOF_16;
  }
  try {
    return std::stoi(value);
  } catch (...) {
    return HAND_TYPE;
  }
}

struct BusConfig {
  std::string name;
  BusType type{BusType::Canfd};
  int channel{0};
  std::string interface;
  int canfd_arb_baudrate{CANFD_ARB_BAUDRATE};
  int canfd_data_baudrate{CANFD_DATA_BAUDRATE};
  int rs485_baudrate{RS485_BAUDRATE};
};

struct HandConfig {
  std::string name;
  std::string bus_name;
  int address{1};
  int hand_type{HAND_TYPE};
};

class BusBase;

class HandDevice {
 public:
  explicit HandDevice(HandConfig config)
      : name_(std::move(config.name)),
        bus_name_(std::move(config.bus_name)),
        address_(config.address),
        hand_type_(config.hand_type),
        sdk_(std::make_shared<HC_SDK_NS::HC_SDK_CLASS>()) {}

  ~HandDevice() { close(); }

  const std::string& name() const { return name_; }
  const std::string& busName() const { return bus_name_; }
  int address() const { return address_; }
  int handType() const { return hand_type_; }
  int activeDof() const { return active_dof_; }
  BusType busType() const { return bus_type_; }
  std::shared_ptr<HC_SDK_NS::HC_SDK_CLASS> sdk() const { return sdk_; }
  void setBus(const std::shared_ptr<BusBase>& bus, BusType type) {
    bus_ = bus;
    bus_type_ = type;
  }

  void setAddress(int address) { address_ = address; }

  void setCanfdSendFunction(
      std::function<bool(unsigned int, const unsigned char*, unsigned int, int)>
          func) {
    canfd_send_function_ = std::move(func);
    sdk_->set_send_canfd_callback_ex(&canfd_send_function_);
  }

  void setRs485SendFunction(
      std::function<bool(const unsigned char*, unsigned int)> func) {
    rs485_send_function_ = std::move(func);
    sdk_->set_send_rs485_callback_ex(&rs485_send_function_);
  }

  void setEthercatSendFunction(
      std::function<bool(const unsigned char*, unsigned int)> func) {
    ethercat_send_function_ = std::move(func);
    sdk_->set_send_rpdo_callback_ex(&ethercat_send_function_);
  }

  void setSiiIdentityVerified(bool value) { sii_identity_verified_ = value; }

  void setEthercatSdoFunction(std::function<bool(unsigned int, unsigned char, int, unsigned int, unsigned int*)> fn) {
    ethercat_sdo_function_ = std::move(fn);
    sdk_->set_ecat_sdo_callback_ex(&ethercat_sdo_function_);
  }

  bool initialize(BusType type, rclcpp::Logger logger) {
    sdk_->set_hand_type(hand_type_);

    int result = HC_SDK_NS::LER_NONE;
    if (type == BusType::Ethercat) {
      result = sdk_->initial(HC_SDK_NS::LCN_ECAT);
    } else if (type == BusType::Canfd) {
      result = sdk_->initial(HC_SDK_NS::LCN_CANFD,
                             static_cast<unsigned int>(address_));
    } else {
      result = sdk_->initial(HC_SDK_NS::LCN_RS485,
                             static_cast<unsigned int>(address_));
    }

    // Manufacturer SDK documents LER_COMM_RECV as metadata unavailable,
    // while TPDO decoding/monitoring is initialized. Accept this only after
    // the independently read, exact SII vendor/product/revision match.
    const bool metadata_unavailable = type == BusType::Ethercat && sii_identity_verified_ && result == HC_SDK_NS::LER_COMM_RECV;
    if (metadata_unavailable) RCLCPP_WARN(logger, "DH116_SDK_METADATA_UNAVAILABLE code=%d; SII identity verified; awaiting decoded TPDO", result);
    if (result != HC_SDK_NS::LER_NONE && !metadata_unavailable) {
      RCLCPP_ERROR(logger, "[%s] 设备 SDK 初始化失败: %d", name_.c_str(),
                   result);
      initialized_ = false;
      return false;
    }

    float firmware = 0;
    char serial[128]{};
    const bool metadata_read = sdk_->get_firmware_version(&firmware) == HC_SDK_NS::LER_NONE && firmware > 0 &&
        sdk_->get_serial_number(serial, sizeof(serial)) == HC_SDK_NS::LER_NONE && serial[0];
    if (metadata_read) RCLCPP_INFO(logger, "DH116_DEVICE_ID firmware=%.3f serial=%s", firmware, serial);
    int total = 0;
    int active = 0;
    sdk_->get_dof(&total, &active);
    active_dof_ = active;
    last_now_angles_.assign(active_dof_ + 1, 0.0f);
    has_last_now_angle_.assign(active_dof_ + 1, false);
    initialized_ = true;
    RCLCPP_INFO(logger, "[%s] 初始化完成 bus=%s address=%d dof=%d/%d",
                name_.c_str(), bus_name_.c_str(), address_, active, total);
    return true;
  }

  void decodeCanfd(uint32_t id, const uint8_t* data, int size) {
    if (!initialized_ || !data || size <= 0) return;
    sdk_->set_canfd_data_decode(id, data, size);
  }

  void decodeRs485(const uint8_t* data, int size) {
    if (!initialized_ || !data || size <= 0) return;
    sdk_->set_rs485_data_decode(data, size);
  }

  void decodeEthercat(const uint8_t* data, int size) {
    if (!data || size <= 0) return;
    const int result = sdk_->set_tpdo_data_decode(data, size);
    if (result == HC_SDK_NS::LER_NONE) {
      last_decode_ns_.store(std::chrono::steady_clock::now().time_since_epoch().count());
      if (!decoded_seen_.exchange(true)) {
        std::ostringstream sample;
        for (int i = 0; i < std::min(size, 32); ++i) sample << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(data[i]);
        std::cout << "DH116_TPDO_DECODE_OK bytes=" << size << " first32=" << sample.str() << std::endl;
      }
    }
  }

  int readNowAngle(int joint_id, float* angle, bool* from_cache) {
    if (!angle) return HC_SDK_NS::LER_UNKNOWN;
    if (from_cache) *from_cache = false;

    return readNowAngleLocked(joint_id, angle, from_cache);
  }

  int readNowAngularVelocity(int joint_id, float* velocity) {
    if (!velocity) return HC_SDK_NS::LER_UNKNOWN;
    return sdk_->get_now_angular_velocity(joint_id, velocity);
  }

  int readNowPosition(int joint_id, int* position) {
    if (!position) return HC_SDK_NS::LER_UNKNOWN;
    return sdk_->get_now_position(joint_id, position);
  }

  int readNowPositionVelocity(int joint_id, int* velocity) {
    if (!velocity) return HC_SDK_NS::LER_UNKNOWN;
    return sdk_->get_now_position_velocity(joint_id, velocity);
  }

  int readNowCurrent(int joint_id, int* current) {
    if (!current) return HC_SDK_NS::LER_UNKNOWN;
    return sdk_->get_now_current(joint_id, current);
  }

  int readNowStatus(int joint_id, int* status) {
    if (!status) return HC_SDK_NS::LER_UNKNOWN;
    return sdk_->get_now_status(joint_id, status);
  }

  int readNowAlarm(int joint_id, int* alarm) {
    if (!alarm) return HC_SDK_NS::LER_UNKNOWN;
    return sdk_->get_now_alarm(joint_id, alarm);
  }

  bool readRealtimeState(hand_control_interfaces::msg::HandJointState& msg) {
    if (!initialized_) return false;
    if (bus_type_ == BusType::Ethercat && (!decoded_seen_.load() ||
        std::chrono::steady_clock::now().time_since_epoch().count() - last_decode_ns_.load() > 200000000)) return false;

    msg.hand_name = name_;
    msg.bus_name = bus_name_;
    msg.address = address_;
    msg.active_dof = active_dof_;
    msg.joint_ids.clear();
    msg.angles.clear();
    msg.angular_velocities.clear();
    msg.positions.clear();
    msg.position_velocities.clear();
    msg.currents.clear();
    msg.statuses.clear();
    msg.alarms.clear();

    msg.joint_ids.reserve(active_dof_);
    msg.angles.reserve(active_dof_);
    msg.angular_velocities.reserve(active_dof_);
    msg.positions.reserve(active_dof_);
    msg.position_velocities.reserve(active_dof_);
    msg.currents.reserve(active_dof_);
    msg.statuses.reserve(active_dof_);
    msg.alarms.reserve(active_dof_);

    for (int joint_id = 1; joint_id <= active_dof_; ++joint_id) {
      float angle = 0.0f;
      if (sdk_->get_now_angle(joint_id, &angle) != HC_SDK_NS::LER_NONE || !std::isfinite(angle)) return false;

      float angular_velocity = 0.0f;
      if (sdk_->get_now_angular_velocity(joint_id, &angular_velocity) !=
          HC_SDK_NS::LER_NONE) {
        return false;
      }

      int position = 0;
      if (sdk_->get_now_position(joint_id, &position) !=
          HC_SDK_NS::LER_NONE) {
        return false;
      }

      int position_velocity = 0;
      if (sdk_->get_now_position_velocity(joint_id, &position_velocity) !=
          HC_SDK_NS::LER_NONE) {
        return false;
      }

      int current = 0;
      if (sdk_->get_now_current(joint_id, &current) !=
          HC_SDK_NS::LER_NONE) {
        return false;
      }

      int status = -1;
      if (sdk_->get_now_status(joint_id, &status) !=
          HC_SDK_NS::LER_NONE) {
        return false;
      }

      int alarm = 0;
      if (sdk_->get_now_alarm(joint_id, &alarm) !=
          HC_SDK_NS::LER_NONE) {
        return false;
      }

      if (!std::isfinite(angular_velocity)) return false;
      msg.joint_ids.push_back(joint_id);
      msg.angles.push_back(static_cast<double>(angle));
      msg.angular_velocities.push_back(static_cast<double>(angular_velocity));
      msg.positions.push_back(position);
      msg.position_velocities.push_back(position_velocity);
      msg.currents.push_back(static_cast<double>(current));
      msg.statuses.push_back(status);
      msg.alarms.push_back(alarm);
    }

    return true;
  }

  void close() {
    if (!sdk_) return;
    sdk_->close();
    initialized_ = false;
  }

  bool isInitialized() const { return initialized_; }

  bool isBusAlive() const;

 private:
  int readNowAngleLocked(int joint_id, float* angle, bool* from_cache) {
    float value = 0.0f;
    const int result = sdk_->get_now_angle(joint_id, &value);
    if (result == HC_SDK_NS::LER_NONE && std::isfinite(value)) {
      *angle = value;
      if (joint_id >= 0 &&
          joint_id < static_cast<int>(last_now_angles_.size())) {
        last_now_angles_[joint_id] = value;
        has_last_now_angle_[joint_id] = true;
      }
      return result;
    }

    if (joint_id >= 0 &&
        joint_id < static_cast<int>(last_now_angles_.size()) &&
        has_last_now_angle_[joint_id]) {
      *angle = last_now_angles_[joint_id];
      if (from_cache) *from_cache = true;
      return result;
    }

    *angle = 0.0f;
    return result;
  }
  std::string name_;
  std::string bus_name_;
  int address_;
  int hand_type_;
  int active_dof_{0};
  BusType bus_type_{BusType::Canfd};
  std::shared_ptr<HC_SDK_NS::HC_SDK_CLASS> sdk_;
  std::weak_ptr<BusBase> bus_;
  std::atomic<bool> initialized_{false};
  std::atomic<bool> decoded_seen_{false};
  bool sii_identity_verified_{false};
  std::atomic<int64_t> last_decode_ns_{0};
  std::function<bool(unsigned int, unsigned char, int, unsigned int, unsigned int*)> ethercat_sdo_function_;
  std::vector<float> last_now_angles_;
  std::vector<bool> has_last_now_angle_;
  std::function<bool(unsigned int, const unsigned char*, unsigned int, int)>
      canfd_send_function_;
  std::function<bool(const unsigned char*, unsigned int)> rs485_send_function_;
  std::function<bool(const unsigned char*, unsigned int)>
      ethercat_send_function_;
};

class BusBase : public std::enable_shared_from_this<BusBase> {
 public:
  explicit BusBase(BusConfig config) : config_(std::move(config)) {}
  virtual ~BusBase() = default;

  const std::string& name() const { return config_.name; }
  BusType type() const { return config_.type; }

  virtual bool start(rclcpp::Logger logger) = 0;
  virtual void stop() = 0;
  virtual bool isAlive() const = 0;
  virtual bool attachHand(const std::shared_ptr<HandDevice>& hand,
                          rclcpp::Logger logger) = 0;

  bool restart(rclcpp::Logger logger) {
    stop();
    const bool ok = start(logger);
    if (!ok) {
      RCLCPP_ERROR(logger, "[%s] 总线重连失败", name().c_str());
    }
    return ok;
  }

 protected:
  std::vector<std::shared_ptr<HandDevice>> attachedHands() const {
    std::lock_guard<std::mutex> lock(hands_mutex_);
    return hands_;
  }

  void addHand(const std::shared_ptr<HandDevice>& hand) {
    std::lock_guard<std::mutex> lock(hands_mutex_);
    hands_.push_back(hand);
  }

  BusConfig config_;
  mutable std::mutex hands_mutex_;
  std::vector<std::shared_ptr<HandDevice>> hands_;
};

bool HandDevice::isBusAlive() const {
  auto bus = bus_.lock();
  return bus && bus->isAlive();
}

class CanfdBus : public BusBase {
 public:
  explicit CanfdBus(BusConfig config) : BusBase(std::move(config)) {}
  ~CanfdBus() override { stop(); }

  bool start(rclcpp::Logger logger) override {
    if (master_ && master_->isConnected()) return true;

    master_ = std::make_shared<CANFDMaster>();
    auto devices = master_->scanDevices();
    if (devices.empty()) {
      RCLCPP_ERROR(logger, "[%s] 未找到 CANFD 通道", name().c_str());
      return false;
    }
    if (config_.channel < 0 ||
        config_.channel >= static_cast<int>(devices.size())) {
      RCLCPP_ERROR(logger, "[%s] CANFD 通道索引无效: %d", name().c_str(),
                   config_.channel);
      return false;
    }

    if (!master_->connect(config_.channel, config_.canfd_arb_baudrate,
                          config_.canfd_data_baudrate)) {
      RCLCPP_ERROR(logger, "[%s] CANFD 连接失败", name().c_str());
      return false;
    }

    master_->setReceiveCallback(
        [this](uint32_t id, const std::vector<uint8_t>& data,
               uint64_t /*timestamp*/) {
          for (const auto& hand : attachedHands()) {
            hand->decodeCanfd(id, data.data(), static_cast<int>(data.size()));
          }
        });

    RCLCPP_INFO(logger, "[%s] CANFD 已连接: channel=%d arb=%d data=%d",
                name().c_str(), config_.channel, config_.canfd_arb_baudrate,
                config_.canfd_data_baudrate);
    return true;
  }

  void stop() override {
    if (master_) {
      master_->disconnect();
    }
  }

  bool isAlive() const override {
    return master_ && master_->isConnected();
  }

  bool attachHand(const std::shared_ptr<HandDevice>& hand,
                  rclcpp::Logger logger) override {
    if (!isAlive()) return false;
    hand->setBus(shared_from_this(), type());
    hand->setCanfdSendFunction(
        [this](unsigned int id, const unsigned char* data, unsigned int size,
               int externflag) {
          std::lock_guard<std::mutex> lock(send_mutex_);
          return master_ && master_->sendData(id, data, size, externflag);
        });
    addHand(hand);
    return hand->initialize(type(), logger);
  }

 private:
  std::shared_ptr<CANFDMaster> master_;
  std::mutex send_mutex_;
};

class Rs485Bus : public BusBase {
 public:
  explicit Rs485Bus(BusConfig config) : BusBase(std::move(config)) {}
  ~Rs485Bus() override { stop(); }

  bool start(rclcpp::Logger logger) override {
    if (master_ && master_->isConnected()) return true;

    master_ = std::make_shared<RS485Master>();
    auto devices = master_->scanDevices();
    if (devices.empty()) {
      RCLCPP_ERROR(logger, "[%s] 未找到 RS485 串口", name().c_str());
      return false;
    }
    if (config_.channel < 0 ||
        config_.channel >= static_cast<int>(devices.size())) {
      RCLCPP_ERROR(logger, "[%s] RS485 串口索引无效: %d", name().c_str(),
                   config_.channel);
      return false;
    }

    if (!master_->connect(config_.channel, config_.rs485_baudrate)) {
      RCLCPP_ERROR(logger, "[%s] RS485 连接失败", name().c_str());
      return false;
    }

    master_->setReceiveCallback([this](const uint8_t* data, size_t size) {
      for (const auto& hand : attachedHands()) {
        hand->decodeRs485(data, static_cast<int>(size));
      }
    });

    RCLCPP_INFO(logger, "[%s] RS485 已连接: channel=%d baudrate=%d",
                name().c_str(), config_.channel, config_.rs485_baudrate);
    return true;
  }

  void stop() override {
    if (master_) {
      master_->disconnect();
    }
  }

  bool isAlive() const override {
    return master_ && master_->isConnected();
  }

  bool attachHand(const std::shared_ptr<HandDevice>& hand,
                  rclcpp::Logger logger) override {
    if (!isAlive()) return false;
    hand->setBus(shared_from_this(), type());
    hand->setRs485SendFunction(
        [this](const unsigned char* data, unsigned int size) {
          std::lock_guard<std::mutex> lock(send_mutex_);
          return master_ && master_->sendData(data, size);
        });
    addHand(hand);
    return hand->initialize(type(), logger);
  }

 private:
  std::shared_ptr<RS485Master> master_;
  std::mutex send_mutex_;
};

class EthercatBus : public BusBase {
 public:
  explicit EthercatBus(BusConfig config) : BusBase(std::move(config)) {}
  ~EthercatBus() override { stop(); }

  bool start(rclcpp::Logger logger) override {
    if (master_ &&
        master_->getState() == EthercatMaster::EthercatState::Operational) {
      ensureMonitorRunning();
      return true;
    }

    master_ = std::make_shared<EthercatMaster>();
    master_->setStatusCallback([logger](const std::string& message) {
      RCLCPP_INFO(logger, "EtherCAT: %s", message.c_str());
    });
    auto devices = master_->scanNetworkInterfaces();
    if (devices.empty()) {
      RCLCPP_ERROR(logger, "[%s] 未找到 EtherCAT 网口", name().c_str());
      return false;
    }
    if (config_.channel < 0 ||
        config_.channel >= static_cast<int>(devices.size())) {
      RCLCPP_ERROR(logger, "[%s] EtherCAT 网口索引无效: %d", name().c_str(),
                   config_.channel);
      return false;
    }
    if (config_.interface.empty()) {
      RCLCPP_ERROR(logger, "Explicit wired interface name required for this adapter");
      return false;
    }
    if (!master_->initInterface(config_.interface) || !master_->start()) {
      RCLCPP_ERROR(logger, "[%s] EtherCAT 启动失败", name().c_str());
      return false;
    }
    master_->run();
    ensureMonitorRunning();
    RCLCPP_INFO(logger, "[%s] EtherCAT 已连接: channel=%d slaves=%d",
                name().c_str(), config_.channel, master_->getSlaveCount());
    return true;
  }

  void stop() override {
    monitor_running_ = false;
    if (monitor_thread_.joinable()) {
      monitor_thread_.join();
    }
    if (master_) {
      master_->stop();
    }
  }

  bool isAlive() const override {
    return master_ &&
           master_->getState() == EthercatMaster::EthercatState::Operational;
  }

  bool attachHand(const std::shared_ptr<HandDevice>& hand,
                  rclcpp::Logger logger) override {
    if (!isAlive()) return false;
    const int slave_id = hand->address();
    if (master_->getSlaveInputSize(slave_id) <= 0 ||
        master_->getSlaveOutputSize(slave_id) <= 0) {
      RCLCPP_ERROR(logger, "[%s/%s] EtherCAT slave_id 无效或无 PDO: %d",
                   name().c_str(), hand->name().c_str(), slave_id);
      return false;
    }
    hand->setBus(shared_from_this(), type());
    hand->setEthercatSendFunction(
        [this, slave_id](const unsigned char* data, unsigned int size) {
          return master_ && master_->setSlaveOutputs(slave_id, data, size);
        });
    if (!master_->verifyDh116Identity(slave_id)) return false;
    hand->setSiiIdentityVerified(true);
    hand->setEthercatSdoFunction([this, slave_id](unsigned int index, unsigned char subindex, int is_write, unsigned int, unsigned int* value) {
      if (is_write || !master_) return false;
      return master_->sdoRead(static_cast<uint16_t>(index), subindex, value, slave_id);
    });
    addHand(hand);
    return hand->initialize(type(), logger);
  }

 private:
  void ensureMonitorRunning() {
    if (monitor_running_ || !master_) return;
    monitor_running_ = true;
    monitor_thread_ = std::thread([this]() {
      while (monitor_running_) {
        for (const auto& hand : attachedHands()) {
          const int slave_id = hand->address();
          const int input_size = master_->getSlaveInputSize(slave_id);
          if (input_size <= 0) continue;
          std::vector<uint8_t> buffer(input_size);
          if (master_->getSlaveInputs(slave_id, buffer.data(), input_size)) {
            hand->decodeEthercat(buffer.data(), input_size);
          }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    });
  }

  std::shared_ptr<EthercatMaster> master_;
  std::atomic<bool> monitor_running_{false};
  std::thread monitor_thread_;
};

std::shared_ptr<BusBase> makeBus(const BusConfig& config) {
  switch (config.type) {
    case BusType::Ethercat:
      return std::make_shared<EthercatBus>(config);
    case BusType::Canfd:
      return std::make_shared<CanfdBus>(config);
    case BusType::Rs485:
      return std::make_shared<Rs485Bus>(config);
  }
  return nullptr;
}

std::string serviceName(const std::string& prefix, const char* name) {
  if (prefix.empty()) return name;
  return prefix + "/" + name;
}

}  // namespace

struct HandControlService::Impl {
  explicit Impl(HandControlService* node) : node_(node) {
    loadConfig();
    createBuses();
    createHands();
    startBuses();
    attachHands();
    RCLCPP_INFO(logger(), "Feedback-only adapter: motor services are not registered");
    registerRealtimeStatePublishers();
    reconnect_timer_ = node_->create_wall_timer(
        std::chrono::seconds(5), [this]() { checkAndReconnect(); });
  }

  ~Impl() {
    realtime_state_timer_.reset();
    realtime_state_publishers_.clear();
    services_.clear();
    hands_.clear();
    for (auto& item : buses_) {
      item.second->stop();
    }
    buses_.clear();
  }

  rclcpp::Logger logger() const { return node_->get_logger(); }

  using HandJointState = hand_control_interfaces::msg::HandJointState;

  struct RealtimeStatePublisher {
    std::shared_ptr<HandDevice> hand;
    rclcpp::Publisher<HandJointState>::SharedPtr publisher;
  };

  void loadConfig() {
    const std::string default_type = defaultBusType();
    bus_names_ =
        node_->declare_parameter<std::vector<std::string>>("bus_names",
                                                           std::vector<std::string>{
                                                               "bus0"});
    hand_names_ =
        node_->declare_parameter<std::vector<std::string>>("hand_names",
                                                           std::vector<std::string>{
                                                               "hand"});
    validateNameList(bus_names_, "bus_names");
    validateNameList(hand_names_, "hand_names");

    compat_single_hand_services_ =
        node_->declare_parameter<bool>("compat_single_hand_services", true);
    default_hand_name_ =
        node_->declare_parameter<std::string>("default_hand", hand_names_[0]);
    publish_realtime_state_ =
        node_->declare_parameter<bool>("publish_realtime_state", true);
    realtime_state_publish_rate_hz_ = node_->declare_parameter<double>(
        "realtime_state_publish_rate_hz", 100.0);

    for (const auto& bus_name : bus_names_) {
      BusConfig cfg;
      cfg.name = bus_name;
      cfg.type = parseBusType(node_->declare_parameter<std::string>(
          "buses." + bus_name + ".type", default_type));
      cfg.channel =
          node_->declare_parameter<int>("buses." + bus_name + ".channel", 0);
      cfg.interface = node_->declare_parameter<std::string>("buses." + bus_name + ".interface", "");
      cfg.canfd_arb_baudrate = node_->declare_parameter<int>(
          "buses." + bus_name + ".canfd_arb_baudrate", CANFD_ARB_BAUDRATE);
      cfg.canfd_data_baudrate = node_->declare_parameter<int>(
          "buses." + bus_name + ".canfd_data_baudrate", CANFD_DATA_BAUDRATE);
      cfg.rs485_baudrate = node_->declare_parameter<int>(
          "buses." + bus_name + ".rs485_baudrate", RS485_BAUDRATE);
      bus_configs_[bus_name] = cfg;
    }

    for (const auto& hand_name : hand_names_) {
      HandConfig cfg;
      cfg.name = hand_name;
      cfg.bus_name = node_->declare_parameter<std::string>(
          "hands." + hand_name + ".bus", bus_names_[0]);
      auto bus_it = bus_configs_.find(cfg.bus_name);
      if (bus_it == bus_configs_.end()) {
        throw std::runtime_error("Hand " + hand_name +
                                 " references unknown bus " + cfg.bus_name);
      }
      cfg.address = node_->declare_parameter<int>(
          "hands." + hand_name + ".address",
          defaultAddressForType(bus_it->second.type));
      const std::string hand_type = node_->declare_parameter<std::string>(
          "hands." + hand_name + ".hand_type", "LAC_DOF_6");
      cfg.hand_type = parseHandType(hand_type);
      hand_configs_[hand_name] = cfg;
    }

    validateTopology();
  }

  void validateNameList(const std::vector<std::string>& names,
                        const std::string& param_name) const {
    if (names.empty()) {
      throw std::runtime_error(param_name + " must not be empty");
    }
    std::set<std::string> unique_names;
    for (const auto& name : names) {
      if (name.empty()) {
        throw std::runtime_error(param_name + " contains an empty name");
      }
      if (!unique_names.insert(name).second) {
        throw std::runtime_error(param_name + " contains duplicate name: " +
                                 name);
      }
    }
  }

  void validateTopology() const {
    std::set<std::pair<std::string, int>> used_addresses;
    for (const auto& item : hand_configs_) {
      const auto key = std::make_pair(item.second.bus_name, item.second.address);
      if (!used_addresses.insert(key).second) {
        throw std::runtime_error("Duplicate address " +
                                 std::to_string(item.second.address) +
                                 " on bus " + item.second.bus_name);
      }
    }
  }

  void createBuses() {
    for (const auto& item : bus_configs_) {
      buses_[item.first] = makeBus(item.second);
      RCLCPP_INFO(logger(), "配置总线: %s type=%s channel=%d",
                  item.first.c_str(), busTypeName(item.second.type).c_str(),
                  item.second.channel);
    }
  }

  void createHands() {
    for (const auto& item : hand_configs_) {
      hands_[item.first] = std::make_shared<HandDevice>(item.second);
      RCLCPP_INFO(logger(), "配置手: %s bus=%s address=%d",
                  item.second.name.c_str(), item.second.bus_name.c_str(),
                  item.second.address);
    }
  }

  void startBuses() {
    for (auto& item : buses_) {
      item.second->start(logger());
    }
  }

  void attachHands() {
    for (auto& item : hands_) {
      const auto& hand = item.second;
      auto bus_it = buses_.find(hand->busName());
      if (bus_it == buses_.end()) {
        RCLCPP_ERROR(logger(), "[%s] 找不到总线 %s", hand->name().c_str(),
                     hand->busName().c_str());
        continue;
      }
      if (!bus_it->second->attachHand(hand, logger())) {
        RCLCPP_ERROR(logger(), "[%s] 绑定到总线 %s 失败", hand->name().c_str(),
                     hand->busName().c_str());
      }
    }
  }

  bool checkReady(const std::shared_ptr<HandDevice>& hand,
                  const char* service_name) const {
    if (!hand || !hand->isInitialized()) {
      RCLCPP_ERROR(logger(), "[%s] 手未初始化", service_name);
      return false;
    }
    if (!hand->isBusAlive()) {
      RCLCPP_ERROR(logger(), "[%s] 总线未连接或通信异常", service_name);
      return false;
    }
    return true;
  }

  bool checkBusType(const std::shared_ptr<HandDevice>& hand, BusType type,
                    const char* service_name) const {
    if (!checkReady(hand, service_name)) return false;
    if (hand->busType() != type) {
      RCLCPP_ERROR(logger(), "[%s] 当前手使用 %s 总线，不适用 %s 配置",
                   service_name, busTypeName(hand->busType()).c_str(),
                   busTypeName(type).c_str());
      return false;
    }
    return true;
  }

  bool hasAddressConflict(const std::shared_ptr<HandDevice>& hand,
                          int address) const {
    for (const auto& item : hands_) {
      const auto& other = item.second;
      if (other.get() == hand.get()) continue;
      if (other->busName() == hand->busName() &&
          other->address() == address) {
        return true;
      }
    }
    return false;
  }

  bool checkJoint(const std::shared_ptr<HandDevice>& hand, int joint_id,
                  const char* service_name) const {
    if (!checkReady(hand, service_name)) return false;
    if (joint_id < 0 || joint_id > hand->activeDof()) {
      RCLCPP_ERROR(logger(), "[%s] 无效的关节ID: %d", service_name, joint_id);
      return false;
    }
    return true;
  }

  template <typename SrvT, typename Callback>
  void addService(const std::shared_ptr<HandDevice>& hand,
                  const std::string& prefix, const char* name,
                  Callback&& callback) {
    const std::string full_name = serviceName(prefix, name);
    services_.push_back(node_->create_service<SrvT>(
        full_name, [hand, cb = std::forward<Callback>(callback)](
                       const std::shared_ptr<typename SrvT::Request> req,
                       std::shared_ptr<typename SrvT::Response> res) {
          cb(hand, req, res);
        }));
  }

  void registerAllServices() {
    for (const auto& item : hands_) {
      registerServicesForHand(item.second, item.second->name());
    }

    auto default_it = hands_.find(default_hand_name_);
    if (compat_single_hand_services_ && default_it != hands_.end()) {
      registerServicesForHand(default_it->second, "");
      RCLCPP_INFO(logger(), "已注册兼容单手服务，默认手: %s",
                  default_hand_name_.c_str());
    }
  }

  void registerRealtimeStatePublishers() {
    if (!publish_realtime_state_) {
      RCLCPP_INFO(logger(), "实时状态话题发布已关闭");
      return;
    }

    for (const auto& item : hands_) {
      addRealtimeStatePublisher(item.second, item.second->name());
    }

    auto default_it = hands_.find(default_hand_name_);
    if (compat_single_hand_services_ && default_it != hands_.end()) {
      addRealtimeStatePublisher(default_it->second, "");
      RCLCPP_INFO(logger(), "已注册兼容实时状态话题，默认手: %s",
                  default_hand_name_.c_str());
    }

    if (realtime_state_publishers_.empty()) return;

    if (realtime_state_publish_rate_hz_ <= 0.0) {
      realtime_state_publish_rate_hz_ = 100.0;
    }

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / realtime_state_publish_rate_hz_));
    realtime_state_timer_ = node_->create_wall_timer(
        period, [this]() { publishRealtimeStates(); });
    RCLCPP_INFO(logger(), "实时状态话题发布频率: %.1f Hz",
                realtime_state_publish_rate_hz_);
  }

  void addRealtimeStatePublisher(const std::shared_ptr<HandDevice>& hand,
                                 const std::string& prefix) {
    const std::string topic_name =
        serviceName(prefix, TOPIC_NAME_REALTIME_STATE);
    auto publisher = node_->create_publisher<HandJointState>(
        topic_name, rclcpp::QoS(10));
    realtime_state_publishers_.push_back({hand, publisher});
    RCLCPP_INFO(logger(), "[%s] 实时状态话题: %s", hand->name().c_str(),
                topic_name.c_str());
  }

  void publishRealtimeStates() {
    for (const auto& item : realtime_state_publishers_) {
      const auto& hand = item.hand;
      if (!hand || !hand->isInitialized() || !hand->isBusAlive()) continue;

      HandJointState msg;
      msg.header.stamp = node_->now();
      msg.header.frame_id = hand->name();
      if (hand->readRealtimeState(msg)) {
        item.publisher->publish(msg);
      }
    }
  }

  void registerServicesForHand(const std::shared_ptr<HandDevice>& hand,
                               const std::string& prefix) {
    using namespace hand_control_interfaces::srv;

    addService<SetEnable>(
        hand, prefix, SRV_NAME_SET_ENABLE,
        [this](auto h, auto req, auto res) {
          if (!checkJoint(h, req->joint_id, SRV_NAME_SET_ENABLE)) {
            res->result = -1;
            return;
          }
          res->result = h->sdk()->set_enable(req->joint_id, req->enable);
        });
    addService<GetEnable>(
        hand, prefix, SRV_NAME_GET_ENABLE,
        [this](auto h, auto req, auto res) {
          if (!checkJoint(h, req->joint_id, SRV_NAME_GET_ENABLE)) {
            res->enable = 0;
            return;
          }
          int value = 0;
          h->sdk()->get_enable(req->joint_id, &value);
          res->enable = value;
        });
    addService<GetNowAlarm>(
        hand, prefix, SRV_NAME_GET_NOW_ALARM,
        [this](auto h, auto req, auto res) {
          if (!checkJoint(h, req->joint_id, SRV_NAME_GET_NOW_ALARM)) {
            res->alarm = 0;
            return;
          }
          int value = 0;
          h->readNowAlarm(req->joint_id, &value);
          res->alarm = value;
        });
    addService<SetClearAlarm>(
        hand, prefix, SRV_NAME_SET_CLEAR_ALARM,
        [this](auto h, auto req, auto res) {
          if (!checkJoint(h, req->joint_id, SRV_NAME_SET_CLEAR_ALARM)) {
            res->result = -1;
            return;
          }
          res->result = h->sdk()->set_clear_alarm(req->joint_id);
        });
    addService<SetAngle>(
        hand, prefix, SRV_NAME_SET_ANGLE,
        [this](auto h, auto req, auto res) {
          if (!checkJoint(h, req->joint_id, SRV_NAME_SET_ANGLE)) {
            res->result = -1;
            return;
          }
          res->result = h->sdk()->set_target_angle(
              req->joint_id, static_cast<float>(req->angle));
        });
    addService<GetAngle>(
        hand, prefix, SRV_NAME_GET_ANGLE,
        [this](auto h, auto req, auto res) {
          if (!checkJoint(h, req->joint_id, SRV_NAME_GET_ANGLE)) {
            res->angle = 0.0;
            return;
          }
          float value = 0.0f;
          h->sdk()->get_target_angle(req->joint_id, &value);
          res->angle = value;
        });
    addService<SetPosition>(
        hand, prefix, SRV_NAME_SET_POSITION,
        [this](auto h, auto req, auto res) {
          if (!checkJoint(h, req->joint_id, SRV_NAME_SET_POSITION)) {
            res->result = -1;
            return;
          }
          res->result =
              h->sdk()->set_target_position(req->joint_id, req->position);
        });
    addService<GetPosition>(
        hand, prefix, SRV_NAME_GET_POSITION,
        [this](auto h, auto req, auto res) {
          if (!checkJoint(h, req->joint_id, SRV_NAME_GET_POSITION)) {
            res->position = 0;
            return;
          }
          int value = 0;
          h->sdk()->get_target_position(req->joint_id, &value);
          res->position = value;
        });
    addService<SetAngularVelocity>(
        hand, prefix, SRV_NAME_SET_ANGULAR_VELOCITY,
        [this](auto h, auto req, auto res) {
          if (!checkJoint(h, req->joint_id, SRV_NAME_SET_ANGULAR_VELOCITY)) {
            res->result = -1;
            return;
          }
          res->result = h->sdk()->set_angular_velocity(
              req->joint_id, static_cast<float>(req->velocity));
        });
    addService<GetAngularVelocity>(
        hand, prefix, SRV_NAME_GET_ANGULAR_VELOCITY,
        [this](auto h, auto req, auto res) {
          if (!checkJoint(h, req->joint_id, SRV_NAME_GET_ANGULAR_VELOCITY)) {
            res->velocity = 0.0;
            return;
          }
          float value = 0.0f;
          h->sdk()->get_angular_velocity(req->joint_id, &value);
          res->velocity = value;
        });
    addService<GetNowAngle>(
        hand, prefix, SRV_NAME_GET_NOW_ANGLE,
        [this](auto h, auto req, auto res) {
          if (!checkJoint(h, req->joint_id, SRV_NAME_GET_NOW_ANGLE)) {
            res->angle = 0.0;
            return;
          }
          float value = 0.0f;
          bool from_cache = false;
          const int result =
              h->readNowAngle(req->joint_id, &value, &from_cache);
          if (result != HC_SDK_NS::LER_NONE) {
            RCLCPP_WARN_THROTTLE(
                logger(), *node_->get_clock(), 2000,
                "[%s] get_now_angle joint=%d 读取失败: %d%s",
                h->name().c_str(), req->joint_id, result,
                from_cache ? "，使用上一帧有效角度" : "");
          }
          res->angle = value;
        });
    addService<GetNowAngularVelocity>(
        hand, prefix, SRV_NAME_GET_NOW_ANGULAR_VELOCITY,
        [this](auto h, auto req, auto res) {
          if (!checkJoint(h, req->joint_id,
                          SRV_NAME_GET_NOW_ANGULAR_VELOCITY)) {
            res->velocity = 0.0;
            return;
          }
          float value = 0.0f;
          h->readNowAngularVelocity(req->joint_id, &value);
          res->velocity = value;
        });
    addService<GetNowPosition>(
        hand, prefix, SRV_NAME_GET_NOW_POSITION,
        [this](auto h, auto req, auto res) {
          if (!checkJoint(h, req->joint_id, SRV_NAME_GET_NOW_POSITION)) {
            res->position = 0;
            return;
          }
          int value = 0;
          h->readNowPosition(req->joint_id, &value);
          res->position = value;
        });
    addService<GetNowPositionVelocity>(
        hand, prefix, SRV_NAME_GET_NOW_POSITION_VELOCITY,
        [this](auto h, auto req, auto res) {
          if (!checkJoint(h, req->joint_id,
                          SRV_NAME_GET_NOW_POSITION_VELOCITY)) {
            res->velocity = 0;
            return;
          }
          int value = 0;
          h->readNowPositionVelocity(req->joint_id, &value);
          res->velocity = value;
        });
    addService<GetNowCurrent>(
        hand, prefix, SRV_NAME_GET_NOW_CURRENT,
        [this](auto h, auto req, auto res) {
          if (!checkJoint(h, req->joint_id, SRV_NAME_GET_NOW_CURRENT)) {
            res->current = 0.0;
            return;
          }
          int value = 0;
          h->readNowCurrent(req->joint_id, &value);
          res->current = static_cast<float>(value);
        });
    addService<GetNowStatus>(
        hand, prefix, SRV_NAME_GET_NOW_STATUS,
        [this](auto h, auto req, auto res) {
          if (!checkJoint(h, req->joint_id, SRV_NAME_GET_NOW_STATUS)) {
            res->status = -1;
            return;
          }
          int value = 0;
          h->readNowStatus(req->joint_id, &value);
          res->status = value;
        });
    addService<GetPositionVelocity>(
        hand, prefix, SRV_NAME_GET_POSITION_VELOCITY,
        [this](auto h, auto req, auto res) {
          if (!checkJoint(h, req->joint_id, SRV_NAME_GET_POSITION_VELOCITY)) {
            res->velocity = 0;
            return;
          }
          int value = 0;
          h->sdk()->get_position_velocity(req->joint_id, &value);
          res->velocity = value;
        });
    addService<SetPositionVelocity>(
        hand, prefix, SRV_NAME_SET_POSITION_VELOCITY,
        [this](auto h, auto req, auto res) {
          if (!checkJoint(h, req->joint_id, SRV_NAME_SET_POSITION_VELOCITY)) {
            res->result = -1;
            return;
          }
          res->result =
              h->sdk()->set_position_velocity(req->joint_id, req->velocity);
        });
    addService<GetMaxCurrent>(
        hand, prefix, SRV_NAME_GET_MAX_CURRENT,
        [this](auto h, auto req, auto res) {
          if (!checkJoint(h, req->joint_id, SRV_NAME_GET_MAX_CURRENT)) {
            res->current = 0.0;
            return;
          }
          int value = 0;
          h->sdk()->get_max_current(req->joint_id, &value);
          res->current = static_cast<float>(value);
        });
    addService<SetMaxCurrent>(
        hand, prefix, SRV_NAME_SET_MAX_CURRENT,
        [this](auto h, auto req, auto res) {
          if (!checkJoint(h, req->joint_id, SRV_NAME_SET_MAX_CURRENT)) {
            res->result = -1;
            return;
          }
          res->result = h->sdk()->set_max_current(req->joint_id, req->current);
        });
    addService<GetControlMode>(
        hand, prefix, SRV_NAME_GET_CONTROL_MODE,
        [this](auto h, auto req, auto res) {
          if (!checkJoint(h, req->joint_id, SRV_NAME_GET_CONTROL_MODE)) {
            res->mode = -1;
            return;
          }
          int value = 0;
          h->sdk()->get_control_mode(req->joint_id, &value);
          res->mode = value;
        });
    addService<SetControlMode>(
        hand, prefix, SRV_NAME_SET_CONTROL_MODE,
        [this](auto h, auto req, auto res) {
          if (!checkJoint(h, req->joint_id, SRV_NAME_SET_CONTROL_MODE)) {
            res->result = -1;
            return;
          }
          res->result = h->sdk()->set_control_mode(req->joint_id, req->mode);
        });
    addService<GetPositionReached>(
        hand, prefix, SRV_NAME_GET_POSITION_REACHED,
        [this](auto h, auto req, auto res) {
          if (!checkJoint(h, req->joint_id, SRV_NAME_GET_POSITION_REACHED)) {
            res->reached = 0;
            return;
          }
          int value = 0;
          h->sdk()->get_position_reached(req->joint_id, &value);
          res->reached = value;
        });
    addService<GetTorqueReached>(
        hand, prefix, SRV_NAME_GET_TORQUE_REACHED,
        [this](auto h, auto req, auto res) {
          if (!checkJoint(h, req->joint_id, SRV_NAME_GET_TORQUE_REACHED)) {
            res->reached = 0;
            return;
          }
          int value = 0;
          h->sdk()->get_torque_reached(req->joint_id, &value);
          res->reached = value;
        });
    addService<SetMoveNoHome>(
        hand, prefix, SRV_NAME_SET_MOVE_NO_HOME,
        [this](auto h, auto req, auto res) {
          if (!checkReady(h, SRV_NAME_SET_MOVE_NO_HOME)) {
            res->result = -1;
            return;
          }
          res->result = h->sdk()->set_move_no_home(req->enable);
        });
    addService<GetFirmwareVersion>(
        hand, prefix, SRV_NAME_GET_FIRMWARE_VERSION,
        [this](auto h, auto req, auto res) {
          (void)req;
          if (!checkReady(h, SRV_NAME_GET_FIRMWARE_VERSION)) {
            res->version = 0.0;
            return;
          }
          float value = 0.0f;
          h->sdk()->get_firmware_version(&value);
          res->version = value;
        });
    addService<GetSerialNumber>(
        hand, prefix, SRV_NAME_GET_SERIAL_NUMBER,
        [this](auto h, auto req, auto res) {
          (void)req;
          if (!checkReady(h, SRV_NAME_GET_SERIAL_NUMBER)) {
            res->serial_number = "";
            return;
          }
          char value[128] = {};
          h->sdk()->get_serial_number(value, static_cast<int>(sizeof(value)));
          res->serial_number = value;
        });
    addService<SetSafeCurrentEnable>(
        hand, prefix, SRV_NAME_SET_SAFE_CURRENT_ENABLE,
        [this](auto h, auto req, auto res) {
          if (!checkReady(h, SRV_NAME_SET_SAFE_CURRENT_ENABLE)) {
            res->result = -1;
            return;
          }
          res->result = h->sdk()->set_safe_current_enable(req->enable);
        });
    addService<GetSafeCurrentEnable>(
        hand, prefix, SRV_NAME_GET_SAFE_CURRENT_ENABLE,
        [this](auto h, auto req, auto res) {
          (void)req;
          if (!checkReady(h, SRV_NAME_GET_SAFE_CURRENT_ENABLE)) {
            res->enable = 0;
            return;
          }
          int value = 0;
          h->sdk()->get_safe_current_enable(&value);
          res->enable = value;
        });
    addService<SetHomeCurrent>(
        hand, prefix, SRV_NAME_SET_HOME_CURRENT,
        [this](auto h, auto req, auto res) {
          if (!checkReady(h, SRV_NAME_SET_HOME_CURRENT)) {
            res->result = -1;
            return;
          }
          res->result = h->sdk()->set_home_current(req->current);
        });
    addService<GetHomeCurrent>(
        hand, prefix, SRV_NAME_GET_HOME_CURRENT,
        [this](auto h, auto req, auto res) {
          (void)req;
          if (!checkReady(h, SRV_NAME_GET_HOME_CURRENT)) {
            res->current = 0;
            return;
          }
          int value = 0;
          h->sdk()->get_home_current(&value);
          res->current = value;
        });
    addService<SetCanNodeId>(
        hand, prefix, SRV_NAME_SET_CAN_NODE_ID,
        [this](auto h, auto req, auto res) {
          if (!checkBusType(h, BusType::Canfd, SRV_NAME_SET_CAN_NODE_ID) ||
              req->node_id <= 0 ||
              hasAddressConflict(h, req->node_id)) {
            res->result = -1;
            return;
          }
          res->result = h->sdk()->set_can_node_id(req->node_id);
          if (res->result == 0 && req->node_id > 0) h->setAddress(req->node_id);
        });
    addService<GetCanNodeId>(
        hand, prefix, SRV_NAME_GET_CAN_NODE_ID,
        [this](auto h, auto req, auto res) {
          (void)req;
          if (!checkBusType(h, BusType::Canfd, SRV_NAME_GET_CAN_NODE_ID)) {
            res->node_id = -1;
            return;
          }
          int value = h->address();
          h->sdk()->get_can_node_id(&value);
          res->node_id = value;
        });
    addService<SetCanfdArbBaudrate>(
        hand, prefix, SRV_NAME_SET_CANFD_ARB_BAUDRATE,
        [this](auto h, auto req, auto res) {
          if (!checkBusType(h, BusType::Canfd,
                            SRV_NAME_SET_CANFD_ARB_BAUDRATE)) {
            res->result = -1;
            return;
          }
          res->result = h->sdk()->set_canfd_arb_baudrate(req->baudrate);
        });
    addService<GetCanfdArbBaudrate>(
        hand, prefix, SRV_NAME_GET_CANFD_ARB_BAUDRATE,
        [this](auto h, auto req, auto res) {
          (void)req;
          if (!checkBusType(h, BusType::Canfd,
                            SRV_NAME_GET_CANFD_ARB_BAUDRATE)) {
            res->baudrate = -1;
            return;
          }
          int value = CANFD_ARB_BAUDRATE;
          h->sdk()->get_canfd_arb_baudrate(&value);
          res->baudrate = value;
        });
    addService<SetCanfdDataBaudrate>(
        hand, prefix, SRV_NAME_SET_CANFD_DATA_BAUDRATE,
        [this](auto h, auto req, auto res) {
          if (!checkBusType(h, BusType::Canfd,
                            SRV_NAME_SET_CANFD_DATA_BAUDRATE)) {
            res->result = -1;
            return;
          }
          res->result = h->sdk()->set_canfd_data_baudrate(req->baudrate);
        });
    addService<GetCanfdDataBaudrate>(
        hand, prefix, SRV_NAME_GET_CANFD_DATA_BAUDRATE,
        [this](auto h, auto req, auto res) {
          (void)req;
          if (!checkBusType(h, BusType::Canfd,
                            SRV_NAME_GET_CANFD_DATA_BAUDRATE)) {
            res->baudrate = -1;
            return;
          }
          int value = CANFD_DATA_BAUDRATE;
          h->sdk()->get_canfd_data_baudrate(&value);
          res->baudrate = value;
        });
    addService<SetRs485NodeId>(
        hand, prefix, SRV_NAME_SET_RS485_NODE_ID,
        [this](auto h, auto req, auto res) {
          if (!checkBusType(h, BusType::Rs485, SRV_NAME_SET_RS485_NODE_ID) ||
              req->node_id <= 0 ||
              hasAddressConflict(h, req->node_id)) {
            res->result = -1;
            return;
          }
          res->result = h->sdk()->set_rs485_node_id(req->node_id);
          if (res->result == 0 && req->node_id > 0) h->setAddress(req->node_id);
        });
    addService<GetRs485NodeId>(
        hand, prefix, SRV_NAME_GET_RS485_NODE_ID,
        [this](auto h, auto req, auto res) {
          (void)req;
          if (!checkBusType(h, BusType::Rs485, SRV_NAME_GET_RS485_NODE_ID)) {
            res->node_id = -1;
            return;
          }
          int value = h->address();
          h->sdk()->get_rs485_node_id(&value);
          res->node_id = value;
        });
    addService<SetRs485Baudrate>(
        hand, prefix, SRV_NAME_SET_RS485_BAUDRATE,
        [this](auto h, auto req, auto res) {
          if (!checkBusType(h, BusType::Rs485, SRV_NAME_SET_RS485_BAUDRATE)) {
            res->result = -1;
            return;
          }
          res->result = h->sdk()->set_rs485_baudrate(req->baudrate);
        });
    addService<GetRs485Baudrate>(
        hand, prefix, SRV_NAME_GET_RS485_BAUDRATE,
        [this](auto h, auto req, auto res) {
          (void)req;
          if (!checkBusType(h, BusType::Rs485, SRV_NAME_GET_RS485_BAUDRATE)) {
            res->baudrate = -1;
            return;
          }
          int value = RS485_BAUDRATE;
          h->sdk()->get_rs485_baudrate(&value);
          res->baudrate = value;
        });
    addService<HomeMotors>(
        hand, prefix, SRV_NAME_HOME_MOTORS,
        [this](auto h, auto req, auto res) {
          if (!checkJoint(h, req->joint_id, SRV_NAME_HOME_MOTORS)) {
            res->result = -1;
            return;
          }
          res->result = h->sdk()->home_motors(req->joint_id);
        });
    addService<MoveMotors>(
        hand, prefix, SRV_NAME_MOVE_MOTORS,
        [this](auto h, auto req, auto res) {
          if (!checkJoint(h, req->joint_id, SRV_NAME_MOVE_MOTORS)) {
            res->result = -1;
            return;
          }
          res->result = h->sdk()->move_motors(req->joint_id);
        });
    addService<ControlMotors>(
        hand, prefix, SRV_NAME_CONTROL_MOTORS,
        [this](auto h, auto req, auto res) {
          if (!checkReady(h, SRV_NAME_CONTROL_MOTORS) || h->activeDof() < 6) {
            res->result = -1;
            return;
          }

          const int positions[6] = {req->position_1, req->position_2,
                                    req->position_3, req->position_4,
                                    req->position_5, req->position_6};
          const int velocities[6] = {req->velocity_1, req->velocity_2,
                                     req->velocity_3, req->velocity_4,
                                     req->velocity_5, req->velocity_6};
          const int currents[6] = {req->current_1, req->current_2,
                                   req->current_3, req->current_4,
                                   req->current_5, req->current_6};

          for (int i = 0; i < 6; ++i) {
            const int joint_id = i + 1;
            int result = h->sdk()->set_target_position(joint_id, positions[i]);
            if (result != 0) {
              res->result = result;
              return;
            }
            result = h->sdk()->set_position_velocity(joint_id, velocities[i]);
            if (result != 0) {
              res->result = result;
              return;
            }
            result = h->sdk()->set_max_current(joint_id, currents[i]);
            if (result != 0) {
              res->result = result;
              return;
            }
          }

          res->result = h->sdk()->move_motors(0);
        });
    addService<StopMotors>(
        hand, prefix, SRV_NAME_STOP_MOTORS,
        [this](auto h, auto req, auto res) {
          if (!checkJoint(h, req->joint_id, SRV_NAME_STOP_MOTORS)) {
            res->result = -1;
            return;
          }
          res->result = h->sdk()->stop_motors(req->joint_id);
        });
    addService<PlayGesture>(
        hand, prefix, SRV_NAME_PLAY_GESTURE,
        [this](auto h, auto req, auto res) {
          if (!checkReady(h, SRV_NAME_PLAY_GESTURE)) {
            res->result = -1;
            return;
          }
          res->result = h->sdk()->play_gesture(req->gesture_id, req->velocity,
                                               req->current);
        });
  }

  void checkAndReconnect() {
    for (auto& item : buses_) {
      if (!item.second->isAlive()) {
        RCLCPP_WARN(logger(), "[%s] 总线异常，尝试重连", item.first.c_str());
        item.second->restart(logger());
      }
    }
  }

  HandControlService* node_;
  std::vector<std::string> bus_names_;
  std::vector<std::string> hand_names_;
  std::string default_hand_name_;
  bool compat_single_hand_services_{true};
  bool publish_realtime_state_{true};
  double realtime_state_publish_rate_hz_{100.0};
  std::unordered_map<std::string, BusConfig> bus_configs_;
  std::unordered_map<std::string, HandConfig> hand_configs_;
  std::unordered_map<std::string, std::shared_ptr<BusBase>> buses_;
  std::unordered_map<std::string, std::shared_ptr<HandDevice>> hands_;
  std::vector<rclcpp::ServiceBase::SharedPtr> services_;
  std::vector<RealtimeStatePublisher> realtime_state_publishers_;
  rclcpp::TimerBase::SharedPtr reconnect_timer_;
  rclcpp::TimerBase::SharedPtr realtime_state_timer_;
};

HandControlService::HandControlService() : Node("hand_control_service") {
  RCLCPP_INFO(this->get_logger(), "手部控制服务已创建");
  impl_ = std::make_unique<Impl>(this);
}

HandControlService::~HandControlService() = default;
