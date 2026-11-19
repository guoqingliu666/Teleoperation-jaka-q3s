// src/hand_control_service.hpp
#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>

class HandControlService : public rclcpp::Node {
 public:
  HandControlService();
  ~HandControlService() override;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
