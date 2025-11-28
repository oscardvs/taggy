/**
 * Unitree Go2 Sport Client Implementation
 * EDTH Hackathon - Laelaps AI
 */
#include "common/sport_client.hpp"
#include "common/time_tools.hpp"

SportClient::SportClient(rclcpp::Node* node) : node_(node) {
  req_publisher_ = node_->create_publisher<unitree_api::msg::Request>(
      "/api/sport/request", 10);
}

void SportClient::Damp(unitree_api::msg::Request& req) {
  req.header.identity.api_id = SportAPI::DAMP;
  req.header.identity.id = unitree::common::GetSystemUptimeInNanoseconds();
  req_publisher_->publish(req);
}

void SportClient::BalanceStand(unitree_api::msg::Request& req) {
  req.header.identity.api_id = SportAPI::BALANCE_STAND;
  req.header.identity.id = unitree::common::GetSystemUptimeInNanoseconds();
  req_publisher_->publish(req);
}

void SportClient::StopMove(unitree_api::msg::Request& req) {
  req.header.identity.api_id = SportAPI::STOP_MOVE;
  req.header.identity.id = unitree::common::GetSystemUptimeInNanoseconds();
  req_publisher_->publish(req);
}

void SportClient::StandUp(unitree_api::msg::Request& req) {
  req.header.identity.api_id = SportAPI::STAND_UP;
  req.header.identity.id = unitree::common::GetSystemUptimeInNanoseconds();
  req_publisher_->publish(req);
}

void SportClient::StandDown(unitree_api::msg::Request& req) {
  req.header.identity.api_id = SportAPI::STAND_DOWN;
  req.header.identity.id = unitree::common::GetSystemUptimeInNanoseconds();
  req_publisher_->publish(req);
}

void SportClient::RecoveryStand(unitree_api::msg::Request& req) {
  req.header.identity.api_id = SportAPI::RECOVERY_STAND;
  req.header.identity.id = unitree::common::GetSystemUptimeInNanoseconds();
  req_publisher_->publish(req);
}

void SportClient::Euler(unitree_api::msg::Request& req, float roll, float pitch, float yaw) {
  nlohmann::json js;
  js["x"] = roll;
  js["y"] = pitch;
  js["z"] = yaw;
  req.parameter = js.dump();
  req.header.identity.api_id = SportAPI::EULER;
  req.header.identity.id = unitree::common::GetSystemUptimeInNanoseconds();
  req_publisher_->publish(req);
}

void SportClient::Move(unitree_api::msg::Request& req, float vx, float vy, float vyaw) {
  nlohmann::json js;
  js["x"] = vx;
  js["y"] = vy;
  js["z"] = vyaw;
  req.parameter = js.dump();
  req.header.identity.api_id = SportAPI::MOVE;
  req.header.identity.id = unitree::common::GetSystemUptimeInNanoseconds();
  req_publisher_->publish(req);
}

void SportClient::Sit(unitree_api::msg::Request& req) {
  req.header.identity.api_id = SportAPI::SIT;
  req.header.identity.id = unitree::common::GetSystemUptimeInNanoseconds();
  req_publisher_->publish(req);
}

void SportClient::RiseSit(unitree_api::msg::Request& req) {
  req.header.identity.api_id = SportAPI::RISE_SIT;
  req.header.identity.id = unitree::common::GetSystemUptimeInNanoseconds();
  req_publisher_->publish(req);
}

void SportClient::SpeedLevel(unitree_api::msg::Request& req, int level) {
  nlohmann::json js;
  js["data"] = level;
  req.parameter = js.dump();
  req.header.identity.api_id = SportAPI::SPEED_LEVEL;
  req.header.identity.id = unitree::common::GetSystemUptimeInNanoseconds();
  req_publisher_->publish(req);
}

void SportClient::Hello(unitree_api::msg::Request& req) {
  req.header.identity.api_id = SportAPI::HELLO;
  req.header.identity.id = unitree::common::GetSystemUptimeInNanoseconds();
  req_publisher_->publish(req);
}

void SportClient::Stretch(unitree_api::msg::Request& req) {
  req.header.identity.api_id = SportAPI::STRETCH;
  req.header.identity.id = unitree::common::GetSystemUptimeInNanoseconds();
  req_publisher_->publish(req);
}

void SportClient::SwitchJoystick(unitree_api::msg::Request& req, bool flag) {
  nlohmann::json js;
  js["data"] = flag;
  req.parameter = js.dump();
  req.header.identity.api_id = SportAPI::SWITCH_JOYSTICK;
  req.header.identity.id = unitree::common::GetSystemUptimeInNanoseconds();
  req_publisher_->publish(req);
}

