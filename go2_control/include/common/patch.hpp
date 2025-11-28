/**
 * ROS2 Foxy compatibility patch for Unitree API messages
 * See: https://github.com/ros2/ros2/issues/1324
 */
#pragma once

#include <rclcpp/rclcpp.hpp>
#include "unitree_api/msg/request.hpp"
#include "unitree_api/msg/response.hpp"

namespace libstatistics_collector::topic_statistics_collector {

template <>
struct TimeStamp<unitree_api::msg::Response> {
  static std::pair<bool, int64_t> value(
      const unitree_api::msg::Response& /*m*/) {
    return std::make_pair(true, 0);
  }
};

}  // namespace libstatistics_collector::topic_statistics_collector

