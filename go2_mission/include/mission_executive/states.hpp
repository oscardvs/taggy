/**
 * Mission Executive State Definitions
 * 
 * Defines the FSM states, transitions, and controller button masks
 * for the Go2 autonomous search-and-find mission.
 * 
 * EDTH Hackathon - Oscar Devos
 */

#ifndef MISSION_EXECUTIVE__STATES_HPP_
#define MISSION_EXECUTIVE__STATES_HPP_

#include <cstdint>
#include <string>
#include <unordered_map>

namespace mission_executive {

/**
 * Mission States
 * 
 * IDLE      - Robot stationary, waiting for trigger
 * LISTENING - Waiting for voice command (target object)
 * SEARCHING - Exploring environment, looking for target
 * TRACKING  - Visual servoing towards detected target
 * ARRIVED   - Reached target, executing terminal action
 * FAILED    - Exploration complete without finding target
 */
enum class State {
  IDLE,
  LISTENING,
  SEARCHING,
  TRACKING,
  ARRIVED,
  FAILED
};

/**
 * State name lookup for logging/debugging
 */
inline std::string state_to_string(State state) {
  static const std::unordered_map<State, std::string> names = {
    {State::IDLE, "IDLE"},
    {State::LISTENING, "LISTENING"},
    {State::SEARCHING, "SEARCHING"},
    {State::TRACKING, "TRACKING"},
    {State::ARRIVED, "ARRIVED"},
    {State::FAILED, "FAILED"}
  };
  auto it = names.find(state);
  return (it != names.end()) ? it->second : "UNKNOWN";
}

/**
 * Unitree Go2 Wireless Controller Button Masks
 * 
 * The keys field in WirelessController.msg is a 16-bit bitmask
 * where each bit represents a button state.
 */
namespace buttons {

// Face buttons
constexpr uint16_t A      = 0x0001;  // Bit 0
constexpr uint16_t B      = 0x0002;  // Bit 1
constexpr uint16_t X      = 0x0004;  // Bit 2
constexpr uint16_t Y      = 0x0008;  // Bit 3

// D-Pad
constexpr uint16_t UP     = 0x0010;  // Bit 4
constexpr uint16_t DOWN   = 0x0020;  // Bit 5

// Shoulder buttons
constexpr uint16_t L1     = 0x0040;  // Bit 6
constexpr uint16_t R1     = 0x0080;  // Bit 7

// Triggers (analog, but also have digital state)
constexpr uint16_t L2     = 0x0100;  // Bit 8
constexpr uint16_t R2     = 0x0200;  // Bit 9

// Menu buttons
constexpr uint16_t SELECT = 0x0400;  // Bit 10
constexpr uint16_t START  = 0x0800;  // Bit 11

// Stick buttons
constexpr uint16_t L3     = 0x1000;  // Bit 12
constexpr uint16_t R3     = 0x2000;  // Bit 13

}  // namespace buttons

/**
 * Trigger mask for activating LISTENING state
 * L2 (Safety Enable) + A (Confirm) pressed simultaneously
 * This two-button combination prevents accidental activation
 */
constexpr uint16_t TRIGGER_MASK = buttons::L2 | buttons::A;

/**
 * Default timeout values (seconds)
 */
namespace timeouts {

constexpr double LISTENING_TIMEOUT = 10.0;  // Return to IDLE if no voice command
constexpr double TARGET_LOST_TIMEOUT = 2.0; // Return to SEARCHING if target lost
constexpr double ARRIVED_DELAY = 5.0;       // Delay before returning to IDLE
constexpr double FAILED_DELAY = 5.0;        // Delay before returning to IDLE

}  // namespace timeouts

/**
 * Default distance thresholds (meters)
 */
namespace thresholds {

constexpr double ARRIVAL_DISTANCE = 0.5;  // Target reached when distance < this

}  // namespace thresholds

/**
 * Posture command strings
 * Sent to /cmd_posture topic
 */
namespace postures {

constexpr const char* STAND = "stand";
constexpr const char* BALANCE = "balance";
constexpr const char* SIT = "sit";
constexpr const char* RECOVERY = "recovery";

}  // namespace postures

}  // namespace mission_executive

#endif  // MISSION_EXECUTIVE__STATES_HPP_

