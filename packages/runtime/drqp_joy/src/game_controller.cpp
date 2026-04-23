// Copyright 2020 Open Source Robotics Foundation, Inc.
// Copyright 2023 CSIRO Data61
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Open Source Robotics Foundation, Inc. nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// This file is a port of the ROS 2 joy package game_controller.cpp to SDL3,
// extended with dual-motor rumble support.
// Original: https://github.com/ros-drivers/joystick_drivers/blob/ros2/joy/src/game_controller.cpp

#include "drqp_joy/game_controller.hpp"
#include "drqp_joy/game_controller_utils.hpp"

#include <SDL3/SDL.h>

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joy_feedback.hpp>

namespace drqp_joy
{

namespace
{

constexpr SDL_InitFlags kRequiredSdlSubsystems = SDL_INIT_GAMEPAD;

std::mutex& sdl_init_mutex()
{
  static std::mutex mutex;
  return mutex;
}

size_t& sdl_init_ref_count()
{
  static size_t count = 0;
  return count;
}

void acquireSdlSubsystems()
{
  std::scoped_lock lock(sdl_init_mutex());

  if (sdl_init_ref_count() == 0 && !SDL_Init(kRequiredSdlSubsystems)) {
    throw std::runtime_error(
      "Failed to initialize SDL3 (gamepad and haptic subsystems): " + std::string(SDL_GetError()));
  }

  ++sdl_init_ref_count();
}

void releaseSdlSubsystems()
{
  std::scoped_lock lock(sdl_init_mutex());

  if (sdl_init_ref_count() == 0) {
    return;
  }

  --sdl_init_ref_count();
  if (sdl_init_ref_count() == 0) {
    SDL_QuitSubSystem(kRequiredSdlSubsystems);
  }
}

}  // namespace

GameController::GameController(const rclcpp::NodeOptions& options)
: rclcpp::Node("game_controller_node", options)
{
  dev_id_ = static_cast<int>(this->declare_parameter("device_id", 0));
  detail::validateDeviceId(dev_id_);
  dev_name_ = this->declare_parameter("device_name", std::string(""));

  // The user specifies the deadzone to us in the range of 0.0 to 1.0.  Later on
  // we'll convert that to the range of 0 to 32767.  Note also that negatives
  // are not allowed, as this is a +/- value.
  scaled_deadzone_ = this->declare_parameter("deadzone", 0.05);
  detail::validateDeadzone(scaled_deadzone_);
  unscaled_deadzone_ = 32767.0 * scaled_deadzone_;
  // According to the SDL documentation, this always returns a value between
  // -32768 and 32767.  However, we want to report a value between -1.0 and 1.0,
  // hence the "scale" dividing by 32767.  Also note that SDL returns the axes
  // with "forward" and "left" as negative.  This is opposite to the ROS
  // convention of "forward" and "left" as positive, so we invert the axes here
  // as well.  Finally, we take into account the amount of deadzone so we truly
  // do get a value between -1.0 and 1.0 (and not -deadzone to +deadzone).
  scale_ = static_cast<float>(-1.0 / (1.0 - scaled_deadzone_) / 32767.0);

  autorepeat_rate_ = this->declare_parameter("autorepeat_rate", 20.0);
  if (autorepeat_rate_ < 0.0) {
    throw std::runtime_error("Autorepeat rate must be >= 0.0");
  } else if (autorepeat_rate_ > 1000.0) {
    throw std::runtime_error("Autorepeat rate must be <= 1000.0");
  } else if (autorepeat_rate_ > 0.0) {
    autorepeat_interval_ms_ = static_cast<int>(1000.0 / autorepeat_rate_);
  } else {
    // If the autorepeat rate is set to 0, the user doesn't want us to
    // publish unless an event happens.  We still wake up every 200
    // milliseconds to check if we need to quit.
    autorepeat_interval_ms_ = 200;
  }

  sticky_buttons_ = this->declare_parameter("sticky_buttons", false);

  coalesce_interval_ms_ = static_cast<int>(this->declare_parameter("coalesce_interval_ms", 1));
  detail::validateCoalesceInterval(coalesce_interval_ms_);

  feedback_rumble_duration_ms_ =
    static_cast<uint32_t>(this->declare_parameter("feedback_rumble_duration_ms", 1000));
  // Make sure to initialize publish_soon_time regardless of whether we are going
  // to use it; this ensures that we are always using the correct time source.
  publish_soon_time_ = this->now();
  last_publish_time_ = publish_soon_time_;

  sdl_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  pub_ = create_publisher<sensor_msgs::msg::Joy>("joy", 10);

  rclcpp::SubscriptionOptions sdl_subscription_options;
  sdl_subscription_options.callback_group = sdl_callback_group_;

  // joy/set_feedback — rumble via sensor_msgs/JoyFeedback.
  //   id=0  → set both low-freq and high-freq motors to intensity
  //   id=1  → low-freq (heavy/left) motor only
  //   id=2  → high-freq (light/right) motor only
  feedback_sub_ = this->create_subscription<sensor_msgs::msg::JoyFeedback>(
    "joy/set_feedback", rclcpp::QoS(10),
    std::bind(&GameController::feedbackCb, this, std::placeholders::_1), sdl_subscription_options);

  joy_msg_.buttons.resize(SDL_GAMEPAD_BUTTON_COUNT);
  joy_msg_.axes.resize(SDL_GAMEPAD_AXIS_COUNT);

  RCLCPP_INFO(
    get_logger(), "Starting game controller node with device_id=%d device_name='%s'", dev_id_,
    dev_name_.c_str());

  acquireSdlSubsystems();
  discoverAvailableGamepads();

  const auto event_poll_period = std::chrono::milliseconds(
    detail::computeEventPollIntervalMs(autorepeat_interval_ms_, coalesce_interval_ms_));
  event_timer_ = this->create_wall_timer(
    event_poll_period, std::bind(&GameController::pollEvents, this), sdl_callback_group_);
}

GameController::~GameController()
{
  if (event_timer_ != nullptr) {
    event_timer_->cancel();
  }

  {
    std::scoped_lock lock(sdl_state_mutex_);
    if (game_controller_ != nullptr) {
      SDL_CloseGamepad(game_controller_);
      game_controller_ = nullptr;
    }
    joystick_instance_id_ = 0;
  }

  releaseSdlSubsystems();
}

// ── Private helpers ─────────────────────────────────────────────────────────

// ── Subscription callbacks ───────────────────────────────────────────────────

void GameController::feedbackCb(const std::shared_ptr<sensor_msgs::msg::JoyFeedback> msg)
{
  if (msg->type != sensor_msgs::msg::JoyFeedback::TYPE_RUMBLE) {
    return;
  }

  if (msg->id > 2) {
    RCLCPP_WARN(get_logger(), "JoyFeedback id must be 0 (both), 1 (low-freq) or 2 (high-freq)");
    return;
  }

  if (msg->intensity < 0.0 || msg->intensity > 1.0) {
    RCLCPP_WARN(get_logger(), "JoyFeedback intensity must be between 0.0 and 1.0");
    return;
  }

  std::scoped_lock lock(sdl_state_mutex_);
  if (game_controller_ == nullptr) {
    return;
  }

  const uint16_t intensity = static_cast<uint16_t>(msg->intensity * 0xFFFF);
  uint16_t low_freq = 0;
  uint16_t high_freq = 0;

  switch (msg->id) {
  case 0:  // Both motors
    low_freq = intensity;
    high_freq = intensity;
    break;
  case 1:  // Low-frequency (heavy/left) motor
    low_freq = intensity;
    break;
  case 2:  // High-frequency (light/right) motor
    high_freq = intensity;
    break;
  }

  // We purposely ignore the return value; if it fails, what can we do?
  SDL_RumbleGamepad(game_controller_, low_freq, high_freq, feedback_rumble_duration_ms_);
}
// ── Axis / button event handlers ──────────────────────────────────────────────

void GameController::discoverAvailableGamepads()
{
  int count = 0;
  SDL_JoystickID* ids = SDL_GetGamepads(&count);
  if (ids == nullptr || count <= 0) {
    RCLCPP_INFO(get_logger(), "No gamepads detected at startup; waiting for SDL hotplug events");
    return;
  }

  std::vector<SDL_JoystickID> joystick_ids(ids, ids + count);
  SDL_free(ids);

  for (const SDL_JoystickID joystick_id : joystick_ids) {
    const char* joystick_name = SDL_GetGamepadNameForID(joystick_id);
    if (matchesRequestedGamepad(joystick_id, joystick_name, true)) {
      openRequestedGamepad(joystick_id, joystick_name);
      return;
    }
  }

  if (dev_name_.empty()) {
    RCLCPP_INFO(
      get_logger(),
      "Detected %d gamepad(s) at startup, but none matched requested device_id=%d; waiting for "
      "hotplug events",
      count, dev_id_);
    return;
  }

  RCLCPP_INFO(
    get_logger(),
    "Detected %d gamepad(s) at startup, but none matched requested device_name='%s'; waiting for "
    "hotplug events",
    count, dev_name_.c_str());
}

bool GameController::matchesRequestedGamepad(
  SDL_JoystickID joystick_id, const char* joystick_name, bool log_mismatch)
{
  if (!dev_name_.empty()) {
    if (joystick_name == nullptr || dev_name_ != joystick_name) {
      if (log_mismatch) {
        RCLCPP_INFO(
          get_logger(), "Gamepad added: id=%u, name=%s — not the requested device_name '%s'",
          joystick_id, joystick_name ? joystick_name : "unknown", dev_name_.c_str());
      }
      return false;
    }

    return true;
  }

  int count = 0;
  SDL_JoystickID* ids = SDL_GetGamepads(&count);
  if (ids == nullptr) {
    if (log_mismatch) {
      RCLCPP_WARN(
        get_logger(), "Unable to enumerate gamepads while matching device_id=%d: %s", dev_id_,
        SDL_GetError());
    }
    return false;
  }

  const std::vector<SDL_JoystickID> joystick_ids(ids, ids + count);
  SDL_free(ids);

  const auto requested_id = detail::getRequestedDeviceByIndex(joystick_ids, dev_id_);
  if (!requested_id.has_value() || requested_id.value() != joystick_id) {
    if (log_mismatch) {
      RCLCPP_INFO(
        get_logger(), "Gamepad added: id=%u, name=%s — not at device_id=%d, ignoring", joystick_id,
        joystick_name ? joystick_name : "unknown", dev_id_);
    }
    return false;
  }

  return true;
}

void GameController::openRequestedGamepad(SDL_JoystickID joystick_id, const char* joystick_name)
{
  RCLCPP_INFO(
    get_logger(), "Gamepad added: id=%u, name=%s", joystick_id,
    joystick_name ? joystick_name : "unknown");

  std::scoped_lock lock(sdl_state_mutex_);
  if (game_controller_ != nullptr) {
    return;
  }

  game_controller_ = SDL_OpenGamepad(joystick_id);
  if (game_controller_ == nullptr) {
    RCLCPP_WARN(get_logger(), "Unable to open gamepad id=%u: %s", joystick_id, SDL_GetError());
    return;
  }

  joystick_instance_id_ = joystick_id;

  for (int i = 0; i < SDL_GAMEPAD_AXIS_COUNT; ++i) {
    const int16_t state = SDL_GetGamepadAxis(game_controller_, static_cast<SDL_GamepadAxis>(i));
    joy_msg_.axes.at(i) = convertRawAxisValueToROS(state);
  }

  RCLCPP_INFO(
    get_logger(), "Opened gamepad: %s  deadzone: %f  rumble: %s",
    SDL_GetGamepadName(game_controller_), scaled_deadzone_,
    SDL_GetBooleanProperty(
      SDL_GetGamepadProperties(game_controller_), SDL_PROP_GAMEPAD_CAP_RUMBLE_BOOLEAN, false)
      ? "Yes"
      : "No");
}

float GameController::convertRawAxisValueToROS(int16_t val)
{
  // SDL reports axis values between -32768 and 32767.  To make sure
  // we report out scaled value between -1.0 and 1.0, we add one to
  // the value iff it is exactly -32768.  This makes all of the math
  // below work properly.
  if (val == -32768) {
    val = -32767;
  }

  // Note that we do all of the math in double space below.  This ensures
  // that the values stay between -1.0 and 1.0.
  double double_val = static_cast<double>(val);
  // Apply the deadzone semantic here.  This allows the deadzone
  // to be "smooth".
  if (double_val > unscaled_deadzone_) {
    double_val -= unscaled_deadzone_;
  } else if (double_val < -unscaled_deadzone_) {
    double_val += unscaled_deadzone_;
  } else {
    double_val = 0.0;
  }

  return static_cast<float>(double_val * scale_);
}

bool GameController::handleGamepadAxis(const SDL_GamepadAxisEvent& e)
{
  bool publish = false;

  if (e.which != joystick_instance_id_) {
    return publish;
  }

  if (e.axis >= SDL_GAMEPAD_AXIS_COUNT) {
    RCLCPP_WARN(get_logger(), "Saw axis too large for this device, ignoring");
    return publish;
  }

  const float last_axis_value = joy_msg_.axes.at(e.axis);
  const float next_axis_value = convertRawAxisValueToROS(e.value);
  joy_msg_.axes.at(e.axis) = next_axis_value;
  if (detail::axisValueChanged(last_axis_value, next_axis_value)) {
    if (coalesce_interval_ms_ > 0 && !publish_soon_) {
      publish_soon_ = true;
      publish_soon_time_ = this->now();
    } else {
      const rclcpp::Duration time_since_publish_soon = this->now() - publish_soon_time_;
      if (time_since_publish_soon.nanoseconds() >= RCL_MS_TO_NS(coalesce_interval_ms_)) {
        publish = true;
        publish_soon_ = false;
      }
    }
  }

  return publish;
}

bool GameController::handleGamepadButtonDown(const SDL_GamepadButtonEvent& e)
{
  bool publish = false;

  if (e.which != joystick_instance_id_) {
    return publish;
  }

  if (e.button >= SDL_GAMEPAD_BUTTON_COUNT) {
    RCLCPP_WARN(get_logger(), "Saw button too large for this device, ignoring");
    return publish;
  }

  if (sticky_buttons_) {
    joy_msg_.buttons.at(e.button) = 1 - joy_msg_.buttons.at(e.button);
  } else {
    joy_msg_.buttons.at(e.button) = 1;
  }
  publish = true;

  return publish;
}

bool GameController::handleGamepadButtonUp(const SDL_GamepadButtonEvent& e)
{
  bool publish = false;

  if (e.which != joystick_instance_id_) {
    return publish;
  }

  if (e.button >= SDL_GAMEPAD_BUTTON_COUNT) {
    RCLCPP_WARN(get_logger(), "Saw button too large for this device, ignoring");
    return publish;
  }

  if (!sticky_buttons_) {
    joy_msg_.buttons.at(e.button) = 0;
    publish = true;
  }

  return publish;
}

// ── Device connect / disconnect ───────────────────────────────────────────────

void GameController::handleGamepadDeviceAdded(const SDL_GamepadDeviceEvent& e)
{
  const SDL_JoystickID new_id = e.which;
  const char* new_name = SDL_GetGamepadNameForID(new_id);

  if (!matchesRequestedGamepad(new_id, new_name, true)) {
    return;
  }

  openRequestedGamepad(new_id, new_name);
}

void GameController::handleGamepadDeviceRemoved(const SDL_GamepadDeviceEvent& e)
{
  if (e.which != joystick_instance_id_) {
    return;
  }

  std::scoped_lock lock(sdl_state_mutex_);
  if (game_controller_ != nullptr) {
    RCLCPP_INFO(get_logger(), "Gamepad removed: %s.", SDL_GetGamepadName(game_controller_));
    SDL_CloseGamepad(game_controller_);
    game_controller_ = nullptr;
  }
}

// ── Event loop ────────────────────────────────────────────────────────────────

void GameController::pollEvents()
{
  bool should_publish = false;
  SDL_Event e;
  while (SDL_PollEvent(&e)) {
    switch (e.type) {
    case SDL_EVENT_GAMEPAD_AXIS_MOTION:
      should_publish = handleGamepadAxis(e.gaxis) || should_publish;
      break;
    case SDL_EVENT_GAMEPAD_BUTTON_DOWN:
      should_publish = handleGamepadButtonDown(e.gbutton) || should_publish;
      break;
    case SDL_EVENT_GAMEPAD_BUTTON_UP:
      should_publish = handleGamepadButtonUp(e.gbutton) || should_publish;
      break;
    case SDL_EVENT_GAMEPAD_ADDED:
      handleGamepadDeviceAdded(e.gdevice);
      break;
    case SDL_EVENT_GAMEPAD_REMOVED:
      handleGamepadDeviceRemoved(e.gdevice);
      break;
    // Joystick events are duplicates of GAMEPAD events; suppress them.
    case SDL_EVENT_JOYSTICK_AXIS_MOTION:
    case SDL_EVENT_JOYSTICK_BALL_MOTION:
    case SDL_EVENT_JOYSTICK_HAT_MOTION:
    case SDL_EVENT_JOYSTICK_BUTTON_DOWN:
    case SDL_EVENT_JOYSTICK_BUTTON_UP:
    case SDL_EVENT_JOYSTICK_ADDED:
    case SDL_EVENT_JOYSTICK_REMOVED:
      break;
    default:
      RCLCPP_DEBUG(get_logger(), "Unhandled SDL event type 0x%x", e.type);
      break;
    }
  }

  const rclcpp::Time now = this->now();
  const rclcpp::Duration diff_since_last_publish = now - last_publish_time_;
  if (
    (autorepeat_rate_ > 0.0 &&
     RCL_NS_TO_MS(diff_since_last_publish.nanoseconds()) >= autorepeat_interval_ms_) ||
    publish_soon_) {
    should_publish = true;
    publish_soon_ = false;
  }

  if (game_controller_ != nullptr && should_publish) {
    last_publish_time_ = now;
    joy_msg_.header.frame_id = "joy";
    joy_msg_.header.stamp = now;
    pub_->publish(joy_msg_);
  }
}

}  // namespace drqp_joy

RCLCPP_COMPONENTS_REGISTER_NODE(drqp_joy::GameController)
