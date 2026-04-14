/*
 * Copyright (c) 2020, Open Source Robotics Foundation.
 * Copyright (c) 2023, CSIRO Data61.
 * Copyright (c) 2024-2025, Anton Matosov (SDL3 migration, dual-motor rumble, haptics)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// This file is a port of the ROS 2 joy package game_controller.cpp to SDL3,
// extended with dual-motor rumble and SDL haptic effect support.
// Original: https://github.com/ros-drivers/joystick_drivers/blob/ros2/joy/src/game_controller.cpp

#include <algorithm>
#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include <SDL3/SDL.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joy_feedback.hpp>

#include "drqp_interfaces/msg/haptic_effect.hpp"
#include "drqp_joy/game_controller.hpp"

namespace drqp_joy
{

using HapticEffect = drqp_interfaces::msg::HapticEffect;

GameController::GameController(const rclcpp::NodeOptions & options)
: rclcpp::Node("game_controller_node", options)
{
  dev_id_ = static_cast<int>(this->declare_parameter("device_id", 0));
  dev_name_ = this->declare_parameter("device_name", std::string(""));

  // The user specifies the deadzone to us in the range of 0.0 to 1.0.  Later on
  // we'll convert that to the range of 0 to 32767.  Note also that negatives
  // are not allowed, as this is a +/- value.
  scaled_deadzone_ = this->declare_parameter("deadzone", 0.05);
  if (scaled_deadzone_ < 0.0 || scaled_deadzone_ > 1.0) {
    throw std::runtime_error("Deadzone must be between 0.0 and 1.0");
  }
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
  if (coalesce_interval_ms_ < 0) {
    throw std::runtime_error("coalesce_interval_ms must be positive");
  }
  // Make sure to initialize publish_soon_time regardless of whether we are going
  // to use it; this ensures that we are always using the correct time source.
  publish_soon_time_ = this->now();

  pub_ = create_publisher<sensor_msgs::msg::Joy>("joy", 10);

  // joy/set_feedback — rumble via sensor_msgs/JoyFeedback.
  //   id=0  → set both low-freq and high-freq motors to intensity
  //   id=1  → low-freq (heavy/left) motor only
  //   id=2  → high-freq (light/right) motor only
  feedback_sub_ = this->create_subscription<sensor_msgs::msg::JoyFeedback>(
    "joy/set_feedback", rclcpp::QoS(10),
    std::bind(&GameController::feedbackCb, this, std::placeholders::_1));

  // joy/set_haptic — full SDL haptic effects via drqp_interfaces/HapticEffect.
  haptic_sub_ = this->create_subscription<HapticEffect>(
    "joy/set_haptic", rclcpp::QoS(10),
    std::bind(&GameController::hapticCb, this, std::placeholders::_1));

  future_ = exit_signal_.get_future();

  // In theory we could do this with just a timer, which would simplify the code
  // a bit.  But then we couldn't react to "immediate" events, so we stick with
  // the thread.
  event_thread_ = std::thread(&GameController::eventThread, this);

  joy_msg_.buttons.resize(SDL_GAMEPAD_BUTTON_COUNT);
  joy_msg_.axes.resize(SDL_GAMEPAD_AXIS_COUNT);

  // SDL_INIT_GAMEPAD implies SDL_INIT_JOYSTICK.
  if (!SDL_Init(SDL_INIT_GAMEPAD | SDL_INIT_HAPTIC)) {
    throw std::runtime_error("SDL could not be initialized: " + std::string(SDL_GetError()));
  }
}

GameController::~GameController()
{
  exit_signal_.set_value();
  event_thread_.join();
  closeHaptic();
  if (game_controller_ != nullptr) {
    SDL_CloseGamepad(game_controller_);
  }
  SDL_Quit();
}

// ── Private helpers ─────────────────────────────────────────────────────────

void GameController::openHaptic()
{
  SDL_Joystick * joystick = SDL_GetGamepadJoystick(game_controller_);
  if (joystick == nullptr) {
    RCLCPP_INFO(get_logger(), "Could not get joystick handle for haptic init: %s", SDL_GetError());
    return;
  }
  haptic_ = SDL_OpenHapticFromJoystick(joystick);
  if (haptic_ == nullptr) {
    RCLCPP_INFO(get_logger(), "Haptic not supported on this gamepad: %s", SDL_GetError());
  } else {
    RCLCPP_INFO(get_logger(), "Haptic device opened successfully");
  }
}

void GameController::closeHaptic()
{
  if (haptic_ == nullptr) {
    return;
  }
  if (haptic_effect_id_ != -1) {
    SDL_StopHapticEffect(haptic_, haptic_effect_id_);
    SDL_DestroyHapticEffect(haptic_, haptic_effect_id_);
    haptic_effect_id_ = -1;
  }
  SDL_CloseHaptic(haptic_);
  haptic_ = nullptr;
}

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
  SDL_RumbleGamepad(game_controller_, low_freq, high_freq, 1000);
}

void GameController::hapticCb(const std::shared_ptr<HapticEffect> msg)
{
  if (haptic_ == nullptr) {
    RCLCPP_WARN_ONCE(get_logger(), "Haptic not available on this gamepad — ignoring set_haptic");
    return;
  }

  // ── Handle stop actions ─────────────────────────────────────────────────
  if (msg->action == HapticEffect::ACTION_STOP_ALL) {
    SDL_StopHapticEffects(haptic_);
    haptic_effect_id_ = -1;
    return;
  }

  if (msg->action == HapticEffect::ACTION_STOP) {
    if (haptic_effect_id_ != -1) {
      SDL_StopHapticEffect(haptic_, haptic_effect_id_);
    }
    return;
  }

  // ── Build SDL_HapticEffect ────────────────────────────────────────────────
  SDL_HapticEffect effect;
  SDL_memset(&effect, 0, sizeof(effect));

  const uint32_t duration = (msg->duration_ms == 0) ? SDL_HAPTIC_INFINITY : msg->duration_ms;
  const auto to_s16 = [](float v) {
    return static_cast<Sint16>(std::clamp(v, -1.0f, 1.0f) * 32767.0f);
  };
  const auto to_u16 = [](float v) {
    return static_cast<Uint16>(std::clamp(v, 0.0f, 1.0f) * 0xFFFF);
  };

  switch (msg->effect_type) {
    case HapticEffect::TYPE_LEFTRIGHT: {
      effect.type = SDL_HAPTIC_LEFTRIGHT;
      effect.leftright.length = duration;
      effect.leftright.large_magnitude = to_u16(msg->large_magnitude);
      effect.leftright.small_magnitude = to_u16(msg->small_magnitude);
      break;
    }

    case HapticEffect::TYPE_CONSTANT: {
      effect.type = SDL_HAPTIC_CONSTANT;
      effect.constant.length = duration;
      effect.constant.delay = static_cast<Uint16>(msg->delay_ms);
      effect.constant.level = to_s16(msg->level);
      effect.constant.attack_length = static_cast<Uint16>(msg->attack_ms);
      effect.constant.fade_length = static_cast<Uint16>(msg->fade_ms);
      break;
    }

    case HapticEffect::TYPE_SINE:
    case HapticEffect::TYPE_TRIANGLE:
    case HapticEffect::TYPE_SAWTOOTHUP:
    case HapticEffect::TYPE_SAWTOOTHDOWN: {
      switch (msg->effect_type) {
        case HapticEffect::TYPE_SINE:
          effect.type = SDL_HAPTIC_SINE;
          break;
        case HapticEffect::TYPE_TRIANGLE:
          effect.type = SDL_HAPTIC_TRIANGLE;
          break;
        case HapticEffect::TYPE_SAWTOOTHUP:
          effect.type = SDL_HAPTIC_SAWTOOTHUP;
          break;
        case HapticEffect::TYPE_SAWTOOTHDOWN:
          effect.type = SDL_HAPTIC_SAWTOOTHDOWN;
          break;
      }
      effect.periodic.length = duration;
      effect.periodic.delay = static_cast<Uint16>(msg->delay_ms);
      effect.periodic.period = static_cast<Uint16>(msg->period_ms);
      effect.periodic.magnitude = to_s16(msg->magnitude);
      effect.periodic.offset = to_s16(msg->offset);
      // phase is 0–65535 representing 0°–360°
      effect.periodic.phase =
        static_cast<Uint16>(std::clamp(msg->phase_degrees, 0.0f, 360.0f) / 360.0f * 65535.0f);
      effect.periodic.attack_length = static_cast<Uint16>(msg->attack_ms);
      effect.periodic.fade_length = static_cast<Uint16>(msg->fade_ms);
      break;
    }

    case HapticEffect::TYPE_RAMP: {
      effect.type = SDL_HAPTIC_RAMP;
      effect.ramp.length = duration;
      effect.ramp.delay = static_cast<Uint16>(msg->delay_ms);
      effect.ramp.start = to_s16(msg->ramp_start);
      effect.ramp.end = to_s16(msg->ramp_end);
      effect.ramp.attack_length = static_cast<Uint16>(msg->attack_ms);
      effect.ramp.fade_length = static_cast<Uint16>(msg->fade_ms);
      break;
    }

    default:
      RCLCPP_WARN(get_logger(), "Unknown haptic effect type: %d", msg->effect_type);
      return;
  }

  if (!SDL_HapticEffectSupported(haptic_, &effect)) {
    RCLCPP_WARN(get_logger(), "Haptic effect type %d not supported by this device", msg->effect_type);
    return;
  }

  // Stop and destroy the previous effect slot before creating a new one
  if (haptic_effect_id_ != -1) {
    SDL_StopHapticEffect(haptic_, haptic_effect_id_);
    SDL_DestroyHapticEffect(haptic_, haptic_effect_id_);
    haptic_effect_id_ = -1;
  }

  haptic_effect_id_ = SDL_CreateHapticEffect(haptic_, &effect);
  if (haptic_effect_id_ < 0) {
    RCLCPP_WARN(get_logger(), "Failed to create haptic effect: %s", SDL_GetError());
    return;
  }

  const Uint32 iterations =
    (msg->iterations == 0) ? SDL_HAPTIC_INFINITY : static_cast<Uint32>(msg->iterations);
  if (!SDL_RunHapticEffect(haptic_, haptic_effect_id_, iterations)) {
    RCLCPP_WARN(get_logger(), "Failed to run haptic effect: %s", SDL_GetError());
  }
}

// ── Axis / button event handlers ──────────────────────────────────────────────

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

bool GameController::handleGamepadAxis(const SDL_GamepadAxisEvent & e)
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
  joy_msg_.axes.at(e.axis) = convertRawAxisValueToROS(e.value);
  if (last_axis_value != joy_msg_.axes.at(e.axis)) {
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

bool GameController::handleGamepadButtonDown(const SDL_GamepadButtonEvent & e)
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

bool GameController::handleGamepadButtonUp(const SDL_GamepadButtonEvent & e)
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

void GameController::handleGamepadDeviceAdded(const SDL_GamepadDeviceEvent & e)
{
  const SDL_JoystickID new_id = e.which;
  const char * new_name = SDL_GetGamepadNameForID(new_id);

  if (!dev_name_.empty()) {
    // Match by device name
    if (new_name == nullptr || std::string(new_name) != dev_name_) {
      RCLCPP_INFO(
        get_logger(), "Gamepad added: id=%u, name=%s — not the requested device_name '%s'",
        new_id, new_name ? new_name : "unknown", dev_name_.c_str());
      return;
    }
  } else {
    // Match by index (dev_id_): check whether new_id is at position dev_id_ in the current list
    int count = 0;
    SDL_JoystickID * ids = SDL_GetGamepads(&count);
    bool matched = false;
    if (ids != nullptr) {
      matched = (count > dev_id_) && (ids[dev_id_] == new_id);
      SDL_free(ids);
    }
    if (!matched) {
      RCLCPP_INFO(
        get_logger(), "Gamepad added: id=%u, name=%s — not at device_id=%d, ignoring",
        new_id, new_name ? new_name : "unknown", dev_id_);
      return;
    }
  }

  RCLCPP_INFO(
    get_logger(), "Gamepad added: id=%u, name=%s", new_id, new_name ? new_name : "unknown");

  if (game_controller_ != nullptr) {
    // Already managing a controller; ignore this one.
    return;
  }

  game_controller_ = SDL_OpenGamepad(new_id);
  if (game_controller_ == nullptr) {
    RCLCPP_WARN(get_logger(), "Unable to open gamepad id=%u: %s", new_id, SDL_GetError());
    return;
  }

  joystick_instance_id_ = new_id;

  // Seed the initial axis state
  for (int i = 0; i < SDL_GAMEPAD_AXIS_COUNT; ++i) {
    const int16_t state =
      SDL_GetGamepadAxis(game_controller_, static_cast<SDL_GamepadAxis>(i));
    joy_msg_.axes.at(i) = convertRawAxisValueToROS(state);
  }

  openHaptic();

  RCLCPP_INFO(
    get_logger(), "Opened gamepad: %s  deadzone: %f  rumble: %s  haptic: %s",
    SDL_GetGamepadName(game_controller_),
    scaled_deadzone_,
    SDL_GamepadHasRumble(game_controller_) ? "Yes" : "No",
    haptic_ ? "Yes" : "No");
}

void GameController::handleGamepadDeviceRemoved(const SDL_GamepadDeviceEvent & e)
{
  if (e.which != joystick_instance_id_) {
    return;
  }

  closeHaptic();

  if (game_controller_ != nullptr) {
    RCLCPP_INFO(
      get_logger(), "Gamepad removed: %s.", SDL_GetGamepadName(game_controller_));
    SDL_CloseGamepad(game_controller_);
    game_controller_ = nullptr;
  }
}

// ── Event loop ────────────────────────────────────────────────────────────────

void GameController::eventThread()
{
  std::future_status status;
  rclcpp::Time last_publish = this->now();

  do {
    bool should_publish = false;
    SDL_Event e;
    int wait_time_ms = autorepeat_interval_ms_;
    if (publish_soon_) {
      wait_time_ms = std::min(wait_time_ms, coalesce_interval_ms_);
    }
    // SDL3: SDL_WaitEventTimeout returns bool (true = event received)
    const bool got_event = SDL_WaitEventTimeout(&e, wait_time_ms);
    if (got_event) {
      switch (e.type) {
        case SDL_EVENT_GAMEPAD_AXIS_MOTION:
          should_publish = handleGamepadAxis(e.gaxis);
          break;
        case SDL_EVENT_GAMEPAD_BUTTON_DOWN:
          should_publish = handleGamepadButtonDown(e.gbutton);
          break;
        case SDL_EVENT_GAMEPAD_BUTTON_UP:
          should_publish = handleGamepadButtonUp(e.gbutton);
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
    } else {
      // Timeout or error — check autorepeat
      const rclcpp::Time now = this->now();
      const rclcpp::Duration diff_since_last_publish = now - last_publish;
      if ((autorepeat_rate_ > 0.0 &&
        RCL_NS_TO_MS(diff_since_last_publish.nanoseconds()) >= autorepeat_interval_ms_) ||
        publish_soon_)
      {
        last_publish = now;
        should_publish = true;
        publish_soon_ = false;
      }
    }

    if (game_controller_ != nullptr && should_publish) {
      joy_msg_.header.frame_id = "joy";
      joy_msg_.header.stamp = this->now();
      pub_->publish(joy_msg_);
    }

    status = future_.wait_for(std::chrono::seconds(0));
  } while (status == std::future_status::timeout);
}

}  // namespace drqp_joy

RCLCPP_COMPONENTS_REGISTER_NODE(drqp_joy::GameController)
