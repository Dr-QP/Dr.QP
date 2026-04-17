/*
 * Copyright (c) 2020, Open Source Robotics Foundation.
 * Copyright (c) 2023, CSIRO Data61
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
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

#ifndef DRQP_JOY__GAME_CONTROLLER_HPP_
#define DRQP_JOY__GAME_CONTROLLER_HPP_

#include <SDL3/SDL.h>

#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joy_feedback.hpp>

#include "drqp_interfaces/msg/haptic_effect.hpp"

namespace drqp_joy
{

/// ROS 2 game-controller node — SDL3 port with dual-motor rumble and haptic support.
///
/// **Topics published**
///   - `joy`  (sensor_msgs/Joy)
///
/// **Topics subscribed**
///   - `joy/set_feedback`  (sensor_msgs/JoyFeedback) — rumble control
///       id=0  → set both low-freq and high-freq motors to `intensity`
///       id=1  → low-freq (heavy/left) motor only
///       id=2  → high-freq (light/right) motor only
///   - `joy/set_haptic`    (drqp_interfaces/HapticEffect) — SDL haptic effects
class GameController final : public rclcpp::Node
{
public:
  explicit GameController(const rclcpp::NodeOptions& options);
  GameController(GameController&& c) = delete;
  GameController& operator=(GameController&& c) = delete;
  GameController(const GameController& c) = delete;
  GameController& operator=(const GameController& c) = delete;

  ~GameController() override;

private:
  void eventThread();

  bool handleGamepadAxis(const SDL_GamepadAxisEvent& e);
  bool handleGamepadButtonDown(const SDL_GamepadButtonEvent& e);
  bool handleGamepadButtonUp(const SDL_GamepadButtonEvent& e);
  void handleGamepadDeviceAdded(const SDL_GamepadDeviceEvent& e);
  void handleGamepadDeviceRemoved(const SDL_GamepadDeviceEvent& e);

  float convertRawAxisValueToROS(int16_t val);

  void feedbackCb(const std::shared_ptr<sensor_msgs::msg::JoyFeedback> msg);
  void hapticCb(const std::shared_ptr<drqp_interfaces::msg::HapticEffect> msg);

  void destroyHapticEffectLocked();
  void openHaptic();
  void closeHaptic();

  // ── Parameters ─────────────────────────────────────────────────────────────
  int dev_id_{0};
  std::string dev_name_;
  double scaled_deadzone_{0.0};
  double unscaled_deadzone_{0.0};
  double scale_{0.0};
  double autorepeat_rate_{0.0};
  int autorepeat_interval_ms_{0};
  bool sticky_buttons_{false};
  int coalesce_interval_ms_{0};
  uint32_t feedback_rumble_duration_ms_{1000};

  // ── State ───────────────────────────────────────────────────────────────────
  SDL_Gamepad* game_controller_{nullptr};
  SDL_JoystickID joystick_instance_id_{0};
  SDL_Haptic* haptic_{nullptr};
  int haptic_effect_id_{-1};

  bool publish_soon_{false};
  rclcpp::Time publish_soon_time_;
  std::mutex sdl_state_mutex_;

  // ── Threading ───────────────────────────────────────────────────────────────
  std::thread event_thread_;
  std::shared_future<void> future_;
  std::promise<void> exit_signal_;

  // ── ROS interfaces ──────────────────────────────────────────────────────────
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::JoyFeedback>::SharedPtr feedback_sub_;
  rclcpp::Subscription<drqp_interfaces::msg::HapticEffect>::SharedPtr haptic_sub_;

  sensor_msgs::msg::Joy joy_msg_;
};

}  // namespace drqp_joy

#endif  // DRQP_JOY__GAME_CONTROLLER_HPP_
