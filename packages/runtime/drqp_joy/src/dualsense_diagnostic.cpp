// Copyright (c) 2026 Anton Matosov
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <SDL3/SDL.h>

#include <exception>
#include <iostream>
#include <numbers>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "drqp_joy/dualsense_diagnostic_utils.hpp"

namespace
{

using AudioDevice = drqp_joy::diagnostics::detail::NamedDevice<SDL_AudioDeviceID>;
using GamepadDevice = drqp_joy::diagnostics::detail::NamedDevice<SDL_JoystickID>;

constexpr int kPs5EffectStateSize = 47;
constexpr Uint16 kSonyVendorId = 0x054c;
constexpr Uint16 kDualSenseProductId = 0x0ce6;
constexpr Uint16 kDualSenseEdgeProductId = 0x0df2;

constexpr Uint8 kDs5EnableSpeakerVolume = 0x20;
constexpr Uint8 kDs5EnableAudioControl = 0x80;
constexpr Uint8 kDs5EnableAudioControl2 = 0x80;
constexpr Uint8 kDs5SpeakerVolumeMax = 0x64;
constexpr Uint8 kDs5SpeakerRouteRightChannel = 0x30;
constexpr Uint8 kDs5SpeakerPreampPlus6dB = 0x02;

struct Options
{
  int gamepad_index{0};
  uint32_t duration_ms{1000};
  double frequency_hz{80.0};
  bool list_only{false};
  bool probe_rumble{false};
  bool skip_audio{false};
  std::string gamepad_name;
  std::string audio_device_name;
};

void printUsage(const char* executable)
{
  std::cout << "Usage: " << executable << " [options]\n"
            << "  --list-only                 List detected devices and exit\n"
            << "  --gamepad-index N           Select gamepad by SDL index (default: 0)\n"
            << "  --gamepad-name TEXT         Select first gamepad whose name contains TEXT\n"
            << "  --audio-device-name TEXT    Select first audio device whose name contains TEXT\n"
            << "  --duration-ms N             Test duration in milliseconds (default: 1000)\n"
            << "  --frequency-hz N            Tone frequency in hertz (default: 80, reasonable range: 20-200)\n"
            << "  --probe-rumble             Run the controller rumble probe\n"
            << "  --skip-audio                Skip the controller speaker probe\n"
            << "  --help                      Show this help\n";
}

int parseNonNegativeInt(const char* value, std::string_view option_name)
{
  char* end = nullptr;
  const auto parsed = std::strtoll(value, &end, 10);
  if (end == value || *end != '\0' || parsed < 0) {
    throw std::runtime_error("Invalid value for " + std::string(option_name) + ": " + value);
  }

  return static_cast<int>(parsed);
}

double parsePositiveDouble(const char* value, std::string_view option_name)
{
  char* end = nullptr;
  const double parsed = std::strtod(value, &end);
  if (end == value || *end != '\0' || parsed <= 0.0) {
    throw std::runtime_error("Invalid value for " + std::string(option_name) + ": " + value);
  }

  return parsed;
}

Options parseArgs(int argc, char** argv)
{
  Options options;

  for (int i = 1; i < argc; ++i) {
    const std::string_view arg(argv[i]);
    if (arg == "--help") {
      printUsage(argv[0]);
      std::exit(0);
    }
    if (arg == "--list-only") {
      options.list_only = true;
      continue;
    }
    if (arg == "--skip-audio") {
      options.skip_audio = true;
      continue;
    }
    if (arg == "--probe-rumble") {
      options.probe_rumble = true;
      continue;
    }

    auto requireValue = [&](std::string_view option_name) -> const char* {
      if (i + 1 >= argc) {
        throw std::runtime_error("Missing value for " + std::string(option_name));
      }
      ++i;
      return argv[i];
    };

    if (arg == "--gamepad-index") {
      options.gamepad_index = parseNonNegativeInt(requireValue(arg), arg);
      continue;
    }
    if (arg == "--duration-ms") {
      options.duration_ms = static_cast<uint32_t>(parseNonNegativeInt(requireValue(arg), arg));
      continue;
    }
    if (arg == "--frequency-hz") {
      options.frequency_hz = parsePositiveDouble(requireValue(arg), arg);
      continue;
    }
    if (arg == "--gamepad-name") {
      options.gamepad_name = requireValue(arg);
      continue;
    }
    if (arg == "--audio-device-name") {
      options.audio_device_name = requireValue(arg);
      continue;
    }

    throw std::runtime_error("Unknown option: " + std::string(arg));
  }

  return options;
}

std::vector<AudioDevice> enumeratePlaybackDevices()
{
  int count = 0;
  SDL_AudioDeviceID* ids = SDL_GetAudioPlaybackDevices(&count);
  if (ids == nullptr) {
    throw std::runtime_error(
      "Failed to enumerate audio playback devices: " + std::string(SDL_GetError()));
  }

  std::vector<AudioDevice> devices;
  devices.reserve(static_cast<size_t>(count));
  for (int index = 0; index < count; ++index) {
    const SDL_AudioDeviceID id = ids[index];
    const char* name = SDL_GetAudioDeviceName(id);
    devices.push_back(
      AudioDevice{id, name == nullptr ? std::string("unknown") : std::string(name)});
  }

  SDL_free(ids);
  return devices;
}

std::vector<GamepadDevice> enumerateGamepads()
{
  int count = 0;
  SDL_JoystickID* ids = SDL_GetGamepads(&count);
  if (ids == nullptr) {
    throw std::runtime_error("Failed to enumerate gamepads: " + std::string(SDL_GetError()));
  }

  std::vector<GamepadDevice> devices;
  devices.reserve(static_cast<size_t>(count));
  for (int index = 0; index < count; ++index) {
    const SDL_JoystickID id = ids[index];
    const char* name = SDL_GetGamepadNameForID(id);
    devices.push_back(
      GamepadDevice{id, name == nullptr ? std::string("unknown") : std::string(name)});
  }

  SDL_free(ids);
  return devices;
}

template <typename Device>
void printDevices(
  const std::vector<Device>& devices, std::string_view kind, bool (*matcher)(std::string_view))
{
  std::cout << kind << " devices:\n";
  if (devices.empty()) {
    std::cout << "  (none)\n";
    return;
  }

  for (size_t index = 0; index < devices.size(); ++index) {
    std::cout << "  [" << index << "] id=" << devices[index].id << " name=\"" << devices[index].name
              << "\"";
    if (matcher(devices[index].name)) {
      std::cout << "  <- likely DualSense";
    }
    std::cout << '\n';
  }
}

std::optional<AudioDevice> selectAudioDevice(
  const std::vector<AudioDevice>& devices, const Options& options)
{
  if (!options.audio_device_name.empty()) {
    return drqp_joy::diagnostics::detail::findDeviceBySubstring(devices, options.audio_device_name);
  }

  return drqp_joy::diagnostics::detail::findFirstMatchingDevice(
    devices, drqp_joy::diagnostics::detail::isLikelyDualSenseAudioDeviceName);
}

std::optional<GamepadDevice> selectGamepad(
  const std::vector<GamepadDevice>& devices, const Options& options)
{
  if (!options.gamepad_name.empty()) {
    return drqp_joy::diagnostics::detail::findDeviceBySubstring(devices, options.gamepad_name);
  }

  if (devices.empty()) {
    return std::nullopt;
  }

  const size_t index = static_cast<size_t>(options.gamepad_index);
  if (index >= devices.size()) {
    return std::nullopt;
  }

  return devices[index];
}

std::vector<int16_t> buildToneSamples(uint32_t duration_ms, double frequency_hz)
{
  constexpr int sample_rate = 3'000;
  constexpr int channels = 2;
  constexpr double start_amplitude = 0.2;
  constexpr double end_amplitude = 1.0;
  constexpr double amplitude_cycles = 5.0;


  const int frame_count =
    static_cast<int>((static_cast<uint64_t>(sample_rate) * duration_ms) / 1000U);
  std::vector<int16_t> samples(static_cast<size_t>(frame_count * channels));

  for (int frame = 0; frame < frame_count; ++frame) {
    const double t = static_cast<double>(frame) / static_cast<double>(sample_rate);
    const double raw = std::sin(2.0 * std::numbers::pi * frequency_hz * t);
    // Aplitude cycles controls how many times the amplitude ramps up and down over the duration of the tone
    const double amplitude = start_amplitude + (end_amplitude - start_amplitude) *      (0.5 * (1.0 - std::cos(2.0 * std::numbers::pi * amplitude_cycles * t / (static_cast<double>(duration_ms) / 1000.0))));

    const auto sample = static_cast<int16_t>(raw * amplitude * 32767.0);
    samples[static_cast<size_t>(frame * channels)] = sample;
    samples[static_cast<size_t>((frame * channels) + 1)] = sample;
  }

  return samples;
}

bool probeAudio(const AudioDevice& device, uint32_t duration_ms, double frequency_hz)
{
  const SDL_AudioSpec spec{SDL_AUDIO_S16, 2, 3'000};
  SDL_AudioStream* stream = SDL_OpenAudioDeviceStream(device.id, &spec, nullptr, nullptr);
  if (stream == nullptr) {
    std::cout << "Speaker probe: failed to open audio device \"" << device.name
              << "\": " << SDL_GetError() << '\n';
    return false;
  }

  const std::vector<int16_t> samples = buildToneSamples(duration_ms, frequency_hz);
  const bool queued = SDL_PutAudioStreamData(
    stream, samples.data(), static_cast<int>(samples.size() * sizeof(int16_t)));
  const bool resumed = queued && SDL_ResumeAudioStreamDevice(stream);
  if (!queued || !resumed) {
    std::cout << "Speaker probe: failed to queue or start playback on \"" << device.name
              << "\": " << SDL_GetError() << '\n';
    SDL_DestroyAudioStream(stream);
    return false;
  }

  SDL_Delay(duration_ms + 150U);
  SDL_DestroyAudioStream(stream);
  std::cout << "Speaker probe: played synthesized tone on \"" << device.name << "\"\n";
  return true;
}

bool isDualSenseGamepad(SDL_Gamepad* gamepad)
{
  SDL_Joystick* joystick = SDL_GetGamepadJoystick(gamepad);
  if (joystick == nullptr) {
    return false;
  }

  const Uint16 vendor = SDL_GetJoystickVendor(joystick);
  const Uint16 product = SDL_GetJoystickProduct(joystick);
  return vendor == kSonyVendorId &&
         (product == kDualSenseProductId || product == kDualSenseEdgeProductId);
}

bool configureDualSenseSpeakerVolume(SDL_Gamepad* gamepad)
{
  if (!isDualSenseGamepad(gamepad)) {
    std::cout << "Speaker probe: selected gamepad is not a DualSense; "
                 "skipping speaker volume configuration\n";
    return false;
  }

  SDL_Joystick* joystick = SDL_GetGamepadJoystick(gamepad);
  if (joystick == nullptr) {
    std::cout << "Speaker probe: failed to get joystick handle for selected gamepad: "
              << SDL_GetError() << '\n';
    return false;
  }

  std::vector<Uint8> payload(kPs5EffectStateSize, 0);
  payload[0] = static_cast<Uint8>(kDs5EnableSpeakerVolume | kDs5EnableAudioControl);
  payload[5] = kDs5SpeakerVolumeMax;
  payload[7] = kDs5SpeakerRouteRightChannel;
  payload[38] = kDs5EnableAudioControl2;
  payload[39] = kDs5SpeakerPreampPlus6dB;

  if (!SDL_SendJoystickEffect(joystick, payload.data(), static_cast<int>(payload.size()))) {
    std::cout << "Speaker probe: failed to set DualSense speaker volume: "
              << SDL_GetError() << '\n';
    return false;
  }

  std::cout << "Speaker probe: configured DualSense speaker volume to maximum\n";
  return true;
}

bool probeRumble(SDL_Gamepad* gamepad, uint32_t duration_ms)
{
  if (!SDL_GetBooleanProperty(
        SDL_GetGamepadProperties(gamepad), SDL_PROP_GAMEPAD_CAP_RUMBLE_BOOLEAN, false)) {
    std::cout << "Rumble probe: gamepad reports no rumble capability\n";
    return false;
  }

  if (!SDL_RumbleGamepad(gamepad, 0xA000, 0x6000, duration_ms)) {
    std::cout << "Rumble probe: failed to start rumble: " << SDL_GetError() << '\n';
    return false;
  }

  SDL_Delay(duration_ms + 50U);
  std::cout << "Rumble probe: started low/high frequency rumble\n";
  return true;
}

}  // namespace

int main(int argc, char** argv)
{
  try {
    const Options options = parseArgs(argc, argv);

    if (!SDL_Init(SDL_INIT_AUDIO | SDL_INIT_GAMEPAD)) {
      throw std::runtime_error(
        "Failed to initialize SDL subsystems: " + std::string(SDL_GetError()));
    }

    const auto playback_devices = enumeratePlaybackDevices();
    const auto gamepads = enumerateGamepads();

    printDevices(
      playback_devices, "Playback",
      drqp_joy::diagnostics::detail::isLikelyDualSenseAudioDeviceName);
    printDevices(gamepads, "Gamepad", drqp_joy::diagnostics::detail::isLikelyDualSenseGamepadName);

    if (options.list_only) {
      SDL_Quit();
      return 0;
    }

    bool overall_success = true;
    const auto selected_gamepad = selectGamepad(gamepads, options);
    SDL_Gamepad* opened_gamepad = nullptr;

    auto ensureGamepadOpen = [&]() -> SDL_Gamepad* {
      if (opened_gamepad != nullptr) {
        return opened_gamepad;
      }
      if (!selected_gamepad.has_value()) {
        return nullptr;
      }

      opened_gamepad = SDL_OpenGamepad(selected_gamepad->id);
      return opened_gamepad;
    };

    if (!options.skip_audio) {
      const auto audio_device = selectAudioDevice(playback_devices, options);
      if (!audio_device.has_value()) {
        std::cout << "Speaker probe: no likely DualSense playback device found"
                  << " (USB audio usually required on Linux)\n";
        overall_success = false;
      } else {
        SDL_Gamepad* gamepad = ensureGamepadOpen();
        if (gamepad == nullptr) {
          std::cout << "Speaker probe: no matching gamepad found for speaker volume setup; "
                       "continuing with audio probe\n";
        } else {
          std::cout << "Selected gamepad: \"" << selected_gamepad->name << "\"\n";
          configureDualSenseSpeakerVolume(gamepad);
        }
        overall_success =
          probeAudio(audio_device.value(), options.duration_ms, options.frequency_hz) &&
          overall_success;
      }
    }

    if (options.probe_rumble) {
      if (!selected_gamepad.has_value()) {
        std::cout << "Rumble probe: no matching gamepad found\n";
        overall_success = false;
      } else {
        SDL_Gamepad* gamepad = ensureGamepadOpen();
        if (gamepad == nullptr) {
          std::cout << "Rumble probe: failed to open gamepad \"" << selected_gamepad->name
                    << "\": " << SDL_GetError() << '\n';
          overall_success = false;
        } else {
          std::cout << "Selected gamepad: \"" << selected_gamepad->name << "\"\n";
          overall_success = probeRumble(gamepad, options.duration_ms) && overall_success;
        }
      }
    }

    if (opened_gamepad != nullptr) {
      SDL_CloseGamepad(opened_gamepad);
    }

    SDL_Quit();
    std::cout << "Overall result: " << (overall_success ? "PASS" : "FAIL") << '\n';
    return overall_success ? 0 : 1;
  } catch (const std::exception& ex) {
    std::cerr << ex.what() << '\n';
    return 2;
  }
}
