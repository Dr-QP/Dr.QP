// Copyright (c) 2017-2025 Anton Matosov
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

#pragma once

#include <cstdint>
#include <stdexcept>

#include <boost/asio/serial_port_base.hpp>

/// @brief Decoded Boost ASIO serial framing options.
///
/// Carries the three framing options that make up an Arduino-style
/// @c transferConfig byte (see @c SERIAL_* macros in SerialProtocol.h).
/// The option objects are constructible standalone and expose @c value(),
/// so this struct can be unit-tested without opening a real serial device.
struct SerialTransferConfig
{
  boost::asio::serial_port_base::character_size characterSize;
  boost::asio::serial_port_base::parity parity;
  boost::asio::serial_port_base::stop_bits stopBits;
};

/// @brief Decode an Arduino @c HardwareSerial framing bitmask into Boost ASIO
///        serial options.
///
/// Bitmask convention (matches the @c SERIAL_* macros in SerialProtocol.h):
/// - bits 2-1: data bits -> 5 + ((cfg >> 1) & 0x03)  => 00=5, 01=6, 10=7, 11=8
/// - bit 3:    stop bits  -> 0=one, 1=two
/// - bits 5-4: parity     -> 00=none, 10=even, 11=odd (01 is invalid)
///
/// @throws std::invalid_argument if the parity field holds the reserved 01
///         value, which does not correspond to any valid framing.
inline SerialTransferConfig decodeSerialTransferConfig(const uint8_t transferConfig)
{
  using boost::asio::serial_port_base;

  const unsigned int dataBits = 5U + ((transferConfig >> 1) & 0x03U);
  const bool twoStopBits = ((transferConfig >> 3) & 0x01U) != 0U;
  const unsigned int parityBits = (transferConfig >> 4) & 0x03U;

  serial_port_base::parity::type parityType = serial_port_base::parity::none;
  switch (parityBits) {
  case 0x00U:
    parityType = serial_port_base::parity::none;
    break;
  case 0x02U:
    parityType = serial_port_base::parity::even;
    break;
  case 0x03U:
    parityType = serial_port_base::parity::odd;
    break;
  default:  // 0x01 is reserved/unused in the Arduino convention.
    throw std::invalid_argument("Invalid parity in transferConfig bitmask");
  }

  return SerialTransferConfig{
    serial_port_base::character_size(static_cast<unsigned int>(dataBits)),
    serial_port_base::parity(parityType),
    serial_port_base::stop_bits(
      twoStopBits ? serial_port_base::stop_bits::two : serial_port_base::stop_bits::one)};
}
