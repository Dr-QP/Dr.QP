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

#include <catch_ros2/catch.hpp>

#include "drqp_serial/SerialProtocol.h"
#include "drqp_serial/SerialTransferConfig.h"

namespace
{
using boost::asio::serial_port_base;

void checkDecode(
  const uint8_t transferConfig, const unsigned int expectedDataBits,
  const serial_port_base::parity::type expectedParity,
  const serial_port_base::stop_bits::type expectedStopBits)
{
  const SerialTransferConfig config = decodeSerialTransferConfig(transferConfig);
  CHECK(config.characterSize.value() == expectedDataBits);
  CHECK(config.parity.value() == expectedParity);
  CHECK(config.stopBits.value() == expectedStopBits);
}
}  // namespace

SCENARIO("decodeSerialTransferConfig maps Arduino bitmask to Boost ASIO options")
{
  GIVEN("the default framing SERIAL_8N1")
  {
    THEN("it decodes to 8 data bits, no parity, one stop bit")
    {
      checkDecode(SERIAL_8N1, 8U, serial_port_base::parity::none, serial_port_base::stop_bits::one);
    }
  }

  GIVEN("even-parity framing SERIAL_8E1")
  {
    THEN("it decodes to 8 data bits, even parity, one stop bit")
    {
      checkDecode(SERIAL_8E1, 8U, serial_port_base::parity::even, serial_port_base::stop_bits::one);
    }
  }

  GIVEN("odd-parity framing SERIAL_8O1")
  {
    THEN("it decodes to 8 data bits, odd parity, one stop bit")
    {
      checkDecode(SERIAL_8O1, 8U, serial_port_base::parity::odd, serial_port_base::stop_bits::one);
    }
  }

  GIVEN("two-stop-bit framing SERIAL_8N2")
  {
    THEN("it decodes to 8 data bits, no parity, two stop bits")
    {
      checkDecode(SERIAL_8N2, 8U, serial_port_base::parity::none, serial_port_base::stop_bits::two);
    }
  }

  GIVEN("seven-data-bit even-parity two-stop framing SERIAL_7E2")
  {
    THEN("it decodes to 7 data bits, even parity, two stop bits")
    {
      checkDecode(SERIAL_7E2, 7U, serial_port_base::parity::even, serial_port_base::stop_bits::two);
    }
  }

  GIVEN("five-data-bit framing SERIAL_5N1")
  {
    THEN("it decodes to 5 data bits, no parity, one stop bit")
    {
      checkDecode(SERIAL_5N1, 5U, serial_port_base::parity::none, serial_port_base::stop_bits::one);
    }
  }

  GIVEN("a bitmask with the reserved parity value 01")
  {
    THEN("decoding throws std::invalid_argument")
    {
      // Parity bits 01 (0x10) is unused in the Arduino convention.
      REQUIRE_THROWS_AS(decodeSerialTransferConfig(0x16), std::invalid_argument);
    }
  }
}
