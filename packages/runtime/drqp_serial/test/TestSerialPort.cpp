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

#include <pty.h>
#include <unistd.h>

#include <boost/asio.hpp>
#include <catch_ros2/catch.hpp>

#include "drqp_serial/UnixSerial.h"

SCENARIO("test unix serial with pseudo terminal")
{
  int master_fd = -1, slave_fd = -1;
  std::array<char, 256> slave_name_arr;

  // Open a pseudo terminal
  REQUIRE(openpty(&master_fd, &slave_fd, slave_name_arr.data(), nullptr, nullptr) != -1);

  GIVEN("UnixSerial is created with slave name")
  {
    std::string slave_name(slave_name_arr.data());
    INFO("Slave name: " << slave_name);
    UnixSerial serial(slave_name);
    serial.begin(115200);

    WHEN("data is written to the serial")
    {
      std::string data = "Hello, World!";
      REQUIRE_NOTHROW(serial.writeBytes(data.c_str(), data.length()));

      THEN("data is readable from the master file descriptor")
      {
        std::string buffer;
        buffer.resize(data.length() + 1, '\0');
        size_t bytes_read = ::read(master_fd, buffer.data(), buffer.size() - 1);
        REQUIRE(bytes_read == data.length());
        buffer.resize(bytes_read);
        REQUIRE(buffer == data);
      }
    }

    WHEN("data is written to the master file descriptor")
    {
      std::string data = "Hello, World!";
      ::write(master_fd, data.c_str(), data.length());

      THEN("data is readable from the serial")
      {
        std::string buffer;
        buffer.resize(data.length() + 1, '\0');
        size_t bytes_read = 0;
        REQUIRE_NOTHROW(bytes_read = serial.readBytes(buffer.data(), buffer.size() - 1));
        REQUIRE(bytes_read == data.length());
        buffer.resize(bytes_read);
        REQUIRE(buffer == data);
      }
    }

    WHEN("no data is written to the master file descriptor")
    {
      THEN("serial.available() returns false")
      {
        REQUIRE(!serial.available());
      }

      THEN("serial.readBytes() throws an exception on timeout")
      {
        std::string buffer;
        buffer.resize(10, '\0');
        REQUIRE_THROWS_AS(
          serial.readBytes(buffer.data(), buffer.size()), boost::system::system_error);
      }
    }
  }
  close(master_fd);
  close(slave_fd);
}
