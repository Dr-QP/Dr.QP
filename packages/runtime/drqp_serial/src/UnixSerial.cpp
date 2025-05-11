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

#include "drqp_serial/UnixSerial.h"
#include "AsioCommon.h"

#include <vector>

#include <boost/asio.hpp>
#include <boost/asio/read.hpp>
#include <boost/asio/write.hpp>
#include <boost/thread.hpp>

struct UnixSerial::Impl
{
  boost::posix_time::time_duration timeout_ = boost::posix_time::milliseconds{500};
  boost::asio::io_service ioService_;
  boost::asio::serial_port serial_;

  explicit Impl(const std::string& fileName) : serial_(ioService_, fileName) {}
};

/// @brief Returns the number of bytes available for reading from a serial
///        port without blocking.
std::size_t get_bytes_available(
  boost::asio::serial_port& serial_port, boost::system::error_code& error)
{
  error = boost::system::error_code();
  int value = 0;
#if defined(BOOST_ASIO_WINDOWS) || defined(__CYGWIN__)
  COMSTAT status;
  if (0 != ::ClearCommError(serial_port.lowest_layer().native_handle(), nullptr, &status)) {
    value = status.cbInQue;
  } else {
    // On error, set the error code.
    error = boost::system::error_code(::GetLastError(), boost::asio::error::get_system_category());
  }
#else  // defined(BOOST_ASIO_WINDOWS) || defined(__CYGWIN__)

  if (::ioctl(serial_port.lowest_layer().native_handle(), FIONREAD, &value) < 0) {
    error = boost::system::error_code(errno, boost::asio::error::get_system_category());
  }

#endif  // defined(BOOST_ASIO_WINDOWS) || defined(__CYGWIN__)

  return error ? static_cast<std::size_t>(0) : static_cast<size_t>(value);
}

/// @brief Returns the number of bytes available for reading from a serial
///        port without blocking.  Throws on error.
std::size_t get_bytes_available(boost::asio::serial_port& serial_port)
{
  boost::system::error_code error;
  std::size_t bytes_available = get_bytes_available(serial_port, error);
  if (error) {
    boost::throw_exception((boost::system::system_error(error)));
  }
  return bytes_available;
}

UnixSerial::UnixSerial(const std::string& fileName) : impl_(std::make_unique<Impl>(fileName)) {}

UnixSerial::~UnixSerial() = default;

void UnixSerial::begin(const uint32_t baudRate, const uint8_t transferConfig)
{
  using boost::asio::serial_port_base;

  impl_->serial_.set_option(serial_port_base::baud_rate(baudRate));
  impl_->serial_.set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));

  // TODO(anton-matosov): Implement parsing of the options or migrate to explicit options from ext
  // SERIAL_8N1
  impl_->serial_.set_option(serial_port_base::character_size(8));                            // 8
  impl_->serial_.set_option(serial_port_base::parity(serial_port_base::parity::none));       // N
  impl_->serial_.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));  // 1
}

size_t UnixSerial::writeBytes(const void* buffer, size_t size)
{
  return doWithTimeout<AsyncOp::Write>(
    impl_->ioService_, impl_->serial_, boost::asio::buffer(buffer, size), impl_->timeout_);
}

void UnixSerial::flushRead()
{
  size_t bytes = get_bytes_available(impl_->serial_);
  if (bytes > 0) {
    std::vector<uint8_t> buf;
    buf.resize(bytes);
    readBytes(&buf[0], bytes);
  }
}

bool UnixSerial::available()
{
  return get_bytes_available(impl_->serial_) != 0;
}

size_t UnixSerial::readBytes(void* buffer, size_t size)
{
  return doWithTimeout<AsyncOp::Read>(
    impl_->ioService_, impl_->serial_, boost::asio::buffer(buffer, size), impl_->timeout_);
}

void UnixSerial::setTimeout(const std::chrono::milliseconds& timeout)
{
  impl_->timeout_ = boost::posix_time::milliseconds(timeout.count());
}

std::chrono::milliseconds UnixSerial::getTimeout() const
{
  return std::chrono::milliseconds(impl_->timeout_.total_milliseconds());
}
