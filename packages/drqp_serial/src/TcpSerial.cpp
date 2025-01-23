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

#include "drqp_serial/TcpSerial.h"

#include <iostream>
#include <optional>
#include <vector>

#include <boost/asio.hpp>
#include <boost/asio/read.hpp>
#include <boost/asio/write.hpp>
#include <boost/thread.hpp>

tcp::resolver::iterator resolve(boost::asio::io_service& ioService, const std::string& ip,
                                uint16_t port) {
  tcp::resolver resolver(ioService);
  tcp::resolver::query query(ip, std::to_string(port));
  return resolver.resolve(query);
}

TcpSerial::TcpSerial(const std::string& ip, uint16_t port)
  : socket_(ioService_), lastRead_(0), everRead_(false) {
  connect(socket_, resolve(ioService_, ip, port));
}

void TcpSerial::begin(const uint32_t baudRate, const uint8_t transferConfig) {}

size_t TcpSerial::write(uint8_t byte) {
  return write(&byte, 1);
}

size_t TcpSerial::write(const uint8_t* data, size_t size) {
  return boost::asio::write(socket_, boost::asio::buffer(data, size));
}

void TcpSerial::flushRead() {
  boost::asio::socket_base::bytes_readable command(true);
  socket_.io_control(command);
  const std::size_t bytesReadable = command.get();

  if (bytesReadable > 0) {
    std::vector<uint8_t> buf;
    buf.resize(bytesReadable);
    readBytes(&buf[0], bytesReadable);
  }
}

bool TcpSerial::available() {
  boost::asio::socket_base::bytes_readable command(true);
  socket_.io_control(command);
  const std::size_t bytesReadable = command.get();

  return bytesReadable != 0;
}

uint8_t TcpSerial::peek() {
  if (!available()) {
    return kNoData;
  }
  if (!everRead_) {
    return read();
  }
  return lastRead_;
}

// template <typename SyncReadStream, typename MutableBufferSequence>
// void readWithTimeout(SyncReadStream& s, const MutableBufferSequence& buffers, const
// boost::asio::deadline_timer::duration_type& expiry_time)
// {
//     boost::optional<boost::system::error_code> timer_result;
//     boost::asio::deadline_timer timer(s.get_io_service());
//     timer.expires_from_now(expiry_time);
//     timer.async_wait([&timer_result] (const boost::system::error_code& error) {
//     timer_result.reset(error); });

//     boost::optional<boost::system::error_code> read_result;
//     boost::asio::async_read(s, buffers, [&read_result] (const boost::system::error_code& error,
//     size_t) { read_result.reset(error); });

//     s.get_io_service().reset();
//     while (s.get_io_service().run_one())
//     {
//         if (read_result)
//             timer.cancel();
//         else if (timer_result)
//             s.cancel();
//     }

//     if (*read_result)
//         throw boost::system::system_error(*read_result);
// }

template <typename MutableBufferSequence>
size_t read_with_timeout(boost::asio::io_service& ioService, tcp::socket& sock,
                         const MutableBufferSequence& buffers) {
  std::optional<boost::system::error_code> timer_result;
  boost::asio::deadline_timer timer(ioService);
  timer.expires_from_now(boost::posix_time::milliseconds(50));
  timer.async_wait([&timer_result](const auto& ec) { timer_result = ec; });

  std::optional<boost::system::error_code> read_result;
  std::optional<std::size_t> bytesTransferred;
  async_read(sock, buffers, [&read_result, &bytesTransferred](const auto& ec, std::size_t bt) {
    read_result = ec;
    bytesTransferred = bt;
  });

  ioService.reset();
  while (ioService.run_one()) {
    if (read_result)
      timer.cancel();
    else if (timer_result)
      sock.cancel();
  }

  if (read_result && *read_result) {
    std::cerr << read_result->message() << "\n";
    return 0;
    // throw boost::system::system_error(*read_result);
  }

  return *bytesTransferred;
}

uint8_t TcpSerial::read() {
  // if (!available()) {
  //     return kNoData;
  // }
  everRead_ = true;
  // read_with_timeout(ioService_, socket_, boost::asio::buffer(&lastRead_, 1));
  return lastRead_;
}

size_t TcpSerial::readBytes(uint8_t* buffer, size_t size) {
  everRead_ = true;
  return read_with_timeout(ioService_, socket_, boost::asio::buffer(buffer, size));
}
