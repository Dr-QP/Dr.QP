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
#include "AsioCommon.h"

#include <vector>

#include <boost/asio.hpp>
#include <boost/asio/read.hpp>
#include <boost/asio/write.hpp>

tcp::resolver::iterator resolve(
  boost::asio::io_service& ioService, const std::string& ip, std::string port)
{
  tcp::resolver resolver(ioService);
  tcp::resolver::query query(ip, port);
  return resolver.resolve(query);
}

TcpSerial::TcpSerial(const std::string& ip, std::string port) : socket_(ioService_)
{
  connect(socket_, resolve(ioService_, ip, port));
}

void TcpSerial::begin(const uint32_t baudRate, const uint8_t transferConfig) {}

size_t TcpSerial::writeBytes(const void* buffer, size_t size)
{
  return doWithTimeout<AsyncOp::Write>(ioService_, socket_, boost::asio::buffer(buffer, size));
}

void TcpSerial::flushRead()
{
  boost::asio::socket_base::bytes_readable command(true);
  socket_.io_control(command);
  const std::size_t bytesReadable = command.get();

  if (bytesReadable > 0) {
    std::vector<uint8_t> buf;
    buf.resize(bytesReadable);
    readBytes(&buf[0], bytesReadable);
  }
}

bool TcpSerial::available()
{
  boost::asio::socket_base::bytes_readable command(true);
  socket_.io_control(command);
  const std::size_t bytesReadable = command.get();

  return bytesReadable != 0;
}

size_t TcpSerial::readBytes(void* buffer, size_t size)
{
  return doWithTimeout<AsyncOp::Read>(ioService_, socket_, boost::asio::buffer(buffer, size));
}
