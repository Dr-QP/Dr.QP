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

#include <iostream>
#include <optional>

#include <boost/asio.hpp>
#include <boost/asio/read.hpp>
#include <boost/asio/write.hpp>

template <typename MutableBufferSequence, typename Stream>
size_t readWithTimeout(
  boost::asio::io_service& ioService, Stream& sock, const MutableBufferSequence& buffers, uint64_t timeoutMs = 5000)
{
  std::optional<boost::system::error_code> timer_result;
  boost::asio::deadline_timer timer(ioService);
  timer.expires_from_now(boost::posix_time::milliseconds(timeoutMs));
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
  }

  return *bytesTransferred;
}
