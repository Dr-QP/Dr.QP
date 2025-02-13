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
#include <utility>
#include <optional>

#include <boost/asio.hpp>
#include <boost/asio/read.hpp>
#include <boost/asio/write.hpp>

enum class AsyncOp { Read, Write };

template <AsyncOp operation>
struct AsyncOperation;

template <>
struct AsyncOperation<AsyncOp::Read>
{
  template <typename Stream, class... Args>
  static void perform(Stream& stream, Args... args)
  {
    return async_read(stream, std::forward<Args>(args)...);
  }
};

template <>
struct AsyncOperation<AsyncOp::Write>
{
  template <typename Stream, class... Args>
  static void perform(Stream& stream, Args... args)
  {
    return async_write(stream, std::forward<Args>(args)...);
  }
};

template <AsyncOp operation, typename MutableBufferSequence, typename Stream>
size_t doWithTimeout(
  boost::asio::io_service& ioService, Stream& stream, const MutableBufferSequence& buffers,
  uint64_t timeoutMs = 5000)
{
  std::optional<boost::system::error_code> timerResult;
  boost::asio::deadline_timer timer(ioService);
  timer.expires_from_now(boost::posix_time::milliseconds(timeoutMs));
  timer.async_wait([&timerResult](const auto& ec) { timerResult = ec; });

  std::optional<boost::system::error_code> operationResult;
  std::optional<std::size_t> bytesTransferred;

  AsyncOperation<operation>::perform(
    stream, buffers, [&operationResult, &bytesTransferred](const auto& ec, std::size_t bt) {
      operationResult = ec;
      bytesTransferred = bt;
    });

  ioService.reset();
  while (ioService.run_one()) {
    if (operationResult)
      timer.cancel();
    else if (timerResult)
      stream.cancel();
  }

  if (operationResult && *operationResult) {
    throw boost::system::system_error(*operationResult);
    return 0;
  }

  return *bytesTransferred;
}
