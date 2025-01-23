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

#include <deque>

#include <boost/serialization/deque.hpp>

namespace RecordingProxy
{
enum class OperationType { kUndefined, kRead, kWrite };

struct Request
{
  std::deque<uint8_t> bytes;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version)
  {
    ar & bytes;
  }

  bool empty() const
  {
    return bytes.empty();
  }
};

struct Response
{
  std::deque<uint8_t> bytes;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version)
  {
    ar & bytes;
  }

  bool empty() const
  {
    return bytes.empty();
  }
};

struct Record
{
  Request request;
  Response response;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version)
  {
    ar & request;
    ar & response;
  }

  bool empty() const
  {
    return request.empty() && response.empty();
  }
};
}  // namespace RecordingProxy
