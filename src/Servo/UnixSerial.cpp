#include "UnixSerial.h"
#include <boost/asio/write.hpp>
#include <boost/asio/read.hpp>
#include <iostream>
#include <vector>
#include <boost/asio.hpp>
#include <boost/thread.hpp>

/// @brief Returns the number of bytes available for reading from a serial
///        port without blocking.
std::size_t get_bytes_available(
    boost::asio::serial_port &serial_port,
    boost::system::error_code &error)
{
    error = boost::system::error_code();
    int value = 0;
#if defined(BOOST_ASIO_WINDOWS) || defined(__CYGWIN__)
    COMSTAT status;
    if (0 != ::ClearCommError(serial_port.lowest_layer().native_handle(), NULL, &status)) {
        value = status.cbInQue;
    }
        // On error, set the error code.
    else {
        error = boost::system::error_code(::GetLastError(), boost::asio::error::get_system_category());
    }
#else // defined(BOOST_ASIO_WINDOWS) || defined(__CYGWIN__)

    if (::ioctl(serial_port.lowest_layer().native_handle(), FIONREAD, &value) < 0) {
        error = boost::system::error_code(errno, boost::asio::error::get_system_category());
    }

#endif // defined(BOOST_ASIO_WINDOWS) || defined(__CYGWIN__)

    return error ? static_cast<std::size_t>(0)
                 : static_cast<size_t>(value);
}

/// @brief Returns the number of bytes available for reading from a serial
///        port without blocking.  Throws on error.
std::size_t get_bytes_available(boost::asio::serial_port &serial_port)
{
    boost::system::error_code error;
    std::size_t bytes_available = get_bytes_available(serial_port, error);
    if (error) {
        boost::throw_exception((boost::system::system_error(error)));
    }
    return bytes_available;
}

UnixSerial::UnixSerial(const std::string &fileName)
    : serial_(ioService_, fileName)
    , everRead_(false)
    , lastRead_(0)
{
}

void UnixSerial::begin(const unsigned long baudRate, const uint8_t transferConfig)
{
    using boost::asio::serial_port_base;

    serial_.set_option(serial_port_base::baud_rate(baudRate));
    serial_.set_option(serial_port_base::character_size(8));
    serial_.set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));
    serial_.set_option(serial_port_base::parity(serial_port_base::parity::none));
    serial_.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
}

size_t UnixSerial::write(uint8_t byte)
{
    return write(&byte, 1);
}

size_t UnixSerial::write(const uint8_t *data, size_t size)
{
    return boost::asio::write(serial_, boost::asio::buffer(data, size));
}

void UnixSerial::flushRead() {
  size_t bytes = get_bytes_available(serial_);
  if (bytes > 0) {
    std::vector<uint8_t> buf;
    buf.resize(bytes);
    readBytes(&buf[0], bytes);
  }
}

bool UnixSerial::available()
{
    return get_bytes_available(serial_) != 0;
}

uint8_t UnixSerial::peek()
{
    if (!available())
    {
        return kNoData;
    }
    if (!everRead_)
    {
        return read();
    }
    return lastRead_;
}

uint8_t UnixSerial::read()
{
    // if (!available()) {
    //     return kNoData;
    // }
    everRead_ = true;
    boost::asio::read(serial_, boost::asio::buffer(&lastRead_, 1));
    return lastRead_;
}

size_t UnixSerial::readBytes(uint8_t *buffer, size_t size)
{
    everRead_ = true;
    return boost::asio::read(serial_, boost::asio::buffer(buffer, size));
}

