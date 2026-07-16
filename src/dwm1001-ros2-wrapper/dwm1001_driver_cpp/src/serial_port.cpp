// Copyright 2023 The Human and Intelligent Vehicle Ensembles (HIVE) Lab
// Copyright 2026 Gianmarco Guerriero (mivia_rover C++ port)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "dwm1001_driver_cpp/serial_port.hpp"

#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

namespace dwm1001_driver_cpp
{

namespace
{

speed_t baudToSpeed(int baudrate)
{
  switch (baudrate) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    default:
      throw SerialError("Unsupported baudrate: " + std::to_string(baudrate));
  }
}

std::string errnoMessage(const std::string & context)
{
  return context + ": " + std::strerror(errno);
}

// Returns true once the fd has data to read, false on timeout. Throws on
// poll() failure or if the fd reports a hangup/error condition -- this is
// how a physically removed USB-serial (CDC-ACM) device gets detected.
// Relying on read() alone is not enough: on such devices read() can keep
// returning 0 bytes forever after unplugging instead of surfacing an
// error, since VTIME-based timeouts are not reliably honored.
bool waitReadable(int fd, int timeout_ms)
{
  struct pollfd pfd {};
  pfd.fd = fd;
  pfd.events = POLLIN;

  int ret = ::poll(&pfd, 1, timeout_ms);
  if (ret < 0) {
    throw SerialError(errnoMessage("poll failed"));
  }
  if (ret == 0) {
    return false;
  }
  if (pfd.revents & (POLLHUP | POLLERR | POLLNVAL)) {
    throw SerialError("Serial port disconnected (poll reported HUP/ERR)");
  }
  return true;
}

}  // namespace

SerialPort::~SerialPort()
{
  if (is_open_) {
    close();
  }
}

void SerialPort::open(const std::string & path, int baudrate, int timeout_deciseconds)
{
  fd_ = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    throw SerialError(errnoMessage("Cannot open serial port '" + path + "'"));
  }

  // Reads are governed by VTIME/VMIN below, not by O_NONBLOCK.
  int flags = fcntl(fd_, F_GETFL, 0);
  fcntl(fd_, F_SETFL, flags & ~O_NONBLOCK);

  struct termios tty {};
  if (tcgetattr(fd_, &tty) != 0) {
    int saved_errno = errno;
    ::close(fd_);
    fd_ = -1;
    errno = saved_errno;
    throw SerialError(errnoMessage("tcgetattr failed on '" + path + "'"));
  }

  speed_t speed = baudToSpeed(baudrate);
  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);

  // 8N1, no hardware/software flow control (mirrors rtscts=False,
  // dsrdtr=False, xonxoff=False in the Python driver).
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag |= CREAD | CLOCAL;

  // Raw (non-canonical) mode: no line editing, no signal generation.
  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

  tty.c_oflag &= ~OPOST;

  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = static_cast<cc_t>(timeout_deciseconds);

  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    int saved_errno = errno;
    ::close(fd_);
    fd_ = -1;
    errno = saved_errno;
    throw SerialError(errnoMessage("tcsetattr failed on '" + path + "'"));
  }

  timeout_ms_ = timeout_deciseconds * 100;
  is_open_ = true;
  buffer_.clear();
}

void SerialPort::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
  is_open_ = false;
  buffer_.clear();
}

bool SerialPort::isOpen() const
{
  return is_open_;
}

void SerialPort::write(const std::string & data)
{
  std::size_t written = 0;
  while (written < data.size()) {
    ssize_t n = ::write(fd_, data.data() + written, data.size() - written);
    if (n < 0) {
      throw SerialError(errnoMessage("write failed"));
    }
    written += static_cast<std::size_t>(n);
  }
}

void SerialPort::drain()
{
  if (tcdrain(fd_) != 0) {
    throw SerialError(errnoMessage("tcdrain failed"));
  }
}

std::optional<std::string> SerialPort::readLine()
{
  auto newline_pos = buffer_.find('\n');
  if (newline_pos != std::string::npos) {
    std::string line = buffer_.substr(0, newline_pos);
    buffer_.erase(0, newline_pos + 1);
    return line;
  }

  if (!waitReadable(fd_, timeout_ms_)) {
    return std::nullopt;
  }

  char chunk[256];
  ssize_t n = ::read(fd_, chunk, sizeof(chunk));
  if (n < 0) {
    throw SerialError(errnoMessage("read failed"));
  }
  if (n == 0) {
    // poll() said readable but read() returned EOF: the other end is
    // gone (some drivers surface a removed device this way instead of
    // POLLHUP/POLLERR).
    throw SerialError("Serial port read returned EOF");
  }

  buffer_.append(chunk, static_cast<std::size_t>(n));

  newline_pos = buffer_.find('\n');
  if (newline_pos == std::string::npos) {
    return std::nullopt;
  }

  std::string line = buffer_.substr(0, newline_pos);
  buffer_.erase(0, newline_pos + 1);
  return line;
}

int SerialPort::bytesWaiting() const
{
  int n = 0;
  if (ioctl(fd_, FIONREAD, &n) < 0) {
    throw SerialError(errnoMessage("FIONREAD failed"));
  }
  return n;
}

std::string SerialPort::readRaw(std::size_t max_bytes)
{
  if (!waitReadable(fd_, timeout_ms_)) {
    return std::string();
  }

  std::string out(max_bytes, '\0');
  ssize_t n = ::read(fd_, out.data(), max_bytes);
  if (n < 0) {
    throw SerialError(errnoMessage("read failed"));
  }
  if (n == 0) {
    throw SerialError("Serial port read returned EOF");
  }
  out.resize(static_cast<std::size_t>(n));
  return out;
}

void SerialPort::setDtr(bool on)
{
  int status = 0;
  if (ioctl(fd_, TIOCMGET, &status) < 0) {
    throw SerialError(errnoMessage("TIOCMGET failed"));
  }
  if (on) {
    status |= TIOCM_DTR;
  } else {
    status &= ~TIOCM_DTR;
  }
  if (ioctl(fd_, TIOCMSET, &status) < 0) {
    throw SerialError(errnoMessage("TIOCMSET (DTR) failed"));
  }
}

void SerialPort::setRts(bool on)
{
  int status = 0;
  if (ioctl(fd_, TIOCMGET, &status) < 0) {
    throw SerialError(errnoMessage("TIOCMGET failed"));
  }
  if (on) {
    status |= TIOCM_RTS;
  } else {
    status &= ~TIOCM_RTS;
  }
  if (ioctl(fd_, TIOCMSET, &status) < 0) {
    throw SerialError(errnoMessage("TIOCMSET (RTS) failed"));
  }
}

void SerialPort::flushInput()
{
  tcflush(fd_, TCIFLUSH);
  buffer_.clear();
}

}  // namespace dwm1001_driver_cpp
