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

#ifndef DWM1001_DRIVER_CPP__SERIAL_PORT_HPP_
#define DWM1001_DRIVER_CPP__SERIAL_PORT_HPP_

#include <optional>
#include <stdexcept>
#include <string>

namespace dwm1001_driver_cpp
{

class SerialError : public std::runtime_error
{
public:
  explicit SerialError(const std::string & message)
  : std::runtime_error(message)
  {}
};

// Raw POSIX termios wrapper covering the subset of pyserial.Serial used by
// the Python DWM1001 nodes: blocking reads bounded by a per-call timeout,
// explicit DTR/RTS control, no hardware/software flow control. Silent by
// design (throws SerialError on failure) -- callers are responsible for
// logging, mirroring how pyserial itself never logs.
class SerialPort
{
public:
  SerialPort() = default;
  ~SerialPort();

  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;

  // timeout_deciseconds: max time a read() call waits for at least one
  // byte, in tenths of a second (termios VTIME units).
  void open(const std::string & path, int baudrate, int timeout_deciseconds);
  void close();
  bool isOpen() const;

  void write(const std::string & data);

  // Blocks until all previously written bytes have been physically
  // transmitted (POSIX tcdrain). Mirrors pyserial's Serial.flush(), which
  // the Python driver calls after every shell command write before
  // sleeping a fixed delay -- without draining, that fixed delay could
  // start counting down before the bytes actually left the port.
  void drain();

  // Reads and buffers available bytes, returning a complete line (without
  // the trailing '\n') once one is assembled. Returns std::nullopt if no
  // new bytes arrived within the configured timeout (mirrors pyserial's
  // readline() returning empty on timeout) -- the caller is expected to
  // call this repeatedly from a loop.
  std::optional<std::string> readLine();

  // Number of bytes currently sitting in the OS input buffer (mirrors
  // pyserial's Serial.in_waiting).
  int bytesWaiting() const;

  // Single best-effort read of up to max_bytes, bypassing the line buffer
  // used by readLine(). Used only for diagnostic "peek" logging, not for
  // line-oriented parsing.
  std::string readRaw(std::size_t max_bytes);

  void setDtr(bool on);
  void setRts(bool on);
  void flushInput();

private:
  int fd_ = -1;
  bool is_open_ = false;
  std::string buffer_;
  int timeout_ms_ = 0;
};

}  // namespace dwm1001_driver_cpp

#endif  // DWM1001_DRIVER_CPP__SERIAL_PORT_HPP_
