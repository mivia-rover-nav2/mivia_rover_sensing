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

#ifndef DWM1001_DRIVER_CPP__DWM1001_SHELL_HPP_
#define DWM1001_DRIVER_CPP__DWM1001_SHELL_HPP_

#include <string>

#include "dwm1001_driver_cpp/serial_port.hpp"

namespace dwm1001_driver_cpp
{

// Low-level primitives for talking to the DWM1001 firmware's interactive
// text shell over a SerialPort. Deliberately minimal: the exact sequencing
// and delays around reset/shell-entry differ between the driver nodes
// (les_mode_tag vs. the pydwm1001-style active/passive nodes), so those
// stay in the caller instead of being baked in here.
class Dwm1001Shell
{
public:
  explicit Dwm1001Shell(SerialPort & serial);

  // Writes `cmd` followed by a carriage return, then sleeps `delay_seconds`.
  void sendCommand(const std::string & cmd, double delay_seconds);

  // Writes a double carriage return to wake up the firmware shell prompt,
  // sleeps `delay_seconds`, then flushes the input buffer.
  void enterShellMode(double delay_seconds = 1.0);

private:
  SerialPort & serial_;
};

}  // namespace dwm1001_driver_cpp

#endif  // DWM1001_DRIVER_CPP__DWM1001_SHELL_HPP_
