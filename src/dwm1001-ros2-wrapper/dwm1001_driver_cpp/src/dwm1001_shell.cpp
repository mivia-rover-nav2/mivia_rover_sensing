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

#include "dwm1001_driver_cpp/dwm1001_shell.hpp"

#include <chrono>
#include <thread>

namespace dwm1001_driver_cpp
{

Dwm1001Shell::Dwm1001Shell(SerialPort & serial)
: serial_(serial)
{}

void Dwm1001Shell::sendCommand(const std::string & cmd, double delay_seconds)
{
  serial_.write(cmd);
  serial_.write("\r");
  // Mirror of pyserial's ser.flush() in the Python driver: wait for the
  // bytes to actually leave the port before starting the fixed delay,
  // rather than just having handed them to the kernel's write buffer.
  serial_.drain();
  std::this_thread::sleep_for(std::chrono::duration<double>(delay_seconds));
}

void Dwm1001Shell::enterShellMode(double delay_seconds)
{
  serial_.write("\r\r");
  serial_.drain();
  std::this_thread::sleep_for(std::chrono::duration<double>(delay_seconds));
  serial_.flushInput();
}

}  // namespace dwm1001_driver_cpp
