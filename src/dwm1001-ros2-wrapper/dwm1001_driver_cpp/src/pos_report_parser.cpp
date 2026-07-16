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

#include "dwm1001_driver_cpp/pos_report_parser.hpp"

#include <sstream>
#include <vector>

namespace dwm1001_driver_cpp
{

namespace
{

// Note: fields are not trimmed. The last field may carry a trailing '\r'
// (SerialPort::readLine() strips '\n' but not '\r'), same as pydwm1001,
// which never strips either -- std::stod/std::stoi (like Python's
// float()/int()) parse the leading numeric part and ignore trailing
// whitespace, so this is harmless.
std::vector<std::string> splitCsv(const std::string & line)
{
  std::vector<std::string> fields;
  std::stringstream ss(line);
  std::string field;
  while (std::getline(ss, field, ',')) {
    fields.push_back(field);
  }
  return fields;
}

// std::stod/std::stoi throw std::invalid_argument/std::out_of_range on a
// non-numeric or out-of-range field -- a structurally valid line (right
// field count, "POS" tag) can still have a garbled numeric field from a
// UART glitch. Wrap them so every parsing failure surfaces as the single
// ParsingError type callers already catch, instead of letting a
// std::logic_error escape uncaught and crash the node.
double parseDouble(const std::string & field)
{
  try {
    return std::stod(field);
  } catch (const std::exception &) {
    throw ParsingError("Position report has a non-numeric field: '" + field + "'.");
  }
}

int parseInt(const std::string & field)
{
  try {
    return std::stoi(field);
  } catch (const std::exception &) {
    throw ParsingError("Position report has a non-numeric field: '" + field + "'.");
  }
}

}  // namespace

PositionReport parseActivePositionReport(const std::string & line)
{
  std::vector<std::string> report = splitCsv(line);

  if (report.size() != 5) {
    throw ParsingError("Position report has unexpected length.");
  }
  if (report[0] != "POS") {
    throw ParsingError("Position report has incorrect tag.");
  }

  PositionReport result;
  result.x = parseDouble(report[1]);
  result.y = parseDouble(report[2]);
  result.z = parseDouble(report[3]);
  result.quality = parseInt(report[4]);
  return result;
}

PassivePositionReport parsePassivePositionReport(const std::string & line)
{
  std::vector<std::string> report = splitCsv(line);

  if (report.size() != 8) {
    throw ParsingError("Position report has unexpected length.");
  }
  if (report[0] != "POS") {
    throw ParsingError("Position report has incorrect tag.");
  }

  PassivePositionReport result;
  result.tag_name = "DW" + report[2];
  result.position.x = parseDouble(report[3]);
  result.position.y = parseDouble(report[4]);
  result.position.z = parseDouble(report[5]);
  result.position.quality = parseInt(report[6]);
  return result;
}

}  // namespace dwm1001_driver_cpp
