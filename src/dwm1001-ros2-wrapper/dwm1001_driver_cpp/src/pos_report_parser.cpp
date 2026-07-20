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
  // "POS,x,y,z,quality" -- 5 comma-separated fields.
  constexpr std::size_t kFieldCount = 5;
  constexpr std::size_t kTagField = 0;
  constexpr std::size_t kXField = 1;
  constexpr std::size_t kYField = 2;
  constexpr std::size_t kZField = 3;
  constexpr std::size_t kQualityField = 4;

  std::vector<std::string> report = splitCsv(line);

  if (report.size() != kFieldCount) {
    throw ParsingError("Position report has unexpected length.");
  }
  if (report[kTagField] != "POS") {
    throw ParsingError("Position report has incorrect tag.");
  }

  PositionReport result;
  result.x = parseDouble(report[kXField]);
  result.y = parseDouble(report[kYField]);
  result.z = parseDouble(report[kZField]);
  result.quality = parseInt(report[kQualityField]);
  return result;
}

PassivePositionReport parsePassivePositionReport(const std::string & line)
{
  // "POS,uwb_addr,tag_short_id,x,y,z,quality,extra" -- 8 comma-separated
  // fields (uwb_addr and the trailing extra field are unused here).
  constexpr std::size_t kFieldCount = 8;
  constexpr std::size_t kTagField = 0;
  constexpr std::size_t kTagIdField = 2;
  constexpr std::size_t kXField = 3;
  constexpr std::size_t kYField = 4;
  constexpr std::size_t kZField = 5;
  constexpr std::size_t kQualityField = 6;

  std::vector<std::string> report = splitCsv(line);

  if (report.size() != kFieldCount) {
    throw ParsingError("Position report has unexpected length.");
  }
  if (report[kTagField] != "POS") {
    throw ParsingError("Position report has incorrect tag.");
  }

  PassivePositionReport result;
  result.tag_name = "DW" + report[kTagIdField];
  result.position.x = parseDouble(report[kXField]);
  result.position.y = parseDouble(report[kYField]);
  result.position.z = parseDouble(report[kZField]);
  result.position.quality = parseInt(report[kQualityField]);
  return result;
}

}  // namespace dwm1001_driver_cpp
