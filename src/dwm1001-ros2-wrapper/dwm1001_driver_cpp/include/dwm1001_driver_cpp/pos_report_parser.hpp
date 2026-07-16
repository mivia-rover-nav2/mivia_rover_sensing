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

#ifndef DWM1001_DRIVER_CPP__POS_REPORT_PARSER_HPP_
#define DWM1001_DRIVER_CPP__POS_REPORT_PARSER_HPP_

#include <stdexcept>
#include <string>

namespace dwm1001_driver_cpp
{

// Mirror of pydwm1001's ParsingError.
class ParsingError : public std::runtime_error
{
public:
  explicit ParsingError(const std::string & message)
  : std::runtime_error(message)
  {}
};

struct PositionReport
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  int quality = 0;
};

struct PassivePositionReport
{
  std::string tag_name;
  PositionReport position;
};

// Parses a "POS,x,y,z,quality" line (5 fields), as emitted after 'lep' by
// an active tag. Throws ParsingError if the field count or the "POS" tag
// don't match -- mirror of pydwm1001's ActiveTag.position property.
PositionReport parseActivePositionReport(const std::string & line);

// Parses a "POS,uwb_addr,tag_short_id,x,y,z,quality,extra" line (8
// fields), as emitted after 'lep' when listening to broadcasts from
// other active tags. Throws ParsingError on mismatch -- mirror of
// pydwm1001's PassiveTag.wait_for_position_report.
PassivePositionReport parsePassivePositionReport(const std::string & line);

}  // namespace dwm1001_driver_cpp

#endif  // DWM1001_DRIVER_CPP__POS_REPORT_PARSER_HPP_
