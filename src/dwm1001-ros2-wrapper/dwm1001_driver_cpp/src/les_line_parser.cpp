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

#include "dwm1001_driver_cpp/les_line_parser.hpp"

#include <regex>
#include <utility>

#include "dwm1001_driver_cpp/string_utils.hpp"

namespace dwm1001_driver_cpp
{

namespace
{

// Groups: 1=id, 2=x, 3=y, 4=z, 5=distance. std::regex (ECMAScript
// grammar) has no named capture groups, unlike Python's re.
const std::regex kAnchorPattern(
  R"(([0-9A-Fa-f]{4})\s*\[\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*,\s*)"
  R"((-?\d+(?:\.\d+)?)\s*\]\s*=\s*(-?\d+(?:\.\d+)?))");

// Groups: 1=x, 2=y, 3=z, 4=quality.
const std::regex kEstPattern(
  R"(est\[\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*,\s*)"
  R"((-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*\])");

}  // namespace

std::vector<AnchorReading> parseAnchors(const std::string & line)
{
  std::vector<AnchorReading> readings;
  for (auto it = std::sregex_iterator(line.begin(), line.end(), kAnchorPattern);
    it != std::sregex_iterator(); ++it)
  {
    const std::smatch & m = *it;
    AnchorReading reading;
    reading.id = toUpper(m[1].str());
    reading.x = std::stod(m[2].str());
    reading.y = std::stod(m[3].str());
    reading.z = std::stod(m[4].str());
    reading.distance = std::stod(m[5].str());
    readings.push_back(std::move(reading));
  }
  return readings;
}

std::optional<TagEstimate> parseEstimate(const std::string & line)
{
  std::smatch m;
  if (!std::regex_search(line, m, kEstPattern)) {
    return std::nullopt;
  }

  TagEstimate estimate;
  estimate.x = std::stod(m[1].str());
  estimate.y = std::stod(m[2].str());
  estimate.z = std::stod(m[3].str());
  estimate.quality = std::stod(m[4].str());
  return estimate;
}

}  // namespace dwm1001_driver_cpp
