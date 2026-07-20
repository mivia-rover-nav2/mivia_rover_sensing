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

std::vector<AnchorReading> parseAnchors(const std::string & line)
{
  // std::regex (ECMAScript grammar) has no named capture groups, unlike
  // Python's re -- name the group indices instead of leaving m[5] etc.
  // to be decoded against the pattern by hand.
  constexpr int kIdGroup = 1;
  constexpr int kXGroup = 2;
  constexpr int kYGroup = 3;
  constexpr int kZGroup = 4;
  constexpr int kDistanceGroup = 5;

  // Function-local static rather than namespace-scope: std::regex's
  // constructor can throw std::regex_error on a malformed pattern, and a
  // namespace-scope object would be constructed before main() even
  // starts, where that exception would be uncatchable (std::terminate()
  // with no diagnostic from this program). A local static is instead
  // constructed on first call, inside code main()'s try/catch can reach.
  static const std::regex kAnchorPattern(
    R"(([0-9A-Fa-f]{4})\s*\[\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*,\s*)"
    R"((-?\d+(?:\.\d+)?)\s*\]\s*=\s*(-?\d+(?:\.\d+)?))");

  std::vector<AnchorReading> readings;
  for (auto it = std::sregex_iterator(line.begin(), line.end(), kAnchorPattern);
    it != std::sregex_iterator(); ++it)
  {
    const std::smatch & m = *it;
    try {
      AnchorReading reading;
      reading.id = toUpper(m[kIdGroup].str());
      reading.x = std::stod(m[kXGroup].str());
      reading.y = std::stod(m[kYGroup].str());
      reading.z = std::stod(m[kZGroup].str());
      reading.distance = std::stod(m[kDistanceGroup].str());
      readings.push_back(std::move(reading));
    } catch (const std::out_of_range &) {
      // The regex only matches well-formed digit runs, so std::stod can't
      // fail on syntax -- but an unusually long run (e.g. a desynced/
      // garbled stream that happens to still match) can overflow double,
      // which std::stod treats as an error where Python's float() would
      // just round. Skip this one match rather than losing the whole
      // line's anchors (or, unhandled, crashing the node via main()'s
      // catch-all -- rclpy tolerates exceptions in a timer callback,
      // rclcpp here does not).
      continue;
    }
  }
  return readings;
}

std::optional<TagEstimate> parseEstimate(const std::string & line)
{
  // Groups: 1=x, 2=y, 3=z, 4=quality. See parseAnchors() above for why
  // this is a function-local static rather than namespace-scope.
  static const std::regex kEstPattern(
    R"(est\[\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*,\s*)"
    R"((-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*\])");

  std::smatch m;
  if (!std::regex_search(line, m, kEstPattern)) {
    return std::nullopt;
  }

  try {
    TagEstimate estimate;
    estimate.x = std::stod(m[1].str());
    estimate.y = std::stod(m[2].str());
    estimate.z = std::stod(m[3].str());
    estimate.quality = std::stod(m[4].str());
    return estimate;
  } catch (const std::out_of_range &) {
    return std::nullopt;
  }
}

}  // namespace dwm1001_driver_cpp
