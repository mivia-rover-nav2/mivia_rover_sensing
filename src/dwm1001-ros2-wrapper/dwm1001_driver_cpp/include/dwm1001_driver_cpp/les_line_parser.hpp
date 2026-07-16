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

#ifndef DWM1001_DRIVER_CPP__LES_LINE_PARSER_HPP_
#define DWM1001_DRIVER_CPP__LES_LINE_PARSER_HPP_

#include <optional>
#include <string>
#include <vector>

namespace dwm1001_driver_cpp
{

struct AnchorReading
{
  std::string id;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double distance = 0.0;
};

struct TagEstimate
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double quality = 0.0;
};

// Parses a DWM1001 "les" mode output line for anchor blocks such as
// "1783[1.00,2.00,1.17]=2.87" (anchor id, its known position, and the
// measured range). A single line can contain several such blocks.
std::vector<AnchorReading> parseAnchors(const std::string & line);

// Parses a "les" mode output line for the tag position estimate, e.g.
// "est[1.23,4.56,0.10,80]". Returns std::nullopt if the line has no
// estimate block.
std::optional<TagEstimate> parseEstimate(const std::string & line);

}  // namespace dwm1001_driver_cpp

#endif  // DWM1001_DRIVER_CPP__LES_LINE_PARSER_HPP_
