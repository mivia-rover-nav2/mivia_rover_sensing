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

#ifndef DWM1001_DRIVER_CPP__STRING_UTILS_HPP_
#define DWM1001_DRIVER_CPP__STRING_UTILS_HPP_

#include <algorithm>
#include <cctype>
#include <string>

namespace dwm1001_driver_cpp
{

inline std::string lstrip(const std::string & s)
{
  std::size_t i = 0;
  while (i < s.size() && std::isspace(static_cast<unsigned char>(s[i]))) {
    ++i;
  }
  return s.substr(i);
}

inline std::string rstrip(const std::string & s)
{
  std::size_t i = s.size();
  while (i > 0 && std::isspace(static_cast<unsigned char>(s[i - 1]))) {
    --i;
  }
  return s.substr(0, i);
}

inline std::string strip(const std::string & s)
{
  return rstrip(lstrip(s));
}

// Strips leading occurrences of `c` (mirrors Python's str.lstrip(chars)),
// as opposed to lstrip() above which strips whitespace.
inline std::string lstripChar(const std::string & s, char c)
{
  std::size_t i = 0;
  while (i < s.size() && s[i] == c) {
    ++i;
  }
  return s.substr(i);
}

inline std::string toUpper(std::string s)
{
  std::transform(
    s.begin(), s.end(), s.begin(),
    [](unsigned char c) {return std::toupper(c);});
  return s;
}

}  // namespace dwm1001_driver_cpp

#endif  // DWM1001_DRIVER_CPP__STRING_UTILS_HPP_
