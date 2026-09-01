/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez
 * Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós,
 * University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * ORB-SLAM3. If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <fmt/core.h>

#include <string>
#include <utility>

#include "Thirdparty/tl/expected.hpp"

namespace ORB_SLAM3 {

class ExpectedError {
 public:
  ExpectedError() = delete;
  ExpectedError(const ExpectedError &) = default;

  explicit ExpectedError(const std::string &errmsg) : err_(errmsg) {}

  // \todo{} I think this isn't very efficient, results in an extra copy?
  //         Although it only happens in "error" conditions.
  template <typename... Args>
  static ExpectedError fmt(fmt::format_string<Args...> rt_fmt_str,
                           Args &&...args) {
    std::string str;
    auto it = std::back_inserter(str);
    fmt::format_to(it, rt_fmt_str, std::forward<Args>(args)...);
    return ExpectedError(str);
  }

  const std::string msg() const { return err_; }

  std::string err_;
};

}  // namespace ORB_SLAM3
