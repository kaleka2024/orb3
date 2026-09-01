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

#include <spdlog/spdlog.h>

#include <memory>
#include <string>
#include <utility>

namespace ORB_SLAM3 {

class Logger {
 public:
  static std::shared_ptr<Logger> &get_instance(
      const std::shared_ptr<spdlog::logger> &logger_in = nullptr) {
    static std::shared_ptr<Logger> s_logger(init(logger_in));
    if ((logger_in) && (logger_in != s_logger->logger_))
      s_logger->logger_ = logger_in;
    return s_logger;
  }

  static std::shared_ptr<spdlog::logger> &get_logger() {
    return Logger::get_instance()->logger_;
  }

  static void set_logger(const std::shared_ptr<spdlog::logger> &s) {
    Logger::get_instance(s);
  }

  static void add_sink(const spdlog::sink_ptr &s) {
    // n.b. per the spdlog documentation, this is _not_ thread safe
    Logger::get_logger()->sinks().push_back(s);
  }

  ~Logger() {}

 private:
  static std::shared_ptr<Logger> init(
      const std::shared_ptr<spdlog::logger> &logger_in = nullptr) {
    // Use new to access private constructor
    return std::shared_ptr<Logger>(new Logger(logger_in));
  }

  explicit Logger(const std::shared_ptr<spdlog::logger> &l = nullptr)
      : logger_(l) {
    if (!logger_) {
      logger_ = std::make_shared<spdlog::logger>("orbslam3");
      spdlog::register_logger(logger_);
    }
  }

  std::shared_ptr<spdlog::logger> logger_;

  Logger(const Logger &) = delete;
  Logger &operator=(const Logger &) = delete;
};

// Convenience wrappers around "oslog::"
namespace oslog {
using spdlog::format_string_t;
using spdlog::source_loc;

template <typename... Args>
inline void log(source_loc source, spdlog::level::level_enum lvl,
                format_string_t<Args...> fmt, Args &&...args) {
  Logger::get_logger()->log(source, lvl, fmt, std::forward<Args>(args)...);
}

template <typename... Args>
inline void log(spdlog::level::level_enum lvl, format_string_t<Args...> fmt,
                Args &&...args) {
  Logger::get_logger()->log(source_loc{}, lvl, fmt,
                            std::forward<Args>(args)...);
}

template <typename... Args>
inline void trace(format_string_t<Args...> fmt, Args &&...args) {
  Logger::get_logger()->trace(fmt, std::forward<Args>(args)...);
}

template <typename... Args>
inline void debug(format_string_t<Args...> fmt, Args &&...args) {
  Logger::get_logger()->debug(fmt, std::forward<Args>(args)...);
}

template <typename... Args>
inline void info(format_string_t<Args...> fmt, Args &&...args) {
  Logger::get_logger()->info(fmt, std::forward<Args>(args)...);
}

template <typename... Args>
inline void warn(format_string_t<Args...> fmt, Args &&...args) {
  Logger::get_logger()->warn(fmt, std::forward<Args>(args)...);
}

template <typename... Args>
inline void error(format_string_t<Args...> fmt, Args &&...args) {
  Logger::get_logger()->error(fmt, std::forward<Args>(args)...);
}

template <typename... Args>
inline void critical(format_string_t<Args...> fmt, Args &&...args) {
  Logger::get_logger()->critical(fmt, std::forward<Args>(args)...);
}

template <typename T>
inline void log(source_loc source, spdlog::level::level_enum lvl,
                const T &msg) {
  Logger::get_logger()->log(source, lvl, msg);
}

template <typename T>
inline void log(spdlog::level::level_enum lvl, const T &msg) {
  Logger::get_logger()->log(lvl, msg);
}
}  // namespace oslog

// Old logging framework for compatibility
class Verbose {
 public:
  enum eLevel {
    VERBOSITY_QUIET = 0,
    VERBOSITY_NORMAL = 1,
    VERBOSITY_VERBOSE = 2,
    VERBOSITY_VERY_VERBOSE = 3,
    VERBOSITY_DEBUG = 4
  };

  static eLevel th;

 public:
  static void PrintMess(std::string str, eLevel lev) {
    switch (lev) {
      case VERBOSITY_DEBUG:
        oslog::debug("{}", str);
        break;
      case VERBOSITY_VERY_VERBOSE:
        oslog::debug("{}", str);
        break;
      case VERBOSITY_VERBOSE:
        oslog::info("{}", str);
        break;
      case VERBOSITY_NORMAL:
        oslog::warn("{}", str);
        break;
      case VERBOSITY_QUIET:
        oslog::critical("{}", str);
        break;
    }
  }

  static void SetTh(eLevel _th) {}
};

}  // namespace ORB_SLAM3
