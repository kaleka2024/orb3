/**
 * This file is part of ORB‑SLAM3
 *
 * Copyright (C) 2017‑2021 Carlos Campos, Richard Elvira, Juan J. Gómez
 * Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014‑2016 Raúl Mur‑Artal, José M.M. Montiel and Juan D. Tardós,
 * University of Zaragoza.
 *
 * ORB‑SLAM3 is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * ORB‑SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * ORB‑SLAM3. If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once // 头文件保护，防止重复包含引发编译重定义
#include <spdlog/spdlog.h> // spdlog高性能日志库主头文件
#include <memory>          // std::shared_ptr智能指针
#include <string>          // std::string字符串类型
#include <utility>         // std::forward完美转发工具

namespace ORB_SLAM3 {

/**
 * @brief 全局单例日志封装类，对spdlog做一层包装，全局共用同一个logger实例
 */
class Logger {
public:
    /**
     * @brief 获取Logger单例实例，静态局部变量实现懒加载单例
     * @param logger_in 外部传入spdlog日志器，可为nullptr；如果传入有效logger则替换内部日志对象
     * @return Logger单例对象的共享指针引用
     */
    static std::shared_ptr<Logger> &get_instance(
        const std::shared_ptr<spdlog::logger> &logger_in = nullptr) {
        // 静态局部变量，程序第一次调用才执行init完成实例创建
        static std::shared_ptr<Logger> s_logger(init(logger_in));
        // 如果传入新logger对象，且与当前持有的logger不一致，则替换内部logger_
        if ((logger_in) && (logger_in != s_logger->logger_))
            s_logger->logger_ = logger_in;
        return s_logger;
    }

    /**
     * @brief 获取底层spdlog::logger原始指针引用
     * @return spdlog日志器共享指针引用
     */
    static std::shared_ptr<spdlog::logger> &get_logger() {
        return Logger::get_instance()->logger_;
    }

    /**
     * @brief 设置外部spdlog日志器，调用get_instance完成替换
     * @param s 外部传入spdlog日志器共享指针
     */
    static void set_logger(const std::shared_ptr<spdlog::logger> &s) {
        Logger::get_instance(s);
    }

    /**
     * @brief 给底层logger增加输出sink（输出目标：控制台/文件等）
     * @param s spdlog输出槽sink智能指针
     * @note 根据spdlog官方文档，该接口本身不是线程安全，外部调用需要自行保证线程安全
     */
    static void add_sink(const spdlog::sink_ptr &s) {
        // n.b. per the spdlog documentation, this is _not_ thread safe
        Logger::get_logger()->sinks().push_back(s);
    }

    /**
     * @brief Logger析构函数，默认空实现，智能指针管理生命周期
     */
    ~Logger() {}
private:
    /**
     * @brief 单例初始化静态函数，使用new调用私有构造，封装到shared_ptr
     * @param logger_in 外部传入spdlog日志器
     * @return Logger对象的shared_ptr
     */
    static std::shared_ptr<Logger> init(
        const std::shared_ptr<spdlog::logger> &logger_in = nullptr) {
        // Use new to access private constructor
        return std::shared_ptr<Logger>(new Logger(logger_in));
    }

    /**
     * @brief 私有构造函数，外部无法直接实例化，保证单例；explicit禁止隐式类型转换
     * @param l spdlog日志器共享指针，可为nullptr
     */
    explicit Logger(const std::shared_ptr<spdlog::logger> &l = nullptr)
        : logger_(l) {
        // 如果外部没有传入logger，则内部新建名为"orbslam3"的spdlog logger并注册
        if (!logger_) {
            logger_ = std::make_shared<spdlog::logger>("orbslam3");
            spdlog::register_logger(logger_);
        }
    }

    std::shared_ptr<spdlog::logger> logger_; // 底层持有的spdlog日志器实例

    Logger(const Logger &) = delete;            // 删除拷贝构造函数，禁止拷贝单例对象
    Logger &operator=(const Logger &) = delete; // 删除拷贝赋值运算符，禁止拷贝赋值
};

// Convenience wrappers around "oslog::"
/**
 * @brief oslog命名空间，对外提供便捷日志模板包装函数，上层业务直接调用
 */
namespace oslog {
using spdlog::format_string_t; // 导入spdlog格式化字符串类型
using spdlog::source_loc;      // 导入源码位置信息结构体（文件名、行号）

/**
 * @brief 带源码位置信息的通用日志模板，支持可变参数格式化输出
 * @tparam Args 可变模板参数包
 * @param source 源码位置信息（文件、行号）
 * @param lvl 日志等级 trace/debug/info/warn/error/critical
 * @param fmt 格式化字符串
 * @param args 可变参数列表，std::forward完美转发保留参数左右值属性
 */
template <typename... Args> inline void log(source_loc source, spdlog::level::level_enum lvl,
                format_string_t<Args...> fmt, Args &&...args) {
    Logger::get_logger()->log(source, lvl, fmt, std::forward<Args>(args)...);
}

/**
 * @brief 不带源码位置的通用日志模板，自动填充空source_loc
 * @tparam Args 可变模板参数包
 * @param lvl 日志等级
 * @param fmt 格式化字符串
 * @param args 可变参数列表
 */
template <typename... Args> inline void log(spdlog::level::level_enum lvl, format_string_t<Args...> fmt,
                Args &&...args) {
    Logger::get_logger()->log(source_loc{}, lvl, fmt,
                              std::forward<Args>(args)...);
}

/**
 * @brief trace级别日志，最细粒度调试信息
 * @tparam Args 可变模板参数包
 * @param fmt 格式化字符串
 * @param args 可变参数
 */
template <typename... Args> inline void trace(format_string_t<Args...> fmt, Args &&...args) {
    Logger::get_logger()->trace(fmt, std::forward<Args>(args)...);
}

/**
 * @brief debug级别日志，调试输出
 * @tparam Args 可变模板参数包
 * @param fmt 格式化字符串
 * @param args 可变参数
 */
template <typename... Args> inline void debug(format_string_t<Args...> fmt, Args &&...args) {
    Logger::get_logger()->debug(fmt, std::forward<Args>(args)...);
}

/**
 * @brief info级别日志，普通运行时信息输出
 * @tparam Args 可变模板参数包
 * @param fmt 格式化字符串
 * @param args 可变参数
 */
template <typename... Args> inline void info(format_string_t<Args...> fmt, Args &&...args) {
    Logger::get_logger()->info(fmt, std::forward<Args>(args)...);
}

/**
 * @brief warn级别日志，警告信息
 * @tparam Args 可变模板参数包
 * @param fmt 格式化字符串
 * @param args 可变参数
 */
template <typename... Args> inline void warn(format_string_t<Args...> fmt, Args &&...args) {
    Logger::get_logger()->warn(fmt, std::forward<Args>(args)...);
}

/**
 * @brief error级别日志，错误信息
 * @tparam Args 可变模板参数包
 * @param fmt 格式化字符串
 * @param args 可变参数
 */
template <typename... Args> inline void error(format_string_t<Args...> fmt, Args &&...args) {
    Logger::get_logger()->error(fmt, std::forward<Args>(args)...);
}

/**
 * @brief critical级别日志，严重致命错误
 * @tparam Args 可变模板参数包
 * @param fmt 格式化字符串
 * @param args 可变参数
 */
template <typename... Args> inline void critical(format_string_t<Args...> fmt, Args &&...args) {
    Logger::get_logger()->critical(fmt, std::forward<Args>(args)...);
}

/**
 * @brief 带源码位置，直接输出对象消息（非格式化字符串版本）
 * @tparam T 消息对象类型，可以直接输出的类型
 * @param source 源码位置信息
 * @param lvl 日志等级
 * @param msg 待输出消息对象
 */
template <typename T> inline void log(source_loc source, spdlog::level::level_enum lvl,
                const T &msg) {
    Logger::get_logger()->log(source, lvl, msg);
}

/**
 * @brief 不带源码位置，直接输出对象消息（非格式化字符串版本）
 * @tparam T 消息对象类型
 * @param lvl 日志等级
 * @param msg 待输出消息对象
 */
template <typename T> inline void log(spdlog::level::level_enum lvl, const T &msg) {
    Logger::get_logger()->log(lvl, msg);
}
} // namespace oslog

// Old logging framework for compatibility
/**
 * @brief 旧版日志兼容类，为兼容老代码保留，底层转发到新oslog接口
 */
class Verbose {
public:
    /**
     * @brief 旧版日志输出等级枚举
     */
    enum eLevel {
        VERBOSITY_QUIET = 0,        // 静默，极少输出
        VERBOSITY_NORMAL = 1,       // 普通模式
        VERBOSITY_VERBOSE = 2,      // 详细输出
        VERBOSITY_VERY_VERBOSE = 3, // 非常详细
        VERBOSITY_DEBUG = 4         // debug调试模式
    };

    static eLevel th; // 全局输出等级阈值，静态成员

public:
    /**
     * @brief 旧版打印消息接口，将旧等级映射到新oslog日志等级输出
     * @param str 待打印字符串
     * @param lev 旧Verbose日志等级
     */
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

    /**
     * @brief 设置输出等级阈值，兼容接口，当前为空实现
     * @param _th 设置的日志等级
     */
    static void SetTh(eLevel _th) {}
};

} // namespace ORB_SLAM3
