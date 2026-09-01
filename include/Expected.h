/**  * This file is part of ORB‑SLAM3  *
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
// 头文件保护，防止头文件重复包含
#pragma once

// fmt格式化库核心头文件，用于字符串格式化输出错误信息
#include <fmt/core.h>
// C++标准字符串
#include <string>
// std::forward完美转发工具
#include <utility>

// tl::expected第三方库，用于函数返回值：成功返回结果，失败返回错误对象
#include "Thirdparty/tl/expected.hpp"

namespace ORB_SLAM3 {

/**
 * @brief 错误信息封装类，配合tl::expected使用，用于函数调用失败时携带错误描述字符串
 */
class ExpectedError {
public:
    /// 删除默认无参构造，不允许构造空的错误对象
    ExpectedError() = delete;
    /// 拷贝构造函数，使用编译器默认实现
    ExpectedError(const ExpectedError &) = default;

    /**
     * @brief 显式构造函数，传入错误消息字符串
     * @param errmsg 错误描述文本
     */
    explicit ExpectedError(const std::string &errmsg) : err_(errmsg) {}

    // \todo{} I think this isn't very efficient, results in an extra copy?
    //         Although it only happens in "error" conditions.
    /**
     * @brief 静态工厂方法，使用fmt库格式化生成ExpectedError错误实例
     * @tparam Args 可变参数包，格式化占位符对应的参数类型
     * @param rt_fmt_str fmt格式化字符串模板
     * @param args 可变参数，会被完美转发给fmt格式化函数
     * @return ExpectedError 生成的错误对象，内部保存格式化完成的错误字符串
     */
    template <typename... Args>
    static ExpectedError fmt(fmt::format_string<Args...> rt_fmt_str,
                             Args &&...args) {
        std::string str;
        // 获取字符串后端插入迭代器，把格式化输出追加到str字符串
        auto it = std::back_inserter(str);
        // fmt格式化输出，使用std::forward实现参数完美转发
        fmt::format_to(it, rt_fmt_str, std::forward<Args>(args)...);
        return ExpectedError(str);
    }

    /**
     * @brief 获取错误消息，只读接口
     * @return const std::string& 错误信息字符串
     */
    const std::string msg() const { return err_; }

    /// 存储实际的错误信息字符串，成员变量公开
    std::string err_;
};

}  // namespace ORB_SLAM3
