/**  * This file was added to ORB‑SLAM3  *
 * Copyright (C) 2026b Aaron Marburg
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

// STL双端队列容器，用于保存最近若干帧的FPS数值
#include <deque>

namespace ORB_SLAM3 {

/**
 * @brief FPS估算器，统计帧时间戳，滑动窗口计算平均帧率
 * 维护固定深度的双端队列，保存最近N帧的fps，做简单算术平均得到当前帧率
 */
class FpsEstimator {
public:
    /// 滑动窗口队列最大深度，最多保存50个FPS样本
    const int DequeDepth = 50;

    /**
     * @brief 显式构造函数，可传入先验FPS初值
     * @param prior 先验FPS初始值，小于等于0时不预填充队列
     */
    explicit FpsEstimator(float prior = -1.0) : prev_ts_(-1.0) {
        // 如果传入合法的先验fps，用该值预先填满整个队列
        if (prior > 0) {
            fps_q_ = deque<float>(DequeDepth, prior);
        }
    }

    /**
     * @brief 推入一帧的时间戳，内部计算帧间隔dt，换算fps存入滑动窗口
     * @param ts 当前帧时间戳，单位一般为秒
     */
    void pushTimestamp(double ts) {
        // 第一帧，只记录上一帧时间戳，不计算间隔
        if (prev_ts_ < 0) {
            prev_ts_ = ts;
            return;
        }

        // 计算两帧时间差，取绝对值防止时间戳回跳
        const double dt = fabs(ts - prev_ts_);
        // 由帧间隔换算瞬时帧率 fps = 1 / delta_t
        const double fps = 1 / dt;
        // 判断fps不是NaN非法数值，才加入队列
        if (!isnan(fps)) {
            // 新的fps放到队列头部
            fps_q_.push_front(fps);

            // 如果队列长度超过设定窗口深度，弹出队尾旧样本，维持窗口大小
            while (fps_q_.size() > DequeDepth) fps_q_.pop_back();
        }

        // 更新上一帧时间戳为本帧时间戳，供下一次调用使用
        prev_ts_ = ts;
    }

    /**
     * @brief 获取当前滑动窗口平均FPS
     * @return float 平均帧率；队列为空时返回0.0
     */
    float fps() const {
        // 队列无有效样本，直接返回0
        if (fps_q_.size() == 0) return 0.0;

        // Very simple average to start with
        // 对队列中全部fps样本求和，做简单算术平均
        float sum = accumulate(fps_q_.begin(), fps_q_.end(), 0.0);
        return sum / fps_q_.size();
    }

private:
    /// 上一帧的时间戳，初始值‑1.0标记尚未收到有效帧
    double prev_ts_;
    /// 双端队列滑动窗口，存储最近DequeDepth个瞬时FPS值
    deque<float> fps_q_;
};

}  // namespace ORB_SLAM3
