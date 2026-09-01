/**
 * This file was added to ORB-SLAM3
 *
 * Copyright (C) 2026b Aaron Marburg
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

#include <deque>

namespace ORB_SLAM3 {

class FpsEstimator {
 public:
  const int DequeDepth = 50;

  explicit FpsEstimator(float prior = -1.0) : prev_ts_(-1.0) {
    if (prior > 0) {
      fps_q_ = deque<float>(DequeDepth, prior);
    }
  }

  void pushTimestamp(double ts) {
    if (prev_ts_ < 0) {
      prev_ts_ = ts;
      return;
    }

    const double dt = fabs(ts - prev_ts_);
    const double fps = 1 / dt;
    if (!isnan(fps)) {
      fps_q_.push_front(fps);

      while (fps_q_.size() > DequeDepth) fps_q_.pop_back();
    }

    prev_ts_ = ts;
  }
  float fps() const {
    if (fps_q_.size() == 0) return 0.0;

    // Very simple average to start with
    float sum = accumulate(fps_q_.begin(), fps_q_.end(), 0.0);
    return sum / fps_q_.size();
  }

 private:
  double prev_ts_;
  deque<float> fps_q_;
};

}  // namespace ORB_SLAM3
