/*
BSD 3-Clause License

Copyright (c) 2024 ETH Zurich, Autonomous Systems Lab, Rik Girod

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#pragma once

#include <memory>
#include <vector>

#include <gtsam/geometry/Point3.h>

#include "rio/common.h"

namespace rio {

class Track {
 public:
  typedef std::shared_ptr<Track> Ptr;

  Track(const CfarDetection& cfar_detection, const uint64_t id,
        const uint64_t max_age = 1)
      : cfar_detection_(cfar_detection), id_(id), max_age_(max_age) {}

  // returns true if the detection was added.
  bool addCfarDetection(const CfarDetection& cfar_detection);
  // returns true if the track is still valid.
  inline bool isValid() const { return age_ < max_age_; };
  inline void update() { age_++; };
  inline uint64_t getId() const { return id_; }
  inline gtsam::Point3 getR_p_RT() const {
    return {cfar_detection_.x, cfar_detection_.y, cfar_detection_.z};
  }
  inline void setAdded() { added_ = true; }
  inline bool isAdded() const { return added_; }

 private:
  CfarDetection cfar_detection_;
  uint64_t age_{0};
  uint64_t id_{0};
  uint64_t max_age_{1};
  bool added_{false};
};

class Tracker {
 public:
  Tracker() = default;
  Tracker(const uint64_t max_age);
  // Add CFAR detections and return the updated tracks at the given time.
  std::vector<Track::Ptr> addCfarDetections(
      const std::vector<CfarDetection>& cfar_detection);

 private:
  bool detectLandmark(const CfarDetection& cfar_detection) const;
  uint64_t id_{0};
  uint64_t max_age_{1};

  std::vector<Track::Ptr> tracks_;
};

}  // namespace rio