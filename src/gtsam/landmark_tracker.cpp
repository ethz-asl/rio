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

#include "rio/gtsam/landmark_tracker.h"

#include <log++.h>

using namespace rio;

bool Track::addCfarDetection(const CfarDetection& cfar_detection) {
  bool equal = cfar_detection.x == cfar_detection_.x &&
               cfar_detection.y == cfar_detection_.y &&
               cfar_detection.z == cfar_detection_.z &&
               cfar_detection.velocity == cfar_detection_.velocity &&
               cfar_detection.snr == cfar_detection_.snr &&
               cfar_detection.noise == cfar_detection_.noise;
  if (!equal) {
    return false;
  }
  LOG(D, "Track " << id_ << " updated with detection: " << cfar_detection);
  LOG_TIMED(I, 5.0, "Zero-velocity track update.");
  age_ = 0;
  return true;
}

Tracker::Tracker(const uint64_t max_age) : max_age_(max_age) {}

bool Tracker::detectLandmark(const CfarDetection& cfar_detection) const {
  return cfar_detection.velocity == 0.0f;
}

std::vector<Track::Ptr> Tracker::addCfarDetections(
    const std::vector<CfarDetection>& cfar_detection) {
  // Filter deprecated tracks.
  std::vector<Track::Ptr> active_tracks;
  std::copy_if(tracks_.begin(), tracks_.end(),
               std::back_inserter(active_tracks),
               [&](const auto& track) { return track->isValid(); });
  std::for_each(active_tracks.begin(), active_tracks.end(),
                [](const auto& track) { track->update(); });

  std::vector<Track::Ptr> updated_tracks;
  for (const auto& cfar_detection : cfar_detection) {
    // Skip detections that are not landmarks.
    if (!detectLandmark(cfar_detection)) {
      continue;
    }
    // Update tracks.
    auto updated_track = std::find_if(
        active_tracks.begin(), active_tracks.end(),
        [&](auto& track) { return track->addCfarDetection(cfar_detection); });
    if (updated_track != active_tracks.end()) {
      updated_tracks.emplace_back(*updated_track);
      continue;
    }
    // Create new tracks.
    updated_tracks.emplace_back(new Track(cfar_detection, id_++, max_age_));
    active_tracks.push_back(updated_tracks.back());
    LOG(D, "New track created with id: " << id_ - 1);
  }

  LOG(D, "Returning " << updated_tracks.size() << " updated tracks.");
  LOG(D, "Total " << active_tracks.size() << " active tracks.");
  tracks_ = active_tracks;
  return updated_tracks;
}