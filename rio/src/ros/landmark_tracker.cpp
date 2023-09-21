#include "rio/ros/landmark_tracker.h"

#include <memory>

using namespace rio;

bool Track::addCfarDetection(
    const mav_sensors::Radar::CfarDetection& cfar_detection,
    const ros::Time& stamp) {
  bool equal = cfar_detection.x == cfar_detection_.x &&
               cfar_detection.y == cfar_detection_.y &&
               cfar_detection.z == cfar_detection_.z &&
               cfar_detection.velocity == cfar_detection_.velocity &&
               cfar_detection.snr == cfar_detection_.snr &&
               cfar_detection.noise == cfar_detection_.noise;
  if (!equal) {
    return false;
  }
  last_stamp_ = stamp;
  return true;
}

bool Track::isValid(const ros::Time& stamp) const {
  return (stamp - last_stamp_).toSec() < max_duration_;
}

Tracker::Tracker(const double max_duration) : max_duration_(max_duration) {}

bool Tracker::detectLandmark(
    const mav_sensors::Radar::CfarDetection& cfar_detection) const {
  return cfar_detection.velocity == 0.0f;
}

std::vector<Track::Ptr> Tracker::addCfarDetections(
    const std::vector<mav_sensors::Radar::CfarDetection>& cfar_detection,
    const ros::Time& stamp) {
  // Filter deprecated tracks.
  std::vector<Track::Ptr> active_tracks;
  std::copy_if(tracks_.begin(), tracks_.end(),
               std::back_inserter(active_tracks),
               [&](const auto& track) { return track->isValid(stamp); });

  for (const auto& cfar_detection : cfar_detection) {
    // Skip detections that are not landmarks.
    if (!detectLandmark(cfar_detection)) {
      continue;
    }
    // Update active tracks.
    bool found = false;
    for (auto& track : active_tracks) {
      if (track->addCfarDetection(cfar_detection, stamp)) {
        found = true;
        break;
      }
    }
    // Create new tracks.
    if (!found) {
      active_tracks.emplace_back(
          new Track(cfar_detection, stamp, max_duration_, id_++));
    }
  }

  tracks_ = active_tracks;
  return active_tracks;
}