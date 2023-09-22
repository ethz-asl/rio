#pragma once

#include <memory>
#include <vector>

#include <gtsam/geometry/Point3.h>
#include <mav_sensors_drivers/sensor_types/Radar.h>

namespace rio {

class Track {
 public:
  typedef std::shared_ptr<Track> Ptr;

  Track(const mav_sensors::Radar::CfarDetection& cfar_detection,
        const uint64_t id, const uint64_t max_age = 1)
      : cfar_detection_(cfar_detection), id_(id), max_age_(max_age) {}

  // returns true if the detection was added.
  bool addCfarDetection(
      const mav_sensors::Radar::CfarDetection& cfar_detection);
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
  mav_sensors::Radar::CfarDetection cfar_detection_;
  ros::Time last_stamp_;
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
      const std::vector<mav_sensors::Radar::CfarDetection>& cfar_detection);

 private:
  bool detectLandmark(
      const mav_sensors::Radar::CfarDetection& cfar_detection) const;
  uint64_t id_{0};
  uint64_t max_age_{1};

  std::vector<Track::Ptr> tracks_;
};

}  // namespace rio