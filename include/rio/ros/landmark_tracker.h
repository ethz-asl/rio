#pragma once

#include <memory>
#include <vector>

#include <mav_sensors_drivers/sensor_types/Radar.h>
#include <ros/time.h>
#include <sensor_msgs/PointCloud2.h>

namespace rio {

class Track {
 public:
  typedef std::shared_ptr<Track> Ptr;

  Track(const mav_sensors::Radar::CfarDetection& cfar_detection,
        const ros::Time& stamp, const double max_duration, const uint64_t id)
      : cfar_detection_(cfar_detection),
        last_stamp_(stamp),
        max_duration_(max_duration),
        id_(id) {}

  // returns true if the detection was added.
  bool addCfarDetection(const mav_sensors::Radar::CfarDetection& cfar_detection,
                        const ros::Time& stamp);
  // returns true if the track is still valid.
  bool isValid(const ros::Time& stamp) const;

 private:
  mav_sensors::Radar::CfarDetection cfar_detection_;
  ros::Time last_stamp_;
  double max_duration_{0.0};
  uint64_t id_{0};
};

class Tracker {
 public:
  Tracker(const double max_duration);
  // Add CFAR detections and return the active tracks at the given time.
  std::vector<Track::Ptr> addCfarDetections(
      const std::vector<mav_sensors::Radar::CfarDetection>& cfar_detection,
      const ros::Time& stamp);

 private:
  bool detectLandmark(
      const mav_sensors::Radar::CfarDetection& cfar_detection) const;
  double max_duration_{0.0};
  uint64_t id_{0};

  std::vector<Track::Ptr> tracks_;
};

}  // namespace rio