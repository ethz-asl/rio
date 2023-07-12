#pragma once

#include <Eigen/Dense>
#include <mav_sensors_drivers/sensor_types/Radar.h>

namespace rio
{

    /**
     * @brief Least squares fit to estimate linear velocity.
     *
     * @param Radar measurement containing CFAR detections.
     * @param velocity pointer to velocity estimate.
     * @return true if velocity estimate successful.
     */
    bool least_squares(const Radar &measurement, Eigen::Vector3d* velocity);

} // namespace rio
