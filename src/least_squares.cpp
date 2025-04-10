#include "rio/least_squares.h"

#include <Eigen/SVD>

bool rio::leastSquares(const mav_sensors::Radar &measurement,
                       Eigen::Vector3d *velocity) {
  if (measurement.cfar_detections.size() < 3) return false;
  // Get detection direction vector r and doppler velocity vd
  Eigen::MatrixXd r(measurement.cfar_detections.size(), 3);
  Eigen::VectorXd vd(measurement.cfar_detections.size());
  for (size_t i = 0; i < measurement.cfar_detections.size(); i++) {
    r.row(i) << measurement.cfar_detections[i].x,
        measurement.cfar_detections[i].y, measurement.cfar_detections[i].z;
    r.rowwise().normalize();
    vd(i) = measurement.cfar_detections[i].velocity;
  }
  // Compute least squares velocity estimate Ax = b, where A = r, b = vd
  auto jacobiSvd = r.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
  if (jacobiSvd.rank() < 3)
    return false;
  else {
    *velocity = jacobiSvd.solve(vd);
    return true;
  }
}