#include "rio/common.h"

#include <sensor_msgs/point_cloud2_iterator.h>

using namespace rio;
using namespace gtsam;

bool rio::loadPreintegratedCombinedMeasurements(
    const ros::NodeHandle& nh, PreintegratedCombinedMeasurements* imu) {
  assert(imu);
  double bias_acc_sigma = 0.0, bias_omega_sigma = 0.0, bias_acc_int_sigma = 0.0,
         bias_omega_int_sigma = 0.0, acc_sigma = 0.0, integration_sigma = 0.0,
         gyro_sigma = 0.0;
  // TODO: Possibly expose "use2ndOrderCoriolis", "omegaCoriolis", "n_gravity"
  // or "body_P_sensor" as parameters. But only really makes sense if we have a
  // earth-centered coordinate frame or not a IMU centered frame.
  if (!loadParam<double>(nh, "imu/bias_acc_sigma", &bias_acc_sigma))
    return false;
  if (!loadParam<double>(nh, "imu/bias_omega_sigma", &bias_omega_sigma))
    return false;
  if (!loadParam<double>(nh, "imu/bias_acc_int_sigma", &bias_acc_int_sigma))
    return false;
  if (!loadParam<double>(nh, "imu/bias_omega_int_sigma", &bias_omega_int_sigma))
    return false;
  if (!loadParam<double>(nh, "imu/acc_sigma", &acc_sigma)) return false;
  if (!loadParam<double>(nh, "imu/integration_sigma", &integration_sigma))
    return false;
  if (!loadParam<double>(nh, "imu/gyro_sigma", &gyro_sigma)) return false;

  auto imu_params = PreintegratedCombinedMeasurements::Params::MakeSharedU();
  imu_params->biasAccCovariance = I_3x3 * std::pow(bias_acc_sigma, 2);
  imu_params->biasOmegaCovariance = I_3x3 * std::pow(bias_omega_sigma, 2);
  imu_params->biasAccOmegaInt.block<3, 3>(0, 0) =
      I_3x3 * std::pow(bias_acc_int_sigma, 2);
  imu_params->biasAccOmegaInt.block<3, 3>(3, 3) =
      I_3x3 * std::pow(bias_omega_int_sigma, 2);

  imu_params->accelerometerCovariance = I_3x3 * std::pow(acc_sigma, 2);
  imu_params->integrationCovariance = I_3x3 * std::pow(integration_sigma, 2);
  imu_params->gyroscopeCovariance = I_3x3 * std::pow(gyro_sigma, 2);

  Vector3 b_a, b_g;
  if (!loadParam<Vector3>(nh, "imu/initial_bias_acc", &b_a)) return false;
  if (!loadParam<Vector3>(nh, "imu/initial_bias_gyro", &b_g)) return false;
  imu_params->print("IMU parameters:");
  *imu = PreintegratedCombinedMeasurements(imu_params, {b_a, b_g});
  imu->print("Initial preintegration parameters:");

  return true;
}

bool rio::loadPriorNoisePose(const ros::NodeHandle& nh,
                             SharedNoiseModel* noise) {
  assert(noise);
  Vector3 prior_noise_R_IB, prior_noise_I_p_IB;
  if (!loadParam<Vector3>(nh, "prior_noise/R_IB", &prior_noise_R_IB))
    return false;
  if (!loadParam<Vector3>(nh, "prior_noise/I_p_IB", &prior_noise_I_p_IB))
    return false;
  *noise = noiseModel::Diagonal::Sigmas(
      (Vector6() << prior_noise_R_IB, prior_noise_I_p_IB).finished());
  std::dynamic_pointer_cast<noiseModel::Diagonal>(*noise)->print(
      "prior_noise_model_I_T_IB: ");
  return true;
}

bool rio::loadPriorNoiseVelocity(const ros::NodeHandle& nh,
                                 SharedNoiseModel* noise) {
  assert(noise);
  Vector3 prior_noise_I_v_IB;
  if (!loadParam<Vector3>(nh, "prior_noise/I_v_IB", &prior_noise_I_v_IB))
    return false;
  *noise = noiseModel::Diagonal::Sigmas(prior_noise_I_v_IB);
  std::dynamic_pointer_cast<noiseModel::Diagonal>(*noise)->print(
      "prior_noise_model_I_v_IB: ");
  return true;
}

bool rio::loadPriorNoiseImuBias(const ros::NodeHandle& nh,
                                SharedNoiseModel* noise) {
  assert(noise);
  Vector3 prior_noise_bias_acc, prior_noise_bias_gyro;
  if (!loadParam<Vector3>(nh, "prior_noise/b_a", &prior_noise_bias_acc))
    return false;
  if (!loadParam<Vector3>(nh, "prior_noise/b_g", &prior_noise_bias_gyro))
    return false;
  *noise = noiseModel::Diagonal::Sigmas(
      (Vector6() << prior_noise_bias_acc, prior_noise_bias_gyro).finished());
  std::dynamic_pointer_cast<noiseModel::Diagonal>(*noise)->print(
      "prior_noise_model_imu_bias: ");
  return true;
}

bool rio::loadNoiseRadarRadialVelocity(const ros::NodeHandle& nh,
                                       SharedNoiseModel* noise) {
  assert(noise);
  double noise_radar_doppler = 0.0;
  if (!loadParam<double>(nh, "noise/radar/doppler", &noise_radar_doppler))
    return false;
  auto radar_gaussian_noise = noiseModel::Diagonal::Sigmas(
      (Vector1() << noise_radar_doppler).finished());
  radar_gaussian_noise->print("radar_gaussian_noise: ");
  int radar_doppler_loss = 0;
  if (!loadParam<int>(nh, "noise/radar/loss", &radar_doppler_loss))
    return false;
  // Select robust loss function. Spread is chosen by c times the standard
  // deviation. c according to Zhang 2012, Parameter Estimation Techniques: A
  // Tutorial with Application to Conic Fitting
  switch (radar_doppler_loss) {
    case 0: {
      *noise = radar_gaussian_noise;
      std::dynamic_pointer_cast<noiseModel::Diagonal>(*noise)->print(
          "Noise model radar radial velocity: ");
      break;
    }
    case 1: {
      const double c = 1.3998;
      *noise = noiseModel::Robust::Create(
          noiseModel::mEstimator::Fair::Create(c), radar_gaussian_noise);
      std::dynamic_pointer_cast<noiseModel::Robust>(*noise)->print(
          "Noise model radar radial velocity: ");
      break;
    }
    case 2: {
      const double k = 1.345;
      *noise = noiseModel::Robust::Create(
          noiseModel::mEstimator::Huber::Create(k), radar_gaussian_noise);
      std::dynamic_pointer_cast<noiseModel::Robust>(*noise)->print(
          "Noise model radar radial velocity: ");
      break;
    }
    case 3: {
      const double c = 2.3849;
      *noise = noiseModel::Robust::Create(
          noiseModel::mEstimator::Cauchy::Create(c), radar_gaussian_noise);
      std::dynamic_pointer_cast<noiseModel::Robust>(*noise)->print(
          "Noise model radar radial velocity: ");
      break;
    }
    case 4: {
      const double c = 1.0;
      *noise = noiseModel::Robust::Create(
          noiseModel::mEstimator::GemanMcClure::Create(c),
          radar_gaussian_noise);
      std::dynamic_pointer_cast<noiseModel::Robust>(*noise)->print(
          "Noise model radar radial velocity: ");
      break;
    }
    case 5: {
      const double c = 2.9846;
      *noise = noiseModel::Robust::Create(
          noiseModel::mEstimator::Welsch::Create(c), radar_gaussian_noise);
      std::dynamic_pointer_cast<noiseModel::Robust>(*noise)->print(
          "Noise model radar radial velocity: ");
      break;
    }
    case 6: {
      const double c = 4.6851;
      *noise = noiseModel::Robust::Create(
          noiseModel::mEstimator::Tukey::Create(c), radar_gaussian_noise);
      std::dynamic_pointer_cast<noiseModel::Robust>(*noise)->print(
          "Noise model radar radial velocity: ");
      break;
    }
    default: {
      LOG(F, "Unknown radar doppler loss function: " << radar_doppler_loss);
      return false;
    }
  }
  return true;
}

bool rio::loadNoiseRadarTrack(const ros::NodeHandle& nh,
                              SharedNoiseModel* noise) {
  assert(noise);
  Vector3 noise_radar_track;
  if (!loadParam<Vector3>(nh, "noise/radar/track", &noise_radar_track))
    return false;
  *noise = gtsam::noiseModel::Diagonal::Sigmas(noise_radar_track);
  std::dynamic_pointer_cast<noiseModel::Diagonal>(*noise)->print(
      "Noise model radar track: ");
  return true;
}

std::vector<CfarDetection> rio::parseRadarMsg(
    const sensor_msgs::PointCloud2Ptr& msg) {
  std::vector<CfarDetection> detections(msg->height *
                                                            msg->width);
  sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*msg, "z");
  sensor_msgs::PointCloud2Iterator<float> iter_doppler(*msg, "doppler");
  sensor_msgs::PointCloud2Iterator<int16_t> iter_snr(*msg, "snr");
  sensor_msgs::PointCloud2Iterator<int16_t> iter_noise(*msg, "noise");
  for (auto& detection : detections) {
    detection.x = *(iter_x);
    detection.y = *(iter_y);
    detection.z = *(iter_z);
    detection.velocity = *(iter_doppler);
    detection.snr = *(iter_snr);
    detection.noise = *(iter_noise);
    ++iter_x;
    ++iter_y;
    ++iter_z;
    ++iter_doppler;
    ++iter_snr;
    ++iter_noise;
  }
  return detections;
}

double rio::computeBaroHeight(double pressure) {
  // https://www.grc.nasa.gov/www/k-12/airplane/atmosmet.html
  return (288.08 * std::pow(pressure / 101290.0, 1.0 / 5.256) - 273.1 - 15.04) /
         (-0.00649);
}

bool rio::loadNoiseLoopClosureT(const ros::NodeHandle& nh,
                                gtsam::SharedNoiseModel* noise) {
  assert(noise);
  Vector3 noise_loop_closure_p;
  if (!loadParam<Vector3>(nh, "noise/loop_closure/p", &noise_loop_closure_p))
    return false;
  Vector3 noise_loop_closure_R;
  if (!loadParam<Vector3>(nh, "noise/loop_closure/R", &noise_loop_closure_R))
    return false;
  *noise = noiseModel::Diagonal::Sigmas(
      (Vector6() << noise_loop_closure_R, noise_loop_closure_p).finished());
  std::dynamic_pointer_cast<noiseModel::Diagonal>(*noise)->print(
      "Noise model loop closure T: ");
  return true;
}

bool rio::loadNoiseZeroVelocityPrior(const ros::NodeHandle& nh,
                                     gtsam::SharedNoiseModel* noise) {
  assert(noise);
  double noise_zero_velocity_prior;
  if (!loadParam<double>(nh, "noise/zero_velocity_prior",
                         &noise_zero_velocity_prior))
    return false;
  *noise = noiseModel::Isotropic::Sigma(3, noise_zero_velocity_prior);
  std::dynamic_pointer_cast<noiseModel::Isotropic>(*noise)->print(
      "Noise model zero velocity prior: ");
  return true;
}

bool rio::loadNoiseBaroHeight(const ros::NodeHandle& nh,
                              gtsam::SharedNoiseModel* noise) {
  assert(noise);
  double noise_baro_height = 0.0;
  if (!loadParam<double>(nh, "noise/baro/height", &noise_baro_height))
    return false;
  auto baro_gaussian_noise =
      noiseModel::Diagonal::Sigmas((Vector1() << noise_baro_height).finished());
  baro_gaussian_noise->print("baro_height_gaussian_noise: ");
  int baro_height_loss = 0;
  if (!loadParam<int>(nh, "noise/baro/loss", &baro_height_loss)) return false;
  // Select robust loss function. Spread is chosen by c times the standard
  // deviation. c according to Zhang 2012, Parameter Estimation Techniques: A
  // Tutorial with Application to Conic Fitting
  switch (baro_height_loss) {
    case 0: {
      *noise = baro_gaussian_noise;
      std::dynamic_pointer_cast<noiseModel::Diagonal>(*noise)->print(
          "Noise model baro height: ");
      break;
    }
    case 1: {
      const double c = 1.3998;
      *noise = noiseModel::Robust::Create(
          noiseModel::mEstimator::Fair::Create(c), baro_gaussian_noise);
      std::dynamic_pointer_cast<noiseModel::Robust>(*noise)->print(
          "Noise model baro height: ");
      break;
    }
    case 2: {
      const double k = 1.345;
      *noise = noiseModel::Robust::Create(
          noiseModel::mEstimator::Huber::Create(k), baro_gaussian_noise);
      std::dynamic_pointer_cast<noiseModel::Robust>(*noise)->print(
          "Noise model baro height: ");
      break;
    }
    case 3: {
      const double c = 2.3849;
      *noise = noiseModel::Robust::Create(
          noiseModel::mEstimator::Cauchy::Create(c), baro_gaussian_noise);
      std::dynamic_pointer_cast<noiseModel::Robust>(*noise)->print(
          "Noise model baro height: ");
      break;
    }
    case 4: {
      const double c = 1.0;
      *noise = noiseModel::Robust::Create(
          noiseModel::mEstimator::GemanMcClure::Create(c), baro_gaussian_noise);
      std::dynamic_pointer_cast<noiseModel::Robust>(*noise)->print(
          "Noise model baro height: ");
      break;
    }
    case 5: {
      const double c = 2.9846;
      *noise = noiseModel::Robust::Create(
          noiseModel::mEstimator::Welsch::Create(c), baro_gaussian_noise);
      std::dynamic_pointer_cast<noiseModel::Robust>(*noise)->print(
          "Noise model baro height: ");
      break;
    }
    case 6: {
      const double c = 4.6851;
      *noise = noiseModel::Robust::Create(
          noiseModel::mEstimator::Tukey::Create(c), baro_gaussian_noise);
      std::dynamic_pointer_cast<noiseModel::Robust>(*noise)->print(
          "Noise model baro height: ");
      break;
    }
    default: {
      LOG(F, "Unknown baro height loss function: " << baro_gaussian_noise);
      return false;
    }
  }
  return true;
}