#include "rio/ros/common.h"

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