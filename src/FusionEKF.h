#ifndef FusionEKF_H_
#define FusionEKF_H_

#include <fstream>
#include <string>
#include <vector>
#include "Eigen/Dense"
#include "kalman_filter.h"
#include "measurement_package.h"
#include "tools.h"

class FusionEKF {
 public:
  /**
   * Constructor.
   */
  FusionEKF();

  /**
   * Destructor.
   */
  ~FusionEKF() = default;

  /**
   * Run the whole flow of the Kalman Filter from here.
   */
  void ProcessMeasurement(const MeasurementPackage &measurement_pack);

  /**
   * Kalman Filter update and prediction math lives in here.
   */
  KalmanFilter ekf_;

 private:
  // check whether the tracking toolbox was initialized or not (first measurement)
  bool is_initialized_ = false;

  // previous timestamp
  uint64_t previous_timestamp_ = 0;

  // tool object used to compute Jacobian and RMSE
  Tools tools;

  // matrices
  Eigen::Matrix2d R_laser_ = Eigen::Matrix2d::Zero();
  Eigen::Matrix3d R_radar_ = Eigen::Matrix3d::Zero();
  Eigen::Matrix<double, 2, 4> H_laser_ = Eigen::Matrix<double, 2, 4>::Zero();

  // noise
  double noise_ax_ = 9.0;
  double noise_ay_ = 9.0;
};

#endif // FusionEKF_H_
