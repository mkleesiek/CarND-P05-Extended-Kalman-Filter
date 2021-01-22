#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Matrix4d;
using Eigen::Vector4d;
using std::cout;
using std::endl;
using std::vector;

FusionEKF::FusionEKF() :
    R_laser_(),
    R_radar_(),
    H_laser_()
{
    // measurement covariance matrix - laser
    R_laser_ << 0.0225, 0.0,
        0, 0.0225;

    // measurement covariance matrix - radar
    R_radar_ << 0.09, 0.0, 0.0,
        0.0, 0.0009, 0.0,
        0.0, 0.0, 0.09;

    // measurement matrix - laser
    H_laser_ << 1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0;
}

void FusionEKF::ProcessMeasurement(const MeasurementPackage& measurement_pack)
{
    /**
     * Initialization
     */
    if (!is_initialized_) {
        cout << "EKF: " << endl;

        // initialize state 'x' with the first measurement
        ekf_.x_ = Vector4d::Zero();

        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
            // convert from polar to cartesian coordinates
            const double rho = measurement_pack.raw_measurements_[0];
            const double phi = measurement_pack.raw_measurements_[1];
            const double rho_dot = measurement_pack.raw_measurements_[2];

            ekf_.x_(0) = rho*cos(phi);
            ekf_.x_(1) = rho*sin(phi);
            ekf_.x_(2) = rho_dot*cos(phi);
            ekf_.x_(3) = rho_dot*sin(phi);
        }
        else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
            ekf_.x_(0) = measurement_pack.raw_measurements_[0];
            ekf_.x_(1) = measurement_pack.raw_measurements_[1];
        }

        // state covariance matrix P
        ekf_.P_ = Matrix4d();
        ekf_.P_ << 1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1000.0, 0.0,
            0.0, 0.0, 0.0, 1000.0;

        // the initial transition matrix F
        ekf_.F_ = Matrix4d();
        ekf_.F_ << 1.0, 0.0, 1.0, 0.0,
            0.0, 1.0, 0.0, 1.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0;

        // first timestamp
        previous_timestamp_ = measurement_pack.timestamp_;

        // done initializing, no need to predict or update
        is_initialized_ = true;

        return;
    }

    // compute the time elapsed between the current and previous measurements
    // dt - expressed in seconds
    const double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1E6;
    previous_timestamp_ = measurement_pack.timestamp_;

    // modify the F matrix so that the time is integrated
    ekf_.F_(0, 2) = ekf_.F_(1, 3) = dt;

    // set the process covariance matrix Q
    const double dt2 = dt  * dt;
    const double dt3 = dt2 * dt;
    const double dt4 = dt2 * dt2;

    ekf_.Q_ = Matrix4d();
    ekf_.Q_ << dt4 / 4.0 * noise_ax_, 0.0, dt3 / 2.0 * noise_ax_, 0.0,
        0.0, dt4 / 4.0 * noise_ay_, 0, dt3 / 2.0 * noise_ay_,
        dt3 / 2.0 * noise_ax_, 0.0, dt2 * noise_ax_, 0.0,
        0.0, dt3 / 2.0 * noise_ay_, 0.0, dt2 * noise_ay_;

    /**
     * Prediction
     */

    ekf_.Predict();

    /**
     * Update
     */

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        // Radar updates
        // (Jacobian is calculated internally)
        ekf_.UpdateEKF(measurement_pack.raw_measurements_, R_radar_);
    }
    else {
        // Laser updates
        ekf_.Update(measurement_pack.raw_measurements_, H_laser_, R_laser_);
    }

    // print the output
    cout << "x_ = " << ekf_.x_ << endl;
    cout << "P_ = " << ekf_.P_ << endl;
}
