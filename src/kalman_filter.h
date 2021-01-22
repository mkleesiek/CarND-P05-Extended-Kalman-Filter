#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include "Eigen/Dense"

class KalmanFilter {
public:
	/**
	 * Constructor
	 */
	KalmanFilter() = default;

	/**
	 * Destructor
	 */
	virtual ~KalmanFilter() = default;

	/**
	 * Prediction Predicts the state and the state covariance
	 * using the process model
	 * @param delta_T Time between k and k+1 in s
	 */
	void Predict();

	/**
	 * Updates the state by using standard Kalman Filter equations
	 * @param z The measurement at k+1
	 */
	void Update(const Eigen::VectorXd& z);

	/**
	 * Updates the state by using Extended Kalman Filter equations
	 * @param z The measurement at k+1
	 */
	void UpdateEKF(const Eigen::VectorXd& z);

	/**
	 * Calculate 'h(x_)', mapping cartesian to polar coordinates.
	 * @return
	 */
	Eigen::VectorXd CalculateH() const;

	// state vector
	Eigen::VectorXd x_ = Eigen::VectorXd::Zero(4);

	// state covariance matrix
	Eigen::MatrixXd P_ = Eigen::VectorXd::Zero(4, 4);

	// state transition matrix
	Eigen::MatrixXd F_ = Eigen::VectorXd::Zero(4, 4);

	// process covariance matrix
	Eigen::MatrixXd Q_ = Eigen::VectorXd::Zero(4, 4);

	// measurement matrix
	Eigen::MatrixXd H_ = Eigen::VectorXd::Zero(4, 4);

	// measurement covariance matrix
	Eigen::MatrixXd R_ = Eigen::VectorXd::Zero(4, 4);
};

#endif // KALMAN_FILTER_H_
