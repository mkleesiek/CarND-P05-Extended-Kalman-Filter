#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include "Eigen/Dense"

class KalmanFilter {
public:
	/**
	* Constructor.
	*/
	KalmanFilter() = default;

	/**
	* Destructor.
	*/
	~KalmanFilter() = default;

	/**
	 * Prediction Predicts the state and the state covariance
	 * using the process model
	 * @param delta_T Time between k and k+1 in s
	 */
	void Predict();

	/**
	 * Updates the state by using standard Kalman Filter equations
	 * @param z The measurement at k+1
	 * @param H Measurement matrix
	 * @param R Measurement covariance matrix
	 */
	void Update(const Eigen::Vector2d& z, const Eigen::Matrix<double, 2, 4>& H, const Eigen::Matrix2d& R);

	/**
	 * Updates the state by using Extended Kalman Filter equations.
	 * The Jacobian is calculated internally.
	 *
	 * @param z The measurement at k+1
	 * @param R Measurement covariance matrix
	 */
	void UpdateEKF(const Eigen::Vector3d& z, const Eigen::Matrix3d& R);

	/**
	 * Calculate Jacobian matrix based on the current state vector
	 */
	Eigen::Matrix<double, 3, 4> CalculateJacobian() const;

	/// State vector
	Eigen::Vector4d x_ = Eigen::Vector4d::Zero();
	/// State covariance matrix
	Eigen::Matrix4d P_ = Eigen::Matrix4d::Zero();
	/// State transition matrix
	Eigen::Matrix4d F_ = Eigen::Matrix4d::Zero();
	/// Process covariance matrix
	Eigen::Matrix4d Q_ = Eigen::Matrix4d::Zero();

private:
	// Calculate 'h(x_)', mapping cartesian to polar coordinates.
	Eigen::Vector3d CalculateH() const;

	// Common sections of ::Update and ::UpdateEKF.
	void UpdateInternal(const Eigen::VectorXd& y, const Eigen::MatrixXd& H, const Eigen::MatrixXd& R);
};

#endif // KALMAN_FILTER_H_
