#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

constexpr double kEpsilon = 1E-6;
constexpr double kPi = 3.14159265358979323846;

namespace
{
	double wrapAngle(double angle)
	{
		while (angle > kPi)
        	angle -= 2.0 * kPi;
		while (angle < -kPi)
			angle += 2.0 * kPi;
		return kPi;
	}
}

VectorXd KalmanFilter::CalculateH() const
{
	const double c = sqrt(x_(0)*x_(0) + x_(1)*x_(1));

	if (fabs(c) < kEpsilon || (fabs(x_(0)) < kEpsilon && fabs(x_(1)) < kEpsilon)) {
		return VectorXd::Zero(3);
	}

	VectorXd h;
	h << c,
	     atan2(x_(1), x_(0)),
		 (x_(0)*x_(2) + x_(1)*x_(3)) / c;

	return h;
}

void KalmanFilter::Predict()
{
	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd& z)
{
	assert(z.rows() == 2);

	VectorXd y = z - H_ * x_;

	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_*P_*Ht+R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_*Ht;
	MatrixXd K = PHt*Si;

	// new estimate
	x_ = x_+(K*y);
	size_t x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I-K*H_)*P_;
}

void KalmanFilter::UpdateEKF(const VectorXd& z)
{
	assert(z.rows() == 3);

	VectorXd y = z - CalculateH();
	y(1) = wrapAngle(y(1));

	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_*P_*Ht+R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_*Ht;
	MatrixXd K = PHt*Si;

	// new estimate
	x_ = x_+(K*y);
	size_t x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I-K*H_)*P_;
}
