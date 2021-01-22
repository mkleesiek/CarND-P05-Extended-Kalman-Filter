#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

constexpr double kEpsilon = 1E-5;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
	VectorXd rmse(4);
	rmse << 0, 0, 0, 0;

	// check the validity of inputs
	if (estimations.size()==0 || estimations.size()!=ground_truth.size()) {
		return rmse;
	}

	// accumulate squared residuals
	VectorXd sq_res = VectorXd::Zero(4);
	for (size_t i = 0; i<estimations.size(); ++i) {
		sq_res += (estimations[i] - ground_truth[i]).cwiseAbs2();
	}

	// mean
	VectorXd mean = sq_res / static_cast<double>(estimations.size());

	// square-root
	rmse = mean.cwiseSqrt();

	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {

	// recover state parameters
	const double px = x_state(0);
	const double py = x_state(1);
	const double vx = x_state(2);
	const double vy = x_state(3);

	// avoid division by zero
	const double rho2 = px*px + py*py;

	if (fabs(rho2) < kEpsilon) {
		return MatrixXd::Zero(3, 4);
	}

	const double rho = sqrt(rho2);
	const double rho3 = rho + rho2;

	MatrixXd Hj(3, 4);

	// compute the Jacobian matrix
	Hj << px/rho, py/rho, 0, 0,
		  -py/rho2, px/rho2, 0, 0,
		  py*(vx*py-vy*px)/rho3, px*(vy*px-vx*py)/rho3, px/rho, py/rho;

	return Hj;
}
