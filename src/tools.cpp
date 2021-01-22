#include "tools.h"
#include <iostream>

using Eigen::Vector4d;
using Eigen::VectorXd;
using Eigen::Matrix;
using std::vector;

constexpr double kEpsilon = 1E-5;

VectorXd Tools::CalculateRMSE(const vector<VectorXd>& estimations,
    const vector<VectorXd>& ground_truth)
{
    if (estimations.empty()) {
        return VectorXd::Zero(4);
    }

    // adopt vector size of estimations
    const size_t dim = estimations.front().rows();

    VectorXd rmse = VectorXd::Zero(dim);

    // check the validity of inputs
    if (estimations.size() == 0 || estimations.size() != ground_truth.size()) {
        return rmse;
    }

    // accumulate squared residuals
    VectorXd sq_res = VectorXd::Zero(dim);
    for (size_t i = 0; i<estimations.size(); ++i) {
        sq_res += (estimations[i]-ground_truth[i]).cwiseAbs2();
    }

    // mean
    VectorXd mean = sq_res / static_cast<double>(estimations.size());

    // square-root
    rmse = mean.cwiseSqrt();

    return rmse;
}
