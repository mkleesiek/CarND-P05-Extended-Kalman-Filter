#include "kalman_filter.h"

using namespace Eigen;

/// used for zero-comparison
constexpr double kEpsilon = 1E-6;

/// mathematical constant pi
constexpr double kPi = 3.14159265358979323846;

namespace
{
    /// wrap an angle (in radians) to the range [-pi, pi]
    double wrapAngle(double angle)
    {
        while (angle > kPi)
            angle -= 2.0 * kPi;
        while (angle < -kPi)
            angle += 2.0 * kPi;
        return angle;
    }
}

void KalmanFilter::Predict()
{
    x_ = F_ * x_;
    const MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const Vector2d& z, const Matrix<double, 2, 4>& H_laser, const Matrix2d& R_laser)
{
    const Vector2d y = z - H_laser * x_;

    UpdateInternal(y, H_laser, R_laser);
}

void KalmanFilter::UpdateEKF(const Vector3d& z, const Eigen::Matrix3d& R_radar)
{
    Vector3d y = z - CalculateH();
    y(1) = wrapAngle(y(1));

    // calculate Jacobian
    const Eigen::Matrix<double, 3, 4> H_radar = CalculateJacobian();

    UpdateInternal(y, H_radar, R_radar);
}

Matrix<double, 3, 4> KalmanFilter::CalculateJacobian() const
{
    // recover state parameters
    const double px = x_(0);
    const double py = x_(1);
    const double vx = x_(2);
    const double vy = x_(3);

    // avoid division by zero
    const double rho2 = px*px+py*py;
    if (fabs(rho2) < kEpsilon) {
        return Matrix<double, 3, 4>::Zero();
    }

    const double rho = sqrt(rho2);
    const double rho3 = rho * rho2;

    Matrix<double, 3, 4> Hj;

    // compute the Jacobian matrix
    Hj << px / rho,  py / rho, 0.0, 0.0,
         -py / rho2, px / rho2, 0.0, 0.0,
          py * (vx*py-vy*px) / rho3, px * (vy*px-vx*py) / rho3, px / rho, py / rho;

    return Hj;
}

Vector3d KalmanFilter::CalculateH() const
{
    const double c = sqrt(x_(0)*x_(0) + x_(1)*x_(1));

    if (fabs(c) < kEpsilon || (fabs(x_(0)) < kEpsilon && fabs(x_(1)) < kEpsilon)) {
        return Vector3d::Zero();
    }

    Vector3d h;
    h << c,
        atan2(x_(1), x_(0)),
        (x_(0)*x_(2) + x_(1)*x_(3)) / c;

    return h;
}

void KalmanFilter::UpdateInternal(const Eigen::VectorXd& y, const Eigen::MatrixXd& H, const Eigen::MatrixXd& R)
{
    const MatrixXd Ht = H.transpose();
    const MatrixXd S = H * P_ * Ht + R;
    const MatrixXd Si = S.inverse();
    const MatrixXd K = P_ * Ht * Si;

    // new estimate
    x_ = x_ + (K * y);

    // new state covariance
    static const Matrix4d I = Matrix4d::Identity();
    P_ = (I - K * H) * P_;
}