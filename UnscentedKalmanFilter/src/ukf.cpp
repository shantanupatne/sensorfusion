#include "ukf.h"
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() 
{
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  n_x_ = 5;
  n_aug_ = 7;

  // initial state vector
  x_ = VectorXd(n_x_);

  // initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_);

  // initial sigma prediction
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 4;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 15;
  
  /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  
  /**
   * End DO NOT MODIFY section for measurement noise values 
   */
  
  // create the weights vector
  lambda_ = 3 - n_aug_;
  weights_.setConstant(2 * n_aug_ + 1, 1/(2 * (lambda_ + n_aug_)));
  weights_(0) = lambda_ / (lambda_ + n_aug_);

  // initialization
  is_initialized_ = false;
  time_us_ = 0.;

  // NIS init
  nis_lidar = 0.;
  nis_radar = 0.;
}

UKF::~UKF() {}

void UKF::Init(MeasurementPackage meas_package) 
{
  double x, y, yaw_d;
  if(meas_package.sensor_type_ == MeasurementPackage::LASER) 
  {
    x = meas_package.raw_measurements_(0);
    y = meas_package.raw_measurements_(1);
    yaw_d = 0;
  } 
  else if (meas_package.sensor_type_ == MeasurementPackage::RADAR) 
  {
    double rho = meas_package.raw_measurements_(0);
    double phi = meas_package.raw_measurements_(1);
    double rho_dot = meas_package.raw_measurements_(2);

    x = rho * cos(phi);
    y = rho * sin(phi);
    yaw_d = rho_dot;

  } else return;
  // px, py, v, yaw, yaw_dot
  x_ << x, y, 0, 0, 0;
  P_.setIdentity();
  
  time_us_ = meas_package.timestamp_;
  is_initialized_ = true;
}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) 
{
  // initiliaze if first measurement
  if ( !is_initialized_ ) {
    Init(meas_package);
    return;
  }

  // compute delta time
  float delta_t = (meas_package.timestamp_ - time_us_) / 1000000.;
  time_us_ = meas_package.timestamp_;
  
  // predict k+1
  Prediction(delta_t);

  // update with measurement k+1
  if (use_radar_ && meas_package.sensor_type_ == MeasurementPackage::RADAR) 
  {
    UpdateRadar(meas_package);
  }
  else if (use_laser_ && meas_package.sensor_type_ == MeasurementPackage::LASER)
  {
    UpdateLidar(meas_package);
  }

}

void UKF::AugmentedSigmaPoints(MatrixXd* Xsig_out) 
{
  // Create Augmented X
  VectorXd x_aug_ = VectorXd::Zero(n_aug_);
  x_aug_.head(n_x_) = x_;

  // Process Noise
  MatrixXd Q = MatrixXd(2, 2);
  Q << std::pow(std_a_, 2), 0, 0, std::pow(std_yawdd_, 2);

  // Augmented Covarianec
  MatrixXd P_aug_ = MatrixXd::Zero(n_aug_, n_aug_);
  P_aug_.topLeftCorner(n_x_, n_x_) = P_;
  P_aug_.bottomRightCorner(2, 2) = Q;

  // Cholesky Decomp for sqrt(P)
  MatrixXd A = P_aug_.llt().matrixL();
  A = sqrt(lambda_ + n_aug_) * A;

  // Generate sigma points
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  Xsig_aug.col(0) = x_aug_;
  for ( int i = 0; i < n_aug_; i++ ) 
  {
    Xsig_aug.col(i + 1)           = x_aug_ + A.col(i);
    Xsig_aug.col(i + n_aug_ + 1)  = x_aug_ - A.col(i);
  } 

  *Xsig_out = Xsig_aug;
}

void UKF::Prediction(double delta_t) 
{
  // generate sigma points
  MatrixXd X_aug_ = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  AugmentedSigmaPoints(&X_aug_);

  // predict sigma points
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
      VectorXd x_k = X_aug_.col(i).head(n_x_);
      double mu_ak = X_aug_(n_x_, i), mu_yawk = X_aug_(n_x_ + 1, i);
      
      double p_x = x_k(0), p_y = x_k(1), v = x_k(2), yaw = x_k(3), yaw_d = x_k(4);
      
      VectorXd x_k_dot = VectorXd::Zero(n_x_);
      // avoid division by zero
      if (yaw_d < 0.001) {
          x_k_dot(0) = v * cos(yaw) * delta_t;
          x_k_dot(1) = v * sin(yaw) * delta_t;
      } else {
          x_k_dot(0) = (v / yaw_d) * (sin(yaw + (yaw_d * delta_t)) - sin(yaw));
          x_k_dot(1) = (v / yaw_d) * (cos(yaw) - cos(yaw + (yaw_d * delta_t)));
          x_k_dot(3) = yaw_d * delta_t;
      } 
      
      // Compute process noise vector
      VectorXd mu_k = VectorXd::Zero(n_x_);
      double dt2 = delta_t * delta_t;
      mu_k(0) = cos(yaw) * mu_ak * dt2 / 2;
      mu_k(1) = sin(yaw) * mu_ak * dt2 / 2;
      mu_k(2) = mu_ak * delta_t;
      mu_k(3) = mu_yawk * dt2 / 2;
      mu_k(4) = mu_yawk * delta_t;
      
      // write predicted sigma points into right column
      VectorXd x_k_new = x_k + x_k_dot + mu_k;
      Xsig_pred_.col(i) = x_k_new;
  }

  // predict state mean
  x_ = Xsig_pred_ * weights_;

  // predict state covariance matrix
  MatrixXd Xc = Xsig_pred_.colwise() - x_;
  normalizeAngles(Xc, 3);

  P_ = Xc * weights_.asDiagonal() * Xc.transpose();

}

void UKF::UpdateLidar(MeasurementPackage meas_package) 
{
  int n_z = 2;

  // create matrix for sigma points in measurement space
  MatrixXd Zsig = Xsig_pred_.block(0, 0, n_z, 2 * n_aug_ + 1);

  // Generate measurement noise
  MatrixXd R(n_z, n_z);
  R << (std_laspx_ * std_laspx_), 0,
        0, (std_laspy_ * std_laspy_);
  
  // Update the state
  UpdateState(meas_package, Zsig, R);
}

void UKF::UpdateRadar(MeasurementPackage meas_package) 
{
  int n_z = 3;

  // create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
  for (int i = 0; i < 2*n_aug_ +1; i++) {
    double px = Xsig_pred_(0, i), py = Xsig_pred_(1, i), v = Xsig_pred_(2, i), yaw = Xsig_pred_(3, i);
    double rho = sqrt(pow(px, 2) + pow(py, 2));
    double phi = atan2(py, px);
    double rho_dot = ((px * cos(yaw) * v) + (py * sin(yaw) * v)) / rho;
    
    Zsig(0, i) = rho;
    Zsig(1, i) = phi;
    Zsig(2, i) = rho_dot;
  }

  // Generate the measurement noise
  MatrixXd R(n_z, n_z);
  R << (std_radr_ * std_radr_), 0, 0,
        0, (std_radphi_ * std_radphi_), 0,
        0, 0, (std_radrd_ * std_radrd_);
    
  // Update state
  UpdateState(meas_package, Zsig, R);

}

void UKF::UpdateState(MeasurementPackage meas_package, Eigen::MatrixXd Zsig, Eigen::MatrixXd R) 
{
  // mean predicted measurement
  VectorXd z_pred = Zsig * weights_;
  
  // measurement covariance matrix 
  MatrixXd Zc = Zsig.colwise() - z_pred;
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    normalizeAngles(Zc, 1);
  }
  
  // Innovation Matrix
  MatrixXd S = Zc * weights_.asDiagonal() * Zc.transpose() + R;

  // calculate cross correlation matrix
  MatrixXd Xc = Xsig_pred_.colwise() - x_;
  normalizeAngles(Xc, 3);

  MatrixXd Tc = Xc * weights_.asDiagonal() * Zc.transpose();

  // calculate Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  // update state mean and covariance matrix
  VectorXd z = meas_package.raw_measurements_;
  
  VectorXd z_diff = (z - z_pred);
  while (z_diff[1] > M_PI) z_diff[1] -= 2 * M_PI;
  while (z_diff[1] < -M_PI) z_diff[1] += 2 * M_PI;

  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();

  // Update NIS
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    nis_radar = z_diff.transpose() * S.inverse() * z_diff;
  } else {
    nis_lidar = z_diff.transpose() * S.inverse() * z_diff;
  }
}

void UKF::normalizeAngles(MatrixXd& matrix, int rowIndex) 
{
  VectorXd rowToNormalize = matrix.row(rowIndex);

  for (int i = 0; i < rowToNormalize.size(); i++) {
  
    double angle = rowToNormalize(i);

    while (angle > M_PI) angle -= 2*M_PI;
    while (angle < -M_PI) angle += 2*M_PI;
    
    rowToNormalize(i) = angle;
  }

  matrix.row(rowIndex) = rowToNormalize;
}