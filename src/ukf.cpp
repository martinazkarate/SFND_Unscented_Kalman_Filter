#include "ukf.h"
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // state dimension
  n_x_ = 5;

  // augmented state dimension
  n_aug_ = 7;

  // initialise lambda to recommended value
  lambda_ = 3 - n_aug_;

  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(n_x_);

  // initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_);

  // initial predicted Sigma points matrix
  Xsig_pred_ = MatrixXd (n_aug_, 2*n_aug_+1);

  // Weights of sigma points
  weights_ = VectorXd(n_aug_);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 5; //30 -> This value seems to high, try with 10 or 5

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 1; //30 -> This value seems to high, try with 1 or less.
  
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

  // update this values in the first call to process measurement.
  is_initialized_ = false;
  time_us_ = 0;

}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  if (!is_initialized_) // First time the method is called
  {
    if (meas_package.sensor_type_ == MeasurementPackage::SensorType::LASER)
    {
      x_.fill(0.0);
      x_.head(2) = meas_package.raw_measurements_;

      P_.fill(0.0);
      P_(0,0) = std_laspx_;
      P_(1,1) = std_laspy_;
      P_(2,2) = 20;
      P_(3,3) = 5;
      P_(4,4) = 1;
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::SensorType::RADAR)
    {
      x_.fill(0.0);
      double rho = meas_package.raw_measurements_(0);
      double phi = meas_package.raw_measurements_(1);
      double rho_dot = meas_package.raw_measurements_(2);
      x_(0) = rho*cos(phi);
      x_(1) = rho*sin(phi);
      x_(2) = rho_dot;
      
      P_.fill(0.0);
      P_(0,0) = std_radr_;
      P_(1,1) = std_radr_;
      P_(2,2) = 2*std_radrd_;
      P_(3,3) = 5;
      P_(4,4) = 1;
    }

    is_initialized_ = true;
    time_us_ = meas_package.timestamp_;
  }

  else // UKF already initialized. 
  {
    Prediction(meas_package.timestamp_-time_us_);
    if (meas_package.sensor_type_ == MeasurementPackage::SensorType::LASER)
    {
      UpdateLidar(meas_package);
    }
    else if(meas_package.sensor_type_ == MeasurementPackage::SensorType::RADAR)
    {
      UpdateRadar(meas_package);
    }
    time_us_ = meas_package.timestamp_;
  }
}

void UKF::Prediction(double delta_t) {
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */
}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */
}