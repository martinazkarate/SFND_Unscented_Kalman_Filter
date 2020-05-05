#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Output stream to store NIS values
//std::ofstream nis_las_;
//std::ofstream nis_rad_;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // state dimension
  n_x_ = 5;

  // augmented state dimension
  n_aug_ = 7;

  // measurement dimension
  n_z_las_ = 2;
  n_z_rad_ = 3;

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
  Xsig_pred_ = MatrixXd (n_x_, 2*n_aug_+1);

  // Weights of sigma points
  weights_ = VectorXd(2*n_aug_+1);
  // set weights
  weights_(0) = lambda_/(lambda_+n_aug_);
  for (int i=1; i<2*n_aug_+1; ++i) {  // 2n+1 weights
    weights_(i) =  0.5/(n_aug_+lambda_);
  }
  // std::cout << "Weights:" << std::endl << weights_ << std::endl;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 3; //30 -> This value seems to high, try with 10 or 5. In the course assigment it was 0.2

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 1; //30 -> This value seems to high, try with 1 or less. In the course assigment it was 0.2
  
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
  std_radphi_ = 0.03;  // in the course assigment value was 0.0175

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;  // in the course assigment value was 0.1
  
  /**
   * End DO NOT MODIFY section for measurement noise values 
   */

  // update this values in the first call to process measurement.
  is_initialized_ = false;
  time_us_ = 0;
/*
  std::string filename = "../nis_las.csv";
  nis_las_(filename, std::ios::binary | std::ios::app);

  if (!nis_las_.is_open()) {
    std::cerr << "failed to open file: " << filename << std::endl;
    return EXIT_FAILURE;
  }
  nis_las_ << "NIS LIDAR" << std::endl;

  filename = "../nis_rad.csv";
  nis_rad_(filename, std::ios::binary | std::ios::app);

  if (!nis_rad_.is_open()) {
    std::cerr << "failed to open file: " << filename << std::endl;
    return EXIT_FAILURE;
  }
  nis_rad_ << "NIS RADAR" << std::endl;
*/
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  if (!is_initialized_) // First time the method is called
  {
    // std::cout << "Process first measurement" << std::endl;
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
      //std::cout << "Initialised state and covariance with LASER" << std::endl;
      //std::cout << x_ << std::endl;
      //std::cout << P_ << std::endl;
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
      //std::cout << "Initialised state and covariance with RADAR" << std::endl;
      //std::cout << x_ << std::endl;
      //std::cout << P_ << std::endl;
    }

    is_initialized_ = true;
    time_us_ = meas_package.timestamp_;
  }
  else // UKF already initialized. 
  {
    if (meas_package.sensor_type_ == MeasurementPackage::SensorType::LASER  && use_laser_)
    {
      //std::cout << "New LASER measurement. Calling Prediction" << std::endl;
      Prediction((meas_package.timestamp_-time_us_)/1000000.0);
      //std::cout << "Calling Update Lidar" << std::endl;
      UpdateLidar(meas_package);
      //std::cout << "End of Update" << std::endl;
      time_us_ = meas_package.timestamp_;
    }
    else if(meas_package.sensor_type_ == MeasurementPackage::SensorType::RADAR && use_radar_)
    {
      //std::cout << "New RADAR measurement. Calling Prediction" << std::endl;
      Prediction((meas_package.timestamp_-time_us_)/1000000.0);
      //std::cout << "Calling Update Radar" << std::endl;
      UpdateRadar(meas_package);
      //std::cout << "End of Update" << std::endl;
      time_us_ = meas_package.timestamp_;
    }  
  }
}

void UKF::Prediction(double delta_t) {
  /**
   * Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2*n_aug_+1);
  //std::cout << "Calling Generate" << std::endl;
  GenerateAugmentedSigmaPoints(&Xsig_aug);
  //std::cout << "Calling SigmaPrediction" << std::endl;
  SigmaPointPrediction(Xsig_aug, delta_t);
  //std::cout << "Calling Mean" << std::endl;
  PredictedMeanAndCovariance();
  //std::cout << "End of Prediction" << std::endl;
}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * Calculate the lidar NIS, at the end.
   */
  int n_z = n_z_las_;
  // measurement matrix
  MatrixXd H_ = MatrixXd(n_z,n_x_);
  H_ << 1, 0, 0, 0, 0,
        0, 1, 0, 0, 0;

  // measurement covariance matrix
  MatrixXd R_ = MatrixXd(n_z,n_z);
  R_ << std_laspx_*std_laspx_, 0,
        0, std_laspy_*std_laspy_;

  VectorXd z_pred = H_ * x_;
  VectorXd y = meas_package.raw_measurements_ - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  MatrixXd I = MatrixXd::Identity(n_x_, n_x_);
  P_ = (I - K * H_) * P_;

  //NIS check
  double nis = y.transpose()*Si*y;
  //std::cout << "NIS LIDAR: " << nis << std::endl;
  //nis_las_ << nis << std::endl;
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * Calculate the radar NIS, at the end.
   */
  int n_z = n_z_rad_; // set measurement dimension, radar can measure r, phi, and r_dot
  VectorXd z_pred = VectorXd(n_z);
  MatrixXd Zsig = MatrixXd(n_z, 2*n_aug_+1);
  MatrixXd S = MatrixXd(n_z, n_z);
  //std::cout << "Calling predict Radar measurement" << std::endl;
  PredictRadarMeasurement(&z_pred, &Zsig, &S);
  //std::cout << "Calling update Radar measurement" << std::endl;
  UpdateRadarMeasurement(meas_package.raw_measurements_, z_pred, Zsig, S);
}

void UKF::GenerateAugmentedSigmaPoints(MatrixXd* Xsig_out) {

  // create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);

  // create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

  // create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
 
  // create augmented mean state
  x_aug.head(n_x_) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  // create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x_,n_x_) = P_;
  P_aug(5,5) = std_a_*std_a_;
  P_aug(6,6) = std_yawdd_*std_yawdd_;

  // create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  // create augmented sigma points
  Xsig_aug.col(0)  = x_aug;
  for (int i = 0; i< n_aug_; ++i) {
    Xsig_aug.col(i+1)        = x_aug + sqrt(lambda_+n_aug_) * L.col(i);
    Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * L.col(i);
  }

  //std::cout << "Augmented Sigma Points: " << std::endl << Xsig_aug << std::endl;

  // write result
  *Xsig_out = Xsig_aug;
}

void UKF::SigmaPointPrediction(MatrixXd Xsig_aug, double delta_t) {

  // predict sigma points
  for (int i = 0; i< 2*n_aug_+1; ++i) {
    // extract values for better readability
    double p_x = Xsig_aug(0,i);
    double p_y = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yawd = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);

    // predicted state values
    double px_p, py_p;

    // avoid division by zero
    if (fabs(yawd) > 0.001) {
        px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
        py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
    } else {
        px_p = p_x + v*delta_t*cos(yaw);
        py_p = p_y + v*delta_t*sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;

    // add noise
    px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
    v_p = v_p + nu_a*delta_t;

    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p = yawd_p + nu_yawdd*delta_t;

    // write predicted sigma point into right column
    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
  }
  //std::cout << "Predicted Sigma Points: " << std::endl << Xsig_pred_ << std::endl;
}

void UKF::PredictedMeanAndCovariance() {

  // predicted state mean
  x_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {  // iterate over sigma points
    x_ = x_ + weights_(i) * Xsig_pred_.col(i);
  }
  while (x_(3)> M_PI) x_(3)-=2.*M_PI;
  while (x_(3)<-M_PI) x_(3)+=2.*M_PI;
  //std::cout << "Predicted State Mean: " << std::endl << x_ << std::endl;

  // predicted state covariance matrix
  P_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {  // iterate over sigma points
    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    // angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    P_ = P_ + weights_(i) * x_diff * x_diff.transpose() ;
  }
  //std::cout << "Predicted State Covariance: " << std::endl << P_ << std::endl;
}

void UKF::PredictRadarMeasurement(VectorXd* z_out, MatrixXd* Zsig_out, MatrixXd* S_out) {
  
  // set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = n_z_rad_;

  // create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  // mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  
  // measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);

  // transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {  // 2n+1 simga points
    // extract values for better readability
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    double v  = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    // measurement model
    Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                       // r
    Zsig(1,i) = atan2(p_y,p_x);                                // phi
    Zsig(2,i) = (p_x*v1 + p_y*v2) / sqrt(p_x*p_x + p_y*p_y);   // r_dot
  }

  // mean predicted measurement
  z_pred.fill(0.0);
  for (int i=0; i < 2*n_aug_+1; ++i) {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  // innovation covariance matrix S
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {  // 2n+1 simga points
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    // angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  // add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z,n_z);
  R <<  std_radr_*std_radr_, 0, 0,
        0, std_radphi_*std_radphi_, 0,
        0, 0,std_radrd_*std_radrd_;
  S = S + R;

  // write result
  *Zsig_out = Zsig;
  *z_out = z_pred;
  *S_out = S;
}

void UKF::UpdateRadarMeasurement(VectorXd z, VectorXd z_pred, MatrixXd Zsig, MatrixXd S) {

  int n_z = n_z_rad_;

  // create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);

  // calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {  // 2n+1 simga points
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    // angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    // angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  // Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  // residual
  VectorXd z_diff = z - z_pred;

  // angle normalization
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

  // update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();
  //std::cout << "Updated State: " << std::endl << x_ << std::endl;
  //std::cout << "Updated Covariance: " << std::endl << P_ << std::endl;

  //NIS Check
  double nis = z_diff.transpose()*S.inverse()*z_diff;
  //std::cout << "NIS RADAR: " << nis << std::endl;
  //nis_rad_ << nis << std::endl;
}