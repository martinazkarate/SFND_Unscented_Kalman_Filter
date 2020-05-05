#ifndef UKF_H
#define UKF_H

#include "Eigen/Dense"
#include "measurement_package.h"

class UKF {
 public:
  /**
   * Constructor
   */
  UKF();

  /**
   * Destructor
   */
  virtual ~UKF();

  /**
   * ProcessMeasurement
   * @param meas_package The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(MeasurementPackage meas_package);

  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
  void Prediction(double delta_t);

  /**
   * Updates the state and the state covariance matrix using a laser measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateLidar(MeasurementPackage meas_package);

  /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateRadar(MeasurementPackage meas_package);

  /**
   * Generates the sigma points for the prediction step
   * @param Xsig_out Matrix containing the sigma points
   */
  void GenerateAugmentedSigmaPoints(Eigen::MatrixXd* Xsig_out);

  /**
   * Prediction of the sigma points using the process function for the prediction step
   * @param Xsig_out Matrix containing the sigma points
   * @param delta_t Time between k and k+1 in s
   */
  void SigmaPointPrediction(Eigen::MatrixXd Xsig_aug, double delta_t);

  /**
   * Calculates Mean and Covariance Matrix of predicted sigma points
   */
  void PredictedMeanAndCovariance();

  /**
   * Prediction of the sigma points using the process function for the prediction step
   * @param z_out predicted mean measurement vector
   * @param Zsig_out Predicted measurement sigma points
   * @param S_out predicted measurement covariance matrix
   */
  void PredictRadarMeasurement(Eigen::VectorXd* z_out, Eigen::MatrixXd* Zsig_out, Eigen::MatrixXd* S_out);

  /**
   * Prediction of the sigma points using the process function for the prediction step
   * @param z measurement vector
   * @param z_pred predicted mean measurement vector
   * @param Zsig Predicted measurement sigma points
   * @param S predicted measurement covariance matrix
   */
  void UpdateRadarMeasurement(Eigen::VectorXd z, Eigen::VectorXd z_pred, Eigen::MatrixXd Zsig, Eigen::MatrixXd S);

  // initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  // if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  // if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  // state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  // predicted sigma points matrix
  Eigen::MatrixXd Xsig_pred_;

  // time when the state is true, in us
  long long time_us_;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  // Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  // Laser measurement noise standard deviation position1 in m
  double std_laspx_;

  // Laser measurement noise standard deviation position2 in m
  double std_laspy_;

  // Radar measurement noise standard deviation radius in m
  double std_radr_;

  // Radar measurement noise standard deviation angle in rad
  double std_radphi_;

  // Radar measurement noise standard deviation radius change in m/s
  double std_radrd_ ;

  // Weights of sigma points
  Eigen::VectorXd weights_;

  // State dimension
  int n_x_;

  // Augmented state dimension
  int n_aug_;

  // Measurment dimension laser
  int n_z_las_;

  // Measurment dimension radar
  int n_z_rad_;

  // Sigma point spreading parameter
  double lambda_;
};

#endif  // UKF_H