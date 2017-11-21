#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // If this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // If this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // If both use_laser_ and ude_radar_ are false
  ignore_laser_radar = !use_laser_ && !use_radar_;

  // Initial state vector
  x_ = VectorXd(5);

  // Initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;

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

  /*****************************************************************************
   *  ADDITIONAL INITIALIZATIONS
   ****************************************************************************/

  ///* Used to calculate time difference between measurements
  dt_ = 0.0;

  ///* Initialize sigma point matrix
  Xsig_aug_ = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  ///* Initialize predictions matrix
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

  ///* Dimensions of lidar and radar measurements
  n_z_lidar_ = 2;
  n_z_radar_ = 3;

  ///* Vectors to store lidar and radar measurements
  z_meas_lidar_ = VectorXd(n_z_lidar_);
  z_meas_radar_ = VectorXd(n_z_radar_);

  ///* Vectors to store lidar and radar predictions
  z_pred_lidar_ = VectorXd(n_z_lidar_);
  z_pred_radar_ = VectorXd(n_z_radar_);

  ///* Matrices to store lidar and radar sigma predictions
  Zsig_lidar_ = MatrixXd(n_z_lidar_, 2 * n_aug_ + 1);
  Zsig_radar_ = MatrixXd(n_z_radar_, 2 * n_aug_ + 1);

  ///* Matrices for UpdateLidar
  S_lidar_ = MatrixXd(n_z_lidar_, n_z_lidar_); // measurement covariance
  R_lidar_ = MatrixXd(n_z_lidar_, n_z_lidar_); // measurement noise
  T_lidar_ = MatrixXd(n_x_, n_z_lidar_); // cross-correlation for diffs

  ///* Matrices for UpdateRadar
  S_radar_ = MatrixXd(n_z_radar_, n_z_radar_); // measurement covariance
  R_radar_ = MatrixXd(n_z_radar_, n_z_radar_); // measurement noise
  T_radar_ = MatrixXd(n_x_, n_z_radar_); // cross-correlation for diffs

  /**
  TODO: UKF Initialization [DONE]

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */

  /**
   * Note: I consecutively checked ukf.h for which vars were already
   * initialized above and then initialized myself missing ones
   */

  ///* Initially set to false, set to true in first call of ProcessMeasurement
  is_initialized_ = false;

  ///* Time when the state is true, in us (initial time is zero)
  time_us_ = 0.0;

  ///* State dimension (as per CTRV model)
  n_x_ = 5;

  ///* Augmented state dimension (state dim + 2 for process noise)
  n_aug_ = n_x_ + 2;

  ///* Weights of sigma points (center point + 2 for each dimension of augmented vector)
  weights_ = VectorXd(2 * n_aug_ + 1);

  ///* Predicted sigma points matrix (rows: state vector length, cols: num of sigma points)
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

  ///* Sigma point spreading parameter (as per classroom solution)
  lambda_ = 3 - n_x_;

}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO: ProcessMeasurement() [DONE]

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */

  /**
   * Workflow follows approach from previous project (EKF)
   * In fact, this is any Bayes Filter approach
   * 1. Initialization
   * 2. Prediction
   * 3. Measurement Update
   */

  /*****************************************************************************
   *  1. Initialization
   ****************************************************************************/

  /**
   * TODO: Initialize [DONE]
   * 1. Set initial state
   * 2. Set initial covariance matrix
   * 3. Process first lidar or radar measurement
   * 4. Set timestamp
   * 5. Set initialization flag
   */

  if (!is_initialized_) {

    // Set initial x. NOTE: [pos1 pos2 vel_abs yaw_angle yaw_rate]
    x_ << 1, 1, 1, 1, 1;

    // Set initial covariance matrix
    P_ << 1, 0, 0, 0, 0,
        0, 1, 0, 0, 0,
        0, 0, 1, 0, 0,
        0, 0, 0, 1, 0,
        0, 0, 0, 0, 1;

    // Process initial measurements
    if (meas_package.sensor_type_ == MeasurementPackage::LASER) {

      // Process initial laser measurement.
      // NOTE: laser gives positions for x and y
      x_(0) = meas_package.raw_measurements_(0);
      x_(1) = meas_package.raw_measurements_(1);

    } else if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {

      // Process initial radar measurement.
      // NOTE: radar gives distance rho, angle phi, speed in rho direction rho_dot
      // Can determine x and y position

      // Extract measurements
      float rho = meas_package.raw_measurements_(0);
      float phi = meas_package.raw_measurements_(1);
//      float rho_dot = meas_package.raw_measurements_(2); // not used (for now)

      // Update state
      x_(0) = rho * cos(phi); // distance projection on vertical axis x
      x_(1) = rho * sin(phi); // distance projection on vertical axis x

    }

    // Set time stamp
    time_us_ = meas_package.timestamp_;

    // Set initialization flag
    is_initialized_ = true;

  }

  /*****************************************************************************
   *  2. Prediction
   ****************************************************************************/

  /**
   * TODO: Predict [DONE]
   * This is the part where UKF magic happens
   */

  // Calculate how much time between measurements (in seconds); time_us_ is old value
  dt_ = (meas_package.timestamp_ - time_us_) * MICROSECONDS_PER_SECOND_;
  // Update time_us_ to current value
  time_us_ = meas_package.timestamp_;

  // Call predict step
  Prediction(dt_);

  /*****************************************************************************
   *  3. Measurement Update
   ****************************************************************************/

  /**
   * TODO: Update [DONE]
   * Call different update function depending on the type of measurement
   */

  if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
    UpdateLidar(meas_package);
  } else if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    UpdateRadar(meas_package);
  }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO: Prediction() [DONE]

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */

  /**
   * Unscented Kalman Filter algorithm is implemented in this module.
   * The steps are:
   * 1. generate augmented sigma points
   * 2. predict augmented sigma points
   * 3. calculate mean/variance of predicted sigma points
   * [these calculated mean and variance are then used to update with lidar / radar data]
   */

  AugmentedSigmaPoints();

  SigmaPointPrediction(delta_t);

  PredictMeanAndCovariance();

}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO: UpdateLidar() [DONE]

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */

  // Extract measurement as VectorXd
  z_meas_lidar_ = meas_package.raw_measurements_;

  // Reset matrix
  Zsig_lidar_.fill(0.0);

  // Calculate sigma points in measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {

    // Extract values for better readability
    double p_x = Xsig_pred_(0, i);
    double p_y = Xsig_pred_(1, i);

    // Measurement model is very simple for lidar
    Zsig_lidar_(0, i) = p_x;
    Zsig_lidar_(1, i) = p_y;

  }

  // Calculate predicted measurement
  // Reset vector
  z_pred_lidar_.fill(0.0);

  for (int i = 0; i < 2 * n_aug_ + 1; i++) {

    z_pred_lidar_ = z_pred_lidar_ + weights_(i) * Zsig_lidar_.col(i);

  }

  // Reset measurement covariance matrix S
  S_lidar_.fill(0.0);

  for (int i = 0; i < 2 * n_aug_ + 1; i++) {

    z_diff_lidar_ = Zsig_lidar_.col(i) - z_pred_lidar_;
    S_lidar_ = S_lidar_ + weights_(i) * z_diff_lidar_ * z_diff_lidar_.transpose();

  }

  // Add measurement noise covariance matrix R
  R_lidar_ << std_laspx_ * std_laspx_, 0,
      0, std_laspy_ * std_laspy_;
  S_lidar_ = S_lidar_ + R_lidar_;

  // Diffs cross correlation matrix T
  T_lidar_.fill(0.0);

  for (int i = 0; i < 2 * n_aug_ + 1; i++) {

    // Measurement difference
    z_diff_lidar_ = Zsig_lidar_.col(i) - z_pred_lidar_;

    // State difference
    x_diff_ = Xsig_pred_.col(i) - x_;

    T_lidar_ = T_lidar_ + weights_(i) * x_diff_ * z_diff_lidar_.transpose();
  }

  // Kalman gain K
  K_lidar_ = T_lidar_ * S_lidar_.inverse();

  // Diff for measurements
  z_diff_lidar_ = z_meas_lidar_ - z_pred_lidar_;

  // Update state mean and covariance matrix
  x_ = x_ + K_lidar_ * z_diff_lidar_;
  P_ = P_ - K_lidar_ * S_lidar_ * K_lidar_.transpose();

}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO: UpdateRadar()

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */

  // Extract measurements for easier access
  z_meas_radar_ = meas_package.raw_measurements_;

  // Transform sigma points to measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {

    // Extract values for better readability
    double p_x = Xsig_pred_(0, i);
    double p_y = Xsig_pred_(1, i);
    double v = Xsig_pred_(2, i);
    double yaw = Xsig_pred_(3, i);

    double v1 = cos(yaw) * v;
    double v2 = sin(yaw) * v;

    // Measurement model
    Zsig_radar_(0, i) = sqrt(p_x * p_x + p_y * p_y);                          //rho
    Zsig_radar_(1, i) = atan2(p_y, p_x);                                      //phi
    Zsig_radar_(2, i) = (p_x * v1 + p_y * v2) / sqrt(p_x * p_x + p_y * p_y);  //rho_dot

  }

  // Mean predicted measurement
  z_pred_radar_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    z_pred_radar_ = z_pred_radar_ + weights_(i) * Zsig_radar_.col(i);
  }

  // Measurement covariance matrix S
  S_radar_.fill(0.0);

  for (int i = 0; i < 2 * n_aug_ + 1; i++) {

    // Measurement diff
    z_diff_radar_ = Zsig_radar_.col(i) - z_pred_radar_;

    // Angle normalization
    z_diff_radar_(1) = tools_.NormalizePhi(z_diff_radar_(1));

    S_radar_ = S_radar_ + weights_(i) * z_diff_radar_ * z_diff_radar_.transpose();

  }

  // Add measurement noise covariance matrix
  R_radar_ << std_radr_ * std_radr_, 0, 0,
      0, std_radphi_ * std_radphi_, 0,
      0, 0, std_radrd_ * std_radrd_;
  S_radar_ = S_radar_ + R_radar_;

  // Diffs cross correlation matrix T
  T_radar_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {

    // Difference of predictions
    z_diff_radar_ = Zsig_radar_.col(i) - z_pred_radar_;

    // Angle normalization
    z_diff_radar_(1) = tools_.NormalizePhi(z_diff_radar_(1));

    // State difference
    x_diff_ = Xsig_pred_.col(i) - x_;

    // Angle normalization
    x_diff_(3) = tools_.NormalizePhi(x_diff_(3));

    T_radar_ = T_radar_ + weights_(i) * x_diff_ * z_diff_radar_.transpose();

  }

  // Kalman gain
  K_radar_ = T_radar_ * S_radar_.inverse();

  // Diff for measurement
  z_diff_radar_ = z_meas_radar_ - z_pred_radar_;

  // Angle normalization
  z_diff_radar_(1) = tools_.NormalizePhi(z_diff_radar_(1));

  // Update state mean and covariance matrix
  x_ = x_ + K_radar_ * z_diff_radar_;
  P_ = P_ - K_radar_ * S_radar_ * K_radar_.transpose();

}


void UKF::AugmentedSigmaPoints() {
  /**
   * TODO: Generate Aug Sigma Points [DONE]
   */

  // Clear old values (possibly not needed)
  Xsig_aug_.fill(0.0);

  // Create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);

  // Create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

  // Create augmented mean state
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  // Create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5, 5) = P_; // top-left 5x5 block
  P_aug(5, 5) = std_a_ * std_a_; // variance of a
  P_aug(6, 6) = std_yawdd_ * std_yawdd_; // variance of yawdd

  // Create square root matrix
  MatrixXd L = P_aug.llt().matrixL(); // Cholesky decomposition

  // Create augmented sigma points
  Xsig_aug_.col(0) = x_aug;
  for (int i = 0; i < n_aug_; i++) {

    // Here we are looping through individual columns of matrix Xsig_aug_
    Xsig_aug_.col(i + 1) = x_aug + sqrt(lambda_ + n_aug_) * L.col(i);
    Xsig_aug_.col(i + 1 + n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * L.col(i);

  }
}

void UKF::SigmaPointPrediction(float delta_t) {
  /**
   * TODO: Predict Aug Sigma Points [DONE]
   */

  // Clear old values (possibly not needed)
  Xsig_pred_.fill(0.0);

  // Loop through generated sigma points and
  // make prediction for each one of them.
  // All math is according to classroom equations
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {

    // Extract values for better readability
    double p_x = Xsig_aug_(0, i);
    double p_y = Xsig_aug_(1, i);
    double v = Xsig_aug_(2, i);
    double yaw = Xsig_aug_(3, i);
    double yawd = Xsig_aug_(4, i);
    double nu_a = Xsig_aug_(5, i);
    double nu_yawdd = Xsig_aug_(6, i);

    // Predicted state values
    double px_p, py_p;

    // Deal with division by zero
    if (fabs(yawd) > 0.001) {

      px_p = p_x + v / yawd * (sin(yaw + yawd * delta_t) - sin(yaw));
      py_p = p_y + v / yawd * (cos(yaw) - cos(yaw + yawd * delta_t));

    } else {

      px_p = p_x + v * delta_t * cos(yaw);
      py_p = p_y + v * delta_t * sin(yaw);

    }

    double v_p = v;
    double yaw_p = yaw + yawd * delta_t;
    double yawd_p = yawd;

    // Add noise according to equations
    px_p = px_p + 0.5 * nu_a * delta_t * delta_t * cos(yaw);
    py_p = py_p + 0.5 * nu_a * delta_t * delta_t * sin(yaw);
    v_p = v_p + nu_a * delta_t;
    yaw_p = yaw_p + 0.5 * nu_yawdd * delta_t * delta_t;
    yawd_p = yawd_p + nu_yawdd * delta_t;

    // Write predicted sigma point into i-th column
    Xsig_pred_(0, i) = px_p;
    Xsig_pred_(1, i) = py_p;
    Xsig_pred_(2, i) = v_p;
    Xsig_pred_(3, i) = yaw_p;
    Xsig_pred_(4, i) = yawd_p;

  }
}

void UKF::PredictMeanAndCovariance() {
  /**
   * TODO: Calculate Mean / Variance [DONE]
   */

  // Set first weight to a special value
  weights_(0) = lambda_ / (lambda_ + n_aug_);

  // Set remaining weights to same value
  for (int i = 1; i < 2 * n_aug_ + 1; i++) {

    weights_(i) = 0.5 / (n_aug_ + lambda_);

  }

  // Reset state mean
  x_.fill(0.0);

  // Iterate over sigma points, update state mean
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {

    x_ = x_ + weights_(i) * Xsig_pred_.col(i);

  }

  // Predicted state covariance matrix
  P_.fill(0.0);

  // Iterate over sigma points
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {

    // State difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;

    // Angle normalization
    x_diff(3) = tools_.NormalizePhi(x_diff(3));

    // Update state covariance matrix
    P_ = P_ + weights_(i) * x_diff * x_diff.transpose();

  }
}
