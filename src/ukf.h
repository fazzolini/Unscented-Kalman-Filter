#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include "tools.h"
#include <vector>
#include <string>
#include <fstream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {
public:

  ///* Initially set to false, set to true in first call of ProcessMeasurement [DONE]
  bool is_initialized_;

  ///* If this is false, laser measurements will be ignored (except for init) [OK]
  bool use_laser_;

  ///* If this is false, radar measurements will be ignored (except for init) [OK]
  bool use_radar_;

  ///* For the case when both laser and radar are ignored (degenerate case?)
  bool ignore_laser_radar;

  ///* State vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad [OK]
  VectorXd x_; // initialized

  ///* State covariance matrix [OK]
  MatrixXd P_;

  ///* Time when the state is true, in us [DONE]
  long long time_us_;

  ///* Process noise standard deviation longitudinal acceleration in m/s^2 [OK]
  double std_a_;

  ///* Process noise standard deviation yaw acceleration in rad/s^2 [OK]
  double std_yawdd_;

  ///* Laser measurement noise standard deviation position1 in m [OK]
  double std_laspx_;

  ///* Laser measurement noise standard deviation position2 in m [OK]
  double std_laspy_;

  ///* Radar measurement noise standard deviation radius in m [OK]
  double std_radr_;

  ///* Radar measurement noise standard deviation angle in rad [OK]
  double std_radphi_;

  ///* Radar measurement noise standard deviation radius change in m/s [OK]
  double std_radrd_;

  ///* State dimension [DONE]
  int n_x_;

  ///* Augmented state dimension [DONE]
  int n_aug_;

  ///* Weights of sigma points [DONE]
  VectorXd weights_;

  ///* Predicted sigma points matrix [DONE]
  MatrixXd Xsig_pred_;

  ///* Sigma point spreading parameter [DONE]
  double lambda_;

  /*****************************************************************************
   *  ADDITIONAL DECLARATIONS OF VARS [START]
   ****************************************************************************/

  ///* Used to calculate time difference between measurements
  float dt_;

  ///* Constant to multiply dt to convert to seconds
  const float MICROSECONDS_PER_SECOND_ = 1000000.0;

  ///* Augmented state matrix
  MatrixXd Xsig_aug_;

  ///* Predicted sigma points
  MatrixXd Xsig_pred_;

  ///* Instance of tools
  Tools tools_;

  ///* Dimensions of lidar and radar measurements
  int n_z_lidar_;
  int n_z_radar_;

  ///* Store difference for state
  VectorXd x_diff_;

  ///* Vectors to store lidar and radar measurements
  VectorXd z_meas_lidar_;
  VectorXd z_meas_radar_;

  ///* Vectors to store lidar and radar predictions
  VectorXd z_pred_lidar_;
  VectorXd z_pred_radar_;

  ///* Vectors to store lidar and radar predictions diffs
  VectorXd z_diff_lidar_;
  VectorXd z_diff_radar_;

  ///* Matrices to store lidar and radar sigma predictions
  MatrixXd Zsig_lidar_;
  MatrixXd Zsig_radar_;

  ///* Matrices for UpdateLidar
  MatrixXd S_lidar_; // measurement covariance
  MatrixXd R_lidar_; // measurement noise
  MatrixXd T_lidar_; // cross-correlation for diffs
  MatrixXd K_lidar_; // kalman gain

  ///* Matrices for UpdateRadar
  MatrixXd S_radar_; // measurement covariance
  MatrixXd R_radar_; // measurement noise
  MatrixXd T_radar_; // cross-correlation for diffs
  MatrixXd K_radar_; // kalman gain

  /*****************************************************************************
   *  ADDITIONAL DECLARATIONS OF VARS [END]
   ****************************************************************************/


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

  /*****************************************************************************
   *  ADDITIONAL DECLARATIONS OF METHODS [START]
   ****************************************************************************/

  /**
   * Creates a matrix that stores augmented sigma points according to Unscented
   * Kalman Filter algorithm covered in classroom.
   * Takes no params and returns nothing because it operates on class variables.
   */
  void AugmentedSigmaPoints();

  /**
   * Uses augmented sigma points matrix to make predictions
   * for each point and store it in a separate matrix.
   * Returns nothing because it operates on class variables.
   */
  void SigmaPointPrediction(float delta_t);

  /**
   * Based on sigma points predictions calculate inferred
   * mean vector and covariance matrix for prediction.
   * This is the last step of prediction step of UKF.
   * Takes no params and returns nothing because it operates on class variables.
   */
  void PredictMeanAndCovariance();
};

/*****************************************************************************
 *  ADDITIONAL DECLARATIONS OF METHODS [END]
 ****************************************************************************/

#endif /* UKF_H */
