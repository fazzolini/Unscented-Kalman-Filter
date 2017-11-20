#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
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
   *  ADDITIONAL DECLARATIONS
   ****************************************************************************/

  ///* Used to calculate time difference between measurements
  float dt;

  ///* constant to multiply dt to convert to seconds
  const float MICROSECONDS_PER_SECOND = 1000000.0;


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
};

#endif /* UKF_H */
