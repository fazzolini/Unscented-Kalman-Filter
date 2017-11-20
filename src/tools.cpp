#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO: RMSE [DONE]
    * Calculate the RMSE here.
  */

  // NOTE: I reused code from EKF project for this part
  // default value is all zeros
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if(estimations.size() != ground_truth.size() || estimations.size() == 0){
    cout << "Invalid estimation or ground_truth data" << endl;
    return rmse;
  }

  //accumulate squared residuals
  for(unsigned int i=0; i < estimations.size(); ++i){

    // difference between truth and estimation
    VectorXd residual;
    residual = estimations[i] - ground_truth[i];

    //coefficient-wise multiplication
    VectorXd squared_residual;
    squared_residual = residual.array() * residual.array();

    // accumulate (element-wise addition)
    rmse += squared_residual;
  }

  //calculate the mean
  rmse = rmse / estimations.size();

  //calculate the squared root
  rmse = rmse.array().sqrt();

  //return the result
  return rmse;

}

float Tools::NormalizePhi(float &original_phi) {
  const auto pi = float(M_PI); // use C++ math.h M_PI, cast double to float
  if (original_phi > pi) {
    while (original_phi > pi) {
      original_phi = original_phi - 2 * pi;
    }
  } else if (original_phi < -pi) {
    while (original_phi < -pi) {
      original_phi = original_phi + 2 * pi;
    }
  }
  return original_phi;
}