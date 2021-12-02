#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using namespace std;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // TODO: YOUR CODE HERE
  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  VectorXd diff;
  // TODO: accumulate squared residuals
  if( estimations.size() == 0 ||
       estimations.size() != ground_truth.size()){
         cout<< "Invalid estimation or ground_truth data" <<endl;
       }
  else{
    for (int i=0; i < estimations.size(); ++i) {
        diff = estimations[i]-ground_truth[i];
        diff = diff.array() * diff.array();
        rmse += diff;
    }
  }
  // TODO: calculate the mean
  rmse = rmse/estimations.size();
  // TODO: calculate the squared root
  rmse = rmse.array().sqrt();
  // return the result
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */

  MatrixXd Hj(3,4);
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

 // Define constants that will be used in H. 
  float c1 = pow(px,2) + pow(py,2);
  float c2 = sqrt(c1);
  float c3 = (vx*py)-(vy*px);
  
  // check division by zero
  if(c1 == 0){
      cout << "Calculation Jacobian () - Error - Divison by Zero"<< endl;
  }
else{
   // Compute the Jacobian matrix
    Hj << (px/c2), (py/c2) , 0 ,0 ,
          (-py/c1), (px/c1),0 ,0 ,
          (py*c3)/(pow(c1,3/2)),(px*c3)/(pow(c1,3/2)),px/c2,  py/c2;
  }
  return Hj;
}
