/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"
#include <vector>
#include <iostream>
#include <math.h>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_maxi, double output_lim_mini) {
   /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   **/
  
  kpi = Kpi;
  kii = Kii;
  kdi = Kdi;
  max_lim = output_lim_maxi;
  min_lim = output_lim_mini;
  p_error = 0;
  i_error = 0;
  d_error = 0;
  dt = 0;

}


void PID::UpdateError(double cte) {
   /**
   * TODO: Update PID errors based on cte.
   **/
  double diff_cte;
  double prev_cte;
  
  diff_cte = cte - prev_cte;
  prev_cte = cte;
  
  p_error = cte;
  i_error += cte * dt;
  d_error = diff_cte / dt;
  
}

double PID::TotalError() {
   /**
   * TODO: Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
   */
    // control signal
    double control = kpi * p_error + kdi * d_error + kii * i_error;
  
    if(control < min_lim) {
      control = min_lim;
    } else if(control > max_lim) {
      control = max_lim;
    }
  
    return control;
}

double PID::UpdateDeltaTime(double new_dt) {
   /**
   * TODO: Update the delta time with new value
   */
   dt = new_dt;
   return dt;
}