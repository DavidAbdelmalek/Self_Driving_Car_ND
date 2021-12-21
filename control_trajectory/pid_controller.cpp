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
  Kp=Kpi;
  Ki=Kii;
  Kd=Kdi;
  output_lim_max=output_lim_maxi;
  output_lim_min=output_lim_mini;

  cte = 0;
  sum_cte = 0.0;
  diff_cte = 0.0;

}

void PID::UpdateError(double current_cte) {
   /**
   * TODO: Update PID errors based on cte.
   **/
   if (std::abs(delta_time) < 0.0001)
        diff_cte = 0;
      else
		   diff_cte = (current_cte - cte);
  sum_cte += current_cte*delta_time;
  cte = current_cte;
}

double PID::TotalError() {
   /**
   * TODO: Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
   */
  double control;
  if (abs(delta_time) < 0.000001)
    {
      return 0.0f;
    }
   control = -Kp * cte - Kd * diff_cte / delta_time - Ki * sum_cte;
   std::cout << "Cte " << cte << " Control: " << control << " P: " << -Kp * cte << " I: " << - Ki * sum_cte << " D: " << - Kd * diff_cte << " T: " << delta_time << std::endl;

   // Control error between minimum and maximum Limit values.
   control = max(control, output_lim_min);
   control = min(control, output_lim_max);
   

    return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
   /**
   * TODO: Update the delta time with new value
   */
  delta_time = new_delta_time;
}