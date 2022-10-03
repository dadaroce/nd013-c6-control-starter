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
   //Initialize PID coefficients (and errors, if needed)
   m_kp = Kpi;
   m_ki = Kii;
   m_kd = Kdi;
   m_max_out = output_lim_maxi;
   m_min_out = output_lim_mini;
   m_int_err = 0.0;
   m_prev_err = 0.0;
  
   m_dt = 1;
}


void PID::UpdateError(double cte) {
   m_prop_err = cte;
   m_int_err =  m_int_err + m_prop_err * m_dt; 
   m_dev_err = (m_prop_err - m_prev_err) / m_dt;
   m_prev_err = m_prop_err;
}

double PID::TotalError() {

   double control = m_kp * m_prev_err + m_ki * m_int_err + m_kd * m_dev_err;
   control = control > m_max_out ? m_max_out : control;
   control = control < m_min_out ? m_min_out : control;
   
   return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
   m_dt = new_delta_time;
}