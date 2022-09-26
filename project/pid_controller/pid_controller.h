/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PID {
public:

   /**
   * TODO: Create the PID class
   **/

    //Errors
    double m_prop_err;
    double m_dev_err;
    double m_int_err;
    double m_prev_err;

    //Coefficients
    double m_kp;
    double m_ki;
    double m_kd;

    //Output limits
    double m_max_out;
    double m_min_out;
  
    //Delta time
    double m_dt;

    //Constructor
    PID();

    //Destructor.
    virtual ~PID();

    //Initialize PID.
    void Init(double Kp, double Ki, double Kd, double output_lim_max, double output_lim_min);

    //Update the PID error variables given cross track error.
    void UpdateError(double cte);

    //Calculate the total PID error.
    double TotalError();
  
    //Update the delta time.
    double UpdateDeltaTime(double new_delta_time);
};

#endif //PID_CONTROLLER_H

