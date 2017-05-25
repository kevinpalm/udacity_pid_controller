#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;
  
  /*
   * Output
   */
   double minrange;
   double maxrange;
  
  /*
  * CTE History
  */
  double prior_cte;
  double cumulative_cte;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd, double min_range, double max_range);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);
  
  /*
  * Get an output value from the PID controller given a cross track error
  */
  double OutputValue(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
};

#endif /* PID_H */
