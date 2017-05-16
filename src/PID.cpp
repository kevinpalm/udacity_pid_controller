#include "PID.h"
#include "math.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	
  // save the input parameters
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  
  // Assume we're starting in the middle of the track with a clean record
  prior_cte = 0.0;
  cumulative_cte = 0.0;

  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
}

void PID::UpdateError(double cte) {
}

double PID::OutputValue(double cte) {
	
	// Update the error
	UpdateError(cte);
	
	// Calculate the steering
	double steering = -Kp * cte - Kd * (prior_cte - cte) - Ki * cumulative_cte;
	
	// Since output values should be in the interval [-1. 1], let's apply a sigmoid transform
	steering = 2 * (1 / (1 + exp(-steering))) - 1;
	
	// Update the cte history variables
	prior_cte = cte;
	cumulative_cte += cte;
	
	return steering;
	
}

double PID::TotalError() {
}

