#include "PID.h"
#include "json.hpp"
#include <cstdlib>
using json = nlohmann::json;

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() 
{
  cte_prev = 0.0;
  cte_sum = 0.0;
}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) 
{
  this->Kp = Kp;
  this->Kd = Kd;
  this->Ki = Ki;
  is_init = false;
}

void PID::UpdateError(double cte) 
{
  p_error = Kp * cte;

  double cte_delta = cte - cte_prev;
  d_error = Kd * cte_delta;

  cte_sum += cte;
  i_error = Ki * cte_sum;
  
  cte_prev = cte;
}

double PID::TotalError(double prev_steer_value) 
{
  double steer_value = - p_error - i_error - d_error;
  double steer_value_delta, steer_delta_threshold = 0.5;

  steer_value = steer_value < -1.0 ? -1.0 : steer_value;
  steer_value = steer_value > 1.0 ? 1.0 : steer_value;

  steer_value_delta = steer_value - prev_steer_value;

  if(steer_value_delta > steer_delta_threshold)
  {
  	steer_value = prev_steer_value + steer_delta_threshold;
  }
  else if(steer_value_delta < -steer_delta_threshold)
  {
  	steer_value = prev_steer_value - steer_delta_threshold;
  }
  
  return steer_value;
}



