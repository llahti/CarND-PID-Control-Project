#include "PID.h"
//#include <chrono>  // high resolution timing
#include <iostream>

using namespace std;

/*
* This file implementents a PID controller.
* This PID controller takes in the error of set-point and real-value and then calculates new control variable.
*/

PID::PID() {
  // Timer initialization
  is_timer_initialized = false;
  i_error_list.clear();
}

PID::PID(double target, double Kp, double Ki, double Kd) {
  // Timer initialization
  is_timer_initialized = false;
  i_error_list.clear();
  // Set target and PID coefficients
  setTarget(target);
  Init(Kp, Ki, Kd);
}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  SetCoefficents(Kp_, Ki_, Kd_);

  // Errors should be zero in the beginning
  d_error = 0.0;
  i_error = 0.0;
  p_error = 0.0;
}

void PID::Reset() {
  is_timer_initialized = false;   // Should work as in next update timers are initialized
  d_error = 0.0;
  i_error = 0.0;
  p_error = 0.0;

  i_error_list.clear();
}

double PID::Update(double measurement) {
  double error = target_value - measurement;
  UpdateError(error);
  double control_variable = TotalError();
  return control_variable;
}

void PID::UpdateError_d(double cte)
{
  // D-Error is time dependent so we need to check that timer is initialized
  if (is_timer_initialized) {
    // Calculate time difference between this and previous update
    timestamp_now = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> delta_t = timestamp_now - previous_timestamp;
    previous_timestamp = timestamp_now;
    // Update D-error
    d_error = cte - p_error;  // Here p_error is effectively a previous cte
    d_error = d_error / delta_t.count();
  }
  else {
    previous_timestamp = std::chrono::high_resolution_clock::now();
    d_error = cte - p_error;  // Here p_error is effectively a previous cte
    start_timestamp = previous_timestamp;
    is_timer_initialized = true;
  }
}

void PID::UpdateError_i(double error)
{
  // I-error is limited to  past n errors
  i_error_list.push_back(error);
  if (i_error_list.size() > i_error_length) {
      i_error_list.pop_front();
    }
  double tmp_i_error = 0.0;
  for (double& e:i_error_list) {
      tmp_i_error += e;
  }
  i_error = tmp_i_error;
}

void PID::UpdateError(double error) {
  // Update D-error
  UpdateError_d(error);

  // Update P-error,
  // (NOTE! Keep this after D-error calculations
  // as d-error is using the previous p_error for delta calculations)
  p_error = error;

  // Update I-error
  UpdateError_i(error);
}

void PID::SetCoefficents(const double Kp_, const double Ki_, const double Kd_){
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
}


double PID::TotalError() {
  return Kp*p_error + Ki*i_error + Kd*d_error;
}

double PID::TotalRuntime() {
  if (is_timer_initialized) {
    std::chrono::duration<double> runtime;
    runtime = std::chrono::high_resolution_clock::now() - start_timestamp;
    return runtime.count();
  }
  else {
    // If timer is not initialized we'll get erroneus result by above calculations so therefore let's return 0.0
    return 0.0;
  }
}

void PID::setTarget(double target) {
  target_value = target;
}

