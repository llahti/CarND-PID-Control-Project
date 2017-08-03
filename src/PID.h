#ifndef PID_H
#define PID_H
#include <chrono>  // high resolution timing


class PID {
private:
  // Timers for D-error calculations
  std::chrono::time_point<std::chrono::high_resolution_clock>  previous_timestamp;
  std::chrono::time_point<std::chrono::high_resolution_clock>  timestamp_now;
  bool is_timer_initialized;
  // Time stamp of very first update event (Needed when total runtime is calculated)
  std::chrono::time_point<std::chrono::high_resolution_clock>  start_timestamp;

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

  /**
  * Constructor
  */
  PID();

  /**
  * Destructor.
  */
  virtual ~PID();

  /**
  * Initialize PID.
  */
  void Init(double Kp_, double Ki_, double Kd_);

  /**
   * @brief Reset resets the PID by setting runtime to zero and resetting errors
   */
  void Reset();

  /**
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);


  /**
   * @brief Set P,I and D coefficients
   * @param Kp_ Controls the proportional gain
   * @param Ki_ Controls the integral gain (sum of previous errors)
   * @Param Kd_ Controls the derivate gain (change rate of error)
   */
  void SetCoefficents(const double Kp_, const double Ki_, const double Kd_);

  /**
  * Calculate the total PID error.
  */
  double TotalError();

  /**
   * Returns the total runtime of PID controller.
   * It is calculated as: time_now - time_firstupdate
   * Time is returned in seconds
   */
  double TotalRuntime();
};

#endif /* PID_H */
