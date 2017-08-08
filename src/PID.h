#ifndef PID_H
#define PID_H
#include <chrono>  // high resolution timing
#include <vector>
#include <list>

/**
 * @brief The PID class implements a PID control algorithm.
 *
 * This class implements a PID-control algorithm. It consist of two
 * principal methods to realize PID-control
 * 1. UpdateError(error)
 *   - Calculates proportional, derivative and integral error terms
 *   - Input to this method is generally the the difference of setpoint
 *     and real measured process value. (I.e. Difference of track-center
 *     and measured track center (cross track error))
 * 2. TotalError()
 *   - Returns the total error which is calculated by multiplying error terms by their
 *     corresponding weights.
 *   - Output from this method is the new control value. (I.e. steering angle)
 *
 * Simple Usage
 * ------------
 *
 * PID pid;
 * pid.Init(-0.5, -0.0001, 0.05);
 * // Control Loop
 * while (true) {
 *   double error = 0; // TODO: Measured error
 *   pid.UpdateError(error);
 *   double ctrl = pid.TotalError();  // Calculate new control value
 *   //ProcessControl(ctrl);   // TODO: Control process by using ctrl value
 * }
 */
class PID {
private:
  // Timers for D-error calculations
  std::chrono::time_point<std::chrono::high_resolution_clock>  previous_timestamp;
  std::chrono::time_point<std::chrono::high_resolution_clock>  timestamp_now;
  bool is_timer_initialized;
  // Time stamp of very first update event (Needed when total runtime is calculated)
  std::chrono::time_point<std::chrono::high_resolution_clock>  start_timestamp;

  // This vector is used to calculate I-error
  std::list<double> i_error_list;
  // maximum number of items in i_error_list
  unsigned int i_error_length = 20;

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
  * This function updates internal error parameters of P, I and D term.
  * P_error = cte
  * D_error = cte/delta_t (delta_t is time between this and last update)
  * I_error = sum of all errors
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
