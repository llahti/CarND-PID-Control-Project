#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include <vector>
#include "PID.h"

class Optimizer {
private:
  std::vector<double> errors_update;  // List of errors, 1 for each update
  std::vector<double> errors_iteration;  // Keep track of error measurement of each iteration
  std::vector<double> control_output;  // List of control value outputs of each update
  std::vector<double> PID_coefficients;  // 3 numbers P, I and D
  std::vector<double> PID_dp;  // Potential change vector of PID coefficients
  double skip_nseconds = 0;  // how many seconds are skipped in beginning
  double iteration_time = 10;  // Maximum runtime of optimizer in secondss
  double best_error;  // Best error in twiddle algorithm
  std::vector<double> best_PID_coefficients;  // 3 numbers P, I and D
  unsigned int n_iteration = 0;  // Iteration counter for twiddle algorithm
  unsigned int i_coeff = 0;  // determine which coefficient is going to be modfied

  unsigned int twiddle_state = 0;  // State of twiddle; 0=first try, 1=second try

  double tolerance = 0.00001;  // Optimizer tolerance. I.E. if sum of change vector is less than tolerance then stop optimization.


  PID* pid;  // Pointer to PID controller so taht we can change it's parameters.
public:
  // Variables
  bool need_reset = false;  // Flag which shows when PID needs reset
  bool auto_reset = false;  // If true then reset PID automatically during update

  /**
  * @brief Constructors default constructor
  */
  Optimizer();

  /**
   * @brief Optimizer takes pid controller as an input and uses it's PID coefficients
   * to initialize optimizer initial pid coefficients
   *
   * @param pid pointer to PID controller
   */
  Optimizer(PID* pid);

  /**
  * Destructor.
  */
  virtual ~Optimizer();

  /**
   * Modify this function according to your needs to provide optimal cost function
   * for your needs.
   * @brief calculateIterationError calculates iteration error
   * @return
   */
  double calculateIterationError();

  /**
   * @brief getAverageError Returns absolute average error of current iteration.
   * @return
   */
  double getAbsAverageError();


  /**
   * @brief getAverageError Returns average error of current iteration.
   * @return
   */
  double getAverageError();


  /**
   * @brief getChangeCoeffSum Returns sum of change coefficients. Small sum means that PID coefficients are well optimized.
   * @return
   */
  double getChangeCoeffSum();

  /**
   * @brief getMaxError returns a maximum error value since last reset.
   * @return
   */
  double getAbsMaxError();

  /**
   * @brief getMSE Mean-Squared-Error
   * @return
   */
  double getMSE();

  /**
   * @brief getRMSE Root-Mean-Squared-Error
   * @return
   */
  double getRMSE();

  /**
   * @brief SetPID sets pointer to PID controller
   * @param pid pointer to PID controller
   */
  void setPID(PID* pid);

  /**
   * @brief setPIDCoefficients Set PID coefficients.
   * @param Kp
   * @param Ki
   * @param Kd
   */
  void setPIDCoefficients(const double Kp, const double Ki,const double Kd);

  /**
   * @brief setChangeCoefficients sets DP(change) coeffecients which are used to calculate new value of PID coefficients.
   * @param Dp
   * @param Di
   * @param Dd
   */
  void setChangeCoefficients(const double Dp, const double Di,const double Dd);

  /**
   * @brief setIterationTime Sets time of one iteration after which parameters are adjusted.
   * @param time
   */
  void setIterationTime(const double time);

  /**
   * @brief setSkipTime Defines how long time is skipped from the beginning before starting to track errors.
   * @param nseconds time in seconds
   */
  void setSkipTime(const double time);

  /**
   * @brief UpdateError updates PID controller error and keeps track of per update error statistics
   * @param error
   */
  void UpdateError(const double error);

  /**
   * @brief TotalError returns total error from PID controller and keeps track of per update control value statistics
   * @return
   */
  double TotalError();
};


#endif /* OPTIMIZER_H */
