#include "optimizer.h"
#include <iostream>
#include <cmath>

Optimizer::Optimizer() {
  setPIDCoefficients(0,0,0);
  setChangeCoefficients(1,1,1);
  errors_update.clear();
}

Optimizer::Optimizer(PID* pid ) {
  this->setPIDCoefficients(0,0,0);
  this->setChangeCoefficients(1,1,1);
  this->pid = pid;
  errors_update.clear();
}

Optimizer::~Optimizer() {}

double Optimizer::getAverageError() {
  double sum=0;
  for (auto& n : errors_update){
    sum += n;
  }
  return sum / errors_update.size();
}

double Optimizer::getChangeCoeffSum() {
  return PID_dp[0] + PID_dp[1] + PID_dp[2];
}


double Optimizer::getRMSE() {
  double squared_sum;
  for (auto& n : errors_update){
    squared_sum += n * n;
  }
  return sqrt(squared_sum / errors_update.size());
}

void Optimizer::setPID(PID* pid) {
  this->pid = pid;
}


void Optimizer::setPIDCoefficients(const double Kp, const double Ki,const double Kd) {
  PID_coefficients.clear();
  PID_coefficients.push_back(Kp);
  PID_coefficients.push_back(Ki);
  PID_coefficients.push_back(Kd);
}

void Optimizer::setChangeCoefficients(const double Dp, const double Di,const double Dd) {
  PID_dp.clear();
  PID_dp.push_back(Dp);
  PID_dp.push_back(Di);
  PID_dp.push_back(Dd);
}

void Optimizer::setIterationTime(const double time) {
  if (skip_nseconds < time) {
    iteration_time = time;
  }
  else {
    std::cout << "MaxRuntime can't be less than SkipTime." << std::endl;
  }
}

void Optimizer::setSkipTime(const double time) {
  if (time < iteration_time) {
    skip_nseconds = time;
  }
  else {
    std::cout << "Skip time can't be more than maximum runtime" << std::endl;
  }
}

void Optimizer::Update(const double error) {
  const double runtime =  pid->TotalRuntime();

  // Skip n seconds from the beginning.
  if (runtime > skip_nseconds) {
    errors_update.push_back(error);
  }

  // One iteration is done --> Prepare things for next iteration
  if (runtime > iteration_time) {
    // START OF TWIDDLE ALGORITHM
    double rmse = getRMSE();
    errors_iteration_rmse.push_back(getRMSE());
    std::cout << n_iteration
              << " RMSE=" << rmse
              << "\tP=" << pid->Kp << " I=" << pid->Ki << " D=" << pid->Kd
              << "\tDp=" << PID_dp[0] << " Di=" << PID_dp[1]<< " Dd=" << PID_dp[2]
              << "\tDP_SUM=" << getChangeCoeffSum() << std::endl;
    // Reset timers
    //pid->Reset();
    need_reset = true;  // PID coefficients will be changed so reset is needed
    // Clear erros from previous iteration
    errors_update.clear();

    if (n_iteration==0) {
      best_error = rmse;
    }
    else {
      double dp_sum = getChangeCoeffSum();
      // Check whether we still need to optimize
      if (dp_sum > tolerance) {
          // Optimizing
          //std::cout << "Optimizer: i=" << n_iteration
          //          << " Twiddle state=" << twiddle_state
          //          << " i_coeff=" << i_coeff << std::endl;
          if (twiddle_state==0) {
              // First step is to change coefficients
              PID_coefficients[i_coeff] += PID_dp[i_coeff];
              pid->SetCoefficents(PID_coefficients[0],
                  PID_coefficients[1],
                  PID_coefficients[2]);
              twiddle_state = 1;  // on next iteration continue from second stage of twiddle
            }
          else if (twiddle_state==1){
            // Second step is to check whether RMSE improved
              if (error < best_error) {
                // RMSE improved.. continue to same direction
                best_error = error;
                PID_dp[i_coeff] *= 1.1;
                twiddle_state = 0;  // start again from the first twiddle state
                i_coeff = (i_coeff +1 ) % 3;  // continue optimizing from the next coeff
              }
              else {
                // RMSE got worse.. change direction
                PID_coefficients[i_coeff] -= 2 * PID_dp[i_coeff];
                pid->SetCoefficents(PID_coefficients[0],
                    PID_coefficients[1],
                    PID_coefficients[2]);
                twiddle_state = 2;
              }
          }
          else if (twiddle_state==2) {
            if (error < best_error) {
              best_error = error;
              PID_dp[i_coeff] *= 1.1;
            }
            else {
              PID_coefficients[i_coeff] += PID_dp[i_coeff];
              PID_dp[i_coeff] *= 0.9;
            }
            i_coeff = (i_coeff + 1) % 3;  // continue optimizing from the next coeff
            twiddle_state = 0;  // start again from the first twiddle state
          }
        //twiddle_state++;
        } // END_IF dp_sum > tolrance
    }
    // Increasce iteration counter
    n_iteration++;
    // END OF TWIDDLE ALGORITHM
    }
  // Do automatic reset when reset is needed
  if (auto_reset && need_reset) {
      pid->Reset();
      need_reset = false;
    }
}
/*
 * def twiddle(tol=0.2):
    p = [0, 0, 0]
    dp = [1, 1, 1]
    robot = make_robot()
    x_trajectory, y_trajectory, best_err = run(robot, p)

    it = 0
    while sum(dp) > tol:
        print("Iteration {}, best error = {}".format(it, best_err))
        for i in range(len(p)):
            p[i] += dp[i]
            robot = make_robot()
            x_trajectory, y_trajectory, err = run(robot, p)

            if err < best_err:
                best_err = err
                dp[i] *= 1.1
            else:
                p[i] -= 2 * dp[i]
                robot = make_robot()
                x_trajectory, y_trajectory, err = run(robot, p)

                if err < best_err:
                    best_err = err
                    dp[i] *= 1.1
                else:
                    p[i] += dp[i]
                    dp[i] *= 0.9
        it += 1
    return p
 */
