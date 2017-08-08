#include "optimizer.h"
#include <iostream>
#include <cmath>
#include <algorithm>

Optimizer::Optimizer() {
  setPIDCoefficients(0,0,0);
  setChangeCoefficients(1,1,1);
  errors_update.clear();
  best_PID_coefficients.clear();
  control_output.clear();
}

Optimizer::Optimizer(PID* pid ) {
  this->setPIDCoefficients(pid->Kp, pid->Ki, pid->Kd);
  this->setChangeCoefficients(1,1,1);
  this->pid = pid;
  errors_update.clear();
  best_PID_coefficients.clear();
  control_output.clear();
}

Optimizer::~Optimizer() {}

double Optimizer::calculateIterationError() {
  double cost = 0.0;
  const double rmse = getRMSE() * 0.5;
  const double abs_max_error = getAbsMaxError() * 0.5;

  double delta_control2 = 0.0;
  for (unsigned i = 1; i < control_output.size(); i++) {
      double delta = control_output[i] - control_output[i-1];
      delta_control2 += delta * delta;
  }
  delta_control2 = delta_control2 / control_output.size();
  // Sum up errors into a cost value
  cost += delta_control2 * 500;  // Penalty for steering angle change
  cost += rmse * 0.7;
  cost += abs_max_error * 0.4;
  std::cout << "Costs --"
            << "  rmse=" << rmse
            << "  abs_max_e=" << abs_max_error*0.2
            << "  delta_ctrl=" << delta_control2 * 10
            << std::endl;
  return cost;
}

double Optimizer::getAbsAverageError() {
  double sum=0;
  for (const auto& n : errors_update){
    sum += abs(n);
  }
  return sum / errors_update.size();
}

double Optimizer::getAverageError() {
  double sum=0;
  for (const auto& n : errors_update){
    sum += n;
  }
  return sum / errors_update.size();
}

double Optimizer::getChangeCoeffSum() {
  return PID_dp[0] + PID_dp[1] + PID_dp[2];
}

double Optimizer::getAbsMaxError() {
  const double max = *std::max_element(errors_update.begin(), errors_update.end());
  return std::abs(max);
}

double Optimizer::getMSE() {
  double squared_sum = 0;
  for (const auto& n : errors_update){
    squared_sum += n * n;
  }
  const double mse = squared_sum / errors_update.size();

  return mse;
}

double Optimizer::getRMSE() {
  double squared_sum = 0;
  for (const auto& n : errors_update){
    squared_sum += n * n;
  }
  const double rmse = sqrt(squared_sum / errors_update.size());

  if (rmse < 0) {
    std::cout << "ERROR: Got negative RMSE. Size of array is" << errors_update.size() << std::endl;
  }

  return rmse;
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

void Optimizer::UpdateError(const double error) {
  // Update pid controller
  pid->UpdateError(error);

  const double runtime =  pid->TotalRuntime();

  // Skip n seconds from the beginning.
  if (runtime > skip_nseconds) {
    errors_update.push_back(error);
  }

  // One iteration is done --> Prepare things for next iteration
  if (runtime > iteration_time) {
    // START OF TWIDDLE ALGORITHM
    // Calculate error metrics for this iteration
    const double error_metrics = calculateIterationError();
    // And then clear erros from previous iterations
    errors_update.clear();
    control_output.clear();

    //const double error_metrics = getAbsAverageError();
    errors_iteration.push_back(error_metrics);
    std::cout << n_iteration
              << " Best ERROR=" << best_error
              << " this ERROR=" << error_metrics
              << "\tP=" << pid->Kp << " I=" << pid->Ki << " D=" << pid->Kd
              //<< "\tDp=" << PID_dp[0] << " Di=" << PID_dp[1]<< " Dd=" << PID_dp[2]
              << "\tDP_SUM=" << getChangeCoeffSum() << std::endl;
    // Reset timers
    //pid->Reset();
    need_reset = true;  // PID coefficients will be changed so reset is needed


    if (n_iteration==0) {
      // This is first iteration so we just save reference value for further rmse comparisons.
      std::cout << "First iteration! ERROR=" << error_metrics << std::endl;
      best_error = error_metrics;
      best_PID_coefficients.clear();
      best_PID_coefficients.push_back(pid->Kp);
      best_PID_coefficients.push_back(pid->Ki);
      best_PID_coefficients.push_back(pid->Kd);
    }
    else {
      // The actual twiddle algorithm starts here
      double dp_sum = getChangeCoeffSum();
      // Check whether we still need to optimize
      if (std::abs(dp_sum) > tolerance) {
        // Optimizing
        std::cout << "\tTwiddle state=" << twiddle_state
                  << " i_coeff=" << i_coeff << std::endl;
        if (twiddle_state==0) {
            // First step is to change coefficients
            std::cout << "\tChange PID coefficients " << PID_coefficients[0] << " " << PID_coefficients[1] << " " << PID_coefficients[2];
            PID_coefficients[i_coeff] += PID_dp[i_coeff];
            std::cout << " --> " << PID_coefficients[0] << " " << PID_coefficients[1] << " " << PID_coefficients[2] << std::endl;
            pid->SetCoefficents(PID_coefficients[0],
                PID_coefficients[1],
                PID_coefficients[2]);
            twiddle_state = 1;  // on next iteration continue from second stage of twiddle
          }
        else if (twiddle_state==1){
          // Second step is to check whether RMSE improved
            if (error_metrics < best_error) {
              // RMSE improved.. continue to same direction
              std::cout << "\tError IMPROVED from " <<  best_error << " to " << error_metrics << std::endl;
              best_error = error_metrics;
              best_PID_coefficients.clear();
              best_PID_coefficients.push_back(pid->Kp);
              best_PID_coefficients.push_back(pid->Ki);
              best_PID_coefficients.push_back(pid->Kd);

              std::cout << "\tChange DP coefficients " << PID_dp[0] << " " << PID_dp[1] << " " << PID_dp[2];
              PID_dp[i_coeff] *= 1.1;
              std::cout << " --> " << PID_dp[0] << " " << PID_dp[1] << " " << PID_dp[2] << std::endl;

              twiddle_state = 0;  // start again from the first twiddle state

              std::cout << "\ti_coef: " << i_coeff;
              i_coeff = (i_coeff +1 ) % 3;  // continue optimizing from the next coeff
              std::cout << " --> " << i_coeff << std::endl;
            }
            else {
              // RMSE got worse.. change direction
              std::cout << "\tError not improved. Best error is " << best_error << " this time we got " << error_metrics << std::endl;
              std::cout << "\tChange PID coefficients " << PID_coefficients[0] << " " << PID_coefficients[1] << " " << PID_coefficients[2];
              PID_coefficients[i_coeff] -= 2 * PID_dp[i_coeff];
              std::cout << " --> " << PID_coefficients[0] << " " << PID_coefficients[1] << " " << PID_coefficients[2] << std::endl;

              pid->SetCoefficents(PID_coefficients[0],
                  PID_coefficients[1],
                  PID_coefficients[2]);
              twiddle_state = 2;
            }
        }
        else if (twiddle_state==2) {
          if (error_metrics < best_error) {
            std::cout << "\tError IMPROVED from " <<  best_error << " to " << error_metrics << std::endl;
            best_error = error_metrics;
            best_PID_coefficients.clear();
            best_PID_coefficients.push_back(pid->Kp);
            best_PID_coefficients.push_back(pid->Ki);
            best_PID_coefficients.push_back(pid->Kd);

            std::cout << "\tIncrease DP coefficients " << PID_dp[0] << " " << PID_dp[1] << " " << PID_dp[2];
            PID_dp[i_coeff] *= 1.1;
            std::cout << " --> " << PID_dp[0] << " " << PID_dp[1] << " " << PID_dp[2] << std::endl;
          }
          else {
            std::cout << "\tError not improved. Best error is " << best_error << " this time we got " << error_metrics << std::endl;
            PID_coefficients[i_coeff] += PID_dp[i_coeff];

            std::cout << "\tDecrease DP coefficients " << PID_dp[0] << " " << PID_dp[1] << " " << PID_dp[2];
            PID_dp[i_coeff] *= 0.7;
            std::cout << " --> " << PID_dp[0] << " " << PID_dp[1] << " " << PID_dp[2] << std::endl;
          }
          std::cout << "\ti_coef: " << i_coeff;
          i_coeff = (i_coeff + 1) % 3;  // continue optimizing from the next coeff
          std::cout << " --> " << i_coeff << std::endl;
          twiddle_state = 0;  // start again from the first twiddle state
        }
        //twiddle_state++;
      } // END_IF dp_sum > tolrance
      else {
        std::cout << "NOTHING TO OPTIMIZE!" << std::endl;
        std::cout << "Best PID coefficients: "
                  << best_PID_coefficients[0]
                  << " " << best_PID_coefficients[1]
                  << " " << best_PID_coefficients[2]
                  << std::endl;
        }
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

double Optimizer::TotalError() {
  double total_error = pid->TotalError();
  control_output.push_back(total_error);
  return total_error;
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
