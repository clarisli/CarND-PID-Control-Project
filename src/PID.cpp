#include "PID.h"

#include <math.h>    
#include <iostream>

const double tolerance = 0.0001;
const int update_count_threshold = 100;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	PID::Kp = Kp;
	PID::Ki = Ki;
	PID::Kd = Kd;

	p_error = 0.0;
	i_error = 0.0;
	d_error = 0.0;
	prev_cte = 0.0;

	// Twiddle
	twiddle_state = Uninitialized;
	optimize_index = 0;
	best_error = 999.0;
	update_count = 0;
	total_error = 0;
	dp = { Kp/10, Kd/10, 0.001};
}

void PID::UpdateError(double cte) {

	std::cout << "UpdateError: " << Kp << ", " << Kd << ", " << Ki << endl;

	// update pid errors
	p_error = cte;
	d_error = cte - prev_cte;
	i_error += cte;
	prev_cte = cte;

	// Update error for twiddle
	if (update_count < update_count_threshold) {
		total_error += cte*cte;
		update_count++;
	}




}

double PID::TotalError() {
	return Kp * p_error + Ki * i_error + Kd * d_error;
}

void PID::Twiddle() {

	std::cout << "Twiddle: " << update_count << ", threshold:" << update_count_threshold << endl; 

	if (update_count < update_count_threshold) {
		return;
	}

	double current_error = total_error / update_count;

	vector<double> p { Kp, Kd, Ki};
	double sum_dp = dp[0] + dp[1] + dp[2];
	std::cout << "sum dp: " << sum_dp << endl; 
	if (sum_dp > tolerance) {
			std::cout << "twiddle: " << twiddle_state << ", current_error: " << current_error << ", best_error: " << best_error << endl;
			switch (twiddle_state) {
				case Uninitialized:
					p[optimize_index] += dp[optimize_index];
					twiddle_state = AddDP;
					break;
				case AddDP:
					if (current_error < best_error) {
						best_error = current_error;
						dp[optimize_index] *= 1.1;
						twiddle_state = Done;
					} else {
						p[optimize_index] -= 2* dp[optimize_index];
						twiddle_state = MinusDP;
					}
					break;
				case MinusDP:
					if (current_error < best_error) {
						best_error = current_error;
						dp[optimize_index] *= 1.1;
					} else {
						p[optimize_index] += dp[optimize_index];
						dp[optimize_index] *= 0.9;
					}
					twiddle_state = Done;
					break;
				case Done:
					optimize_index = (optimize_index+1) % p.size();
					twiddle_state = Uninitialized; 
					update_count = 0;
					total_error = 0;
					break;
			}

			Kp = p[0];
    		Kd = p[1];
			Ki = p[2];
			// p_error = 0.0;
			// i_error = 0.0;
			// d_error = 0.0;
	}
}



