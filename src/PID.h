#ifndef PID_H
#define PID_H

#include <vector>
using namespace std;

class PID {
private:
  double prev_cte;
  /*
  * Twiddle
  */
  double best_error;
  int optimize_index;
  int update_count;
  double total_error;
  enum {
   Uninitialized,
   AddDP,
   MinusDP,
   Done
  } twiddle_state; 
  vector<double> dp;

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
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
  
  /*
  * Twiddle algorithm, optimize the parameters Kp, Ki, Kd
  */
  void Twiddle();
};

#endif /* PID_H */
