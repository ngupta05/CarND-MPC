#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>

using namespace std;

class MPC {
	vector<double> mpc_x, mpc_y;
 public:
  MPC();
  virtual ~MPC();

  vector<double> get_mpc_x() { return mpc_x; }
  vector<double> get_mpc_y() {return mpc_y; }

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
