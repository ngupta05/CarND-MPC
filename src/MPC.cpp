#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// TODO: Set the timestep length and duration
size_t N = 10;
double dt = 0.4;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;
const double ref_v = 35 * 1609.34/3600; // m/s

// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we should to establish
// when one variable starts and another ends to make our lifes easier.
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;

class FG_eval {
public:
	// Fitted polynomial coefficients
	Eigen::VectorXd coeffs;
	FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

	typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
	void operator()(ADvector& fg, const ADvector& vars) {
		fg[0] = 0;


		fg[1 + x_start] = vars[x_start];
		fg[1 + y_start] = vars[y_start];
		fg[1 + psi_start] = vars[psi_start];
		fg[1 + v_start] = vars[v_start];
		fg[1 + cte_start] = vars[cte_start];
		fg[1 + epsi_start] = vars[epsi_start];

		// The rest of the constraints
		for (unsigned int t = 1; t < N; t++) {
			AD<double> x1 = vars[x_start + t];
			AD<double> x0 = vars[x_start + t - 1];

			AD<double> y1 = vars[y_start + t];
			AD<double> y0 = vars[y_start + t - 1];

			AD<double> psi1 = vars[psi_start + t];
			AD<double> psi0 = vars[psi_start + t - 1];

			AD<double> v1 = vars[v_start + t];
			AD<double> v0 = vars[v_start + t - 1];

			AD<double> cte1 = vars[cte_start + t];
			AD<double> cte0 = vars[cte_start + t - 1];

			AD<double> epsi1 = vars[epsi_start + t];
			AD<double> epsi0 = vars[epsi_start + t - 1];

			AD<double> a0 = vars[a_start + t -1];
			AD<double> delta0 = vars[delta_start + t -1];

			AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * CppAD::pow(x0, 2);
			AD<double> psides0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0);

			fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
			fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
			fg[1 + psi_start + t] = psi1 - (psi0 + (v0/Lf) * delta0 * dt);
			fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
			fg[1 + cte_start + t] = cte1 - ((f0 - y0) + v0 * dt * CppAD::sin(epsi0));
			fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) + (v0/Lf)*dt*delta0);
		}

		// Add to cost
		for (unsigned int t = 0; t < N; t++) {
			fg[0] += 100 * CppAD::pow(vars[cte_start + t], 2);
			fg[0] += 100 * CppAD::pow(vars[epsi_start + t], 2);
			fg[0] += 0.2 * CppAD::pow(vars[v_start + t] - ref_v, 2);
		}

		// Cost to smoothen out transition
		for (unsigned int t = 0; t < N - 1; t++) {
			fg[0] += CppAD::pow(vars[delta_start + t], 2);
			fg[0] += CppAD::pow(vars[a_start + t], 2);
		}

		// Cost to smoothen out changes in actuations
		for (unsigned int t = 0; t < N - 2; t++) {
			fg[0] += 10 * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2); // TODO
			fg[0] += 10 * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2); // TODO
		}
	}
};

//
// MPC class definition implementation.
//
MPC::MPC() {
}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
	typedef CPPAD_TESTVECTOR(double) Dvector;

	double x = state[0];
	double y = state[1];
	double psi = state[2];
	double v = state[3];
	double cte = state[4];
	double epsi = state[5];

	size_t n_vars = N * 6 + (N - 1) * 2;
	// Number of constraints
	size_t n_constraints = N * 6;

	// Initial value of the independent variables.
	// Should be 0 except for the initial values.
	Dvector vars(n_vars);
	for (unsigned int i = 0; i < n_vars; i++) {
		vars[i] = 0.0;
	}
	// Set the initial variable values
	vars[x_start] = x;
	vars[y_start] = y;
	vars[psi_start] = psi;
	vars[v_start] = v;
	vars[cte_start] = cte;
	vars[epsi_start] = epsi;

	// Lower and upper limits for x
	Dvector vars_lowerbound(n_vars);
	Dvector vars_upperbound(n_vars);

	// Set all non-actuators upper and lowerlimits
	// to the max negative and positive values.
	for (unsigned int i = 0; i < delta_start; i++) {
		vars_lowerbound[i] = -1.0e19;
		vars_upperbound[i] = 1.0e19;
	}

	// The upper and lower limits of delta are set to -25 and 25
	// degrees (values in radians).
	// NOTE: Feel free to change this to something else.
	for (unsigned int i = delta_start; i < a_start; i++) {
		vars_lowerbound[i] = -0.436332;
		vars_upperbound[i] = 0.436332;
	}

	// Acceleration/decceleration upper and lower limits.
	// NOTE: Feel free to change this to something else.
	for (unsigned int i = a_start; i < n_vars; i++) {
		vars_lowerbound[i] = -1.0;
		vars_upperbound[i] = 1.0;
	}

	// Lower and upper limits for constraints
	// All of these should be 0 except the initial
	// state indices.
	Dvector constraints_lowerbound(n_constraints);
	Dvector constraints_upperbound(n_constraints);
	for (unsigned int i = 0; i < n_constraints; i++) {
		constraints_lowerbound[i] = 0;
		constraints_upperbound[i] = 0;
	}
	constraints_lowerbound[x_start] = x;
	constraints_lowerbound[y_start] = y;
	constraints_lowerbound[psi_start] = psi;
	constraints_lowerbound[v_start] = v;
	constraints_lowerbound[cte_start] = cte;
	constraints_lowerbound[epsi_start] = epsi;

	constraints_upperbound[x_start] = x;
	constraints_upperbound[y_start] = y;
	constraints_upperbound[psi_start] = psi;
	constraints_upperbound[v_start] = v;
	constraints_upperbound[cte_start] = cte;
	constraints_upperbound[epsi_start] = epsi;


	// object that computes objective and constraints
	FG_eval fg_eval(coeffs);

	//
	// NOTE: You don't have to worry about these options
	//
	// options for IPOPT solver
	std::string options;
	// Uncomment this if you'd like more print information
	options += "Integer print_level  0\n";
	// NOTE: Setting sparse to true allows the solver to take advantage
	// of sparse routines, this makes the computation MUCH FASTER. If you
	// can uncomment 1 of these and see if it makes a difference or not but
	// if you uncomment both the computation time should go up in orders of
	// magnitude.
	options += "Sparse  true        forward\n";
	options += "Sparse  true        reverse\n";
	// NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
	// Change this as you see fit.
	options += "Numeric max_cpu_time          0.5\n";

	// place to return solution
	mpc_x.clear();
	mpc_y.clear();
	CppAD::ipopt::solve_result<Dvector> solution;

	// solve the problem
	CppAD::ipopt::solve<Dvector, FG_eval>(
			options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
			constraints_upperbound, fg_eval, solution);

	// Check some of the solution values
	bool ok = solution.status == CppAD::ipopt::solve_result<Dvector>::success;

	mpc_x = vector<double>(&(solution.x[x_start]), &(solution.x[x_start]) + N);
	mpc_y = vector<double>(&(solution.x[y_start]), &(solution.x[y_start]) + N);

	// Cost
	// auto cost = solution.obj_value;
	if (ok) {
		return {solution.x[x_start + N - 1],   solution.x[y_start + N - 1],
			solution.x[psi_start + 1], solution.x[v_start + 1],
			solution.x[cte_start + 1], solution.x[epsi_start + 1],
			solution.x[delta_start],   solution.x[a_start]};
	} else {
		return {};
	}
}
