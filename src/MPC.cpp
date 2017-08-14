#include "MPC.h"

#include <cmath>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>

#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

double polyeval(Eigen::VectorXd coeffs, double x);

// TODO: Set the timestep length and duration
constexpr size_t N = 10;
constexpr double dt = 0.1;

constexpr size_t Nstate = 6;
constexpr size_t Nactu  = 2;

// Essentially:
// vars = [ x0, x1, x2, ..., xN,
//          y0, y1, y2, ..., yN,
//          psi0, psi1, ..., psiN,
//          v0, v1, v2, ..., vN,
//          ...,
//          speed0, speed1, ..., speed(N-1),
//          steer0, steer1, ..., steer(N-1),
//        ]
// The following writes down all the (x,y,psi,v,...)0 positions in vars.
constexpr size_t x_first = 0;
constexpr size_t y_first = x_first + N;
constexpr size_t psi_first = y_first + N;
constexpr size_t v_first = psi_first + N;
constexpr size_t cte_first = v_first + N;
constexpr size_t eps_first = cte_first + N;

constexpr size_t delta_first = eps_first + N;
constexpr size_t a_first = delta_first + N - 1;

constexpr double Vref = 60.;


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

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // TODO: implement MPC
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.
    
    // fg = [ cost,
    //        x0 constraint, x1 constraint, x2 constraint, x3 constraint, ...,
    //        y0 constraint, y1 constraint, ...,
    //        ...,
    //      ]
    //
    // > Finally, fg[1 + psi_start + t] is reserved for the t-th of N values of ψ that the solver computes.
    
    //
    // Calculate cost -> fg[0]
    //
    fg[0] = 0;
    
    // Punish large CTE and Epsi, and deviation from reference velocity
    for (int t = 0; t < N; t++) {
      fg[0] += CppAD::pow(vars[cte_first + t], 2);
      fg[0] += CppAD::pow(vars[eps_first + t], 2);
      fg[0] += CppAD::pow(vars[v_first + t] - Vref, 2);
    }
    
    // Punish large actuator movements (hard throttle, turns)
    for (int t = 0; t < N - 1; t++) {
      fg[0] += CppAD::pow(vars[delta_first + t], 2);
      fg[0] += CppAD::pow(vars[a_first + t], 2);
    }
    
    // Punish large differences between consecutive actuator movements
    for (int t = 0; t < N - 2; t++) {
      fg[0] += 20 * CppAD::pow(vars[delta_first + t + 1] - vars[delta_first + t], 2);
      fg[0] += 20 * CppAD::pow(vars[a_first + t + 1] - vars[a_first + t], 2);
    }
    // TL;DR: make a huge 2d matrix of initial states, and auto-solvable functions for further states.
    
    fg[x_first   + 1] = vars[x_first];
    fg[y_first   + 1] = vars[y_first];
    fg[psi_first + 1] = vars[psi_first];
    fg[v_first   + 1] = vars[v_first];
    fg[cte_first + 1] = vars[cte_first];
    fg[eps_first + 1] = vars[eps_first];
    
    
    // Type down the function x1(x0) in a form that fits the solver.
    // If x1 = x0 + v dt, then it fits the solver better to find x1 - (x0 + v dt) = 0, and make it solve for x1.
    for (int t = 1; t < N; t++) {
      AD<double> x_1   = vars[x_first + t];
      AD<double> y_1   = vars[y_first + t];
      AD<double> psi_1 = vars[psi_first + t];
      AD<double> v_1   = vars[v_first + t];
      AD<double> cte_1 = vars[cte_first + t];
      AD<double> eps_1   = vars[eps_first + t];
      
      
      AD<double> x_0   = vars[x_first + t-1];
      AD<double> y_0   = vars[y_first + t-1];
      AD<double> psi_0 = vars[psi_first + t-1];
      AD<double> v_0   = vars[v_first + t-1];
      AD<double> cte_0 = vars[cte_first + t-1];
      AD<double> eps_0 = vars[eps_first + t-1];
      
      AD<double> delta_0   = vars[delta_first + t-1];
      AD<double> a_0   = vars[a_first + t-1];
      
      AD<double> f_0   = coeffs[0] + coeffs[1] * x_0 + coeffs[2] * x_0 * x_0 + coeffs[3] * x_0 * x_0 * x_0;
      AD<double> psi_desired = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x_0 + 3 * coeffs[3] * x_0 * x_0);
      
      // 0 = x1 - (x0 + v0 cos(psi0) dt);
      fg[x_first + t+1] = x_1 - (x_0 + v_0 * CppAD::cos(psi_0) * dt);
      // 0 = y1 - (x0 + v0 cos(psi0) dt);
      fg[y_first + t+1] = y_1 - (y_0 + v_0 * CppAD::sin(psi_0) * dt);
      // 0 = psi1 - psi0 + v0/Lf deltat dt
      fg[psi_first + t+1] = psi_1 - (psi_0 + v_0 / Lf * delta_0 * dt);
      // 0 = v1 - at dt
      fg[v_first + t+1] = v_1 - (v_0 + a_0 * dt);
      // 0 = cte1 - (f(x0) - y0 + v0 * sin(epsi0) * dt)
      fg[cte_first + t+1] = cte_1 - ( (f_0 - y_0) + v_0 * CppAD::sin(eps_0) * dt);
      // 0 = psi0 - psidesired0 + v0 delta0/Lf dt
      fg[eps_first + t+1] = eps_1 - (psi_0 - psi_desired + v_0 * delta_0 / Lf * dt);
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

/**
* Predict the car's state in dt seconds.
* Takes into account:
* @param v the car's velocity
* @param cte the cross track error
* @param epsi the psi error
* @param delta the steering wheel's position
* @param a the throttle+brake pos
* @param coeffs the waypoint polyfit coefficients
* @param dt the time in seconds
* @returns an Eigen::VectorXd of:
*          x, y, psi, v, cte, epsi
*/

Solution MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // TODO: Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  size_t n_vars = Nstate * N + Nactu * (N-1);
  // TODO: Set the number of constraints
  size_t n_constraints = n_vars;
 
  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }
  
  
  // Initial state. Predict 100ms ahead for delays.
  
  vars[x_first] = state[0];
  vars[y_first] = state[1];
  vars[psi_first] = state[2];
  vars[v_first] = state[3];
  vars[cte_first] = state[4];
  vars[eps_first] = state[5];

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  
  // TODO: Set lower and upper limits for variables.
  
  // Variables can be anything
  for (int i = 0; i < delta_first; i++) {
    vars_lowerbound[i] = - 1e20;
    vars_upperbound[i] =   1e20;
  }
  
  // Delta can be -25..25deg --> -..+ 0.43633 rad
  for (int i = delta_first; i < a_first; i++) {
    vars_lowerbound[i] = - 0.436332;
    vars_upperbound[i] =   0.436332;
  }
  
  // Acceleration can be -1..1
  for (int i = a_first; i < n_vars; i++) {
    vars_lowerbound[i] = - 1;
    vars_upperbound[i] =   1;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
  
  // Initial state limits should be themselves.
  constraints_lowerbound[x_first]     = constraints_upperbound[x_first]   = state[0];
  constraints_lowerbound[y_first]     = constraints_upperbound[y_first]   = state[1];
  constraints_lowerbound[psi_first]   = constraints_upperbound[psi_first] = state[2];
  constraints_lowerbound[v_first]     = constraints_upperbound[v_first]   = state[3];
  constraints_lowerbound[cte_first] = constraints_upperbound[cte_first] = state[4];
  constraints_lowerbound[eps_first] = constraints_upperbound[eps_first] = state[5];
  

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
  options += "Numeric max_cpu_time          0.1\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;
  
  
  // TODO: Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.
  
  Solution sol;
  
  sol.accel = solution.x[a_first];
  sol.steer = solution.x[delta_first];
  
  for (int i = 0; i < N-1; i++) {
    sol.x_pred.push_back(solution.x[x_first + i]);
    sol.y_pred.push_back(solution.x[y_first + i]);
  }
  
  return sol;
}
