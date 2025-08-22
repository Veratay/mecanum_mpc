#ifndef MPCC_SOLVER_H
#define MPCC_SOLVER_H

#include "f.h"
#include <memory>

#define casadi_real double

class Solver {
private:
    casadi_int sz_arg;
    casadi_int sz_res;
    casadi_int sz_iw;
    casadi_int sz_w;

    std::unique_ptr<const double*[]> arg;
    std::unique_ptr<double*[]> res;
    std::unique_ptr<casadi_int[]> iw;
    std::unique_ptr<double[]> w;

    int mem;

public:
    Solver();
    ~Solver();

    int solve(casadi_real target[], casadi_real x0[], casadi_real u_last[], casadi_real p[], casadi_real x_out[], casadi_real u_out[]);
};

#endif // SOLVER_H
