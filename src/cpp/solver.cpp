#include "solver.hpp"
#include <cassert>
#include <chrono>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <stdio.h>

Solver::Solver() {
    casadi_int n_in = f_n_in();
    casadi_int n_out = f_n_out();

    /* Get sizes of the required work vectors */
    casadi_int sz_arg = n_in, sz_res = n_out, sz_iw = 0, sz_w = 0;

    if (f_work(&sz_arg, &sz_res, &sz_iw, &sz_w))
        throw std::runtime_error("Failed to get sizes of work vectors");

    /* Allocate input/output buffers and work vectors*/
    this->arg = std::make_unique<const double *[]>(sz_arg);
    this->res = std::make_unique<double *[]>(sz_res);
    this->iw = std::make_unique<casadi_int[]>(sz_iw);
    this->w = std::make_unique<double[]>(sz_w);

    this->sz_arg = sz_arg;
    this->sz_res = sz_res;
    this->sz_iw = sz_iw;
    this->sz_w = sz_w;

    f_incref();

    this->mem = f_checkout();
}

Solver::~Solver() {
    if(this->mem) f_release(this->mem);
    f_decref();
}

int Solver::solve(
        casadi_real target[],
        casadi_real x0[],
        casadi_real u_last[],
        casadi_real p[],
        casadi_real x_out[],
        casadi_real u_out[]
        ) {
    this->arg[0] = target;
    this->arg[1] = x0;
    this->arg[2] = u_last;
    this->arg[3] = p;

    this->res[0] = x_out;
    this->res[1] = u_out;

    f(this->arg.get(), this->res.get(), this->iw.get(), this->w.get(), this->mem);

    return 0;
}

extern "C" {
    typedef void* SolverHandle;

    SolverHandle solver_create() {
        return new Solver();
    }

    void solver_destroy(SolverHandle handle) {
        delete static_cast<Solver*>(handle);
    }

    // Solve function
    int solver_solve(SolverHandle handle,
                     double target[], double x0[], double u_last[],
                     double p[], double x_out[], double u_out[]) {
        Solver* solver = static_cast<Solver*>(handle);
        return solver->solve(target, x0, u_last, p, x_out, u_out);
    }
}
