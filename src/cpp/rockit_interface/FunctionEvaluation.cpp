/*
 * Fatrop - A fast trajectory optimization solver
 *  Copyright (C) 2022 - 2024 Lander Vanroye, KU Leuven. All rights reserved.
 *
 * This file is part of Fatrop.
 *
 * Fatrop is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Fatrop is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with Fatrop.  If not, see <http://www.gnu.org/licenses/>. */
#include "FunctionEvaluation.hpp"
#include <blasfeo.h>
#include <cassert>
using namespace fatrop;
using namespace std;

#define casadi_int int

fatrop_int EvalBase::eval_bf(const double **arg, MAT *bf_mat)
{
//#if DEBUG
    assert(bf_mat->m >= out_m);
    assert(bf_mat->n >= out_n);
    assert(bf_mat->m == out_m);
    assert(bf_mat->n == out_n);
//#endif
    #ifndef ENABLE_MULTITHREADING
    double *buffer_p = buffer.data();

    #else
    double *buffer_p = buffer[omp_get_thread_num()].data();
    #endif
    // todo make this static polymorphism using CRTP
    fatrop_int res = eval_buffer(arg);

    double *sparse_out = buffer.data(); 

    const bool is_dense = (out_nnz == out_m * out_n);

    std::vector<double> dense_buf;
    dense_buf.assign(out_m * out_n, 0.0);
    double *dense = dense_buf.data();

    if (true) {
        // CasADi sometimes still labels as “sparse” but nnz==m*n. Just memcpy.
        std::memcpy(dense, sparse_out, sizeof(double) * (size_t)out_m * (size_t)out_n);
    } else {
        // CasADi CSC layout encoded in sparsity_out:
        // [0]=nrow, [1]=ncol, [2..2+ncol]=colptr (ncol+1 entries), then rowidx (nnz entries)
        const casadi_int *sp = sparsity_out.data();
        const casadi_int nrow = sp[0];
        const casadi_int ncol = sp[1];
        const casadi_int *colptr = sp + 2;
        const casadi_int *rowidx = sp + 2 + ncol + 1;

        // Safety
        if (nrow != out_m || ncol != out_n) {
            fprintf(stderr, "[EvalBase::eval_bf] sparsity dims mismatch: sp=(%d x %d) out=(%d x %d)\n",
                    (int)nrow, (int)ncol, out_m, out_n);
            return -3;
        }

        // Fill dense in column-major
        for (casadi_int j = 0; j < ncol; ++j) {
            for (casadi_int k = colptr[j]; k < colptr[j+1]; ++k) {
                const casadi_int i = rowidx[k];
                // column-major index
                dense[(size_t)i + (size_t)j * (size_t)out_m] = sparse_out[k];
            }
        }
    }

    // Final pack to BLASFEO
    // dense is column-major, leading dimension out_m
    blasfeo_pack_dmat(out_m, out_n, dense, out_m, bf_mat, 0, 0);
    return res;
}

fatrop_int EvalBase::eval_array(const double **arg, double *array)
{
    #ifndef ENABLE_MULTITHREADING
    double *buffer_p = buffer.data();
    #else
    double *buffer_p = buffer[omp_get_thread_num()].data();
    #endif
    // todo make this static polymorphism using CRTP
    fatrop_int res = eval_buffer(arg);
    memcpy(array, buffer_p, out_nnz * sizeof(double));
    return res;
}
