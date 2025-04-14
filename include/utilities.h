#ifndef ATG_SIMPLE_2D_CONSTRAINT_SOLVER_UTILITIES_H
#define ATG_SIMPLE_2D_CONSTRAINT_SOLVER_UTILITIES_H

#if defined(__APPLE__)
#define scs_force_inline inline
#else
#define scs_force_inline __forceinline
#endif

namespace atg_scs {
    void freeArray(double *&data);
    void freeArray(int *&data);
} /* atg_scs */

#endif /* ATG_SIMPLE_2D_CONSTRAINT_SOLVER_UTILITIES_H */
#include "../include/utilities.h"

void atg_scs::freeArray(double *&data) {
    delete[] data;
    data = nullptr;
}

void atg_scs::freeArray(int *&data) {
    delete[] data;
    data = nullptr;
}
