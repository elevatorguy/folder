#ifndef ATG_SIMPLE_2D_CONSTRAINT_SOLVER_SLE_SOLVER_H
#define ATG_SIMPLE_2D_CONSTRAINT_SOLVER_SLE_SOLVER_H

#include "matrix.h"
#include "sparse_matrix.h"

class SleSolver {
    public:
        SleSolver(bool supportsLimits);

        virtual bool solve(
                SparseMatrix<3> &J,
                Matrix &W,
                Matrix &right,
                Matrix *result,
                Matrix *previous);
        virtual bool solveWithLimits(
                SparseMatrix<3> &J,
                Matrix &W,
                Matrix &right,
                Matrix &limits,
                Matrix *result,
                Matrix *previous);

        bool supportsLimits() const { return m_supportsLimits; }

    private:
        bool m_supportsLimits;
};

#endif /* ATG_SIMPLE_2D_CONSTRAINT_SOLVER_SLE_SOLVER_H */
