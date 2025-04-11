#ifndef ATG_SIMPLE_2D_CONSTRAINT_SOLVER_GAUSSIAN_ELIMINATION_SLE_SOLVER_H
#define ATG_SIMPLE_2D_CONSTRAINT_SOLVER_GAUSSIAN_ELIMINATION_SLE_SOLVER_H

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

#include "utilities.h"

class GaussianEliminationSleSolver {
    public:
        GaussianEliminationSleSolver();
        virtual ~GaussianEliminationSleSolver();

        virtual bool solve(
                SparseMatrix<3> &J,
                Matrix &W,
                Matrix &right,
                Matrix *result,
                Matrix *previous);

        static scs_force_inline double fastAbs(double v) {
            return (v > 0)
                ? v
                : -v;
        }

    protected:
        Matrix m_a;
        Matrix m_M;
        SparseMatrix<3> m_reg;
};

#endif /* ATG_SIMPLE_2D_CONSTRAINT_SOLVER_GAUSSIAN_ELIMINATION_SLE_SOLVER_H */
