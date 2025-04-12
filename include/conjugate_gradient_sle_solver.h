#ifndef ATG_SIMPLE_2D_CONSTRAINT_SOLVER_CONJUGATE_GRADIENT_SLE_SOLVER_H
#define ATG_SIMPLE_2D_CONSTRAINT_SOLVER_CONJUGATE_GRADIENT_SLE_SOLVER_H

#include "matrix.h"
#include "sparse_matrix.h"

void init_SleSolver(bool supportsLimits);

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

bool m_supportsLimits;

void init_ConjugateGradientSleSolver();
virtual ~ConjugateGradientSleSolver();

virtual bool solve(
        SparseMatrix<3> &J,
        Matrix &W,
        Matrix &right,
        Matrix *result,
        Matrix *previous);

void setMaxIterations(int maxIterations) { m_maxIterations = maxIterations; }
int getMaxIterations() const { return m_maxIterations; }

void setMaxError(double maxError) { m_maxError = maxError; }
double getMaxError() const { return m_maxError; }

void setMinError(double minError) { m_minError = minError; }
double getMinError() const { return m_minError; }

void multiply(SparseMatrix<3> &J, Matrix &W, Matrix &x, Matrix *target);
bool sufficientlySmall(Matrix &x, Matrix &target) const;

Matrix
    m_mreg0,
    m_mreg1,
    m_Ap,
    m_x,
    m_r,
    m_p,
    m_A;

int m_maxIterations;
double m_maxError;
double m_minError;

#endif /* ATG_SIMPLE_2D_CONSTRAINT_SOLVER_CONJUGATE_GRADIENT_SLE_SOLVER_H */
