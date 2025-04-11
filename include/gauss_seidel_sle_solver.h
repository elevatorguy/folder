#ifndef ATG_SIMPLE_2D_CONSTRAINT_SOLVER_SEIDEL_SLE_SOLVER_H
#define ATG_SIMPLE_2D_CONSTRAINT_SOLVER_SEIDEL_SLE_SOLVER_H

#include "matrix.h"
#include "sparse_matrix.h"

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

bool m_supportsLimits;

GaussSeidelSleSolver();
virtual ~GaussSeidelSleSolver();

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

int m_maxIterations;
double m_minDelta;

double solveIteration(
        Matrix &left,
        Matrix &right,
        Matrix *result,
        Matrix *previous);
double solveIteration(
        Matrix &left,
        Matrix &right,
        Matrix &limits,
        Matrix *result,
        Matrix *previous);

Matrix m_M;
SparseMatrix<3> m_reg;

#endif /* ATG_SIMPLE_2D_CONSTRAINT_SOLVER_SEIDEL_SLE_SOLVER_H */
