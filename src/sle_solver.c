#include "sle_solver.h"

SleSolver::SleSolver(bool supportsLimits) {
    m_supportsLimits = supportsLimits;
}

bool SleSolver::solve(
        SparseMatrix<3> &J,
        Matrix &W,
        Matrix &right,
        Matrix *result,
        Matrix *previous)
{
    return false;
}

bool SleSolver::solveWithLimits(
        SparseMatrix<3> &J,
        Matrix &W,
        Matrix &right,
        Matrix &limits,
        Matrix *result,
        Matrix *previous)
{
    return false;
}
