#ifndef ATG_SIMPLE_2D_CONSTRAINT_SOLVER_MATRIX_H
#define ATG_SIMPLE_2D_CONSTRAINT_SOLVER_MATRIX_H

#include "utilities.h"
#include <assert.h>

void init_Matrix(void);
void init_Matrix(int width, int height, double value = 0.0);
void deinit_Matrix(void);

void initialize(int width, int height, double value);
void initialize(int width, int height);
void resize(int width, int height);
void destroy(void);

void set(const double *data);

scs_force_inline void set(int column, int row, double value) {
    assert(column >= 0 && column < m_width);
    assert(row >= 0 && row < m_height);

    m_matrix[row][column] = value;
}

scs_force_inline void add(int column, int row, double value) {
    assert(column >= 0 && column < m_width);
    assert(row >= 0 && row < m_height);

    m_matrix[row][column] += value;
}

scs_force_inline double get(int column, int row) {
    assert(column >= 0 && column < m_width);
    assert(row >= 0 && row < m_height);

    return m_matrix[row][column];
}

void set(Matrix *reference);

void multiply(Matrix &b, Matrix *target);
void componentMultiply(Matrix &b, Matrix *target);
void transposeMultiply(Matrix &b, Matrix *target);
void leftScale(Matrix &scale, Matrix *target);
void rightScale(Matrix &scale, Matrix *target);
void scale(double s, Matrix *target);
void subtract(Matrix &b, Matrix *target);
void add(Matrix &b, Matrix *target);
void negate(Matrix *target);
bool equals(Matrix &b, double err = 1e-6);
double vectorMagnitudeSquared() const;
double dot(Matrix &b) const;

void madd(Matrix &b, double s);
void pmadd(Matrix &b, double s);

void transpose(Matrix *target);
int getWidth(void) const { return m_width; }
int getHeight(void) const { return m_height; }

scs_force_inline void fastRowSwap(int a, int b) {
    double *temp = m_matrix[a];
    m_matrix[a] = m_matrix[b];
    m_matrix[b] = temp;
}

double **m_matrix;
double *m_data;
int m_width;
int m_height;
int m_capacityWidth;
int m_capacityHeight;

#endif /* ATG_SIMPLE_2D_CONSTRAINT_SOLVER_MATRIX_H */
