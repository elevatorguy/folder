#ifndef ATG_SIMPLE_2D_CONSTRAINT_SOLVER_UTILITIES_H
#define ATG_SIMPLE_2D_CONSTRAINT_SOLVER_UTILITIES_H

#if defined(__APPLE__)
#define scs_force_inline inline
#else
#define scs_force_inline __forceinline
#endif

void allocateArray(double *&data, int num_elements);
void allocateArray(int *&data, int num_elements);

void freeArray(double *&data);
void freeArray(int *&data);

#endif /* ATG_SIMPLE_2D_CONSTRAINT_SOLVER_UTILITIES_H */
