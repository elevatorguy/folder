#ifndef ATG_SIMPLE_2D_CONSTRAINT_SOLVER_SYSTEM_STATE_H
#define ATG_SIMPLE_2D_CONSTRAINT_SOLVER_SYSTEM_STATE_H

void init_SystemState(void);
void deinit_SystemState(void);

void copy(const SystemState *state);
void resize(int bodyCount, int constraintCount);
void destroy(void);

void localToWorld(double x, double y, double *x_t, double *y_t, int body);
void velocityAtPoint(double x, double y, double *v_x, double *v_y, int body);
void applyForce(double x_l, double y_l, double f_x, double f_y, int body);

int *indexMap;

double *a_theta;
double *v_theta;
double *theta;

double *a_x;
double *a_y;
double *v_x;
double *v_y;
double *p_x;
double *p_y;

double *f_x;
double *f_y;
double *t;

double *r_x;
double *r_y;
double *r_t;

double *m;

int n;
int n_c;
double dt;

#endif /* ATG_SIMPLE_2D_CONSTRAINT_SOLVER_SYSTEM_STATE_H */
