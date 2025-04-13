#ifndef ATG_SIMPLE_2D_CONSTRAINT_SOLVER_SPRING_H
#define ATG_SIMPLE_2D_CONSTRAINT_SOLVER_SPRING_H

#include "system_state.h"

virtual void apply(SystemState *system) = 0;

int m_index;

struct RigidBody {
    public:
        RigidBody(void);

        void localToWorld(double x, double y, double *w_x, double *w_y);
        void worldToLocal(double x, double y, double *l_x, double *l_y);

        double p_x;
        double p_y;

        double v_x;
        double v_y;

        double theta;
        double v_theta;

        double m;
        double I;

        int index;

        void reset(void);
        double energy() const;
};

void init_Spring(void);

virtual void apply(SystemState *state);

void getEnds(double *x_1, double *y_1, double *x_2, double *y_2);
double energy(void) const;

double m_restLength;
double m_ks;
double m_kd;

double m_p1_x;
double m_p1_y;

double m_p2_x;
double m_p2_y;

RigidBody *m_body1;
RigidBody *m_body2;

#endif /* ATG_SIMPLE_2D_CONSTRAINT_SOLVER_SPRING_H */
