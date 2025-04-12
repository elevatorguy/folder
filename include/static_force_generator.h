#ifndef ATG_SIMPLE_2D_CONSTRAINT_SOLVER_STATIC_FORCE_GENERATOR_H
#define ATG_SIMPLE_2D_CONSTRAINT_SOLVER_STATIC_FORCE_GENERATOR_H

#include "system_state.h"

virtual void apply(SystemState *system) = 0;

int m_index;

struct RigidBody {
    public:
        RigidBody();

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

        void reset();
        double energy() const;
};

void init_StaticForceGenerator();

virtual void apply(SystemState *state);

void setForce(double f_x, double f_y);
void setPosition(double p_x, double p_y);

double m_f_x;
double m_f_y;

double m_p_x;
double m_p_y;

RigidBody *m_body;

#endif /* ATG_SIMPLE_2D_CONSTRAINT_SOLVER_STATIC_FORCE_GENERATOR_H */
