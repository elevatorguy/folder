#ifndef ATG_SIMPLE_2D_CONSTRAINT_ROTATION_FRICTION_CONSTRAINT_H
#define ATG_SIMPLE_2D_CONSTRAINT_ROTATION_FRICTION_CONSTRAINT_H

#include "system_state.h"
#include "matrix.h"
#include "utilities.h"

#include <cfloat>

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

static constexpr int MaxConstraintCount = 3;
static constexpr int MaxBodyCount = 2;

struct Output {
    double C[MaxConstraintCount];
    double J[MaxConstraintCount][3 * MaxBodyCount];
    double J_dot[MaxConstraintCount][3 * MaxBodyCount];
    double v_bias[MaxConstraintCount];
    double limits[MaxConstraintCount][2];
    double ks[MaxConstraintCount];
    double kd[MaxConstraintCount];
};

void init_Constraint(int constraintCount, int bodyCount);

scs_force_inline int getConstraintCount() const { return m_constraintCount; }

int m_index;
int m_bodyCount;
RigidBody *m_bodies[MaxBodyCount];

double F_x[MaxConstraintCount][MaxBodyCount];
double F_y[MaxConstraintCount][MaxBodyCount];
double F_t[MaxConstraintCount][MaxBodyCount];

inline void noLimits(Output *output) {
    for (int i = 0; i < MaxConstraintCount; ++i) {
        output->limits[i][0] = -DBL_MAX;
        output->limits[i][1] = DBL_MAX;
    }
}

int m_constraintCount;

void init_RotationFrictionConstraint();

void setBody(RigidBody *body) { m_bodies[0] = body; }

virtual void calculate(Output *output, SystemState *system);

double m_maxTorque;
double m_minTorque;

double m_ks;
double m_kd;

#endif /* ATG_SIMPLE_2D_CONSTRAINT_ROTATION_FRICTION_CONSTRAINT_H */
