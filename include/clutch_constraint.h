#ifndef ATG_SIMPLE_2D_CONSTRAINT_SOLVER_CLUTCH_CONSTRAINT_H
#define ATG_SIMPLE_2D_CONSTRAINT_SOLVER_CLUTCH_CONSTRAINT_H

#include "constraint.h"

namespace atg_scs {
    class ClutchConstraint : public Constraint {
        public:
            ClutchConstraint();
            virtual ~ClutchConstraint();
            
            void setBody1(RigidBody *body) { m_bodies[0] = body; }
            void setBody2(RigidBody *body) { m_bodies[1] = body; }

            virtual void calculate(Output *output, SystemState *system);

            double m_maxTorque;
            double m_minTorque;

            double m_ks;
            double m_kd;
    };
} /* namespace atg_scs */

#endif /* ATG_SIMPLE_2D_CONSTRAINT_SOLVER_CLUTCH_CONSTRAINT_H */
#include "../include/clutch_constraint.h"

#include <cmath>
#include <cfloat>

atg_scs::ClutchConstraint::ClutchConstraint() : Constraint(1, 2) {
    m_ks = 10.0;
    m_kd = 1.0;

    m_maxTorque = DBL_MAX;
    m_minTorque = -DBL_MAX;
}

atg_scs::ClutchConstraint::~ClutchConstraint() {
    /* void */
}

void atg_scs::ClutchConstraint::calculate(
        Output *output,
        SystemState *state)
{
    output->C[0] = 0;

    output->J[0][0] = 0.0;
    output->J[0][1] = 0.0;
    output->J[0][2] = -1.0;

    output->J[0][3] = 0.0;
    output->J[0][4] = 0.0;
    output->J[0][5] = 1.0;

    output->J_dot[0][0] = 0;
    output->J_dot[0][1] = 0;
    output->J_dot[0][2] = 0;

    output->J_dot[0][3] = 0;
    output->J_dot[0][4] = 0;
    output->J_dot[0][5] = 0;

    output->kd[0] = m_kd;
    output->ks[0] = m_ks;

    output->v_bias[0] = 0;

    output->limits[0][0] = m_minTorque;
    output->limits[0][1] = m_maxTorque;
}
