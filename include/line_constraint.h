#ifndef ATG_SIMPLE_2D_CONSTRAINT_SOLVER_LINE_CONSTRAINT_H
#define ATG_SIMPLE_2D_CONSTRAINT_SOLVER_LINE_CONSTRAINT_H

#include "constraint.h"

namespace atg_scs {
    class LineConstraint : public Constraint {
        public:
            LineConstraint();
            virtual ~LineConstraint();
            
            void setBody(RigidBody *body) { m_bodies[0] = body; }

            virtual void calculate(Output *output, SystemState *system);

            double m_local_x;
            double m_local_y;
            double m_p0_x;
            double m_p0_y;
            double m_dx;
            double m_dy;

            double m_ks;
            double m_kd;
    };
} /* namespace atg_scs */

#endif /* ATG_SIMPLE_2D_CONSTRAINT_SOLVER_LINE_CONSTRAINT_H */
#include "../include/line_constraint.h"

#include <cmath>

atg_scs::LineConstraint::LineConstraint() : Constraint(1, 1) {
    m_local_x = m_local_y = 0.0;
    m_p0_x = m_p0_y = 0.0;
    m_dx = m_dy = 0.0;
    m_ks = 10.0;
    m_kd = 1.0;
}

atg_scs::LineConstraint::~LineConstraint() {
    /* void */
}

void atg_scs::LineConstraint::calculate(
        Output *output,
        SystemState *state)
{
    const int body = m_bodies[0]->index;

    const double q1 = state->p_x[body];
    const double q2 = state->p_y[body];
    const double q3 = state->theta[body];

    const double q3_dot = state->v_theta[body];

    const double cos_q3 = std::cos(q3);
    const double sin_q3 = std::sin(q3);

    const double bodyX = q1 + cos_q3 * m_local_x - sin_q3 * m_local_y;
    const double bodyY = q2 + sin_q3 * m_local_x + cos_q3 * m_local_y;

    const double perpX = -m_dy;
    const double perpY = m_dx;

    const double deltaX = bodyX - m_p0_x;
    const double deltaY = bodyY - m_p0_y;

    const double C = deltaX * perpX + deltaY * perpY;

    const double dC_dq1 = 1.0 * perpX;
    const double dC_dq2 = 1.0 * perpY;
    const double dC_dq3 =
        (-sin_q3 * m_local_x - cos_q3 * m_local_y) * perpX +
        (cos_q3 * m_local_x - sin_q3 * m_local_y) * perpY;

    output->J[0][0] = dC_dq1;
    output->J[0][1] = dC_dq2;
    output->J[0][2] = dC_dq3;

    output->J_dot[0][0] = 0.0;
    output->J_dot[0][1] = 0.0;
    output->J_dot[0][2] =
        (-cos_q3 * q3_dot * m_local_x + sin_q3 * q3_dot * m_local_y) * perpX +
        (-sin_q3 * q3_dot * m_local_x - cos_q3 * q3_dot * m_local_y) * perpY;

    output->ks[0] = m_ks;
    output->kd[0] = m_kd;

    output->C[0] = C;
    output->v_bias[0] = 0;

    noLimits(output);
}
