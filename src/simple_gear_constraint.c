#include "simple_gear_constraint.h"

#include <cmath>
#include <cfloat>

Constraint::Constraint(int constraintCount, int bodyCount) {
    assert(constraintCount <= MaxConstraintCount);
    assert(bodyCount <= MaxBodyCount);

    m_constraintCount = constraintCount;
    m_bodyCount = bodyCount;

    m_index = -1;

    memset(m_bodies, 0, sizeof(int) * MaxBodyCount);

    for (int i = 0; i < MaxConstraintCount; ++i) {
        for (int j = 0; j < MaxBodyCount; ++j) {
            F_x[i][j] = F_y[i][j] = F_t[i][j] = 0;
        }
    }
}

SimpleGearConstraint::SimpleGearConstraint(1, 2) {
    m_ks = 10.0;
    m_kd = 1.0;

    m_ratio = 1.0;
    m_neutral = false;
}

void SimpleGearConstraint::calculate(
    Output *output,
    SystemState *state) {
    output->C[0] = 0;

    output->J[0][0] = 0.0;
    output->J[0][1] = 0.0;
    output->J[0][2] = 1.0;

    output->J[0][3] = 0.0;
    output->J[0][4] = 0.0;
    output->J[0][5] = -m_ratio;

    output->J_dot[0][0] = 0;
    output->J_dot[0][1] = 0;
    output->J_dot[0][2] = 0;

    output->J_dot[0][3] = 0;
    output->J_dot[0][4] = 0;
    output->J_dot[0][5] = 0;

    output->kd[0] = m_kd;
    output->ks[0] = m_ks;

    output->v_bias[0] = 0;

    if (!m_neutral) {
        noLimits(output);
    }
    else {
        output->limits[0][0] = output->limits[0][1] = 0;
    }
}

