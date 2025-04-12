#include "fixed_rotation_constraint.h"

void init_Constraint(int constraintCount, int bodyCount) {
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

void init_FixedRotationConstraint(1, 1) {
    m_rotation = 0;
    m_ks = 10.0;
    m_kd = 1.0;
}

void calculate(Output *output, SystemState *state) {
    const int body = m_bodies[0]->index;

    const double q3 = state->theta[body];
    const double C = q3 - m_rotation;

    output->J[0][0] = 0;
    output->J[0][1] = 0;
    output->J[0][2] = 1.0;

    output->J_dot[0][0] = 0;
    output->J_dot[0][1] = 0;
    output->J_dot[0][2] = 0;

    output->ks[0] = m_ks;
    output->kd[0] = m_kd;

    output->C[0] = C;

    output->v_bias[0] = 0;

    noLimits(output);
}
