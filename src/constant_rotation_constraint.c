#include "constant_rotation_constraint.h"

#include <limits>
#include <cmath>

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

ConstantRotationConstraint::ConstantRotationConstraint(1, 1) {
    m_rotationSpeed = 0.0;
    m_maxTorque = DBL_MAX;
    m_minTorque = -DBL_MAX;
    m_ks = 10.0;
    m_kd = 1.0;
}

void ConstantRotationConstraint::calculate(Output *output, SystemState *state) {
    output->J[0][0] = 0;
    output->J[0][1] = 0;
    output->J[0][2] = 1;

    output->J_dot[0][0] = 0;
    output->J_dot[0][1] = 0;
    output->J_dot[0][2] = 0;

    output->ks[0] = m_ks;
    output->kd[0] = m_kd;

    output->C[0] = 0;

    output->v_bias[0] = m_rotationSpeed;

    output->limits[0][0] = m_minTorque;
    output->limits[0][1] = m_maxTorque;
}
