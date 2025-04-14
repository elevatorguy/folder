#ifndef ATG_SIMPLE_2D_CONSTRAINT_CONSTANT_SPEED_MOTOR_H
#define ATG_SIMPLE_2D_CONSTRAINT_CONSTANT_SPEED_MOTOR_H

#include "force_generator.h"

#include "rigid_body.h"

namespace atg_scs {
    class ConstantSpeedMotor : public ForceGenerator {
        public:
            ConstantSpeedMotor();
            virtual ~ConstantSpeedMotor();

            virtual void apply(SystemState *state);

            double m_ks;
            double m_kd;
            double m_maxTorque;
            double m_speed;

            RigidBody *m_body0;
            RigidBody *m_body1;
    };
} /* namespace atg_scs */

#endif /* ATG_SIMPLE_2D_CONSTRAINT_SOLVER_SPRING_H */
#include "../include/constant_speed_motor.h"

#include <cmath>

atg_scs::ConstantSpeedMotor::ConstantSpeedMotor() {
    m_ks = 1.0f;
    m_kd = 1.0f;
    m_maxTorque = 500.0f;
    m_speed = 1.0f;

    m_body0 = nullptr;
    m_body1 = nullptr;
}

atg_scs::ConstantSpeedMotor::~ConstantSpeedMotor() {
    /* void */
}

void atg_scs::ConstantSpeedMotor::apply(SystemState *state) {
    double v1;
    double a1;

    if (m_body0->index == -1) {
        v1 = a1 = 0;
    }
    else {
        v1 = state->v_theta[m_body0->index];
        a1 = state->a_theta[m_body0->index];
    }

    const double rel_v =
        state->v_theta[m_body1->index] - v1;
    const double rel_a =
        state->a_theta[m_body1->index] - a1;
    const double delta = m_speed - rel_v;

    const double torque = delta * m_ks;
    const double dampingTorque = -rel_a * m_kd;
    const double totalTorque = torque + dampingTorque;
    const double limitedTorque =
        std::fmin(m_maxTorque, std::fmax(-m_maxTorque, totalTorque));

    if (m_body0->index != -1) state->t[m_body0->index] -= limitedTorque;
    state->t[m_body1->index] += limitedTorque;
}
