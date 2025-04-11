#ifndef ATG_SIMPLE_2D_CONSTRAINT_CONSTANT_SPEED_MOTOR_H
#define ATG_SIMPLE_2D_CONSTRAINT_CONSTANT_SPEED_MOTOR_H

#include "system_state.h"

class ForceGenerator {
    public:
        virtual void apply(SystemState *system) = 0;

        int m_index;
};

#include "rigid_body.h"

class ConstantSpeedMotor {
    public:
        ConstantSpeedMotor();

        virtual void apply(SystemState *state);

        double m_ks;
        double m_kd;
        double m_maxTorque;
        double m_speed;

        RigidBody *m_body0;
        RigidBody *m_body1;
};

#endif /* ATG_SIMPLE_2D_CONSTRAINT_SOLVER_SPRING_H */
