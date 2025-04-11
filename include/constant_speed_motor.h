#ifndef ATG_SIMPLE_2D_CONSTRAINT_CONSTANT_SPEED_MOTOR_H
#define ATG_SIMPLE_2D_CONSTRAINT_CONSTANT_SPEED_MOTOR_H

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

ConstantSpeedMotor();

virtual void apply(SystemState *state);

double m_ks;
double m_kd;
double m_maxTorque;
double m_speed;

RigidBody *m_body0;
RigidBody *m_body1;

#endif /* ATG_SIMPLE_2D_CONSTRAINT_SOLVER_SPRING_H */
