#ifndef ATG_SIMPLE_2D_CONSTRAINT_SOLVER_ROLLING_CONSTRAINT_H
#define ATG_SIMPLE_2D_CONSTRAINT_SOLVER_ROLLING_CONSTRAINT_H

#include "constraint.h"

class RollingConstraint : public Constraint {
    public:
        RollingConstraint();

        void setBaseBody(RigidBody *body) { m_bodies[0] = body; }
        void setRollingBody(RigidBody *body) { m_bodies[1] = body; }

        virtual void calculate(Output *output, SystemState *system);

        double m_local_x;
        double m_local_y;
        double m_dx;
        double m_dy;
        double m_radius;

        double m_ks;
        double m_kd;
};

#endif /* ATG_SIMPLE_2D_CONSTRAINT_SOLVER_ROLLING_CONSTRAINT_H */
