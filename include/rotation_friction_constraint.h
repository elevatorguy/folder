#ifndef ATG_SIMPLE_2D_CONSTRAINT_ROTATION_FRICTION_CONSTRAINT_H
#define ATG_SIMPLE_2D_CONSTRAINT_ROTATION_FRICTION_CONSTRAINT_H

#include "constraint.h"

class RotationFrictionConstraint : public Constraint {
    public:
        RotationFrictionConstraint();
        
        void setBody(RigidBody *body) { m_bodies[0] = body; }

        virtual void calculate(Output *output, SystemState *system);

        double m_maxTorque;
        double m_minTorque;

        double m_ks;
        double m_kd;
};

#endif /* ATG_SIMPLE_2D_CONSTRAINT_ROTATION_FRICTION_CONSTRAINT_H */
