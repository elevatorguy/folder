#ifndef ATG_SIMPLE_2D_CONSTRAINT_SOLVER_FIXED_ROTATION_CONSTRAINT_H
#define ATG_SIMPLE_2D_CONSTRAINT_SOLVER_FIXED_ROTATION_CONSTRAINT_H

#include "constraint.h"

class FixedRotationConstraint : public Constraint {
public:
    FixedRotationConstraint();
    virtual ~FixedRotationConstraint();

    void setBody(RigidBody *body) { m_bodies[0] = body; }

    virtual void calculate(Output *output, SystemState *state);

    double m_rotation;
    double m_ks;
    double m_kd;
};

#endif /* ATG_SIMPLE_2D_CONSTRAINT_SOLVER_FIXED_ROTATION_CONSTRAINT_H */

