#ifndef ATG_SIMPLE_2D_CONSTRAINT_SOLVER_CLUTCH_CONSTRAINT_H
#define ATG_SIMPLE_2D_CONSTRAINT_SOLVER_CLUTCH_CONSTRAINT_H

#include "constraint.h"

class ClutchConstraint : public Constraint {
    public:
        ClutchConstraint();
        virtual ~ClutchConstraint();
        
        void setBody1(RigidBody *body) { m_bodies[0] = body; }
        void setBody2(RigidBody *body) { m_bodies[1] = body; }

        virtual void calculate(Output *output, SystemState *system);

        double m_maxTorque;
        double m_minTorque;

        double m_ks;
        double m_kd;
};

#endif /* ATG_SIMPLE_2D_CONSTRAINT_SOLVER_CLUTCH_CONSTRAINT_H */
