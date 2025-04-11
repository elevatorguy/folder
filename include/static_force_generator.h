#ifndef ATG_SIMPLE_2D_CONSTRAINT_SOLVER_STATIC_FORCE_GENERATOR_H
#define ATG_SIMPLE_2D_CONSTRAINT_SOLVER_STATIC_FORCE_GENERATOR_H

#include "system_state.h"

class ForceGenerator {
    public:
        virtual void apply(SystemState *system) = 0;

        int m_index;
};

#include "rigid_body.h"

class StaticForceGenerator {
    public:
        StaticForceGenerator();

        virtual void apply(SystemState *state);

        void setForce(double f_x, double f_y);
        void setPosition(double p_x, double p_y);

        double m_f_x;
        double m_f_y;

        double m_p_x;
        double m_p_y;

        RigidBody *m_body;
};

#endif /* ATG_SIMPLE_2D_CONSTRAINT_SOLVER_STATIC_FORCE_GENERATOR_H */
