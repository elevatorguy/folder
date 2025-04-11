#ifndef ATG_SIMPLE_2D_CONSTRAINT_SOLVER_GRAVITY_FORCE_GENERATOR_H
#define ATG_SIMPLE_2D_CONSTRAINT_SOLVER_GRAVITY_FORCE_GENERATOR_H

#include "system_state.h"

class ForceGenerator {
    public:
        virtual void apply(SystemState *system) = 0;

        int m_index;
};

#include "rigid_body.h"

class GravityForceGenerator {
    public:
        GravityForceGenerator();

        virtual void apply(SystemState *state);

        double m_g;
};

#endif /* ATG_SIMPLE_2D_CONSTRAINT_SOLVER_GRAVITY_FORCE_GENERATOR_H */
