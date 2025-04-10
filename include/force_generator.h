#ifndef ATG_SIMPLE_2D_CONSTRAINT_SOLVER_FORCE_GENERATOR_H
#define ATG_SIMPLE_2D_CONSTRAINT_SOLVER_FORCE_GENERATOR_H

#include "system_state.h"

class ForceGenerator {
    public:
        virtual void apply(SystemState *system) = 0;

        int m_index;
};

#endif /* ATG_SIMPLE_2D_CONSTRAINT_SOLVER_FORCE_GENERATOR_H */
