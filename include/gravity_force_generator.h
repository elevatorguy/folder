#ifndef ATG_SIMPLE_2D_CONSTRAINT_SOLVER_GRAVITY_FORCE_GENERATOR_H
#define ATG_SIMPLE_2D_CONSTRAINT_SOLVER_GRAVITY_FORCE_GENERATOR_H

#include "force_generator.h"

#include "rigid_body.h"

namespace atg_scs {
    class GravityForceGenerator : public ForceGenerator {
        public:
            GravityForceGenerator();
            virtual ~GravityForceGenerator();

            virtual void apply(SystemState *state);

            double m_g;
    };
} /* namespace atg_scs */

#endif /* ATG_SIMPLE_2D_CONSTRAINT_SOLVER_GRAVITY_FORCE_GENERATOR_H */
#include "../include/gravity_force_generator.h"

atg_scs::GravityForceGenerator::GravityForceGenerator() {
    m_g = 9.81;
}

atg_scs::GravityForceGenerator::~GravityForceGenerator() {
    /* void */
}

void atg_scs::GravityForceGenerator::apply(SystemState *state) {
    const int n = state->n;

    for (int i = 0; i < n; ++i) {
        state->f_y[i] += -state->m[i] * m_g;
    }
}
