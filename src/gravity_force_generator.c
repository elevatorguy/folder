#include "gravity_force_generator.h"

void init_GravityForceGenerator(void) {
    m_g = 9.81;
}

void apply(SystemState *state) {
    const int n = state->n;

    for (int i = 0; i < n; ++i) {
        state->f_y[i] += -state->m[i] * m_g;
    }
}
