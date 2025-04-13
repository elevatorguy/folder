#include "nsv_ode_solver.h"

void init_OdeSolver(void) {
    m_dt = 0.0;
}

void start(SystemState *initial, double dt) {
    m_dt = dt;
}

bool step(SystemState *system) {
    return true;
}

void start(SystemState *initial, double dt) {
    OdeSolver::start(initial, dt);
}

bool step(SystemState *system) {
    system->dt = m_dt;
    return true;
}

void solve(SystemState *system) {
    system->dt = m_dt;

    for (int i = 0; i < system->n; ++i) {
        system->v_x[i] += system->a_x[i] * m_dt;
        system->v_y[i] += system->a_y[i] * m_dt;
        system->v_theta[i] += system->a_theta[i] * m_dt;

        system->p_x[i] += system->v_x[i] * m_dt;
        system->p_y[i] += system->v_y[i] * m_dt;
        system->theta[i] += system->v_theta[i] * m_dt;
    }
}
