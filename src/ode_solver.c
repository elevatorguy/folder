#include "ode_solver.h"

OdeSolver::OdeSolver() {
    m_dt = 0.0;
}

void OdeSolver::start(SystemState *initial, double dt) {
    m_dt = dt;
}

bool OdeSolver::step(SystemState *system) {
    return true;
}
