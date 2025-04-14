#ifndef ATG_SIMPLE_2D_CONSTRAINT_SOLVER_EULER_ODE_SOLVER_H
#define ATG_SIMPLE_2D_CONSTRAINT_SOLVER_EULER_ODE_SOLVER_H

#include "ode_solver.h"

namespace atg_scs {
    class EulerOdeSolver : public OdeSolver {
        public:
            EulerOdeSolver();
            virtual ~EulerOdeSolver();

            virtual void start(SystemState *initial, double dt);
            virtual bool step(SystemState *system);
            virtual void solve(SystemState *system);
            virtual void end();
    };
} /* namespace atg_scs */

#endif /* ATG_SIMPLE_2D_CONSTRAINT_SOLVER_EULER_ODE_SOLVER_H */
#include "../include/euler_ode_solver.h"

atg_scs::EulerOdeSolver::EulerOdeSolver() {
    /* void */
}

atg_scs::EulerOdeSolver::~EulerOdeSolver() {
    /* void */
}

void atg_scs::EulerOdeSolver::start(SystemState *initial, double dt) {
    OdeSolver::start(initial, dt);
}

bool atg_scs::EulerOdeSolver::step(SystemState *system) {
    system->dt = m_dt;
    return true;
}

void atg_scs::EulerOdeSolver::solve(SystemState *system) {
    system->dt = m_dt;

    for (int i = 0; i < system->n; ++i) {
        system->p_x[i] += system->v_x[i] * m_dt;
        system->p_y[i] += system->v_y[i] * m_dt;
        system->theta[i] += system->v_theta[i] * m_dt;

        system->v_x[i] += system->a_x[i] * m_dt;
        system->v_y[i] += system->a_y[i] * m_dt;
        system->v_theta[i] += system->a_theta[i] * m_dt;
    }
}

void atg_scs::EulerOdeSolver::end() {
    /* void */
}
