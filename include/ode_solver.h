#ifndef ATG_SIMPLE_2D_CONSTRAINT_SOLVER_ODE_SOLVER_H
#define ATG_SIMPLE_2D_CONSTRAINT_SOLVER_ODE_SOLVER_H

#include "system_state.h"

namespace atg_scs {
    class OdeSolver {
        public:
            OdeSolver();
            virtual ~OdeSolver();

            virtual void start(SystemState *initial, double dt);
            virtual bool step(SystemState *system);
            virtual void solve(SystemState *system);
            virtual void end();

        protected:
            double m_dt;
    };
} /* namespace atg_scs */

#endif /* ATG_SIMPLE_2D_CONSTRAINT_SOLVER_ODE_SOLVER_H */
#include "../include/ode_solver.h"

atg_scs::OdeSolver::OdeSolver() {
    m_dt = 0.0;
}

atg_scs::OdeSolver::~OdeSolver() {
    /* void */
}

void atg_scs::OdeSolver::start(SystemState *initial, double dt) {
    m_dt = dt;
}

bool atg_scs::OdeSolver::step(SystemState *system) {
    return true;
}

void atg_scs::OdeSolver::solve(SystemState *system) {
    /* void */
}

void atg_scs::OdeSolver::end() {
    /* void */
}
