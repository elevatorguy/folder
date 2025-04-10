#ifndef ATG_SIMPLE_2D_CONSTRAINT_SOLVER_ODE_SOLVER_H
#define ATG_SIMPLE_2D_CONSTRAINT_SOLVER_ODE_SOLVER_H

#include "system_state.h"

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

#endif /* ATG_SIMPLE_2D_CONSTRAINT_SOLVER_ODE_SOLVER_H */
