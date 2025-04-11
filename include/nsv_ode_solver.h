#ifndef ATG_SIMPLE_2D_CONSTRAINT_SOLVER_NSV_ODE_SOLVER_H
#define ATG_SIMPLE_2D_CONSTRAINT_SOLVER_NSV_ODE_SOLVER_H

#include "system_state.h"

OdeSolver();

virtual void start(SystemState *initial, double dt);
virtual bool step(SystemState *system);

double m_dt;

virtual void start(SystemState *initial, double dt);
virtual bool step(SystemState *system);
virtual void solve(SystemState *system);

#endif /* ATG_SIMPLE_2D_CONSTRAINT_SOLVER_NSV_ODE_SOLVER_H */
