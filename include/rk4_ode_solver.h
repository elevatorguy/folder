#ifndef ATG_SIMPLE_2D_CONSTRAINT_SOLVER_RK4_ODE_SOLVER_H
#define ATG_SIMPLE_2D_CONSTRAINT_SOLVER_RK4_ODE_SOLVER_H

#include "system_state.h"

void init_OdeSolver(void);

virtual void start(SystemState *initial, double dt);
virtual bool step(SystemState *system);

double m_dt;

enum class RkStage {
    Stage_1,
    Stage_2,
    Stage_3,
    Stage_4,
    Complete,
    Undefined
};

void init_Rk4OdeSolver(void);
void deinit_Rk4OdeSolver(void);

virtual void start(SystemState *initial, double dt);
virtual bool step(SystemState *system);
virtual void solve(SystemState *system);
virtual void end(void);

static RkStage getNextStage(RkStage stage);

RkStage m_stage;
RkStage m_nextStage;

SystemState m_initialState;
SystemState m_accumulator;

#endif /* ATG_SIMPLE_2D_CONSTRAINT_SOLVER_ODE_SOLVER_H */
