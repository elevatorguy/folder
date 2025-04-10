#ifndef ATG_SIMPLE_2D_CONSTRAINT_SOLVER_CONSTRAINT_H
#define ATG_SIMPLE_2D_CONSTRAINT_SOLVER_CONSTRAINT_H

#include "system_state.h"
#include "rigid_body.h"
#include "matrix.h"
#include "utilities.h"

#include <cfloat>

class Constraint {
    public:
        static constexpr int MaxConstraintCount = 3;
        static constexpr int MaxBodyCount = 2;

        struct Output {
            double C[MaxConstraintCount];
            double J[MaxConstraintCount][3 * MaxBodyCount];
            double J_dot[MaxConstraintCount][3 * MaxBodyCount];
            double v_bias[MaxConstraintCount];
            double limits[MaxConstraintCount][2];
            double ks[MaxConstraintCount];
            double kd[MaxConstraintCount];
        };

    public:
        Constraint(int constraintCount, int bodyCount);

        scs_force_inline int getConstraintCount() const { return m_constraintCount; }

        int m_index;
        int m_bodyCount;
        RigidBody *m_bodies[MaxBodyCount];

        double F_x[MaxConstraintCount][MaxBodyCount];
        double F_y[MaxConstraintCount][MaxBodyCount];
        double F_t[MaxConstraintCount][MaxBodyCount];

    protected:
        inline void noLimits(Output *output) {
            for (int i = 0; i < MaxConstraintCount; ++i) {
                output->limits[i][0] = -DBL_MAX;
                output->limits[i][1] = DBL_MAX;
            }
        }

    protected:
        int m_constraintCount;
};

#endif /* ATG_SIMPLE_2D_CONSTRAINT_SOLVER_CONSTRAINT_H */
