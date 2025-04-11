#ifndef ATG_SIMPLE_2D_CONSTRAINT_SOLVER_SPRING_H
#define ATG_SIMPLE_2D_CONSTRAINT_SOLVER_SPRING_H

#include "system_state.h"

class ForceGenerator {
    public:
        virtual void apply(SystemState *system) = 0;

        int m_index;
};

#include "rigid_body.h"

class Spring {
    public:
        Spring();

        virtual void apply(SystemState *state);
        
        void getEnds(double *x_1, double *y_1, double *x_2, double *y_2);
        double energy() const;

        double m_restLength;
        double m_ks;
        double m_kd;

        double m_p1_x;
        double m_p1_y;

        double m_p2_x;
        double m_p2_y;

        RigidBody *m_body1;
        RigidBody *m_body2;
};

#endif /* ATG_SIMPLE_2D_CONSTRAINT_SOLVER_SPRING_H */
