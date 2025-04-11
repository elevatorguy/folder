#ifndef ATG_SIMPLE_2D_CONSTRAINT_SOLVER_GENERIC_RIGID_BODY_SYSTEM_H
#define ATG_SIMPLE_2D_CONSTRAINT_SOLVER_GENERIC_RIGID_BODY_SYSTEM_H

#include "rigid_body.h"
#include "constraint.h"
#include "force_generator.h"
#include "matrix.h"
#include "sparse_matrix.h"
#include "system_state.h"

#include <vector>

class RigidBodySystem {
    public:
        static const int ProfilingSamples = 60 * 10;

    public:
        RigidBodySystem();
        virtual ~RigidBodySystem();

        virtual void reset();

        void addRigidBody(RigidBody *body);
        void removeRigidBody(RigidBody *body);
        RigidBody *getRigidBody(int i);

        void addConstraint(Constraint *constraint);
        void removeConstraint(Constraint *constraint);

        void addForceGenerator(ForceGenerator *generator);
        void removeForceGenerator(ForceGenerator *generator);

        int getRigidBodyCount() const { return (int)m_rigidBodies.size(); }
        int getConstraintCount() const { return (int)m_constraints.size(); }
        int getForceGeneratorCount() const { return (int)m_forceGenerators.size(); }

        int getFullConstraintCount() const;

        float getOdeSolveMicroseconds() const;
        float getConstraintSolveMicroseconds() const;
        float getForceEvalMicroseconds() const;
        float getConstraintEvalMicroseconds() const;

        inline const SystemState *state() const { return &m_state; }

    protected:
        static float findAverage(long long *samples);

        void populateSystemState();
        void populateMassMatrices(Matrix *M, Matrix *M_inv);
        void processForces();

    protected:
        std::vector<RigidBody *> m_rigidBodies;
        std::vector<Constraint *> m_constraints;
        std::vector<ForceGenerator *> m_forceGenerators;

        SystemState m_state;

        long long *m_odeSolveMicroseconds;
        long long *m_constraintSolveMicroseconds;
        long long *m_forceEvalMicroseconds;
        long long *m_constraintEvalMicroseconds;
        long long m_frameIndex;
};

#include "sle_solver.h"
#include "ode_solver.h"

class GenericRigidBodySystem {
    public:
        GenericRigidBodySystem();

        void initialize(SleSolver *sleSolver, OdeSolver *odeSolver);
        virtual void process(double dt, int steps = 1);

    protected:
        void processConstraints(
                long long *evalTime,
                long long *solveTime);

    protected:
        OdeSolver *m_odeSolver;
        SleSolver *m_sleSolver;

    protected:
        struct IntermediateValues {
            SparseMatrix<3> J_sparse, J_dot_sparse, sreg0;
            Matrix J_T;
            Matrix M, M_inv;
            Matrix C;
            Matrix ks, kd;
            Matrix q_dot;

            Matrix reg0, reg1, reg2;

            Matrix right;
            Matrix F_ext, F_C, R;

            // Results
            Matrix lambda;
        } m_iv;
};

#endif /* ATG_SIMPLE_2D_CONSTRAINT_SOLVER_GENERIC_RIGID_BODY_SYSTEM_H */
