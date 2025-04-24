#ifndef ATG_SIMPLE_2D_CONSTRAINT_SOLVER_GENERIC_RIGID_BODY_SYSTEM_H
#define ATG_SIMPLE_2D_CONSTRAINT_SOLVER_GENERIC_RIGID_BODY_SYSTEM_H

#include "matrix.h"
#include "sparse_matrix.h"
#include "system_state.h"

#include <vector>

struct RigidBody {
    public:
        RigidBody(void);

        void localToWorld(double x, double y, double *w_x, double *w_y);
        void worldToLocal(double x, double y, double *l_x, double *l_y);

        double p_x;
        double p_y;

        double v_x;
        double v_y;

        double theta;
        double v_theta;

        double m;
        double I;

        int index;

        void reset(void);
        double energy(void) const;
};

static const int ProfilingSamples = 60 * 10;

void init_RigidBodySystem(void);
void deinit_RigidBodySystem(void);

virtual void reset(void);

void addRigidBody(RigidBody *body);
void removeRigidBody(RigidBody *body);
RigidBody *getRigidBody(int i);

void addConstraint(Constraint *constraint);
void removeConstraint(Constraint *constraint);

void addForceGenerator(ForceGenerator *generator);
void removeForceGenerator(ForceGenerator *generator);

int getRigidBodyCount(void) const { return (int)m_rigidBodies.size(); }
int getConstraintCount(void) const { return (int)m_constraints.size(); }
int getForceGeneratorCount(void) const { return (int)m_forceGenerators.size(); }

int getFullConstraintCount(void) const;

float getOdeSolveMicroseconds(void) const;
float getConstraintSolveMicroseconds(void) const;
float getForceEvalMicroseconds(void) const;
float getConstraintEvalMicroseconds(void) const;

inline const SystemState *state(void) const { return &m_state; }

static float findAverage(long long *samples);

void populateSystemState(void);
void populateMassMatrices(Matrix *M, Matrix *M_inv);
void processForces(void);

vector<RigidBody *> m_rigidBodies;
vector<Constraint *> m_constraints;
vector<ForceGenerator *> m_forceGenerators;

SystemState m_state;

long long *m_odeSolveMicroseconds;
long long *m_constraintSolveMicroseconds;
long long *m_forceEvalMicroseconds;
long long *m_constraintEvalMicroseconds;
long long m_frameIndex;

#include "sle_solver.h"
#include "ode_solver.h"

void init_GenericRigidBodySystem(void);

void initialize(SleSolver *sleSolver, OdeSolver *odeSolver);
virtual void process(double dt, int steps = 1);

void processConstraints(
        long long *evalTime,
        long long *solveTime);

OdeSolver *m_odeSolver;
SleSolver *m_sleSolver;

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

#endif /* ATG_SIMPLE_2D_CONSTRAINT_SOLVER_GENERIC_RIGID_BODY_SYSTEM_H */
