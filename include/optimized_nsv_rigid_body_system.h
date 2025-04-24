#ifndef ATG_SIMPLE_2D_CONSTRAINT_SOLVER_OPTIMIZED_NSV_RIGID_BODY_SYSTEM_H
#define ATG_SIMPLE_2D_CONSTRAINT_SOLVER_OPTIMIZED_NSV_RIGID_BODY_SYSTEM_H

#include "matrix.h"
#include "sparse_matrix.h"
#include "system_state.h"

#include <vector>

struct RigidBody {
    public:
        RigidBody();

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

        void reset();
        double energy() const;
};

static const int ProfilingSamples = 60 * 10;

void init_RigidBodySystem();
void deinit_RigidBodySystem();

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

static float findAverage(long long *samples);

void populateSystemState();
void populateMassMatrices(Matrix *M, Matrix *M_inv);
void processForces();

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
#include "nsv_ode_solver.h"

void init_OptimizedNsvRigidBodySystem();
void deinit_OptimizedNsvRigidBodySystem();

void initialize(SleSolver *sleSolver);
virtual void process(double dt, int steps = 1);

inline double timeElapsed() const { return m_t; }

double m_biasFactor;

void propagateResults();
void processConstraints(
        double dt,
        long long *evalTime,
        long long *solveTime);

NsvOdeSolver m_odeSolver;
SleSolver *m_sleSolver;

double m_t;

struct IntermediateValues {
    SparseMatrix<3> J_sparse, sreg0;
    Matrix C;
    Matrix M, M_inv;
    Matrix b_err, v_bias;
    Matrix limits;

    Matrix q_dot, q_dot_prime;

    Matrix reg0, reg1, reg2;

    Matrix right;
    Matrix F_ext, F_C, R;

    // Results
    Matrix lambda;
} m_iv;

#endif /* ATG_SIMPLE_2D_CONSTRAINT_SOLVER_OPTIMIZED_NSV_RIGID_BODY_SYSTEM_H */
