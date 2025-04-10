#include "static_force_generator.h"

#include <cmath>

StaticForceGenerator::StaticForceGenerator() {
    m_f_x = m_f_y = 0.0;
    m_p_x = m_p_y = 0.0;

    m_body = nullptr;
}

StaticForceGenerator::~StaticForceGenerator() {
    /* void */
}

void StaticForceGenerator::apply(SystemState *state) {
    state->applyForce(
        m_p_x,
        m_p_y,
        m_f_x,
        m_f_y,
        m_body->index
    );
}

void StaticForceGenerator::setForce(double f_x, double f_y) {
    m_f_x = f_x;
    m_f_y = f_y;
}

void StaticForceGenerator::setPosition(double p_x, double p_y) {
    m_p_x = p_x;
    m_p_y = p_y;
}
