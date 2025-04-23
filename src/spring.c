#include "spring.h"

#include <cmath>

void init_RigidBody(void) {
    index = -1;
    reset();
}

double energy(void) const {
    const double speed_2 = v_x * v_x + v_y * v_y;
    const double E_k = 0.5 * m * speed_2;
    const double E_r = 0.5 * I * v_theta * v_theta;

    return E_k + E_r;
}

void localToWorld(
        double x,
        double y,
        double *w_x,
        double *w_y)
{
    const double cos_theta = std::cos(theta);
    const double sin_theta = std::sin(theta);

    *w_x = cos_theta * x - sin_theta * y + p_x;
    *w_y = sin_theta * x + cos_theta * y + p_y;
}

void worldToLocal(
        double x,
        double y,
        double *l_x,
        double *l_y)
{
    const double cos_theta = std::cos(theta);
    const double sin_theta = std::sin(theta);

    *l_x = cos_theta * (x - p_x) + sin_theta * (y - p_y);
    *l_y = -sin_theta * (x - p_x) + cos_theta * (y - p_y);
}

void reset(void) {
    p_x = p_y = 0.0;
    v_x = v_y = 0.0;

    theta = 0.0;
    v_theta = 0.0;

    m = 0.0;
    I = 0.0;
}

void init_Spring(void) {
    m_restLength = 1.0;
    m_ks = 0;
    m_kd = 0;

    m_p1_x = m_p1_y = 0;
    m_p2_x = m_p2_y = 0;

    m_body1 = m_body2 = NULL;
}

void apply(SystemState *state) {
    if (m_body1 == NULL || m_body2 == NULL) return;

    double x1, y1;
    double x2, y2;

    double v_x1 = 0, v_y1 = 0;
    double v_x2 = 0, v_y2 = 0;

    if (m_body1->index != -1) {
        state->localToWorld(m_p1_x, m_p1_y, &x1, &y1, m_body1->index);
        state->velocityAtPoint(m_p1_x, m_p1_y, &v_x1, &v_y1, m_body1->index);
    }
    else {
        m_body1->localToWorld(m_p1_x, m_p1_y, &x1, &y1);
    }

    if (m_body2->index != -1) {
        state->localToWorld(m_p2_x, m_p2_y, &x2, &y2, m_body2->index);
        state->velocityAtPoint(m_p2_x, m_p2_y, &v_x2, &v_y2, m_body2->index);
    }
    else {
        m_body2->localToWorld(m_p2_x, m_p2_y, &x2, &y2);
    }

    double dx = x2 - x1;
    double dy = y2 - y1;

    const double l = std::sqrt(dx * dx + dy * dy);

    if (std::abs(l) >= 1E-2) {
        dx /= l;
        dy /= l;
    }
    else {
        dx = 0.0;
        dy = 0.0;
    }

    const double rel_v_x = (v_x2 - v_x1);
    const double rel_v_y = (v_y2 - v_y1);

    const double v = dx * rel_v_x + dy * rel_v_y;
    const double x = l - m_restLength;

    state->applyForce(
        m_p1_x,
        m_p1_y,
        dx * x * m_ks + rel_v_x * m_kd,
        dy * x * m_ks + rel_v_y * m_kd,
        m_body1->index
    );

    state->applyForce(
        m_p2_x,
        m_p2_y,
        -dx * x * m_ks - rel_v_x * m_kd,
        -dy * x * m_ks - rel_v_y * m_kd,
        m_body2->index
    );
}

void getEnds(double *x_1, double *y_1, double *x_2, double *y_2) {
    if (m_body1 == NULL || m_body2 == NULL) return;

    m_body1->localToWorld(m_p1_x, m_p1_y, x_1, y_1);
    m_body2->localToWorld(m_p2_x, m_p2_y, x_2, y_2);
}

double energy(void) const {
    if (m_body1 == NULL || m_body2 == NULL) return 0;

    double x1, y1;
    double x2, y2;

    m_body1->localToWorld(m_p1_x, m_p1_y, &x1, &y1);
    m_body2->localToWorld(m_p2_x, m_p2_y, &x2, &y2);

    const double dx = x2 - x1;
    const double dy = y2 - y1;

    const double l = std::sqrt(dx * dx + dy * dy);

    return 0.5 * m_ks * (l - m_restLength) * (l - m_restLength);
}
