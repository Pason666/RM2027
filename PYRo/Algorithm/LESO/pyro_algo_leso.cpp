/**
 * @file pyro_algo_leso.cpp
 * @brief Implementation file for the PYRO C++ LESO class.
 *
 * This file implements the `pyro::leso_t` methods, including the discrete
 * forward Euler integration logic for the 3rd-order Linear Extended State
 * Observer.
 *
 * @author Lucky
 * @version 1.0.0
 * @date 2026-05-06
 * @copyright [Copyright Information Here]
 */

/* Includes ------------------------------------------------------------------*/
#include "pyro_algo_leso.h"
#include "pyro_dwt_drv.h" // For pyro::dwt_drv_t
#include <algorithm>      // For std::clamp

namespace pyro
{

/* Constructor Implementation ------------------------------------------------*/

leso_t::leso_t(const float omega_o, const float b, const float z3_limit)
    : _omega_o(omega_o), _b(b), _z3_limit(z3_limit)
{
    clear();
    calculate_betas();
}

/* Public Methods ------------------------------------------------------------*/

/**
 * @brief Updates the LESO states using Forward Euler integration.
 */
void leso_t::update(const float measure, const float u)
{
    // Get time delta from DWT (consistent with pid_t logic)
    _dt = dwt_drv_t::get_delta_t(&_dwt_cnt);

    // Prevent calculation if dt is too small or 0
    if (_dt < 1e-9f)
    {
        return;
    }

    // Calculate observation error (z1 is estimated output, measure is actual y)
    const float err     = _z1 - measure;

    // --- State Update Equations (Forward Euler) ---
    // z1(k) = z1(k-1) + dt * (z2(k-1) - beta1 * err)
    // z2(k) = z2(k-1) + dt * (z3(k-1) - beta2 * err + b * u)
    // z3(k) = z3(k-1) + dt * (- beta3 * err)

    // Store old values for simultaneous update
    const float next_z1 = _z1 + _dt * (_z2 - _beta1 * err);
    const float next_z2 = _z2 + _dt * (_z3 - _beta2 * err + _b * u);
    float next_z3       = _z3 + _dt * (-_beta3 * err);

    // --- Apply Disturbance Limit (Anti-windup for LESO) ---
    if (_z3_limit > 0.0f)
    {
        next_z3 = std::clamp(next_z3, -_z3_limit, _z3_limit);
    }

    // Update internal states
    _z1 = next_z1;
    _z2 = next_z2;
    _z3 = next_z3;
}

/**
 * @brief Clears all internal states.
 */
void leso_t::clear()
{
    _z1      = 0.0f;
    _z2      = 0.0f;
    _z3      = 0.0f;
    _dwt_cnt = 0;
    _dt      = 0.0f;
}

/**
 * @brief Sets new observer bandwidth and plant gain.
 */
void leso_t::set_params(const float omega_o, const float b)
{
    _omega_o = omega_o;
    _b       = b;
    calculate_betas();
}

/**
 * @brief Sets a new limit for z3.
 */
void leso_t::set_z3_limit(const float limit)
{
    _z3_limit = limit > 0.0f ? limit : 0.0f;
}

/* Private Helper Functions --------------------------------------------------*/

/**
 * @brief Calculates observer gains based on the bandwidth parameterization
 * method (Gao Zhiqiang).
 *
 * Configures the characteristic polynomial to (s + omega_o)^3 = 0.
 */
void leso_t::calculate_betas()
{
    _beta1 = 3.0f * _omega_o;
    _beta2 = 3.0f * _omega_o * _omega_o;
    _beta3 = _omega_o * _omega_o * _omega_o;
}

} // namespace pyro