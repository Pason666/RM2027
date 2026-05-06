/**
 * @file pyro_algo_leso.h
 * @brief Header file for the PYRO C++ Linear Extended State Observer (LESO)
 * class.
 *
 * This file defines the `pyro::leso_t` class, which encapsulates a 3rd-order
 * Linear Extended State Observer used in Active Disturbance Rejection Control
 * (ADRC) for a 2nd-order system. It estimates position, velocity, and total
 * disturbance.
 *
 * @author Lucky
 * @version 1.0.0
 * @date 2026-05-06
 * @copyright [Copyright Information Here]
 */

#ifndef __PYRO_ALGO_LESO_H__
#define __PYRO_ALGO_LESO_H__

#include <cstdint>

namespace pyro
{

/**
 * @brief 3rd-Order Linear Extended State Observer (LESO) C++ Class.
 *
 * Designed for 2nd-order plants. Automatically uses `dwt_drv_t` for time
 * delta calculation. Employs bandwidth parameterization for easy tuning.
 */
class leso_t
{
  public:
    /**
     * @brief Constructor for LESO.
     * @param omega_o Observer bandwidth (rad/s). The only tuning parameter for
     * observer dynamics.
     * @param b       Control gain of the plant (b0 in ADRC theory).
     * @param z3_limit Maximum absolute value for the disturbance estimation
     * (z3) to prevent windup.
     */
    leso_t(float omega_o, float b, float z3_limit = 0.0f);

    /**
     * @brief Updates the LESO states.
     * @param measure The current measured system output (y, e.g., position).
     * @param u       The control effort applied to the system in the LAST cycle
     * (u).
     */
    void update(float measure, float u);

    /**
     * @brief Clears the internal LESO states (z1, z2, z3) and timers.
     */
    void clear();

    /**
     * @brief Dynamically updates the observer parameters.
     * @param omega_o New observer bandwidth.
     * @param b       New system control gain.
     */
    void set_params(float omega_o, float b);

    /**
     * @brief Sets a hard limit for the disturbance estimation (z3).
     * @param limit Maximum absolute value for z3. Set to 0 to disable.
     */
    void set_z3_limit(float limit);

    // --- Getters ---

    /**
     * @brief Gets the estimated system output (e.g., Position).
     */
    [[nodiscard]] float get_z1() const
    {
        return _z1;
    }

    /**
     * @brief Gets the estimated derivative of the output (e.g., Velocity).
     */
    [[nodiscard]] float get_z2() const
    {
        return _z2;
    }

    /**
     * @brief Gets the estimated total disturbance.
     */
    [[nodiscard]] float get_z3() const
    {
        return _z3;
    }

  private:
    // --- Private Helper Functions ---
    void calculate_betas();

    // --- Private Member Variables ---

    // Configuration
    float _omega_o;  ///< Observer bandwidth (rad/s)
    float _b;        ///< Plant control gain parameter
    float _z3_limit; ///< Limit for total disturbance estimation

    // Observer Gains (Calculated from omega_o)
    float _beta1{};
    float _beta2{};
    float _beta3{};

    // States
    float _z1         = 0.0f; ///< Estimated output
    float _z2         = 0.0f; ///< Estimated derivative of output
    float _z3         = 0.0f; ///< Estimated total disturbance

    // Dependencies
    uint32_t _dwt_cnt = 0;    ///< Counter for DWT delta-time calculation
    float _dt         = 0.0f; ///< Delta-time from DWT
};

} // namespace pyro

#endif // __PYRO_ALGO_LESO_H__