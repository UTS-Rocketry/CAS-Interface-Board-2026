#ifndef AIRBRAKE_H
#define AIRBRAKE_H

#include <stdint.h>
#include <stdbool.h>
#include "flight_sensors.h"

/*
 * ============================================================================
 *  AIRBRAKE CONTROLLER - public interface
 * ============================================================================
 *
 *  DESIGN PRINCIPLE: controller-agnostic interface.
 *
 *  The functions below describe WHAT the controller does (take state, return a
 *  deploy fraction) without exposing HOW. main.c and the FSM only ever call
 *  this interface, so the algorithm inside airbrake.c can be rewritten without
 *  changing a single line in main.c or the servo driver. This separation
 *  (interface vs implementation) is one of the most important ideas in
 *  embedded/systems software.
 *
 *  It has already paid off once: the implementation moved from a pure
 *  proportional law on an energy-method prediction to an inverse-model
 *  feedforward on a drag-aware prediction, and nothing outside airbrake.c
 *  needed to change - only one getter was added.
 *
 *  THE ABSTRACTION: the controller speaks in "deploy fraction" (0.0 = stowed,
 *  1.0 = fully deployed), NOT microseconds or servo angles. It knows physics,
 *  not hardware. The mapping from fraction to servo pulse lives at the
 *  boundary (servo_set_fraction). This keeps the controller unit-testable on a
 *  PC with no hardware: feed it altitude/velocity, check the fraction it
 *  returns.
 * ============================================================================
 */

/* Reset internal state. Call once before flight (e.g. at FSM init); the
 * controller assumes brakes start stowed. */
void airbrake_init(void);

/*
 * Run one control update.
 *
 *   data : current flight state. Uses kalman_altitude, kalman_velocity,
 *          pressure (PASCALS), temperature (CELSIUS) and flight_state.
 *   dt   : time since last update, seconds (for slew limiting)
 *
 * Returns the commanded deploy fraction in [0.0, 1.0].
 *
 * NOTE: unlike the previous revision, this DOES command the servo (via
 * servo_set_fraction / servo_set_us) as well as returning the fraction. The
 * return value is the same number that was applied.
 */
float airbrake_update(const FlightSensorData *data, float dt);

/* ---- telemetry / debugging ------------------------------------------------
 * Log all four of these. The pair of apogee predictions lets the drag-aware
 * model be checked against the old energy method on real flight data, and
 * airbrake_target_reachable() answers the question the old telemetry could
 * not: was the target ever achievable in the first place?
 */

/* Drag-aware predicted apogee at the commanded deployment, metres AGL.
 * This is the one used for control. */
float airbrake_get_predicted_apogee(void);

/* Old energy-method prediction, metres AGL. Computed for comparison ONLY -
 * never used for control. Ignores drag, so it always reads high. */
float airbrake_get_predicted_apogee_energy(void);

/* Last commanded deploy fraction actually applied, 0..1 (post slew limit). */
float airbrake_get_last_fraction(void);

/*
 * False when the target apogee lies outside what the brakes can achieve from
 * the current state - either below the full-brakes apogee (will overshoot) or
 * above the stowed apogee (will undershoot). In that case the commanded
 * fraction is pinned at the corresponding rail.
 *
 * This is a VEHICLE CAPABILITY signal, not a controller fault. On every L1410
 * test flight the 1800 m target was unreachable, and no amount of gain tuning
 * could have fixed it.
 */
bool airbrake_target_reachable(void);
void airbrake_log_zero_clock(void);

#endif /* AIRBRAKE_H */