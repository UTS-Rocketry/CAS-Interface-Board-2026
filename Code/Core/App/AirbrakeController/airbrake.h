#ifndef AIRBRAKE_H
#define AIRBRAKE_H

#include <stdint.h>
#include <stdbool.h>

/*
 * ============================================================================
 *  AIRBRAKE CONTROLLER - public interface
 * ============================================================================
 *
 *  DESIGN PRINCIPLE: controller-agnostic interface.
 *
 *  The functions below describe WHAT the controller does (take state, return a
 *  deploy fraction) without exposing HOW (proportional now; PID or LQR or MPC
 *  later). main.c and the FSM only ever call this interface. That means you
 *  can completely rewrite the algorithm inside airbrake.c - swap the simple
 *  proportional law for LQR after you have flight data - WITHOUT changing a
 *  single line in main.c or the servo driver. This separation (interface vs
 *  implementation) is one of the most important ideas in embedded/systems
 *  software. It is why we can fly a dumb-but-safe controller first and upgrade
 *  later with zero risk to the surrounding plumbing.
 *
 *  THE ABSTRACTION: the controller speaks in "deploy fraction" (0.0 = stowed,
 *  1.0 = fully deployed), NOT microseconds or servo angles. It knows physics,
 *  not hardware. The mapping from fraction to servo pulse lives at the
 *  boundary (servo_set_fraction). This keeps the controller unit-testable on a
 *  PC with no hardware: feed it altitude/velocity, check the fraction it
 *  returns.
 * ============================================================================
 */

/* Reset internal state. Call once before flight (e.g. at FSM init), and the
 * controller assumes brakes start stowed. */
void airbrake_init(void);

/*
 * Run one control update.
 *
 *   altitude_m : current altitude AGL in metres (from Kalman filter)
 *   velocity_ms: current VERTICAL velocity in m/s, POSITIVE UP (from Kalman)
 *   dt_s       : time since last update, seconds (for slew limiting)
 *
 * Returns the commanded deploy fraction in [0.0, 1.0].
 *
 * This function is PURE in spirit: same inputs -> same output (aside from the
 * remembered previous fraction used for slew limiting). It does NOT touch the
 * servo, the FSM, or any global - it just computes a number. The caller
 * decides whether to apply it. That makes it trivial to test and reason about.
 */
float airbrake_update(float altitude_m, float velocity_ms, float dt_s);

/* For telemetry / debugging: expose what the controller last computed. */
float airbrake_get_predicted_apogee(void);   /* metres AGL */
float airbrake_get_last_fraction(void);       /* 0..1 */

#endif /* AIRBRAKE_H */