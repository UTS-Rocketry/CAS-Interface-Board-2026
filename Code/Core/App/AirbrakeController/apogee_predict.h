#ifndef APOGEE_PREDICT_H
#define APOGEE_PREDICT_H

#include <stdbool.h>

/*
 * ============================================================================
 *  DRAG-AWARE APOGEE PREDICTION
 * ============================================================================
 *
 *  Replaces the energy method  h_apogee = h + v^2/(2g).
 *
 *  The energy method ignores drag, so it always predicts too high. On flight
 *  data that error was ~200 m on average and up to ~1100 m early in coast -
 *  exactly when the controller has the most authority and most needs a good
 *  number. Forward-integrating with a real drag model cut mean error to ~12 m
 *  (see apogee_predict.c header comment for the per-flight validation).
 *
 *  Two entry points:
 *
 *    apogee_predict()          "where am I heading if brakes stay put?"
 *    apogee_required_fraction() "what deployment lands me on target?"
 *
 *  The second is an inverse-model feedforward: it solves directly for the
 *  answer instead of nudging toward it. Use it as the primary command and keep
 *  a small proportional term for trim. That is a strictly better use of a known
 *  drag model than feeding a better prediction into a pure P law.
 * ============================================================================
 */

/* Live atmospheric state, taken from the barometer - NOT a standard-atmosphere
 * table. The vehicle already measures pressure and temperature every cycle;
 * using them is both more accurate and cheaper than a lookup. */
typedef struct {
    float altitude_m;      /* current altitude, m AGL (Kalman)  */
    float velocity_ms;     /* current vertical velocity, m/s (Kalman), positive up */
    float pressure_pa;     /* measured static pressure, Pa      */
    float temperature_k;   /* measured temperature, KELVIN      */
    float mass_kg;         /* CURRENT vehicle mass - post-burnout dry mass */
} ApogeeState;

/*
 * Predict apogee with the airbrakes held at `deploy_fraction` (0.0 = stowed,
 * 1.0 = fully deployed) for the remainder of the coast.
 *
 * Returns the predicted apogee in metres AGL. If already descending
 * (velocity <= 0) it returns the current altitude.
 */
float apogee_predict(const ApogeeState *s, float deploy_fraction);

/*
 * Solve for the deployment fraction whose predicted apogee equals
 * `target_apogee_m`.
 *
 * Returns the required fraction, clamped to [0,1].
 *
 * `out_reachable` (may be NULL) is set false when the target lies outside what
 * the brakes can achieve from the current state:
 *   - target below the full-brakes apogee  -> returns 1.0, unreachable (will overshoot)
 *   - target above the stowed apogee       -> returns 0.0, unreachable (will undershoot)
 * Log this flag. On the L1410 test flights the 1800 m target was unreachable on
 * every single flight, and nothing visible in the old telemetry said so.
 */
float apogee_required_fraction(const ApogeeState *s,
                               float target_apogee_m,
                               bool *out_reachable);

#endif /* APOGEE_PREDICT_H */
