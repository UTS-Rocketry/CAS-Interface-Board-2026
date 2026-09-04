#include "apogee_predict.h"
#include "apogee_predict_config.h"

#include <math.h>

/*
 * ============================================================================
 *  VALIDATION (flight data, Cesaroni L1410, 25 Hz flash logs)
 * ============================================================================
 *
 *  Mean absolute error of predicted vs. actual apogee, evaluated at every
 *  logged record from t = 3.6 s after ignition (clear of burnout) to apogee:
 *
 *      Flight    energy method      this model      improvement
 *      -------   --------------     -----------     -----------
 *      1              34.5 m           12.0 m           2.9x
 *      2             198.1 m           11.3 m          17.6x
 *      3             197.1 m           11.8 m          16.6x
 *
 *  Worst-case error also collapses: Flight 3 peaked at 1134.9 m with the energy
 *  method and 47.2 m with this one.
 *
 *  Flight 4 is excluded: its tumble corrupted the Kalman velocity estimate
 *  badly enough that altitude and velocity disagree (logged velocity cannot
 *  reach logged apogee even with zero drag), so it cannot validate anything.
 *
 *  Drag model: Cd interpolated linearly between CD_RETRACTED (0.33, from
 *  OpenRocket) and CD_FULL_DEPLOY (0.95, joint fit across Flights 1-3, RMS
 *  apogee error 2.6%). The linear interpolation between them is an ASSUMPTION -
 *  no intermediate deployment angle has been measured. Replace with a proper
 *  Cd(angle, Mach) table when CFD lands.
 * ============================================================================
 */

/* ---- numerical integration ------------------------------------------------
 * RK2 (midpoint). Plain Euler needs a much smaller step for the same accuracy
 * because drag is strongly nonlinear in v; midpoint buys that back cheaply.
 *
 * Cost: no exp(), no pow(), no divisions inside the loop (see the density
 * comment below). Roughly 250 steps x ~12 flops for a typical coast, so a few
 * tens of microseconds on an F405 with the FPU enabled - negligible against a
 * 40 ms control period, even with the bisection solve running 10 of them.
 */

static float clampf(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

float apogee_predict(const ApogeeState *s, float deploy_fraction)
{
    /* --- guards: never let a bad sample produce a wild command ---
     * NaN needs its own explicit check: `x < 150.0f` and `x > 400.0f` are
     * BOTH false when x is NaN, so the range checks below silently let a NaN
     * temperature or mass through instead of catching it - same failure
     * class as an unguarded comparison anywhere else in the pipeline. */
    if (s == NULL) return 0.0f;
    if (isnan(s->altitude_m) || isnan(s->velocity_ms) ||
        isnan(s->pressure_pa) || isnan(s->temperature_k) || isnan(s->mass_kg)) {
        return s->altitude_m;
    }
    if (s->velocity_ms <= 0.0f) return s->altitude_m;   /* past apogee */
    if (s->mass_kg < 0.1f)      return s->altitude_m;   /* nonsense mass */
    if (s->pressure_pa <= 0.0f) return s->altitude_m;   /* nonsense pressure - was unguarded */
    if (s->temperature_k < 150.0f || s->temperature_k > 400.0f) {
        return s->altitude_m;                            /* bad temp reading */
    }

    const float frac = clampf(deploy_fraction, 0.0f, 1.0f);
    const float Cd   = APOGEE_CD_RETRACTED
                     + (APOGEE_CD_FULL_DEPLOY - APOGEE_CD_RETRACTED) * frac;

    /* Air density NOW, from the vehicle's own sensors (ideal gas law).
     * Using measured rho rather than a standard-atmosphere table removes the
     * error from launch-site pressure, altitude and temperature in one step. */
    float rho = s->pressure_pa / (APOGEE_R_AIR * s->temperature_k);

    /* Drag acceleration = k * rho * v^2. Precompute k once. */
    const float k = 0.5f * Cd * APOGEE_REF_AREA_M2 / s->mass_kg;

    /* Density falls off exponentially with altitude: rho(h) = rho0*exp(-dh/H),
     * scale height H = R*T/g (~8600 m at 20 C).
     *
     * Rather than call exp() every step, note that each step climbs only
     * v*dt (~5-10 m) so dh/H ~ 1e-3. Over that range the series
     *     exp(-x) ~= 1 - x + x^2/2
     * is accurate to ~1e-10 - far below sensor noise - and costs 3 flops
     * instead of a libm call. We apply it incrementally as a multiplier. */
    const float inv_H = APOGEE_G / (APOGEE_R_AIR * s->temperature_k);

    float h  = s->altitude_m;
    float v  = s->velocity_ms;
    const float dt = APOGEE_PREDICT_DT_S;

    for (int i = 0; i < APOGEE_PREDICT_MAX_STEPS; ++i) {
        if (v <= 0.0f) break;

        const float a = -APOGEE_G - k * rho * v * v;

        /* midpoint state */
        const float v_mid = v + 0.5f * a * dt;
        if (v_mid <= 0.0f) {
            /* Apogee falls inside this step. Close it analytically with the
             * energy relation using the CURRENT total deceleration, instead of
             * overshooting or truncating - keeps the last step from being the
             * dominant error term. */
            h += (v * v) / (2.0f * (APOGEE_G + k * rho * v * v));
            v = 0.0f;
            break;
        }

        /* density at the midpoint altitude, incremental exp approximation */
        const float dh_mid = 0.5f * v * dt;
        const float x_mid  = dh_mid * inv_H;
        const float rho_mid = rho * (1.0f - x_mid + 0.5f * x_mid * x_mid);

        const float a_mid = -APOGEE_G - k * rho_mid * v_mid * v_mid;

        /* advance a full step using midpoint derivatives */
        const float dh = v_mid * dt;
        h += dh;
        v += a_mid * dt;

        /* carry density forward to the new altitude */
        const float x = dh * inv_H;
        rho *= (1.0f - x + 0.5f * x * x);
    }

    return h;
}

float apogee_required_fraction(const ApogeeState *s,
                               float target_apogee_m,
                               bool *out_reachable)
{
    if (out_reachable) *out_reachable = true;

    if (s == NULL || s->velocity_ms <= 0.0f) {
        if (out_reachable) *out_reachable = false;
        return 0.0f;
    }

    /* Bound the problem first. Two integrations tell us whether the target is
     * even achievable; if it is not, there is no point searching. */
    const float apogee_full   = apogee_predict(s, 1.0f);
    const float apogee_stowed = apogee_predict(s, 0.0f);

    if (apogee_full > target_apogee_m) {
        /* Even 100% brakes overshoot. Deploy everything and flag it: this is a
         * vehicle-capability problem, not something the gain can fix. */
        if (out_reachable) *out_reachable = false;
        return 1.0f;
    }
    if (apogee_stowed < target_apogee_m) {
        /* Already going to undershoot - brakes can only remove energy. */
        if (out_reachable) *out_reachable = false;
        return 0.0f;
    }

    /* Predicted apogee decreases monotonically with deployment, so bisection is
     * safe and needs no derivative. Fixed iteration count = bounded,
     * deterministic runtime, which matters more than the last bit of precision
     * in a hard-real-time loop. */
    float lo = 0.0f;   /* apogee(lo) >= target */
    float hi = 1.0f;   /* apogee(hi) <= target */

    for (int i = 0; i < APOGEE_SOLVE_ITERATIONS; ++i) {
        const float mid = 0.5f * (lo + hi);
        if (apogee_predict(s, mid) > target_apogee_m) {
            lo = mid;
        } else {
            hi = mid;
        }
    }

    return clampf(0.5f * (lo + hi), 0.0f, 1.0f);
}