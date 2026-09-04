#include "airbrake.h"
#include "airbrake_config.h"
#include "flight_sensors.h"
#include "servo.h"
#include "flight_state.h"
#include "apogee_predict.h"
#include "apogee_predict_config.h"

static float s_last_fraction;        /* previous deploy command, for slew limiting */
static float s_predicted_apogee;     /* drag-aware prediction, for telemetry       */
static float s_predicted_apogee_energy; /* old energy-method prediction, LOGGED ONLY */
static bool  s_target_reachable;     /* false => target outside brake authority    */

/* Force x into [lo, hi]. Saturation is fundamental to real actuators: a servo
 * cannot deploy to 1.3 or -0.2, so every controller MUST clamp its output. */
static float clampf(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

/*
 * Energy-method apogee prediction:  h_apogee = h + v^2 / (2g)
 *
 * NO LONGER USED FOR CONTROL. Retained purely so both predictions can be
 * logged side by side and compared on real flight data.
 *
 * It ignores drag, so it always predicts high. Against flight data (Flights 1-3,
 * every record from t = 3.6 s to apogee) its mean absolute error was 34.5 / 198.1
 * / 197.1 m, peaking at 1134.9 m early in Flight 3's coast - exactly when the
 * controller has the most authority and most needs a good number. The drag-aware
 * predictor scored 12.0 / 11.3 / 11.8 m on the same records.
 *
 * Keep logging both for at least one flight: if the drag-aware number does not
 * beat this one in flight the way it does offline, something is wrong with the
 * sensor plumbing, and that is much better learned from a log than from an
 * apogee.
 */
static float predict_apogee_energy(float altitude_m, float velocity_ms) {
    return altitude_m + (velocity_ms * velocity_ms) / (2.0f * AIRBRAKE_G);
}

static bool airbrakes_allowed(const FlightSensorData *d) {
    /* Uses d->flight_state rather than FSM_get_state() so the gate decision and
     * the control calculation below are made from the SAME snapshot, instead of
     * a state that could change between the two calls. */
    if (d->flight_state != STATE_COAST)                    return false;
    if (d->kalman_velocity <= 0.0f)                        return false;
    if (d->kalman_velocity > AIRBRAKE_V_BRAKE_MAX_MS)      return false;
    if (d->kalman_altitude < AIRBRAKE_ALT_BRAKE_MIN_M)     return false;
    if (d->kalman_velocity < AIRBRAKE_MIN_COAST_SPEED_MS)  return false;
    return true;
}

/* ---- public API ----------------------------------------------------------- */

void airbrake_init(void) {
    s_last_fraction           = 0.0f;   /* assume brakes start stowed */
    s_predicted_apogee        = 0.0f;
    s_predicted_apogee_energy = 0.0f;
    s_target_reachable        = true;
}

float airbrake_update(const FlightSensorData *data, float dt)
{
    /*
     * STEP 0 - GATING / SAFETY INTERLOCKS
     * Defence in depth: the caller should already only invoke this during
     * COAST, and airbrakes_allowed() checks again. A single bug in either
     * place cannot deploy brakes at the wrong time.
     */
    if (!airbrakes_allowed(data)) {
        servo_set_us(SERVO_AIRBRAKE, SERVO_US_MIN);   /* stow */
        s_last_fraction = 0.0f;
        return s_last_fraction;
    }

    /*
     * STEP 1 - ASSEMBLE STATE FROM LIVE SENSORS
     *
     * UNITS - verify these against the sensor driver before flight:
     *   pressure    must be PASCALS. Flash logs read ~99700, which is Pa, but
     *               some BMP388 drivers return hPa. If hPa slipped through,
     *               density comes out 100x too low, drag effectively vanishes,
     *               and this predictor silently degrades to the energy method -
     *               no crash, no warning, just quietly wrong numbers.
     *   temperature must be CELSIUS here (converted to Kelvin below). A missed
     *               273.15 corrupts density by ~13x.
     *
     * The debug block below prints computed density once so this can be checked
     * on the bench: expect ~1.0-1.25 kg/m^3 near the ground. If it reads ~0.01,
     * pressure is in hPa.
     */
    ApogeeState st;
    st.altitude_m    = data->kalman_altitude;
    st.velocity_ms   = data->kalman_velocity;
    st.pressure_pa   = data->pressure;
    st.temperature_k = data->temperature + 273.15f;
    st.mass_kg       = APOGEE_DRY_MASS_KG;   /* DRY mass - coast only, propellant gone */

    /*
     * STEP 2 - SOLVE FOR THE REQUIRED DEPLOYMENT (inverse-model feedforward)
     *
     * With a validated drag model we can solve directly for the deployment that
     * lands on target, rather than nudging toward it with a gain and hoping.
     *
     * This is NOT open loop. The solve is redone every cycle from freshly
     * MEASURED altitude and velocity, so model error corrects itself: if Cd is
     * wrong, the real trajectory diverges from the prediction, the next cycle
     * measures where we actually are, and the new solve accounts for it. That
     * is receding-horizon feedback - the same principle as MPC - and it is why
     * no separate proportional trim term is needed here.
     *
     * (A P trim on top of this solve would be a no-op anyway: the bisection
     * converges until predicted apogee equals target, so the residual it would
     * act on is the solver's own convergence error, ~1 m, not real model error.)
     *
     * s_target_reachable goes false when the target lies outside what the brakes
     * can do from the current state. LOG IT. On every L1410 test flight the
     * 1800 m target was below the full-brakes apogee - unreachable - and nothing
     * in the telemetry said so. Flight 3 flew 2170 m when the best achievable
     * was ~2179 m: the controller was already at the floor, and no amount of
     * gain tuning could have helped.
     */
    float commanded = apogee_required_fraction(&st,
                                               AIRBRAKE_TARGET_APOGEE_M,
                                               &s_target_reachable);

    /*
     * STEP 3 - PREDICT AT THE COMMANDED DEPLOYMENT (telemetry)
     * Where we are actually heading given what we just commanded.
     */
    s_predicted_apogee = apogee_predict(&st, commanded);

    /* Old predictor, computed for comparison only - never used for control. */
    s_predicted_apogee_energy = predict_apogee_energy(st.altitude_m, st.velocity_ms);

#ifdef DEBUG
    /* Keep DEBUG OFF for flight builds: a blocking UART write inside a 40 ms
     * control loop costs milliseconds and jitters dt. */
    printf("apogee drag=%.1f energy=%.1f | cmd=%.3f reach=%d | rho=%.3f\n\r",
           s_predicted_apogee,
           s_predicted_apogee_energy,
           commanded,
           (int)s_target_reachable,
           st.pressure_pa / (287.058f * st.temperature_k));
#endif

    /*
     * STEP 4 - SATURATE
     * apogee_required_fraction() already returns a value in [0,1], but clamp
     * against the configured limits, which may be tighter (e.g. DEPLOY_MAX
     * lowered to limit authority on an early flight).
     */
    commanded = clampf(commanded, AIRBRAKE_DEPLOY_MIN, AIRBRAKE_DEPLOY_MAX);

    /*
     * STEP 5 - SLEW LIMIT
     * Respects how fast the servo can physically move and avoids jerking the
     * airframe with instantaneous full-throw commands.
     *
     * dt is clamped first: see AIRBRAKE_MAX_DT_S in airbrake_config.h for why
     * an unclamped dt can silently defeat this limiter on the first call.
     */
    const float dt_bounded = clampf(dt, 0.0f, AIRBRAKE_MAX_DT_S);
    const float max_step = AIRBRAKE_SLEW_RATE_PER_SEC * dt_bounded;
    const float step     = clampf(commanded - s_last_fraction, -max_step, max_step);
    s_last_fraction = clampf(s_last_fraction + step,
                             AIRBRAKE_DEPLOY_MIN, AIRBRAKE_DEPLOY_MAX);

    servo_set_fraction(SERVO_AIRBRAKE, s_last_fraction);
    return s_last_fraction;
}

float airbrake_get_predicted_apogee(void)        { return s_predicted_apogee;        }
float airbrake_get_predicted_apogee_energy(void) { return s_predicted_apogee_energy; }
float airbrake_get_last_fraction(void)           { return s_last_fraction;           }
bool  airbrake_target_reachable(void)            { return s_target_reachable;        }