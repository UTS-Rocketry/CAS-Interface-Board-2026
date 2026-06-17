#include "airbrake.h"
#include "airbrake_config.h"

/*
 * ============================================================================
 *  AIRBRAKE CONTROLLER - implementation
 * ============================================================================
 *
 *  THE CONTROL PROBLEM IN ONE PARAGRAPH
 *  ------------------------------------
 *  We want to hit a target apogee. We cannot command apogee directly - we can
 *  only add drag, right now, by deploying brakes. So each cycle we (1) predict
 *  the apogee we are currently heading for, (2) compare it to the target to get
 *  an error, (3) deploy brakes in proportion to that error. Deploying brakes
 *  adds drag, bleeds energy, and lowers the apogee we'll actually reach. It is
 *  a feedback loop: predict -> compare -> actuate -> (physics responds) ->
 *  predict again. This is the essence of closed-loop control.
 *
 *  WHY "FEEDBACK" AND NOT "OPEN LOOP"
 *  ----------------------------------
 *  An open-loop approach would be "deploy brakes on a fixed schedule." That
 *  fails because every flight differs - motor impulse varies, winds vary, mass
 *  varies. Feedback measures the actual state and corrects, so it is robust to
 *  those variations. The price is you must measure state well (your Kalman
 *  filter) and predict sensibly (here, the energy method).
 * ============================================================================
 */

/* ---- internal state (remembered between calls) ---------------------------- */
static float s_last_fraction;       /* previous deploy command, for slew limiting */
static float s_predicted_apogee;    /* last predicted apogee, for telemetry */

/* ---- small helpers -------------------------------------------------------- */

/* Force x into [lo, hi]. Saturation is fundamental to real actuators: a servo
 * cannot deploy to 1.3 or -0.2, so every controller MUST clamp its output. */
static float clampf(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

/*
 * Predict apogee from current state using the ENERGY METHOD.
 *
 *   Conservation of mechanical energy during coast, ignoring drag:
 *       KE_now + PE_now = PE_apogee     (KE at apogee = 0, since v = 0 there)
 *       (1/2) m v^2 + m g h = m g h_apogee
 *   Mass cancels:
 *       h_apogee = h + v^2 / (2 g)
 *
 *  The term v^2/(2g) is "how much more height this velocity buys us if gravity
 *  were the only force." Because we ignore drag, this OVERESTIMATES apogee (real
 *  drag will steal some of that energy). Overestimating is the SAFE direction
 *  for airbrakes: it makes the controller deploy slightly early/eager, biasing
 *  toward undershoot rather than blowing past the target.
 *
 *  Limitation: the overestimate is worst when fast (drag ~ v^2 is largest), and
 *  shrinks as we slow near apogee. Once you have flight data you'll either add a
 *  drag term (-> forward integration) or fold the bias into Kp.
 */
static float predict_apogee_energy(float altitude_m, float velocity_ms) {
    return altitude_m + (velocity_ms * velocity_ms) / (2.0f * AIRBRAKE_G);
}

/* ---- public API ----------------------------------------------------------- */

void airbrake_init(void) {
    s_last_fraction   = 0.0f;   /* assume brakes start stowed */
    s_predicted_apogee = 0.0f;
}

float airbrake_update(float altitude_m, float velocity_ms, float dt_s) {
    /*
     * STEP 0 - GATING / SAFETY INTERLOCKS
     * -----------------------------------
     * The controller must do nothing unless we are genuinely coasting upward
     * fast enough to matter. Two guards:
     *
     *   (a) velocity must be UP (positive). The energy prediction is only
     *       meaningful while ascending. If velocity is negative we're past
     *       apogee (descending) - brakes must be stowed.
     *   (b) speed must exceed MIN_COAST_SPEED. Near the top the rocket is
     *       barely moving; actuating there just twitches the servo for no
     *       benefit and risks leaving brakes out into descent.
     *
     * Note: this is defence-in-depth. The CALLER should also only invoke the
     * controller during the FSM's COAST state. Gating in both places means a
     * single bug (wrong FSM state, or a caller mistake) cannot deploy brakes at
     * the wrong time. Layered safety is standard practice for actuators that
     * can affect a vehicle's trajectory.
     */
    if (velocity_ms < AIRBRAKE_MIN_COAST_SPEED_MS) {
        /* not ascending fast enough (or descending): retract and reset target */
        s_predicted_apogee = altitude_m;
        /* slew toward stowed rather than snapping, for mechanical kindness */
        float target = AIRBRAKE_DEPLOY_MIN;
        float delta  = clampf(target - s_last_fraction,
                              -AIRBRAKE_MAX_SLEW_PER_UPDATE,
                               AIRBRAKE_MAX_SLEW_PER_UPDATE);
        s_last_fraction = clampf(s_last_fraction + delta,
                                 AIRBRAKE_DEPLOY_MIN, AIRBRAKE_DEPLOY_MAX);
        return s_last_fraction;
    }

    /*
     * STEP 1 - PREDICT
     * ----------------
     * Where are we heading right now?
     */
    s_predicted_apogee = predict_apogee_energy(altitude_m, velocity_ms);

    /*
     * STEP 2 - ERROR
     * --------------
     * error = predicted - target.
     *   error > 0 : predicted to OVERSHOOT -> we need MORE drag -> deploy.
     *   error <= 0 : predicted to undershoot or hit target -> NO brakes
     *               (brakes can only ADD drag; they cannot add energy, so
     *                there is nothing to do if we're already low).
     */
    float overshoot_m = s_predicted_apogee - AIRBRAKE_TARGET_APOGEE_M;

    /*
     * STEP 3 - PROPORTIONAL CONTROL LAW
     * ---------------------------------
     * The simplest feedback law: command is proportional to error.
     *
     *     raw_command = Kp * overshoot
     *
     * This is the "P" in PID. It says: the more we're predicted to overshoot,
     * the harder we brake. If we're only slightly high, deploy a little; if
     * we're way high, deploy a lot. Kp sets the aggressiveness (see config).
     *
     * Why start with P only (no I, no D)?
     *   - I (integral) accumulates past error to kill steady-state offset, but
     *     it suffers "windup" when the actuator saturates (brakes maxed out) -
     *     a real hazard here. Adding I safely needs anti-windup logic. Skip
     *     until needed.
     *   - D (derivative) reacts to the RATE of error change for damping, but it
     *     amplifies sensor noise. Your altitude/velocity come from a Kalman
     *     filter that already smooths things; add D later if you see oscillation.
     *   P alone is robust, predictable, and easy to tune. It is the correct
     *   first controller. Earn complexity only when the simple thing proves
     *   insufficient.
     */
    float raw_command = AIRBRAKE_KP * overshoot_m;

    /*
     * STEP 4 - SATURATE
     * -----------------
     * Clamp into the physically achievable range. If overshoot is huge,
     * raw_command might be 3.0 - but the brakes max out at 1.0. The clamp is
     * not optional; it reflects reality.
     */
    float commanded = clampf(raw_command, AIRBRAKE_DEPLOY_MIN, AIRBRAKE_DEPLOY_MAX);

    /*
     * STEP 5 - SLEW LIMIT (rate limit)
     * --------------------------------
     * Don't allow the command to jump further than MAX_SLEW per update. This
     * smooths motion, respects how fast the servo can actually move, and avoids
     * jerking the airframe with instantaneous full-throw commands. We move the
     * previous command toward the new target by at most one slew step.
     *
     * (dt is available if you want slew expressed per-second instead of
     * per-update; here we keep it per-update for simplicity. To make it
     * time-based: max_step = SLEW_RATE_PER_SEC * dt_s.)
     */
    (void)dt_s;  /* unused for now; kept in the signature for time-based slew later */
    float step = clampf(commanded - s_last_fraction,
                        -AIRBRAKE_MAX_SLEW_PER_UPDATE,
                         AIRBRAKE_MAX_SLEW_PER_UPDATE);
    s_last_fraction = clampf(s_last_fraction + step,
                             AIRBRAKE_DEPLOY_MIN, AIRBRAKE_DEPLOY_MAX);

    return s_last_fraction;
}

float airbrake_get_predicted_apogee(void) { 
    return s_predicted_apogee; 
}
float airbrake_get_last_fraction(void) { 
    return s_last_fraction; 
}