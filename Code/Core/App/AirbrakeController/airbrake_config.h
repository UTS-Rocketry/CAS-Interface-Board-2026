#ifndef AIRBRAKE_CONFIG_H
#define AIRBRAKE_CONFIG_H

/*
 * ============================================================================
 *  AIRBRAKE CONTROLLER CONFIGURATION
 * ============================================================================
 *
 *  Every number a human might want to change lives here, separated from the
 *  control logic in airbrake.c. This is a deliberate software-engineering
 *  choice: tuning values change often (per flight, per motor, after test
 *  data), while the control *algorithm* changes rarely. Keeping them apart
 *  means you re-tune without touching tested logic.
 *
 *  UNITS: SI throughout (metres, m/s, seconds, kg). Pick one unit system and
 *  never mix - unit confusion is a classic cause of lost vehicles
 *  (cf. Mars Climate Orbiter). Altitudes are in METRES AGL (above ground
 *  level), not feet, not MSL.
 * ============================================================================
 */

/* ---- TARGET ----------------------------------------------------------------
 * The apogee we are trying to hit, in metres AGL.
 *   AURC competition target: 10,000 ft = 3048 m
 *   Test flights with smaller motors: set this lower to match the motor.
 * This is the single most important number to set correctly before a flight.
 */
#define AIRBRAKE_TARGET_APOGEE_M     1700.0f   /* 10,000 ft. CHANGE per flight. */

/* ---- PHYSICAL CONSTANTS ----------------------------------------------------*/
#define AIRBRAKE_G                   9.80665f  /* gravity, m/s^2 */

/* ---- PROPORTIONAL GAIN -----------------------------------------------------
 * Kp maps "how far we are predicted to overshoot" (metres) to "how far to
 * deploy the brakes" (fraction 0..1).
 *
 *   deploy_fraction = Kp * overshoot_metres
 *
 * Intuition: if Kp = 0.001, then a predicted 500 m overshoot commands
 * 0.001 * 500 = 0.5 = 50% deployment. A 1000 m overshoot would saturate to
 * 100%. Tune this so a *realistic* overshoot gives sensible deployment.
 *
 * Start SMALL and conservative. Too large -> brakes slam fully open at the
 * slightest overshoot, oscillate, and you lose energy unpredictably. Too
 * small -> brakes barely move and you overshoot the target. You will tune
 * this in simulation first, then refine on real flight data.
 */
#define AIRBRAKE_KP                  0.005f   /* deploy fraction per metre of overshoot */

/* ---- DEPLOYMENT LIMITS -----------------------------------------------------
 * Safety clamps on commanded deployment. The controller output is always
 * forced into [MIN, MAX]. For a FIRST flight you may deliberately cap MAX
 * below 1.0 (or set it to 0.0 to fly with brakes locked while you validate
 * the rest of the system) - see your own flight plan note about locking
 * brakes on flight one.
 */
#define AIRBRAKE_DEPLOY_MIN          0.0f      /* fully stowed */
#define AIRBRAKE_DEPLOY_MAX          1.0f      /* fully deployed; lower to limit authority */

/* ---- ACTIVATION GATES ------------------------------------------------------
 * The controller must only act during the COAST phase: after motor burnout
 * (no more thrust) and before apogee. Outside this window the brakes stay
 * stowed. These gates are defence-in-depth in case the FSM state is ever
 * wrong or noisy.
 *
 * MIN_SPEED: below this upward speed we're near apogee - stop actuating to
 *   avoid twitching the servo at the top where it does nothing useful.
 * The controller also refuses to act unless velocity is upward (ascending),
 * because energy-method apogee prediction only makes sense while climbing.
 */
#define AIRBRAKE_MIN_COAST_SPEED_MS  5.0f     /* stop actuating below this upward speed */

/* ---- SLEW LIMIT (rate limiting) --------------------------------------------
 * Maximum change in deployment fraction per control update. This prevents the
 * controller from commanding instantaneous full-throw moves that the servo
 * can't physically achieve and that would jerk the airframe. It smooths the
 * command. Set based on your control rate and how fast you want the brakes to
 * be allowed to move.
 *
 * Example: at 20 Hz (dt = 0.05 s) a slew of 0.05 means the brakes can move at
 * most 0.05 fraction per cycle = 1.0 fraction per second (full travel in 1 s).
 */
#define AIRBRAKE_MAX_SLEW_PER_UPDATE 0.08f     /* max change in fraction per call */


#define AIRBRAKE_V_BRAKE_MAX_MS      250.0f   // structural/Mach cap - from sim
#define AIRBRAKE_ALT_BRAKE_MIN_M     300.0f   // clear of rail transient

//#define AIRBRAKE_MIN_COAST_SPEED_MS   20.0f

#define AIRBRAKE_SLEW_RATE_PER_SEC     2.0f   // fraction/sec

//#define AIRBRAKE_DEPLOY_MIN            0.0f
//#define AIRBRAKE_DEPLOY_MAX            1.0f

//gain
// #define AIRBRAKE_KP                    0.05f

//#define AIRBRAKE_G                     9.80665f

//#define AIRBRAKE_V_BRAKE_MAX_MS        250.0f   /* structural/Mach cap - FROM SIM */
//#define AIRBRAKE_ALT_BRAKE_MIN_M       300.0f   /* clear of rail transient - FROM SIM */

#endif /* AIRBRAKE_CONFIG_H */