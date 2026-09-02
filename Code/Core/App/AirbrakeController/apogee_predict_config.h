#ifndef APOGEE_PREDICT_CONFIG_H
#define APOGEE_PREDICT_CONFIG_H

/*
 * ============================================================================
 *  APOGEE PREDICTOR CONFIGURATION
 * ============================================================================
 *  UNITS: SI throughout (metres, m/s, seconds, kg, Pa, Kelvin).
 * ============================================================================
 */

/* ---- PHYSICAL CONSTANTS ----------------------------------------------------
 * NOTE: kalman.c currently uses 9.81f while airbrake_config.h uses 9.80665f.
 * The difference is negligible numerically (0.07%) but pick one project-wide.
 */
#define APOGEE_G                  9.80665f   /* gravity, m/s^2                */
#define APOGEE_R_AIR              287.058f   /* specific gas constant, dry air */

/* ---- VEHICLE ---------------------------------------------------------------
 * Reference area is the BODY CROSS-SECTION, not the brake paddle area. The Cd
 * values below are defined against this area - change one and you must change
 * the other.
 */
#define APOGEE_REF_AREA_M2        0.016286f  /* pi*(0.144/2)^2, 144 mm airframe */

/* ---- DRAG COEFFICIENTS -----------------------------------------------------
 * CD_RETRACTED   : OpenRocket component analysis, Mach 0.1-0.3 (flat: 0.327-0.331).
 *                  Weakly constrained by flight data (all flights spent most of
 *                  their influential coast with brakes out); flight best-fit was
 *                  0.375, reassuringly close. Treat +/-0.05 as indicative.
 *
 * CD_FULL_DEPLOY : joint fit across Flights 1, 2 and 3. RMS apogee error 2.6%;
 *                  band 0.88-1.04. Flight 1 anchors it (scripted open-loop test,
 *                  KNOWN deployment position); Flights 2 and 3 are out-of-sample
 *                  and land at -3.0% and +3.2%.
 *
 * CAUTION: video review suggests the brakes may not have physically reached
 * commanded deployment. If so the TRUE full-deployment Cd is higher than 0.95,
 * and this value describes drag actually achieved rather than mechanical full
 * travel. Servo-angle telemetry will settle it.
 *
 * Measured at Mach 0.2-0.3. No Mach dependence is modelled. If the competition
 * flight deploys brakes at meaningfully higher Mach, this carries more
 * uncertainty than the band above suggests - replace with a Cd(angle, Mach)
 * table once CFD is available.
 */
#define APOGEE_CD_RETRACTED       0.33f
#define APOGEE_CD_FULL_DEPLOY     0.95f

/* ---- INTEGRATION -----------------------------------------------------------
 * DT: prediction integration step. Checked against a 0.002 s reference on real
 *     Flight 3 telemetry: max difference 0.003 m across the coast. RK2 is doing
 *     the work here - the step size is not the limiting error, the Cd
 *     uncertainty is (+/-0.08 on Cd_full moves apogee by tens of metres).
 *     Smaller is not free: the solver runs this loop
 *     APOGEE_SOLVE_ITERATIONS+2 times per control cycle.
 *
 * MAX_STEPS: hard bound on loop iterations = bounded worst-case runtime.
 *     400 * 0.05 s = 20 s of predicted coast, comfortably longer than any coast
 *     flown so far (~9-13 s). If the loop ever hits this bound the prediction is
 *     truncated (returns a low apogee) rather than hanging - fail-safe direction
 *     for a brake controller, since it commands less deployment.
 */
#define APOGEE_PREDICT_DT_S       0.05f
#define APOGEE_PREDICT_MAX_STEPS  400

/* ---- INVERSE SOLVE ---------------------------------------------------------
 * Bisection iterations for apogee_required_fraction(). Each halves the bracket,
 * so 10 gives 1/1024 ~ 0.1% resolution in deployment fraction - far finer than
 * the servo can position, and finer than the Cd uncertainty justifies.
 */
#define APOGEE_SOLVE_ITERATIONS   10

/* ---- MASS ------------------------------------------------------------------
 * Post-burnout dry mass. The predictor only ever runs during coast, so the
 * propellant is already gone - do NOT use wet mass here.
 *   L1410 test config: 18.0 kg wet - 2.875 kg propellant = 15.125 kg.
 * MUST be updated for the competition motor and any mass changes.
 */
#define APOGEE_DRY_MASS_KG        15.125f

#endif /* APOGEE_PREDICT_CONFIG_H */
