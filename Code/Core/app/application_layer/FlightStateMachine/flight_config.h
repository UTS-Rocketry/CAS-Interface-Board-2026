#ifndef FLIGHT_CONFIG_H
#define FLIGHT_CONFIG_H

/* ── Launch detect ─────────────────────────────────── */
#define LAUNCH_ACCEL_THRESHOLD_MG       3000.0f   /* 3g on IMU z axis */
#define LAUNCH_CONFIRM_SAMPLES          5          /* consecutive samples */

/* ── Burnout detect ────────────────────────────────── */
#define BURNOUT_ACCEL_THRESHOLD_MG      2000.0f   /* back to ~1g after motor out */
#define BURNOUT_CONFIRM_SAMPLES         5

/* ── Apogee detect ─────────────────────────────────── */
#define APOGEE_VELOCITY_THRESHOLD       -0.5f     /* m/s, negative = descending */
#define APOGEE_CONFIRM_SAMPLES          10

/* ── Deployment altitudes ──────────────────────────── */
#define MAIN_DEPLOY_ALT_M               300.0f    /* AGL meters */

/* ── Landing detect ────────────────────────────────── */
#define LAND_VELOCITY_THRESHOLD         0.5f      /* m/s absolute */
#define LAND_ALT_THRESHOLD_M            10.0f     /* AGL meters */

/* ── State timeouts ────────────────────────────────── */
#define BOOST_TIMEOUT_MS                10000     /* 10s max burn */
#define COAST_TIMEOUT_MS                60000     /* 30s max coast */
#define DROGUE_TIMEOUT_MS               300000     
#define PARAFOIL_TIMEOUT_MS             300000    /* 5 min max descent */

/* ── Arming ────────────────────────────────────────── */
#define ARM_AUTO_DELAY_MS               10000     /* temp: auto arm after 10s */
#define MAIN_ALT_CONFIRM_SAMPLES        5
/* ── Airbrake control ──────────────────────────────── */
/*
 * Keep this disabled until the rocket-specific values below are filled from
 * sim/test data. With it disabled, the airbrake module always commands retract.
 */
#define AIRBRAKE_CONTROL_ENABLED        0

/* ABG estimator gains. These are starter values and should be tuned from logs. */
#define AIRBRAKE_ABG_ALPHA              0.65f
#define AIRBRAKE_ABG_BETA               0.20f
#define AIRBRAKE_ABG_GAMMA              0.02f

/* Mission target. Set this to the desired apogee above ground level. */
#define AIRBRAKE_TARGET_APOGEE_M        0.0f

/* Rocket aero model after burnout. Must be set before enabling control. */
#define AIRBRAKE_ROCKET_MASS_KG         0.0f
#define AIRBRAKE_REFERENCE_AREA_M2      0.0f
#define AIRBRAKE_CD_CLEAN               0.0f
#define AIRBRAKE_CD_FULL_DEPLOY         0.0f
#define AIRBRAKE_AIR_DENSITY_KG_M3      1.225f
#define AIRBRAKE_GRAVITY_MPS2           9.80665f

/* Actuator mapping: deployment 0.0 -> closed, 1.0 -> full deploy. */
#define AIRBRAKE_SERVO_CLOSED_US        1000.0f
#define AIRBRAKE_SERVO_FULL_US          2000.0f
#define AIRBRAKE_MAX_DEPLOYMENT         1.0f
#define AIRBRAKE_MAX_DEPLOY_RATE_PER_S  1.0f

/* Predictor/controller timing. */
#define AIRBRAKE_CONTROL_DT_FALLBACK_S  0.02f
#define AIRBRAKE_PREDICT_DT_S           0.02f
#define AIRBRAKE_PREDICT_MAX_STEPS      4000
#define AIRBRAKE_SEARCH_STEPS           20
#define AIRBRAKE_TARGET_DEADBAND_M      5.0f
#define AIRBRAKE_MIN_COAST_VELOCITY_MPS 1.0f
#define AIRBRAKE_MAX_VALID_DT_S         0.25f


#endif
