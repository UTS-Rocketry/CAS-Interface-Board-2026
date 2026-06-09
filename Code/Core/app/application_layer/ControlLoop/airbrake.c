#include "airbrake.h"
#include "flight_config.h"
#include <math.h>
#include <string.h>

#ifndef AIRBRAKE_CONTROL_ENABLED
#define AIRBRAKE_CONTROL_ENABLED        0
#endif

#ifndef AIRBRAKE_TARGET_APOGEE_M
#define AIRBRAKE_TARGET_APOGEE_M        0.0f
#endif

#ifndef AIRBRAKE_ROCKET_MASS_KG
#define AIRBRAKE_ROCKET_MASS_KG         0.0f
#endif

#ifndef AIRBRAKE_REFERENCE_AREA_M2
#define AIRBRAKE_REFERENCE_AREA_M2      0.0f
#endif

#ifndef AIRBRAKE_CD_CLEAN
#define AIRBRAKE_CD_CLEAN               0.0f
#endif

#ifndef AIRBRAKE_CD_FULL_DEPLOY
#define AIRBRAKE_CD_FULL_DEPLOY         0.0f
#endif

#ifndef AIRBRAKE_AIR_DENSITY_KG_M3
#define AIRBRAKE_AIR_DENSITY_KG_M3      1.225f
#endif

#ifndef AIRBRAKE_GRAVITY_MPS2
#define AIRBRAKE_GRAVITY_MPS2           9.80665f
#endif

#ifndef AIRBRAKE_ABG_ALPHA
#define AIRBRAKE_ABG_ALPHA              0.65f
#endif

#ifndef AIRBRAKE_ABG_BETA
#define AIRBRAKE_ABG_BETA               0.20f
#endif

#ifndef AIRBRAKE_ABG_GAMMA
#define AIRBRAKE_ABG_GAMMA              0.02f
#endif

#ifndef AIRBRAKE_SERVO_CLOSED_US
#define AIRBRAKE_SERVO_CLOSED_US        1000.0f
#endif

#ifndef AIRBRAKE_SERVO_FULL_US
#define AIRBRAKE_SERVO_FULL_US          2000.0f
#endif

#ifndef AIRBRAKE_MAX_DEPLOYMENT
#define AIRBRAKE_MAX_DEPLOYMENT         1.0f
#endif

#ifndef AIRBRAKE_MAX_DEPLOY_RATE_PER_S
#define AIRBRAKE_MAX_DEPLOY_RATE_PER_S  1.0f
#endif

#ifndef AIRBRAKE_CONTROL_DT_FALLBACK_S
#define AIRBRAKE_CONTROL_DT_FALLBACK_S  0.02f
#endif

#ifndef AIRBRAKE_PREDICT_DT_S
#define AIRBRAKE_PREDICT_DT_S           0.02f
#endif

#ifndef AIRBRAKE_PREDICT_MAX_STEPS
#define AIRBRAKE_PREDICT_MAX_STEPS      4000
#endif

#ifndef AIRBRAKE_SEARCH_STEPS
#define AIRBRAKE_SEARCH_STEPS           20
#endif

#ifndef AIRBRAKE_TARGET_DEADBAND_M
#define AIRBRAKE_TARGET_DEADBAND_M      5.0f
#endif

#ifndef AIRBRAKE_MIN_COAST_VELOCITY_MPS
#define AIRBRAKE_MIN_COAST_VELOCITY_MPS 1.0f
#endif

#ifndef AIRBRAKE_MAX_VALID_DT_S
#define AIRBRAKE_MAX_VALID_DT_S         0.25f
#endif

static AirbrakeAbgState_t abg;
static AirbrakeCommand_t last_command;
static AirbrakePwmWriter_t pwm_writer;
static float current_deployment;

static float clampf(float value, float min_value, float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

static uint8_t valid_float(float value)
{
    return (uint8_t)(!isnan(value) && !isinf(value));
}

static float deployment_to_pwm_us(float deployment)
{
    float limited = clampf(deployment, 0.0f, AIRBRAKE_MAX_DEPLOYMENT);
    return AIRBRAKE_SERVO_CLOSED_US +
           limited * (AIRBRAKE_SERVO_FULL_US - AIRBRAKE_SERVO_CLOSED_US);
}

static float cd_for_deployment(float deployment)
{
    float limited = clampf(deployment, 0.0f, AIRBRAKE_MAX_DEPLOYMENT);
    return AIRBRAKE_CD_CLEAN +
           limited * (AIRBRAKE_CD_FULL_DEPLOY - AIRBRAKE_CD_CLEAN);
}

static uint8_t airbrake_config_valid(void)
{
    if (AIRBRAKE_CONTROL_ENABLED == 0) return 0;
    if (AIRBRAKE_TARGET_APOGEE_M <= 0.0f) return 0;
    if (AIRBRAKE_ROCKET_MASS_KG <= 0.0f) return 0;
    if (AIRBRAKE_REFERENCE_AREA_M2 <= 0.0f) return 0;
    if (AIRBRAKE_CD_CLEAN <= 0.0f) return 0;
    if (AIRBRAKE_CD_FULL_DEPLOY < AIRBRAKE_CD_CLEAN) return 0;
    if (AIRBRAKE_AIR_DENSITY_KG_M3 <= 0.0f) return 0;
    if (AIRBRAKE_PREDICT_DT_S <= 0.0f) return 0;
    if (AIRBRAKE_SEARCH_STEPS <= 0) return 0;
    return 1;
}

static float select_measured_altitude(const FlightSensorData *sensorData)
{
    if (valid_float(sensorData->kalman_altitude)) {
        return sensorData->kalman_altitude;
    }

    return sensorData->altitude;
}

static void update_abg(float measured_altitude_m,
                       float measured_velocity_mps,
                       float dt_s)
{
    if (abg.initialized == 0) {
        abg.altitude_m = measured_altitude_m;
        abg.velocity_mps = valid_float(measured_velocity_mps) ?
                           measured_velocity_mps : 0.0f;
        abg.acceleration_mps2 = 0.0f;
        abg.initialized = 1;
        return;
    }

    float x_pred = abg.altitude_m +
                   abg.velocity_mps * dt_s +
                   0.5f * abg.acceleration_mps2 * dt_s * dt_s;
    float v_pred = abg.velocity_mps + abg.acceleration_mps2 * dt_s;
    float a_pred = abg.acceleration_mps2;
    float residual = measured_altitude_m - x_pred;

    abg.altitude_m = x_pred + AIRBRAKE_ABG_ALPHA * residual;
    abg.velocity_mps = v_pred + (AIRBRAKE_ABG_BETA / dt_s) * residual;
    abg.acceleration_mps2 = a_pred +
        ((2.0f * AIRBRAKE_ABG_GAMMA) / (dt_s * dt_s)) * residual;
}

static float rate_limit_deployment(float requested, float dt_s)
{
    float limited_request = clampf(requested, 0.0f, AIRBRAKE_MAX_DEPLOYMENT);
    float max_delta = AIRBRAKE_MAX_DEPLOY_RATE_PER_S * dt_s;
    float delta = limited_request - current_deployment;

    delta = clampf(delta, -max_delta, max_delta);
    current_deployment += delta;
    current_deployment = clampf(current_deployment, 0.0f, AIRBRAKE_MAX_DEPLOYMENT);

    return current_deployment;
}

static float choose_deployment(float altitude_m,
                               float velocity_mps,
                               float *chosen_apogee_m)
{
    float clean_apogee = airbrake_predict_apogee(altitude_m, velocity_mps, 0.0f);

    if (clean_apogee <= AIRBRAKE_TARGET_APOGEE_M + AIRBRAKE_TARGET_DEADBAND_M) {
        *chosen_apogee_m = clean_apogee;
        return 0.0f;
    }

    float best_deployment = 0.0f;
    float best_apogee = clean_apogee;
    float best_error = fabsf(clean_apogee - AIRBRAKE_TARGET_APOGEE_M);

    for (int i = 1; i <= AIRBRAKE_SEARCH_STEPS; i++) {
        float deployment = AIRBRAKE_MAX_DEPLOYMENT *
                           ((float)i / (float)AIRBRAKE_SEARCH_STEPS);
        float predicted_apogee =
            airbrake_predict_apogee(altitude_m, velocity_mps, deployment);
        float error = fabsf(predicted_apogee - AIRBRAKE_TARGET_APOGEE_M);

        if (error < best_error) {
            best_error = error;
            best_deployment = deployment;
            best_apogee = predicted_apogee;
        }
    }

    *chosen_apogee_m = best_apogee;
    return best_deployment;
}

static AirbrakeCommand_t build_command(float deployment,
                                       float predicted_apogee_m,
                                       uint8_t enabled,
                                       uint8_t valid)
{
    AirbrakeCommand_t command;

    command.deployment = clampf(deployment, 0.0f, AIRBRAKE_MAX_DEPLOYMENT);
    command.pwm_us = deployment_to_pwm_us(command.deployment);
    command.predicted_apogee_m = predicted_apogee_m;
    command.cd_effective = cd_for_deployment(command.deployment);
    command.abg_altitude_m = abg.altitude_m;
    command.abg_velocity_mps = abg.velocity_mps;
    command.abg_acceleration_mps2 = abg.acceleration_mps2;
    command.enabled = enabled;
    command.valid = valid;

    return command;
}

static void apply_command(const AirbrakeCommand_t *command)
{
    last_command = *command;

    if (pwm_writer != NULL) {
        pwm_writer(command->pwm_us);
    }
}

void airbrake_init(void)
{
    memset(&abg, 0, sizeof(abg));
    memset(&last_command, 0, sizeof(last_command));
    current_deployment = 0.0f;

    last_command = build_command(0.0f, 0.0f, 0, 0);
    apply_command(&last_command);
}

void airbrake_set_pwm_writer(AirbrakePwmWriter_t writer)
{
    pwm_writer = writer;
}

AirbrakeCommand_t airbrake_update(const FlightSensorData *sensorData,
                                  float dt_s,
                                  uint8_t coast_active)
{
    uint8_t valid = 1;
    uint8_t enabled = 0;
    float measured_altitude_m = 0.0f;
    float measured_velocity_mps = 0.0f;
    float requested_deployment = 0.0f;
    float predicted_apogee_m = 0.0f;

    if (sensorData == NULL) {
        airbrake_retract();
        return last_command;
    }

    if (dt_s <= 0.0f || dt_s > AIRBRAKE_MAX_VALID_DT_S ||
        !valid_float(dt_s)) {
        dt_s = AIRBRAKE_CONTROL_DT_FALLBACK_S;
    }

    measured_altitude_m = select_measured_altitude(sensorData);
    measured_velocity_mps = sensorData->kalman_velocity;

    if (!valid_float(measured_altitude_m)) valid = 0;
    if (!valid_float(measured_velocity_mps)) measured_velocity_mps = 0.0f;

    if (valid != 0) {
        update_abg(measured_altitude_m, measured_velocity_mps, dt_s);
    }

    enabled = (uint8_t)(coast_active != 0 &&
                        valid != 0 &&
                        airbrake_config_valid() != 0 &&
                        abg.velocity_mps > AIRBRAKE_MIN_COAST_VELOCITY_MPS);

    if (enabled != 0) {
        requested_deployment = choose_deployment(abg.altitude_m,
                                                 abg.velocity_mps,
                                                 &predicted_apogee_m);
    } else {
        predicted_apogee_m = valid ? abg.altitude_m : 0.0f;
    }

    float deployment = 0.0f;

    if (enabled != 0) {
        deployment = rate_limit_deployment(requested_deployment, dt_s);
    } else {
        current_deployment = 0.0f;
    }

    AirbrakeCommand_t command = build_command(deployment,
                                              predicted_apogee_m,
                                              enabled,
                                              valid);
    apply_command(&command);

    return last_command;
}

void airbrake_retract(void)
{
    current_deployment = 0.0f;
    AirbrakeCommand_t command = build_command(0.0f, 0.0f, 0, 0);
    apply_command(&command);
}

float airbrake_predict_apogee(float altitude_m,
                              float velocity_mps,
                              float deployment)
{
    if (!airbrake_config_valid()) return altitude_m;
    if (!valid_float(altitude_m) || !valid_float(velocity_mps)) return altitude_m;
    if (velocity_mps <= 0.0f) return altitude_m;

    float altitude = altitude_m;
    float velocity = velocity_mps;
    float dt = AIRBRAKE_PREDICT_DT_S;
    float cd = cd_for_deployment(deployment);

    for (int i = 0; i < AIRBRAKE_PREDICT_MAX_STEPS && velocity > 0.0f; i++) {
        float drag_force = 0.5f *
                           AIRBRAKE_AIR_DENSITY_KG_M3 *
                           velocity * velocity *
                           cd *
                           AIRBRAKE_REFERENCE_AREA_M2;
        float acceleration = -AIRBRAKE_GRAVITY_MPS2 -
                             (drag_force / AIRBRAKE_ROCKET_MASS_KG);

        altitude += velocity * dt + 0.5f * acceleration * dt * dt;
        velocity += acceleration * dt;
    }

    return altitude;
}

float airbrake_get_deployment(void)
{
    return current_deployment;
}

float airbrake_get_pwm_us(void)
{
    return last_command.pwm_us;
}

const AirbrakeCommand_t *airbrake_get_last_command(void)
{
    return &last_command;
}
