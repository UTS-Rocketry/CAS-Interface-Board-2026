#ifndef AIRBRAKE_H
#define AIRBRAKE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "flight_sensors.h"
#include <stdint.h>

typedef void (*AirbrakePwmWriter_t)(float pwm_us);

typedef struct {
    float altitude_m;
    float velocity_mps;
    float acceleration_mps2;
    uint8_t initialized;
} AirbrakeAbgState_t;

typedef struct {
    float deployment;
    float pwm_us;
    float predicted_apogee_m;
    float cd_effective;
    float abg_altitude_m;
    float abg_velocity_mps;
    float abg_acceleration_mps2;
    uint8_t enabled;
    uint8_t valid;
} AirbrakeCommand_t;

void airbrake_init(void);
void airbrake_set_pwm_writer(AirbrakePwmWriter_t writer);
AirbrakeCommand_t airbrake_update(const FlightSensorData *sensorData,
                                  float dt_s,
                                  uint8_t coast_active);
void airbrake_retract(void);

float airbrake_predict_apogee(float altitude_m,
                              float velocity_mps,
                              float deployment);
float airbrake_get_deployment(void);
float airbrake_get_pwm_us(void);
const AirbrakeCommand_t *airbrake_get_last_command(void);

#ifdef __cplusplus
}
#endif

#endif
